#!/usr/bin/env bash
#
# dosongle.sh -- FAT-over-HTTP toolkit for the T-Dongle S3 DOSONGLE volume.
#
# Wraps the messy owner-swap + macOS-unmount + PUT + verify dance documented
# in HARNESS.md into a small CLI. Also sourceable as a library so the FAT-side
# primitives live in one place if you want to script on top of it.
#
# CLI:
#   dosongle.sh ls
#   dosongle.sh status [--raw]
#   dosongle.sh mode
#   dosongle.sh push <local-file> [<remote>]
#   dosongle.sh pull <remote> [<local>]
#   dosongle.sh cat  <remote>
#   dosongle.sh rm   <remote> -y
#   dosongle.sh swap host|device
#   dosongle.sh format -y
#   dosongle.sh reset
#   dosongle.sh help
#
# As a library:
#   . dosongle.sh
#   # then call: dosongle_push / dosongle_pull / dosongle_rm /
#   #            dosongle_swap / dosongle_verify_remote / dosongle_mac_unmount
#
# Env: DONGLE_HOST=dosongle.local (override mDNS name)
#      CURL_TIMEOUT=15            (per-request, seconds)
#
# Exit codes:
#   0  success
#   1  dongle unreachable
#   2  bad args / missing local file
#   3  owner swap failed
#   4  PUT / DELETE / GET failed
#   5  post-op verification failed
#   6  destructive op without -y

set -euo pipefail

# ----- config (caller-overridable) ------------------------------------------

DONGLE_HOST="${DONGLE_HOST:-dosongle.local}"
BASE_URL="${BASE_URL:-http://$DONGLE_HOST}"
CURL_TIMEOUT="${CURL_TIMEOUT:-15}"
CURL_PUT_TIMEOUT="${CURL_PUT_TIMEOUT:-120}"

# ----- low-level primitives -------------------------------------------------

# 200-only curl; -f makes non-2xx exit non-zero AND skip writing -o.
dosongle_curl_q()   { curl -fsS --max-time "$CURL_TIMEOUT" "$@"; }
# Permissive curl for endpoints where any response is informative.
dosongle_curl_any() { curl  -sS --max-time "$CURL_TIMEOUT" "$@"; }

dosongle_up() { dosongle_curl_q "$BASE_URL/status" >/dev/null 2>&1; }

dosongle_status_json() { dosongle_curl_any "$BASE_URL/status"; }

# Current FAT ownership: "host-write" or "device-write" (or "?" on error).
# /status reports this as the top-level "mode" field. NB: this is the FAT
# ownership, NOT the CDC personality (SLIP/MODEM) -- that isn't exposed
# over HTTP.
dosongle_mode() {
  dosongle_status_json | sed -n 's/.*"mode":"\([^"]*\)".*/\1/p' \
    | head -1 | tr -d '\n' ; echo
}

# Switch FAT ownership and verify the switch took. Idempotent.
dosongle_swap() {
  local target="$1"
  case "$target" in host-write|device-write) ;; \
    *) echo "dosongle_swap: bad target '$target'" >&2; return 2 ;;
  esac
  local cur ; cur="$(dosongle_mode)"
  if [[ "$cur" == "$target" ]]; then return 0; fi
  dosongle_curl_q -X POST "$BASE_URL/$target" >/dev/null || return 3
  cur="$(dosongle_mode)"
  [[ "$cur" == "$target" ]] || { echo "dosongle_swap: want=$target got=$cur" >&2; return 3; }
}

# No forced dismount. The firmware's 409 guard is the safeguard: it
# refuses /fs reads while the host is actively transferring and /fs
# writes while the host has the volume mounted, returning fast instead
# of wedging. Under normal use the host isn't contending (reads while
# idle are safe; writes happen when the volume isn't host-mounted), so
# plain ops just work; a 409 is the safety net telling you to retry
# when idle or unmount manually.
#
# We deliberately do NOT call POST /eject here: on macOS that's a
# one-way door (diskarbitrationd tears down the IOMedia node and won't
# rebuild it from any device-side signal -- verified, even a full USB
# re-enumeration doesn't bring it back; only a physical replug does).
# If you genuinely need the host off the volume, `diskutil unmount
# /Volumes/DOSONGLE` is the reversible host-side primitive (pairs with
# `diskutil mount`); /eject is reserved as a manual "force off" hammer.
dosongle_host_release()  { :; }   # retained as no-op for call-site compatibility
dosongle_host_restore()  { :; }
dosongle_mac_unmount()   { :; }   # back-compat alias

# Derive the 8.3 uppercase remote name from a local path.
dosongle_remote_name() {
  basename "$1" | tr '[:lower:]' '[:upper:]'
}

# Validate strict DOS 8.3: 1..8 chars stem, optional .ext of 1..3 chars,
# alphabet [A-Z 0-9 _ $ ~]. The firmware enforces this server-side and
# returns 400 on violation; we check client-side to fail fast (otherwise
# we'd do the whole owner-swap dance just to get a 400). Synthesised
# long-name shadows that contain '~' are accepted (they're how the
# firmware itself exposes macOS-side long-named files).
dosongle_valid_83() {
  local n="$1"
  # DOS 8.3 accepts hyphens too (P-MTCP-L.BAT etc.). Char set: A-Z 0-9
  # _ $ ~ -. Stem 1..8, optional .ext 1..3. The firmware rejects only
  # the symbols that DOS itself reserves (space, " * + , / : ; < = > ?
  # [ \ ] |), so we mirror that policy here client-side.
  if [[ ! "$n" =~ ^[A-Z0-9_$~-]{1,8}(\.[A-Z0-9_$~-]{1,3})?$ ]]; then
    echo "dosongle: '$n' is not a valid DOS 8.3 name" >&2
    echo "        stem 1..8 chars, optional .ext 1..3 chars, [A-Z 0-9 _ \$ ~ -]" >&2
    return 2
  fi
}

# Verify a remote name appears in /list. Best effort -- /list returns
# text in either ownership mode.
dosongle_verify_remote() {
  local remote="$1"
  local listing
  listing="$(dosongle_curl_any "$BASE_URL/list" 2>/dev/null || true)"
  echo "$listing" | grep -qi -- "$remote"
}

# ----- high-level operations ------------------------------------------------

# push <local> [<remote>] -- owner swap, mac-unmount, PUT, swap back, verify.
# Always ends in host-write (so the host can see the change after re-mounting).
dosongle_push() {
  local local_path="$1"
  local remote="${2:-$(dosongle_remote_name "$local_path")}"
  [[ -f "$local_path" ]] || { echo "dosongle push: no such file: $local_path" >&2; return 2; }
  dosongle_valid_83 "$remote" || return 2
  dosongle_up || { echo "dosongle push: $BASE_URL unreachable" >&2; return 1; }

  # This firmware has no host-write/device-write ownership endpoints --
  # the PUT handler coordinates internally (disk_take_for_firmware) and
  # the host-mounted 409 guard + eject is what keeps us safe. So the
  # flow is just: release the host -> PUT -> restore.
  dosongle_host_release
  if ! curl -fsS --max-time "$CURL_PUT_TIMEOUT" \
            -T "$local_path" "$BASE_URL/fs/$remote" >/dev/null ; then
    dosongle_host_restore
    echo "dosongle push: PUT failed for $remote" >&2
    return 4
  fi
  if ! dosongle_verify_remote "$remote"; then
    dosongle_host_restore
    echo "dosongle push: warning -- $remote not visible in /list after PUT" >&2
    return 5
  fi
  dosongle_host_restore   # re-present so the host remounts + sees the change
  printf '%s -> %s/fs/%s  (%s bytes)\n' \
    "$local_path" "$BASE_URL" "$remote" "$(wc -c <"$local_path" | tr -d ' ')"
}

# pull <remote> [<local>] -- GET to a file. Brackets the read with
# host_release/host_restore: the firmware 409s /fs reads while the host
# (macOS) is actively probing the block device, so we eject first for a
# clean read, then remount. On a host that isn't touching the volume
# (idle DOS at the prompt) the release is a cheap no-op.
dosongle_pull() {
  local remote="$1"
  local local_path="${2:-./$remote}"
  dosongle_up || { echo "dosongle pull: $BASE_URL unreachable" >&2; return 1; }
  dosongle_host_release
  if ! dosongle_curl_q -o "$local_path" "$BASE_URL/fs/$remote" ; then
    rm -f "$local_path"
    dosongle_host_restore
    echo "dosongle pull: GET failed for $remote (does it exist? $0 ls to check)" >&2
    return 4
  fi
  dosongle_host_restore
  printf '%s/fs/%s -> %s  (%s bytes)\n' \
    "$BASE_URL" "$remote" "$local_path" "$(wc -c <"$local_path" | tr -d ' ')"
}

# cat <remote> -- GET to stdout. Useful for PROFILE.LOG-like text files.
# Bracketed like pull (the firmware 409s reads while the host is probing).
dosongle_cat() {
  local remote="$1"
  dosongle_host_release
  local rc=0
  dosongle_curl_q "$BASE_URL/fs/$remote" || rc=$?
  dosongle_host_restore
  return $rc
}

# rm <remote> -- requires -y; no interactive prompt (so this is scriptable).
dosongle_rm() {
  local remote="$1"
  dosongle_valid_83 "$remote" || return 2
  dosongle_up || { echo "dosongle rm: $BASE_URL unreachable" >&2; return 1; }
  dosongle_host_release
  if ! dosongle_curl_q -X DELETE "$BASE_URL/fs/$remote" >/dev/null ; then
    dosongle_host_restore
    echo "dosongle rm: DELETE failed for $remote" >&2
    return 4
  fi
  dosongle_host_restore
  echo "deleted $remote"
}

dosongle_ls() {
  dosongle_host_release
  local rc=0
  dosongle_curl_q "$BASE_URL/list" || rc=$?
  dosongle_host_restore
  return $rc
}

dosongle_format() {
  dosongle_up || { echo "dosongle format: $BASE_URL unreachable" >&2; return 1; }
  dosongle_mac_unmount
  dosongle_curl_q -X POST "$BASE_URL/format" >/dev/null \
    || { echo "dosongle format: POST /format failed" >&2; return 4; }
  # /format is asynchronous; poll /status.format.running until done.
  local deadline=$(( SECONDS + 60 ))
  while (( SECONDS < deadline )); do
    local running last_ok
    running="$(dosongle_status_json | sed -n 's/.*"format":{"running":\([a-z]*\),"last_ok":\([a-z]*\).*/\1/p')"
    [[ "$running" == "false" ]] && break
    sleep 2
  done
  local last_ok
  last_ok="$(dosongle_status_json | sed -n 's/.*"format":{"running":[a-z]*,"last_ok":\([a-z]*\).*/\1/p')"
  if [[ "$last_ok" != "true" ]]; then
    dosongle_host_restore
    echo "dosongle format: completed but last_ok=$last_ok (check /status)" >&2
    return 4
  fi
  dosongle_host_restore   # re-present the freshly-formatted volume
  echo "format ok"
}

dosongle_reset() {
  dosongle_curl_any -X POST "$BASE_URL/reset" >/dev/null || true
  echo "reset issued"
}

# ota <firmware.bin> -- HTTP OTA. POSTs the raw app firmware.bin to /ota;
# the firmware writes the inactive OTA slot, sets it as boot, replies,
# then esp_restarts. After the POST returns we give it a moment to
# actually start the reboot, then poll /status until it's back up.
#
# This is the preferred OTA path -- it works regardless of whether the
# dongle is plugged into the Mac or the Pocket386, because everything
# travels over WiFi. USB-CDC OTA via flash.sh is the fallback for when
# WiFi is broken or unconfigured.
#
# Only accepts firmware.bin (app slot only). To replace the bootloader
# or partition table use flash.sh --boot (ROM bootloader + esptool).
dosongle_ota() {
  local fw="$1"
  [[ -f "$fw" ]] || { echo "dosongle ota: no $fw" >&2; return 2; }
  dosongle_up || { echo "dosongle ota: $BASE_URL unreachable" >&2; return 1; }
  local sz; sz=$(wc -c <"$fw" | tr -d ' ')
  echo "ota: POST $fw ($sz bytes) -> $BASE_URL/ota"
  if ! curl -fsS --max-time 180 -X POST --data-binary @"$fw" \
              -H 'Content-Type: application/octet-stream' \
              "$BASE_URL/ota" ; then
    echo "dosongle ota: POST /ota failed" >&2
    return 4
  fi
  # Firmware sends its OTA OK reply, then calls esp_restart. Give the
  # reboot a moment to actually start before we begin polling.
  sleep 3
  echo "ota: waiting up to 60s for the dongle to come back"
  local deadline=$(( SECONDS + 60 ))
  while (( SECONDS < deadline )); do
    if dosongle_up; then echo "ota: back up"; return 0; fi
    sleep 2
  done
  echo "dosongle ota: dongle did not come back within 60s" >&2
  return 4
}

# type <body> -- POST raw body to /type. The body is parsed by the
# firmware's typing DSL (see dongle_kbd.h: <ENTER> <TAB> <F1>..<F12>
# <CTRL+x> <DELAY=ms>, plain ASCII passes through, '<<' is literal '<').
# Keystrokes are emitted as USB HID reports to whatever host the dongle
# is plugged into. This is NOT a CDC write -- it cannot escape a
# SLIP-stuck dongle, and the keystrokes land in whatever window has
# focus on the receiving host. Echoes the firmware's response ("typed
# N bytes" on success).
dosongle_type() {
  local body="$1"
  dosongle_up || { echo "dosongle type: $BASE_URL unreachable" >&2; return 1; }
  dosongle_curl_q -X POST --data-binary "$body" "$BASE_URL/type"
}

# type-log [N] -- GET /type-log (optionally tailed to the last N lines).
# Firmware-side ring buffer of recent type events: "type begin", "token",
# "literal", "emit key=", "type end". Useful for confirming an HID
# lever actually fired and what was parsed.
dosongle_type_log() {
  local n="${1:-}"
  if [[ -n "$n" ]]; then
    dosongle_curl_q "$BASE_URL/type-log" | tail -"$n"
  else
    dosongle_curl_q "$BASE_URL/type-log"
  fi
}

# disk-log [N] -- GET /disk-log (optionally tailed to the last N lines).
# RAM ring buffer of disk-side diagnostics. The T-Dongle S3 has no exposed
# UART pads, so this is the ONLY way to recover the firmware's MSC + raw-FAT
# failure traces from the Mac side. Survives any DOS / host crash; lost
# only on actual dongle reset.
dosongle_disk_log() {
  local n="${1:-}"
  if [[ -n "$n" ]]; then
    dosongle_curl_q "$BASE_URL/disk-log" | tail -"$n"
  else
    dosongle_curl_q "$BASE_URL/disk-log"
  fi
}

# usb-log [N] -- GET /usb-log. Device-side USB enumeration events
# (tud_ready transitions, suspend/resume, deferred-WiFi markers). Use
# this to confirm whether SET_CONFIGURATION actually succeeded when a
# host complains about enumeration.
dosongle_usb_log() {
  local n="${1:-}"
  if [[ -n "$n" ]]; then
    dosongle_curl_q "$BASE_URL/usb-log" | tail -"$n"
  else
    dosongle_curl_q "$BASE_URL/usb-log"
  fi
}

# ----- CLI dispatch ---------------------------------------------------------

# Detect source-vs-execute: when sourced, just expose functions and stop.
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  return 0 2>/dev/null || true
fi

usage() {
  cat <<EOF
usage: $(basename "$0") <command> [args...]

  ls                              list files on the DOSONGLE FAT
  status [--raw]                  /status (pretty by default, --raw is the JSON)
  mode                            current FAT ownership (host-write|device-write)
  push <local> [<remote>]         upload (8.3 uppercased from local basename
                                  unless given); owner-swap, PUT, swap back
  pull <remote> [<local>]         download (defaults to ./<remote>); no swap
  cat  <remote>                   GET <remote> to stdout
  rm   <remote> -y                delete (requires -y; owner-swap, DELETE)
  swap host|device                explicit owner-swap (alias targets accepted)
  format -y                       wipe + 512-byte-sector FAT12 (requires -y)
  reset                           POST /reset (esp_restart) -- also fixes a
                                  SLIP-stuck dongle (NVS default is MODEM)
  ota <firmware.bin>              HTTP OTA upload + wait for reboot; this is
                                  the preferred firmware-flash path because
                                  it works regardless of where the dongle is
                                  plugged in (Mac vs Pocket386)
  type <body>                     POST <body> to /type; firmware parses the
                                  typing DSL (<ENTER> <TAB> <F1>..<F12>
                                  <CTRL+x> <DELAY=ms>, '<<' = literal '<')
                                  and emits USB HID keystrokes to whatever
                                  host has the dongle. NOT a CDC write.
  type-log [N]                    GET /type-log (last N lines if given)
  disk-log [N]                    GET /disk-log -- MSC + raw-FAT failure
                                  trace in RAM (survives DOS crashes);
                                  the only way to recover dongle-side
                                  diagnostics on T-Dongle S3 (no UART pads)
  usb-log  [N]                    GET /usb-log -- USB device-state events
                                  (tud_ready, suspend/resume, deferred-WiFi
                                  markers). Confirms whether SET_CONFIG
                                  actually succeeded.
  help                            this

env: DONGLE_HOST=$DONGLE_HOST  CURL_TIMEOUT=$CURL_TIMEOUT
EOF
}

cmd="${1:-help}"; shift || true
case "$cmd" in
  ls)        dosongle_ls ;;
  status)
    if [[ "${1:-}" == "--raw" ]]; then dosongle_status_json
    else
      json="$(dosongle_status_json)"
      if command -v jq >/dev/null 2>&1; then echo "$json" | jq .
      else echo "$json"; fi
    fi ;;
  mode)      dosongle_mode ;;
  push)      [[ $# -ge 1 ]] || { usage >&2; exit 2; }
             dosongle_push "$@" ;;
  pull)      [[ $# -ge 1 ]] || { usage >&2; exit 2; }
             dosongle_pull "$@" ;;
  cat)       [[ $# -ge 1 ]] || { usage >&2; exit 2; }
             dosongle_cat "$@" ;;
  rm)
    [[ $# -ge 1 ]] || { usage >&2; exit 2; }
    name="$1"; shift
    [[ "${1:-}" == "-y" ]] || { echo "rm refuses without -y" >&2; exit 6; }
    dosongle_rm "$name" ;;
  swap)
    case "${1:-}" in
      host|host-write)     dosongle_swap host-write ;;
      device|device-write) dosongle_swap device-write ;;
      *) echo "swap target: host|device" >&2; exit 2 ;;
    esac ;;
  format)
    [[ "${1:-}" == "-y" ]] || { echo "format refuses without -y" >&2; exit 6; }
    dosongle_format ;;
  reset)     dosongle_reset ;;
  ota)
    [[ $# -ge 1 ]] || { usage >&2; exit 2; }
    dosongle_ota "$1" ;;
  type)
    [[ $# -ge 1 ]] || { usage >&2; exit 2; }
    dosongle_type "$1" ;;
  type-log)  dosongle_type_log "${1:-}" ;;
  disk-log)  dosongle_disk_log "${1:-}" ;;
  usb-log)   dosongle_usb_log  "${1:-}" ;;
  -h|--help|help) usage ;;
  *) echo "unknown: $cmd" >&2; usage >&2; exit 2 ;;
esac
