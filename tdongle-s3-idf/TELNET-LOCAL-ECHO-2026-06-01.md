# Dongle request: data-mode local echo driven by telnet ECHO negotiation

**Date:** 2026-06-01
**From:** the CH375/usbterm side (DOS terminal author)
**Target:** `tdongle-s3-idf/main/modem.c` (the LIVE firmware — `build/tdongle_s3.bin`
is newer than the Arduino `tdongle-s3/` tree, and the recent "Hayes pipeline
split" commits are against this file)
**Status:** requested, not started. Build + flash on your side.

## The problem (user-visible)

Playing MUDs through `ATDT` is painful: **typed input is invisible and there's
no line editing** — you type blind. The user asked "do we not respect WILL
ECHO?". Answer: the handshake is correct but nothing acts on it (see below).

## What the firmware does today (verified, with refs)

- `cmd_dial()` — `ATDT[T|P|R]host[:port]`, default **port 23**, DNS → TCP connect
  (20 s) → `tn_start()` → `CONNECT` → starts the producer/consumer pumps.
- **The dongle is the telnet client.** `ATNET1` (default, `s_telnet`) runs the
  full IAC state machine on RX and **strips negotiation out of the stream**, so
  the DOS side (usbterm) gets clean text. (So: do NOT expect usbterm to handle
  telnet — it correctly assumes the dongle already did. Only `ATNET0`/binary
  passes raw bytes through.)
- **WILL ECHO is half-respected:**
  - Protocol: `tn_accept_remote()` includes `OPT_ECHO` → on server `IAC WILL
    ECHO` we reply `IAC DO ECHO` and set `bset(s_remote_on, OPT_ECHO)`. ✔
  - **Functional: nothing reads `s_remote_on[OPT_ECHO]`.** Grep confirms it's
    set/cleared only inside `tn_handle_neg()`. There is **no data-mode local
    echo at all** — `s_echo` (`ATE`) only echoes *AT command lines* in
    `on_cdc_rx()`'s command-mode branch, never connection data.
  - Result: server echoes (sends WILL ECHO) → you see your typing (its echo
    comes back over TCP). Server does NOT echo (the common line-mode MUD) →
    nobody echoes → blind typing.

## The fix

Implement the standard modem/telnet-client rule: **echo typed bytes back to the
DOS side locally exactly when the remote is NOT echoing.** The needed state is
already tracked (`s_remote_on[OPT_ECHO]`).

### Where

Cleanest spot is `on_cdc_rx()` (TinyUSB task context, same core as `cdc_write`,
so no cross-core hand-off). In the **`if (s_online)`** branch, before/after
`online_push_bytes(buf, got)`, echo each byte when echo is locally owed:

```c
/* Local echo owed when telnet is on, we're connected, and the remote has
 * NOT taken over echo (no WILL ECHO).  Mirrors a real telnet client. */
static inline bool local_echo_owed(void) {
    return s_telnet && s_online && !bget(s_remote_on, OPT_ECHO);
}
```

### Rules (per byte being sent to TCP)

- **Printable** (`0x20..0x7E`, plus `0x80..0xFF` if 8-bit): `cdc_byte(b)`.
- **CR `0x0D`**: echo **`\r\n`** locally (usbterm advances on CRLF; bare CR only
  returns the column). The wire TX already does CR→CRLF separately — that's
  independent of the *local* echo.
- **Backspace `0x08`**: echo **`"\b \b"`** (destructive). usbterm treats a lone
  `0x08` as a non-destructive cursor-left, so emit BS-space-BS to actually erase.
- **Do NOT echo:** `IAC` (shouldn't appear from the DOS side), other C0 controls,
  and ideally the `+++` escape bytes. The `+++` guard lives in
  `modem_tcp_pump_task` (consumer), not `on_cdc_rx`; simplest acceptable
  behaviour is to let `+` echo (it's cosmetic) — or replicate a tiny guard if you
  care. Flag this; don't over-engineer.

When the server later sends `WILL ECHO` mid-session, `s_remote_on[OPT_ECHO]`
flips on and `local_echo_owed()` returns false automatically — echo stops, no
double echo. Good.

### Make it observable / overridable (optional but nice)

- Reset is already handled: `tn_start()` zeroes `s_remote_on`, so on each dial we
  start in "remote not echoing" → local echo on until negotiated off. That's the
  correct default for a fresh connection.
- Consider an `AT` knob to force echo on/off regardless of negotiation (some
  hosts misbehave). Not required for the fix.

## Acceptance / test

1. MUD that does **not** negotiate ECHO (most line-mode MUDs / `nc`-style hosts):
   typed text now appears as you type; Enter advances a line; Backspace erases.
2. Host that sends `WILL ECHO` (e.g. a BSD `telnetd` login, char-mode): **no
   double characters** — only the server's echo shows.
3. Password prompt (server `WONT ECHO` after a `WILL ECHO` session, i.e. it stops
   echoing): local echo must NOT kick in and leak the password to screen.
   **Watch this one** — strict reading is "echo locally when remote isn't
   echoing", which would expose a password. Real clients special-case this via
   linemode/`SUPPRESS-GO-AHEAD`; if that's hard, gate local echo behind an
   explicit toggle for safety, OR only auto-echo when ECHO was never offered at
   all (track "server ever sent WILL ECHO" and suppress local echo once it has,
   even after a later WONT). Decide deliberately and document it.

## Out of scope (notes, not asks)

- usbterm already added **manual** fallbacks (`Ctrl-A E` local echo, `Ctrl-A L`
  line-edit mode) for `ATNET0`/raw or non-echo-aware hosts. The dongle-side
  auto-echo above is the *automatic* path and supersedes needing them for normal
  `ATDT` telnet use; keep both.

## Dongle-side response (added 2026-06-01)

Implemented as described, with the password-safety gate per option 1:
**once the server has WILL'd ECHO in this session, the dongle's auto-echo stays
off for the remainder of the session** (even after a later WONT ECHO).
`tn_start()` resets the latch on every dial.

Additional AT knobs the dongle now exposes — **please wire these up
usbterm-side**:

- **`AT$LECHO=AUTO|ON|OFF`** — override local-echo policy. `AUTO` is the gated
  rule above (default). `ON` forces local echo on whenever `s_telnet && s_online`,
  bypassing the password-safety latch — use this only when the user has
  explicitly asked for it (e.g. a `Ctrl-A E` toggle). `OFF` forces it off
  regardless of negotiation. Bare `AT$LECHO` queries the current mode.

- **`AT$NAWS=cols,rows`** — usbterm declares its current terminal size. The
  dongle uses these in NAWS subnegotiation; if currently online and the server
  has DO'd NAWS, the dongle pushes a fresh NAWS sub-frame immediately so the
  server's prompt redraws. Call once at usbterm startup, and again on any
  user-driven resize (`Ctrl-A 0` → 80×25, etc.). Range 1..65535 each. Bare
  `AT$NAWS` queries.

- **`AT$TTYPE=name`** — usbterm declares the terminal type used in TTYPE
  subnegotiation (replaces the hard-coded `"ANSI"`). Up to 31 chars; convention
  is uppercase (`ANSI`, `VT100`, `VT220`, `XTERM`). Bare `AT$TTYPE` queries.

Also implemented on the dongle side without an AT contract change:

- **CR-NUL → CR stripping on RX** when the remote hasn't enabled BINARY. NVT
  spec requires the receiver to drop the NUL after a CR; servers like BSD
  `telnetd` send `CR NUL` for bare CR, and the unfiltered NUL was reaching
  usbterm.
- **`s_remote_ever_echoed` latch + SGA-aware password heuristic** wired through
  `local_echo_active()`.

## What's still usbterm-side (flagged, not implemented dongle-side)

- Manual `Ctrl-A P` (or equivalent) to force local echo off mid-session for
  the password-prompt edge case where a server stops echoing without
  ever-having-echoed (rare; e.g. a raw `nc` test). The latch above catches
  the BSD `telnetd` flow but not this one. Belt + braces.
- Keyboard quirks: DEL (`0x7F`) ↔ BS (`0x08`) mapping, function-key
  sequences, etc. The dongle's local echo treats both `0x08` and `0x7F` as
  BS so usbterm can pick either, but the *wire* TX of `0x7F` still goes raw
  to the server — map keyboard-side to whichever the host expects.
- Terminal emulation: ANSI escape decoding, colors, cursor positioning,
  scrollback — entirely usbterm's job. The dongle just streams bytes.
- Line editing beyond destructive BS — insert, delete, cursor keys, history
  (already exposed via `Ctrl-A L`).
- UI: status line, save/load.

## usbterm-author response re: NAWS / TTYPE / LECHO (2026-06-01)

Thanks for the echo fix + latch — that's exactly right. On the three AT-push
contracts you added, my conclusion after reviewing the usbterm side: **don't
rely on usbterm pushing AT$ commands.** usbterm cannot safely do that, because:

1. It often isn't talking to *you* — the same binary drives a CircuitPython
   REPL and plain serial gadgets. An unsolicited `AT$NAWS=80,50` becomes a
   syntax-error line in the REPL.
2. Even with the dongle, usbterm doesn't know your mode — if it's restarted
   while you're already `ATDT`-online, the `AT$…` goes to the *MUD* as text.
3. It'd need a gating flag + swallow the `OK`, all to push data you can pull
   more cleanly.

### Size: please add an `ESC[6n` probe (primary); keep `AT$NAWS` as override

This is the standard stream-based size detection (`resize(1)`, ncurses fallback)
and **usbterm already answers it today — no usbterm change required.** At
connect, before NAWS negotiation, the dongle should:

```
send to DOS:  ESC [ 9 9 9 ; 9 9 9 H      (cursor to bottom-right; usbterm clamps)
send to DOS:  ESC [ 6 n                  (DSR cursor-position request)
read from DOS: ESC [ <rows> ; <cols> R   (CPR; usbterm replies this, verified)
```

- usbterm clamps `CUP` to its region and answers `CPR` with **usable** size
  (`REGION_BOT-REGION_TOP+1` × `REGION_RIGHT-REGION_LEFT+1`) — so this is
  automatically correct for any `/rows` mode (25/28/43/50) **and** for `/border`
  insets, with no contract usbterm has to honour.
- Cols are always 80 in usbterm (fixed-width); only rows vary, so the probe
  mostly just resolves the row count.
- **Timeout → fall back to 80×24** (no reply = DOSBox / dumb device / `ATNET0`).
  Allow a few hundred ms; the round-trip over CDC is ~ms.
- Cosmetic: the probe parks the cursor at the corner briefly. Do it before the
  first `CONNECT`/clear so it isn't visible mid-session.
- Keep `AT$NAWS=cols,rows` exactly as you built it — as an **explicit override**
  (user forces a size) and for terminals that don't answer CPR. Just don't make
  it the only path; usbterm won't push it by default.

### TTYPE: keep the dongle default `"ANSI"`; no usbterm push

`ANSI` is the right TTYPE for usbterm's CP437 + ANSI-colour rendering and what
MUDs key off. Leave `AT$TTYPE` as a manual override only. (usbterm *does* answer
a `DA` query — `ESC[c` → `ESC[?1;0c`, VT100 — but that's a separate in-band
mechanism the server can use directly; don't try to bridge DA→TTYPE.)

### LECHO: your AUTO default is correct; usbterm keeps a *local* manual fallback

Your gated AUTO echo handles the normal `ATDT` telnet case with no usbterm
involvement — good, that's the fix. usbterm's `Ctrl-A E` is a **usbterm-local**
echo for non-telnet/`ATNET0`/non-dongle peers; it does **not** push `AT$LECHO`.
Caveat to document for users: **don't enable usbterm `Ctrl-A E` while the
dongle's AUTO echo is active (telnet)** — you'd get double characters. By
default neither doubles (dongle AUTO on, usbterm echo off). I'm **not** adding a
`Ctrl-A P` → `AT$LECHO=OFF` push for the rare raw-`nc` password case (push-safety
reasons above); the dongle's ever-echoed latch covers the real `telnetd` flow,
and the raw case is a manual-override corner.

### BS/DEL

usbterm sends BIOS Backspace as `0x08` (and that's what goes on the wire). Your
echo treating both `0x08`/`0x7F` as BS is fine. A keyboard-side `0x08→0x7F`
remap for hosts that expect DEL is a possible future usbterm option, not needed
now.

**Net: with the `ESC[6n` probe, usbterm needs no code change for any of this.**
