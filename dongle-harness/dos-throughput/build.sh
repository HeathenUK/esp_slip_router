#!/usr/bin/env bash
#
# Build THROUGHPUT.EXE for real DOS using Open Watcom v2 (medium model,
# matching the usbterm.exe build out of ~/CH375).
#
# Mirrors the "Hosted Build Used Here" recipe in CH375/BUILD-WATCOM.md:
#   bwcl    -- C front end (compile only, -c, no link)
#   bwlink  -- linker, takes a response file
# crtmm.lib (C run-time, medium model, DOS .086) is reused from the CH375
# build tree -- we don't redo bwlib here. If that .lib is missing, follow
# BUILD-WATCOM.md to rebuild it once.
#
# Output: throughput.exe in this directory.
#
set -eu

OW=${OW:-$HOME/CH375/ow}
BWCL=${BWCL:-$OW/build/binbuild/bwcl}
BWLINK=${BWLINK:-$OW/build/binbuild/bwlink}
CRTMM=${CRTMM:-$HOME/CH375/crtmm.lib}
HDR=${HDR:-$OW/bld/hdr/dos/h}

cd "$(dirname "$0")"

if [ ! -x "$BWCL" ];   then echo "build: bwcl not found at $BWCL"   >&2; exit 1; fi
if [ ! -x "$BWLINK" ]; then echo "build: bwlink not found at $BWLINK" >&2; exit 1; fi
if [ ! -f "$CRTMM" ];  then echo "build: crtmm.lib not found at $CRTMM (rebuild via CH375/BUILD-WATCOM.md)" >&2; exit 1; fi

# Compile.
#   -bt=dos  target DOS real-mode
#   -mm      medium memory model (same as usbterm.c)
#   -0       no opt -- bring-up parity with usbterm
#   -os      favour size
#   -zq      quiet
#   -wx      warnings-as-errors
#   -lr      DOS real-mode (link metadata; harmless at compile)
#   -c       compile only, no link
#   -fo=throughput.obj   IMPORTANT: explicit .obj output. bwcl otherwise
#                        emits .o, and bwlink silently picks up any stale
#                        throughput.obj sitting in the dir, producing a
#                        binary built from a previous compile. See user
#                        memory note "usbterm manual build gotcha".
# Build both throughput.exe and httpget.exe with the same recipe.
build_one() {
    local src="$1"
    local stem="${src%.c}"
    env PATH="$OW/build/binbuild:$PATH" \
      "$BWCL" -zq -bt=dos -lr -mm -0 -os -wx -c \
        -i="$HDR" \
        -fo="${stem}.obj" \
        "$src"
    cat > "${stem}_mm.rsp" <<EOF
option quiet
format dos
file ${stem}.obj
library crtmm
name ${stem}.exe
EOF
    env PATH="$OW/build/binbuild:$PATH" "$BWLINK" @"${stem}_mm.rsp"
}

cp "$CRTMM" ./crtmm.lib
build_one throughput.c
build_one httpget.c
build_one timer.c
build_one magicout.c
build_one atq.c
build_one snap.c

echo "build: ok"
ls -l throughput.exe httpget.exe timer.exe magicout.exe atq.exe snap.exe
