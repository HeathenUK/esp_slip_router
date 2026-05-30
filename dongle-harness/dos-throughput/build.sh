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
env PATH="$OW/build/binbuild:$PATH" \
  "$BWCL" -zq -bt=dos -lr -mm -0 -os -wx -c \
    -i="$HDR" \
    -fo=throughput.obj \
    throughput.c

# Copy the run-time lib in next to us so the linker response file can
# refer to it by bare name. (bwlink's library search path handling under
# the hosted build is finicky; co-locating sidesteps it.)
cp "$CRTMM" ./crtmm.lib

# Write a linker response file. Format taken from CH375's usbterm_mm.rsp.
cat > throughput_mm.rsp <<'EOF'
option quiet
format dos
file throughput.obj
library crtmm
name throughput.exe
EOF

env PATH="$OW/build/binbuild:$PATH" "$BWLINK" @throughput_mm.rsp

echo "build: ok -- throughput.exe"
ls -l throughput.exe
