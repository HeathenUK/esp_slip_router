#!/bin/bash
# Sync the minimal PROFILE-only TDONGLE set to /Volumes/95GD5428/TDONGLE/.
#
# Legacy individual-phase BATs and EXEs (ATQ, HTTPGET, MAGICOUT, MTCPGET,
# THRPUT, USBTERM, P-*.BAT, T-*.BAT, SETPATH.BAT, README.TXT, P-README.TXT)
# are archived in cf-deploy/legacy/ and on the CF in TDONGLE/LEGACY/.
# PROFILE.EXE is the single-binary harness that replaces all of them.

set -u
DEST=/Volumes/95GD5428/TDONGLE
SRC=$(cd "$(dirname "$0")" && pwd)
if [ ! -d "$DEST" ]; then
    echo "CF not mounted at /Volumes/95GD5428 -- plug it in first" >&2
    exit 1
fi
mkdir -p "$DEST"

# Binaries. PROFILE.EXE replaces ATQ/HTTPGET/MAGICOUT/MTCPGET/THRPUT.
# TIMER.EXE is mTCP's per-CPU clock calibration -- still needed for first
# boot to populate TIMER.DAT. BNU.COM is the FOSSIL driver, loaded at
# boot via CONFIG.SYS/AUTOEXEC.BAT outside this script's scope.
cp "$SRC/timer.exe"                  "$DEST/TIMER.EXE"
cp "$SRC/../mtcpget-src/PROFILE.EXE" "$DEST/PROFILE.EXE"
cp "$HOME/FOSSLIP/FOSSLIP.EXE"       "$DEST/FOSSLIP.EXE"

# Scripts + configs from cf-deploy/. PROFILE.BAT is the thin wrapper that
# sets MTCPCFG; MTCP.CFG is the mTCP config the wrapper points at.
cp "$SRC/cf-deploy/PROFILE.BAT" "$DEST/PROFILE.BAT"
cp "$SRC/cf-deploy/MTCP.CFG"    "$DEST/MTCP.CFG"

sync
echo "deployed:"
ls -la "$DEST/" | grep -vE "^total|^d|^_|\\._"
