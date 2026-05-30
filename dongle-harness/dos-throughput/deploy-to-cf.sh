#!/bin/bash
# Sync everything to /Volumes/95GD5428/TDONGLE/ when CF is on the Mac.
set -u
DEST=/Volumes/95GD5428/TDONGLE
SRC=$(cd "$(dirname "$0")" && pwd)
if [ ! -d "$DEST" ]; then
    echo "CF not mounted at /Volumes/95GD5428 -- plug it in first" >&2
    exit 1
fi
mkdir -p $DEST
# binaries
cp $SRC/throughput.exe $DEST/THRPUT.EXE
cp $SRC/httpget.exe   $DEST/HTTPGET.EXE
cp $SRC/timer.exe     $DEST/TIMER.EXE
cp $SRC/atq.exe       $DEST/ATQ.EXE
cp $SRC/magicout.exe  $DEST/MAGICOUT.EXE
cp $SRC/../mtcpget-src/MTCPGET.EXE $DEST/MTCPGET.EXE
cp $HOME/FOSSLIP/FOSSLIP.EXE $DEST/FOSSLIP.EXE
cp $HOME/Downloads/bnu202/BNU.COM $DEST/BNU.COM
# scripts + configs (already CRLF / CR-only in cf-deploy/)
cp $SRC/cf-deploy/*.BAT $SRC/cf-deploy/*.TXT $SRC/cf-deploy/*.CFG $DEST/ 2>/dev/null || true
cp $SRC/cf-deploy/MTCP.CFG $DEST/MTCP.CFG
sync
echo "deployed:"
ls -la $DEST/ | grep -vE "^total|^d|^_|\\._"
