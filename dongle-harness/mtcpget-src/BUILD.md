# Building MTCPGET.EXE

Patched mTCP HTGET with periodic progress (every ~1s) and final KB/s line.

Prereqs:
- Open Watcom v2 at ~/CH375/ow (hosted build, contains bwpp/bwasm/bwlink/wmake)
- mTCP source 2025-01-10 unpacked at /tmp/mTCP-src_2025-01-10
- This MTCPGET.CPP, MAKEFILE, mtcpget.rsp copied into mTCP/APPS/HTGET/

Build:
```sh
cd ~/CH375/ow/build/binbuild
ln -sf bwpp wpp; ln -sf bwasm wasm; ln -sf bwlink wlink   # one-time

cd /tmp/mTCP-src_2025-01-10/APPS/HTGET
cp /Users/gadyke/esp_slip_router/dongle-harness/mtcpget-src/MTCPGET.CPP HTGET.CPP
cp /Users/gadyke/esp_slip_router/dongle-harness/mtcpget-src/MAKEFILE     MAKEFILE
cp /Users/gadyke/esp_slip_router/dongle-harness/mtcpget-src/mtcpget.rsp  htget.rsp
cp ~/CH375/open-watcom-v2/bld/clib/library/msdos.086/ml/clibl.lib        ./clibl.lib
cp ~/CH375/open-watcom-v2/bld/cpplib/library/generic.086/ml/plibl.lib    ./plibl.lib

env PATH="$HOME/CH375/ow/build/binbuild:$PATH" WATCOM="$HOME/CH375/ow" \
    INCLUDE="$HOME/CH375/ow/bld/hdr/dos/h" \
    ~/CH375/ow/build/binbuild/wmake htget.obj
env PATH="$HOME/CH375/ow/build/binbuild:$PATH" \
    ~/CH375/ow/build/binbuild/wlink @htget.rsp
# Output: htget.exe (90 KB). Rename to MTCPGET.EXE on deploy.
```

Patch points in MTCPGET.CPP (vs upstream HTGET.CPP):
- around line 1073: add bodyStart / lastProgress* trackers
- in the recv loop (~line 1180): emit `... NNNN bytes (KK.K KB/s)` carriage-return progress every ~TIMER_TICKS_PER_SEC
- after the final `Received ...` line: print `MTCPGET: <bytes> in <s> = <KB/s>` summary

Everything else is stock mTCP HTGET.
