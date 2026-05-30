# Building MTCPGET.EXE (clean-room)

Our own HTTP/1.0 downloader -- ~340 lines of C++ -- that uses mTCP as a
library (its TCPLIB .obj files + public TCPINC headers). The source
here is MIT and was written from scratch; the only mTCP-derived bits
are the linked .obj files from `mTCP-src/TCPLIB/`.

## Prereqs
* Open Watcom v2 at `~/CH375/ow` (hosted build; provides
  bwpp / bwasm / bwlink / wmake; symlink them as `wpp` / `wasm` /
  `wlink` once if you haven't).
* mTCP source 2025-01-10 unpacked at `/tmp/mTCP-src_2025-01-10`.
  Get from <http://www.brutman.com/mTCP/download/mTCP-src_2025-01-10.zip>.

## Build
```sh
# one-time: alias the OW tools to mTCP's expected names
cd ~/CH375/ow/build/binbuild
ln -sf bwpp wpp; ln -sf bwasm wasm; ln -sf bwlink wlink

# set up build dir under mTCP source
mkdir -p /tmp/mTCP-src_2025-01-10/APPS/MTCPGET
cp MTCPGET.CPP MTCPGET.CFG MAKEFILE mtcpget.rsp \
   /tmp/mTCP-src_2025-01-10/APPS/MTCPGET/

# co-locate the runtimes (linker needs them by bare name)
cd /tmp/mTCP-src_2025-01-10/APPS/MTCPGET
cp ~/CH375/open-watcom-v2/bld/clib/library/msdos.086/ml/clibl.lib   ./clibl.lib
cp ~/CH375/open-watcom-v2/bld/cpplib/library/generic.086/ml/plibl.lib ./plibl.lib

env PATH="$HOME/CH375/ow/build/binbuild:$PATH" \
    WATCOM="$HOME/CH375/ow" \
    INCLUDE="$HOME/CH375/ow/bld/hdr/dos/h" \
    ~/CH375/ow/build/binbuild/wmake
# Output: mtcpget.exe (~76 KB)
```

## What it does (vs HTGET)
* HTTP/1.0 only -- no chunked encoding, no redirects, no basic auth
* Status 200 required (anything else aborts)
* `Connection: close` always; body ends at remote-close OR Content-Length
* Periodic progress `... NNNN bytes (KK.K KB/s)\r` once per BIOS tick second
* Final `MTCPGET: <bytes> in <s>s = <KB/s>` summary

## Usage
```
MTCPGET [-o filename] <URL>
```
URL is `[http://]host[:port]/path`. Default output file is `GOT.BIN`.
Reads `MTCPCFG` env var for the mTCP config (PACKETINT, IPADDR, etc.).
