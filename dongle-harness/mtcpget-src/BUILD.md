# Building MTCPGET.EXE and PROFILE.EXE (clean-room)

This directory is the **checkpoint** copy of the source + the shipped
.EXE. Builds actually happen in `$MTCP_SRC_ROOT/APPS/<app>/`
(default `/tmp/mTCP-src_2025-01-10/APPS/<app>/`) because the Watcom
MAKEFILEs reference `../../TCPLIB/` and `../../TCPINC/`. The in-repo
`MAKEFILE` here is the MTCPGET-targeted one (`all : mtcpget.exe`);
running `make` or `wmake` in this directory does NOT build PROFILE.

Both tools are HTTP/1.0 downloaders using mTCP as a library
(TCPLIB .obj files + public TCPINC headers). Source is MIT and was
written from scratch; the only mTCP-derived bits are the linked .obj
files from `mTCP-src/TCPLIB/`.

- `MTCPGET.EXE` -- ~340-line single-URL downloader
- `PROFILE.EXE` -- multi-phase profiler. PHASE 0 baseline AT,
  PHASE 1 raw FOSSIL SLIP UDP, PHASE 2 Hayes HTTP, PHASE 3
  mTCP/SLIP LAN, PHASE 4 mTCP/SLIP internet. Writes PROFILE.LOG
  (committed per line via INT 21h AH=68h).

## Prereqs
* Open Watcom v2 at `~/CH375/ow` (hosted build; provides
  bwpp / bwasm / bwlink / wmake; symlink them as `wpp` / `wasm` /
  `wlink` once if you haven't).
* mTCP source 2025-01-10 unpacked at `/tmp/mTCP-src_2025-01-10`.
  Get from <http://www.brutman.com/mTCP/download/mTCP-src_2025-01-10.zip>.

## Build

### One-time setup (both apps)

```sh
# Alias the OW tools to mTCP's expected names.
cd ~/CH375/ow/build/binbuild
ln -sf bwpp wpp; ln -sf bwasm wasm; ln -sf bwlink wlink
```

### MTCPGET

```sh
mkdir -p /tmp/mTCP-src_2025-01-10/APPS/MTCPGET
cp MTCPGET.CPP MTCPGET.CFG MAKEFILE mtcpget.rsp \
   /tmp/mTCP-src_2025-01-10/APPS/MTCPGET/

# Co-locate the runtimes (linker needs them by bare name).
cd /tmp/mTCP-src_2025-01-10/APPS/MTCPGET
cp ~/CH375/open-watcom-v2/bld/clib/library/msdos.086/ml/clibl.lib   ./clibl.lib
cp ~/CH375/open-watcom-v2/bld/cpplib/library/generic.086/ml/plibl.lib ./plibl.lib

env PATH="$HOME/CH375/ow/build/binbuild:$PATH" \
    WATCOM="$HOME/CH375/ow" \
    INCLUDE="$HOME/CH375/ow/bld/hdr/dos/h" \
    ~/CH375/ow/build/binbuild/wmake
# Output: mtcpget.exe (~76 KB)
```

### PROFILE

Same shape, different build dir. The Watcom MAKEFILE / response file
/ PROFILE.CFG already live at `/tmp/mTCP-src_2025-01-10/APPS/PROFILE/`
from prior setup; the loop is just **sync source in, build, copy
.exe out**:

```sh
# 1. sync the edited source from the repo into the build tree
cp PROFILE.CPP /tmp/mTCP-src_2025-01-10/APPS/PROFILE/PROFILE.CPP

# 2. build
cd /tmp/mTCP-src_2025-01-10/APPS/PROFILE
rm -f profile.obj profile.exe
env PATH="$HOME/CH375/ow/build/binbuild:$PATH" \
    WATCOM="$HOME/CH375/ow" \
    INCLUDE="$HOME/CH375/ow/bld/hdr/dos/h" \
    wmake
# Output: profile.exe (~88 KB; lowercase on disk in the build tree)

# 3. copy the EXE back to the repo for shipping
cp profile.exe "$REPO/dongle-harness/mtcpget-src/PROFILE.EXE"
```

The makefile in this repo dir is the **MTCPGET** one
(`all : mtcpget.exe`); the PROFILE-targeted makefile lives only in
the build tree.

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
