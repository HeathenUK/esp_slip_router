T-DONGLE S3 -- PROFILING PACK
=============================

Goal: figure out where T-MTCP's throughput is going (~10 KB/s vs
58 KB/s Hayes baseline). Runs every test in sequence and saves all
output to .LOG files for offline review.

Prereqs
-------
* CHUSB loaded (FOSSIL on COM1)
* Dongle WiFi up (AT$WIFI=...)
* Dongle in MODEM mode

Just run
--------
  C:\>CD TDONGLE
  C:\TDONGLE>P-ALL

This runs all four phases, switches dongle modes automatically, and
sends the SLIP magic-frame escape (MAGICOUT.EXE) to recover after
each SLIP-mode test.

Files written by P-ALL
----------------------
  P-BASE-S.LOG    pre-run baseline: AT, AT$STATS, AT$WIFI?, AT$NETIF
  P-SLIP.LOG      raw FOSSIL SLIP UDP -- THRPUT.EXE output
  P-HAYES.LOG     Hayes HTTP 1MB internet -- HTTPGET.EXE output
  P-HAYES-S.LOG   dongle AT$STATS after Hayes
  P-MTCP-L.LOG    mTCP HTTP 1MB from LAN (192.168.1.226:8765) -- MTCPGET
  P-MTCP-L-S.LOG  dongle AT$STATS after LAN mTCP
  P-MTCP-N.LOG    mTCP HTTP 1MB internet -- MTCPGET
  P-MTCP-N-S.LOG  dongle AT$STATS after internet mTCP

Individual phases (if you want to run just one)
-----------------------------------------------
  P-SLIP        raw FOSSIL SLIP UDP
  P-HAYES       Hayes 1MB internet
  P-MTCP        mTCP 1MB internet (no URL arg) or P-MTCP <url>
  P-MTCP-L      mTCP 1MB from LAN target (default 192.168.1.226:8765/1M.bin)
  P-STATS       read dongle AT$STATS/$WIFI?/$NETIF
  P-CLEAR       zero dongle AT$STATS counters
  P-MAGIC       send SLIP magic-frame escape (recover to MODEM)

LAN target setup (for P-MTCP-L)
-------------------------------
On the Mac (or any LAN box), run:
  head -c 1048576 /dev/urandom > 1M.bin
  python3 -m http.server 8765
Then P-MTCP-L uses http://192.168.1.226:8765/1M.bin. Edit P-MTCP-L.BAT
if your LAN target isn't that IP.

What to send back
-----------------
After P-ALL finishes, archive all P-*.LOG files (and STAT-* if
present) for analysis. The LAN-vs-internet T-MTCP gap is what
discriminates between RTT-bound (LAN much faster) and stack-bound
(LAN same speed).

Recovery
--------
If a test leaves the dongle in SLIP and can't talk AT:
  C:\TDONGLE>P-MAGIC     (sends magic-frame escape via FOSSIL)
or long-press BOOT on the dongle.
