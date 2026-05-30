@ECHO OFF
REM ============================================================
REM   P-ALL: full profiling battery. Each phase -> .LOG file.
REM   Prereq: CHUSB loaded, dongle WiFi up.
REM
REM   Logs (in C:\TDONGLE\):
REM     P-BASE-S.LOG       baseline AT$STATS / AT$WIFI? / AT$NETIF
REM     P-SLIP.LOG         raw FOSSIL SLIP UDP throughput
REM     P-HAYES.LOG        Hayes 1MB internet
REM     P-HAYES-S.LOG        + dongle AT$STATS after Hayes
REM     P-MTCP-L.LOG       mTCP HTTP from LAN target
REM     P-MTCP-L-S.LOG       + dongle AT$STATS after LAN run
REM     P-MTCP-N.LOG       mTCP HTTP from thinkbroadband internet
REM     P-MTCP-N-S.LOG       + dongle AT$STATS after internet run
REM ============================================================
C:
CD \TDONGLE

ECHO === phase 0: dongle baseline =====================================
ECHO Started %TIME%
MAGICOUT.EXE COM1
ATQ.EXE AT > P-BASE-S.LOG
ATQ.EXE AT$WIFI?  >> P-BASE-S.LOG
ATQ.EXE AT$NETIF  >> P-BASE-S.LOG
ATQ.EXE AT$STATS  >> P-BASE-S.LOG
ATQ.EXE AT$STATS=0

ECHO === phase 1: T-SLIP (raw FOSSIL SLIP UDP) ========================
ECHO Started %TIME%
ECHO Started %TIME% > P-SLIP.LOG
THRPUT.EXE COM1 192.168.240.99 5 >> P-SLIP.LOG
ECHO Ended %TIME% >> P-SLIP.LOG

ECHO === phase 2: T-HAYES (1MB internet) ==============================
ECHO Started %TIME%
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS=0
ECHO Started %TIME% > P-HAYES.LOG
HTTPGET.EXE COM1 http://ipv4.download.thinkbroadband.com/1MB.zip >> P-HAYES.LOG
ECHO Ended %TIME% >> P-HAYES.LOG
ATQ.EXE AT$STATS > P-HAYES-S.LOG

ECHO === phase 3: T-MTCP via LAN ======================================
ECHO Started %TIME%
SET MTCPCFG=C:\TDONGLE\MTCP.CFG
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS=0
ATQ.EXE AT$MODE=SLIP
ECHO Started %TIME% > P-MTCP-L.LOG
FOSSLIP.EXE 0x60 1 115200 >> P-MTCP-L.LOG
MTCPGET.EXE -o GOT.BIN http://192.168.1.226:8765/1M.bin >> P-MTCP-L.LOG
C:\MTCP\PKTTOOL.EXE stats  >> P-MTCP-L.LOG
C:\MTCP\PKTTOOL.EXE unload >> P-MTCP-L.LOG
ECHO Ended %TIME% >> P-MTCP-L.LOG
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS > P-MTCP-L-S.LOG

ECHO === phase 4: T-MTCP via INTERNET =================================
ECHO Started %TIME%
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS=0
ATQ.EXE AT$MODE=SLIP
ECHO Started %TIME% > P-MTCP-N.LOG
FOSSLIP.EXE 0x60 1 115200 >> P-MTCP-N.LOG
MTCPGET.EXE -o GOT.BIN http://ipv4.download.thinkbroadband.com/1MB.zip >> P-MTCP-N.LOG
C:\MTCP\PKTTOOL.EXE stats  >> P-MTCP-N.LOG
C:\MTCP\PKTTOOL.EXE unload >> P-MTCP-N.LOG
ECHO Ended %TIME% >> P-MTCP-N.LOG
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS > P-MTCP-N-S.LOG

ECHO === done =========================================================
ECHO Ended %TIME%
DIR P-*.LOG
