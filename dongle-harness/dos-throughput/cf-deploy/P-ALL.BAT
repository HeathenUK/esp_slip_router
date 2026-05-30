@ECHO OFF
REM ============================================================
REM   P-ALL: full profiling battery. Each phase -> .LOG file +
REM   TYPE'd to console at end of phase. All log names are
REM   8.3-safe to avoid DOS filename truncation collisions.
REM
REM   Logs in C:\TDONGLE\:
REM     PBASES.LOG       baseline dongle state
REM     PSLIP.LOG        raw FOSSIL SLIP UDP throughput
REM     PHAYES.LOG       Hayes 1MB internet
REM     PHAYESS.LOG      AT$STATS after Hayes
REM     PMTCPL.LOG       mTCP HTTP from LAN
REM     PMTCPLS.LOG      AT$STATS after LAN run
REM     PMTCPN.LOG       mTCP HTTP from thinkbroadband
REM     PMTCPNS.LOG      AT$STATS after internet run
REM ============================================================
C:
CD \TDONGLE

ECHO === phase 0: dongle baseline =====================================
ECHO Started %TIME%
MAGICOUT.EXE COM1
ATQ.EXE AT > PBASES.LOG
ATQ.EXE AT$WIFI?  >> PBASES.LOG
ATQ.EXE AT$NETIF  >> PBASES.LOG
ATQ.EXE AT$STATS  >> PBASES.LOG
ATQ.EXE AT$STATS=0
TYPE PBASES.LOG

ECHO === phase 1: T-SLIP (raw FOSSIL SLIP UDP) ========================
ECHO Started %TIME%
ECHO Started %TIME% > PSLIP.LOG
THRPUT.EXE COM1 192.168.240.99 5 >> PSLIP.LOG
ECHO Ended %TIME% >> PSLIP.LOG
TYPE PSLIP.LOG

ECHO === phase 2: T-HAYES (1MB internet) ==============================
ECHO Started %TIME%
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS=0
ECHO Started %TIME% > PHAYES.LOG
HTTPGET.EXE COM1 http://ipv4.download.thinkbroadband.com/1MB.zip >> PHAYES.LOG
ECHO Ended %TIME% >> PHAYES.LOG
ATQ.EXE AT$STATS > PHAYESS.LOG
TYPE PHAYES.LOG
TYPE PHAYESS.LOG

ECHO === phase 3: T-MTCP via LAN ======================================
ECHO Started %TIME%
SET MTCPCFG=C:\TDONGLE\MTCP.CFG
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS=0
ATQ.EXE AT$MODE=SLIP
ECHO Started %TIME% > PMTCPL.LOG
FOSSLIP.EXE 0x60 1 115200 >> PMTCPL.LOG
MTCPGET.EXE -o GOT.BIN http://192.168.1.226:8765/1M.bin >> PMTCPL.LOG
C:\MTCP\PKTTOOL.EXE stats  >> PMTCPL.LOG
C:\MTCP\PKTTOOL.EXE unload >> PMTCPL.LOG
ECHO Ended %TIME% >> PMTCPL.LOG
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS > PMTCPLS.LOG
TYPE PMTCPL.LOG
TYPE PMTCPLS.LOG

ECHO === phase 4: T-MTCP via INTERNET =================================
ECHO Started %TIME%
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS=0
ATQ.EXE AT$MODE=SLIP
ECHO Started %TIME% > PMTCPN.LOG
FOSSLIP.EXE 0x60 1 115200 >> PMTCPN.LOG
MTCPGET.EXE -o GOT.BIN http://ipv4.download.thinkbroadband.com/1MB.zip >> PMTCPN.LOG
C:\MTCP\PKTTOOL.EXE stats  >> PMTCPN.LOG
C:\MTCP\PKTTOOL.EXE unload >> PMTCPN.LOG
ECHO Ended %TIME% >> PMTCPN.LOG
MAGICOUT.EXE COM1
ATQ.EXE AT$STATS > PMTCPNS.LOG
TYPE PMTCPN.LOG
TYPE PMTCPNS.LOG

ECHO === done =========================================================
ECHO Ended %TIME%
DIR P*.LOG
