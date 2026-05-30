@ECHO OFF
REM ============================================================
REM   P-ALL: full profiling battery. Saves each phase to a .LOG.
REM   Prerequisite: CHUSB loaded, dongle WiFi up, dongle in MODEM
REM   mode. Will switch into SLIP and back as needed.
REM
REM   Output files in C:\TDONGLE\:
REM     P-SLIP.LOG       raw FOSSIL SLIP UDP throughput (no internet)
REM     P-HAYES.LOG      Hayes 1 MB HTTP from thinkbroadband
REM     P-HAYES-S.LOG    dongle AT$STATS after Hayes run
REM     P-MTCP-L.LOG     mTCP HTTP from LAN target (no internet RTT)
REM     P-MTCP-L-S.LOG   dongle AT$STATS after LAN run
REM     P-MTCP-N.LOG     mTCP HTTP from thinkbroadband (1 MB)
REM     P-MTCP-N-S.LOG   dongle AT$STATS after internet run
REM ============================================================
C:
CD \TDONGLE
ECHO === phase 0: dongle baseline =====================================
ECHO Started %TIME%
USBTERM.EXE -s STAT-RD.TXT -l P-BASE-S.LOG COM1 9600

ECHO === phase 1: T-SLIP (raw FOSSIL SLIP UDP) ========================
ECHO Started %TIME%
ECHO Started %TIME% > P-SLIP.LOG
THRPUT.EXE COM1 192.168.240.99 5 >> P-SLIP.LOG
ECHO Ended %TIME% >> P-SLIP.LOG

ECHO === phase 2: T-HAYES (1MB internet) ==============================
ECHO Started %TIME%
USBTERM.EXE -s STAT-CLR.TXT -l STAT-CLR-RX.LOG COM1 9600
ECHO Started %TIME% > P-HAYES.LOG
HTTPGET.EXE COM1 http://ipv4.download.thinkbroadband.com/1MB.zip >> P-HAYES.LOG
ECHO Ended %TIME% >> P-HAYES.LOG
USBTERM.EXE -s STAT-RD.TXT -l P-HAYES-S.LOG COM1 9600

ECHO === phase 3: T-MTCP via LAN ======================================
ECHO Started %TIME%
SET MTCPCFG=C:\TDONGLE\MTCP.CFG
USBTERM.EXE -s STAT-CLR.TXT -l STAT-CLR-RX.LOG COM1 9600
USBTERM.EXE -s TO-SLIP.TXT  -l STAT-CLR-RX.LOG COM1 9600
ECHO Started %TIME% > P-MTCP-L.LOG
FOSSLIP.EXE 0x60 1 115200 >> P-MTCP-L.LOG
MTCPGET.EXE -v -o GOT.BIN http://192.168.1.226:8765/1M.bin >> P-MTCP-L.LOG
C:\MTCP\PKTTOOL.EXE stats >> P-MTCP-L.LOG
ECHO Ended %TIME% >> P-MTCP-L.LOG
MAGICOUT.EXE COM1
USBTERM.EXE -s STAT-RD.TXT -l P-MTCP-L-S.LOG COM1 9600

ECHO === phase 4: T-MTCP via INTERNET =================================
ECHO Started %TIME%
USBTERM.EXE -s STAT-CLR.TXT -l STAT-CLR-RX.LOG COM1 9600
USBTERM.EXE -s TO-SLIP.TXT  -l STAT-CLR-RX.LOG COM1 9600
ECHO Started %TIME% > P-MTCP-N.LOG
FOSSLIP.EXE 0x60 1 115200 >> P-MTCP-N.LOG
MTCPGET.EXE -v -o GOT.BIN http://ipv4.download.thinkbroadband.com/1MB.zip >> P-MTCP-N.LOG
C:\MTCP\PKTTOOL.EXE stats >> P-MTCP-N.LOG
ECHO Ended %TIME% >> P-MTCP-N.LOG
MAGICOUT.EXE COM1
USBTERM.EXE -s STAT-RD.TXT -l P-MTCP-N-S.LOG COM1 9600

ECHO === done =========================================================
ECHO Ended %TIME%
ECHO All logs in C:\TDONGLE\:
DIR P-*.LOG
