@ECHO OFF
REM Zero dongle SLIP counters before a run.
C:
CD \TDONGLE
USBTERM.EXE -s STAT-CLR.TXT -l STAT-CLR-RX.LOG COM1 9600
