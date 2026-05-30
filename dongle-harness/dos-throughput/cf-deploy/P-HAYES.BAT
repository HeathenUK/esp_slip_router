@ECHO OFF
REM Hayes (dongle terminates TCP) -- internet 1MB.
C:
CD \TDONGLE
ECHO Started at %TIME% > P-HAYES.LOG
HTTPGET.EXE COM1 http://ipv4.download.thinkbroadband.com/1MB.zip >> P-HAYES.LOG
ECHO Ended at %TIME% >> P-HAYES.LOG
TYPE P-HAYES.LOG
