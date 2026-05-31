@ECHO OFF
REM Zero dongle SLIP counters.
C:
CD \TDONGLE
ATQ.EXE AT$STATS=0
