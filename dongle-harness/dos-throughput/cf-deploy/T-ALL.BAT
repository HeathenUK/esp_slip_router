@ECHO OFF
REM Run all three tests. Switch dongle modes manually between them.
C:
CD \TDONGLE
ECHO Logging to T-ALL.LOG
ECHO ---- T-SLIP  ---- > T-ALL.LOG
TIME /T >> T-ALL.LOG
CALL T-SLIP.BAT >> T-ALL.LOG
ECHO ---- T-HAYES ---- >> T-ALL.LOG
TIME /T >> T-ALL.LOG
CALL T-HAYES.BAT >> T-ALL.LOG
ECHO ---- T-MTCP  ---- >> T-ALL.LOG
TIME /T >> T-ALL.LOG
CALL T-MTCP.BAT >> T-ALL.LOG
TIME /T >> T-ALL.LOG
ECHO Done. Results in T-ALL.LOG.
