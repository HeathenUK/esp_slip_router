@ECHO OFF
REM T-Dongle S3 -- raw FOSSIL SLIP UDP throughput (no internet)
C:
CD \TDONGLE
ECHO === T-SLIP: raw FOSSIL SLIP UDP ===
THRPUT.EXE COM1 192.168.240.99 5
