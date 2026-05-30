@ECHO OFF
REM T-Dongle S3 -- Hayes HTTP download
C:
CD \TDONGLE
ECHO === T-HAYES: Hayes-mode HTTP download ===
IF "%1"=="" GOTO DEFAULT
HTTPGET.EXE COM1 %1
GOTO END
:DEFAULT
HTTPGET.EXE COM1 http://ipv4.download.thinkbroadband.com/1MB.zip
:END
