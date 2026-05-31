@ECHO OFF
REM T-Dongle S3 -- mTCP + FOSSLIP HTTP download (dongle must be in SLIP mode)
C:
CD \TDONGLE
ECHO === T-MTCP: mTCP HTGET over FOSSLIP + SLIP ===
SET MTCPCFG=C:\TDONGLE\MTCP.CFG
FOSSLIP.EXE 0x60 1 115200
IF "%1"=="" GOTO DEFAULT
C:\MTCP\HTGET.EXE -v -o GOT.BIN %1
GOTO END
:DEFAULT
C:\MTCP\HTGET.EXE -v -o GOT.BIN http://ipv4.download.thinkbroadband.com/1MB.zip
:END
