@ECHO OFF
REM mTCP+FOSSLIP -- LAN target (override hostname in MTCP-LAN.URL or arg)
C:
CD \TDONGLE
IF "%1"=="" GOTO DEFAULT
CALL P-MTCP.BAT %1
GOTO END
:DEFAULT
ECHO Edit P-MTCP-L.BAT or pass URL:  P-MTCP-L http://192.168.1.x:8765/1M.bin
CALL P-MTCP.BAT http://192.168.1.226:8765/1M.bin
:END
