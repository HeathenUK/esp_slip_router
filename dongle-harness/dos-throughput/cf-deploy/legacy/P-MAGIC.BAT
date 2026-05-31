@ECHO OFF
REM Send the SLIP magic-frame escape to flip dongle back to MODEM.
C:
CD \TDONGLE
MAGICOUT.EXE COM1
