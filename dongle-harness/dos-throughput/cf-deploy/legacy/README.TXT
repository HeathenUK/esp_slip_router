T-DONGLE S3 -- DOS test pack
============================

REQUIREMENTS
  CHUSB loaded (provides FOSSIL on INT 14h via CH375).
  T-Dongle plugged into the CH375 USB port.

QUICK START
  1. Load CHUSB (puts FOSSIL on COM1).
  2. Run SETPATH so the T-*.BAT files work from any directory:
       C:\>CD TDONGLE
       C:\TDONGLE>SETPATH
     (or just stay in C:\TDONGLE and the BATs will run by name.)
  3. Open USBTERM and set WiFi creds (one-time, persists in dongle NVS):
       AT$WIFI=YourSSID,YourPassword
       AT$WIFI?       <-- confirm IP, RSSI, DNS
       AT$DNS=google.com   <-- DNS smoke test
       AT$PING=1.1.1.1     <-- ICMP smoke test
     Exit terminal (Ctrl-A then Q in usbterm).
  4. Run a test.

THE THREE TESTS
  T-HAYES [URL]   Hayes-mode HTTP download. Dongle is in MODEM mode
                  (default). DOS streams HTTP, dongle terminates TCP.
                  No mode switch needed before/after.
                  Default URL is a 1MB file from thinkbroadband.

  T-MTCP  [URL]   Full SLIP stack: mTCP TCP/IP -> FOSSLIP packet
                  driver -> FOSSIL -> dongle. Dongle MUST be in SLIP
                  mode first: open USBTERM, type "AT$MODE=SLIP",
                  exit terminal, then run T-MTCP. Restore MODEM by
                  long-pressing BOOT on the dongle (~1.5s).

  T-SLIP          Raw FOSSIL SLIP UDP flood at a sentinel IP. Switches
                  dongle to SLIP itself and back at the end. No
                  internet involved -- measures DOS->dongle USB-CDC
                  intake only.

DIAGNOSTICS (from USBTERM, dongle in MODEM mode)
  AT$WIFI?              SSID, IP, GW, DNS, RSSI, channel
  AT$NETIF              lwIP netifs + NAPT bits
  AT$STATS              SLIP byte/pkt counters since last clear
  AT$STATS=0            reset counters
  AT$DNS=<host>         one-shot lwIP DNS resolution
  AT$PING=<ip>          one ICMP echo from the dongle itself
  AT$MODE?              MODEM | SLIP
  AT$MODE=SLIP          switch to SLIP routing
  AT$HELP               full command list

RECOVERY
  If a test leaves the dongle in SLIP and you can't reach it:
    - long-press the BOOT button (~1.5s), OR
    - re-run T-SLIP (its restore step works)

LOG OUTPUT
  HTTPGET prints phase timings: dial / headers / body / total.
  mTCP HTGET with -v prints connection-level progress.
  All test stdout can be captured with " > somefile.log" suffix.

URL NOTES
  proof.ovh.net is HTTPS-only -- not usable by mTCP HTGET or HTTPGET.
  Default is http://ipv4.download.thinkbroadband.com/1MB.zip (plain HTTP).
