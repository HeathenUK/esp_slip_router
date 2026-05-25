# esp_slip_router — T-Dongle S3 port

A PlatformIO/Arduino-ESP32 port of [esp_slip_router](../README.md) for the
**LilyGO T-Dongle S3** (ESP32-S3). It turns the dongle into a WiFi NAT router
that talks to a host over **USB CDC** using **SLIP** (RFC1055) — effectively a
"WiFi card" for any machine that can drive a USB serial device, including DOS
boxes with a USB CDC-ACM serial driver.

## Why this is simpler than the ESP8266 original

The ESP8266 version needed a hand-patched `liblwip_open_napt.a` to make NAT/DNS
work (the whole "privileged/high port" saga). On ESP32 the bundled lwIP already
ships with `IP_FORWARD`, `IPV4_NAPT` and `NAPT_PORTMAP` enabled, so **NAT and DNS
work out of the box** — no library surgery. The link is plain transparent SLIP;
no Hayes/AT modem handshake is involved.

## Architecture

```
Host (DOS)  ──USB CDC-ACM (SLIP frames)──►  T-Dongle S3  ──WiFi STA (NAPT)──►  internet
            ◄─────────────────────────────               ◄──────────────────
```

- **USB CDC** is the SLIP data link. Default env presents **TinyUSB CDC-ACM**
  (the same USB class CircuitPython uses), so a host driver that talks to
  CircuitPython ESP32 boards sees this identically.
- SLIP framing is implemented in `src/main.cpp` (lwIP's `slipif` is **not**
  compiled into the Arduino lwIP build). Incoming frames are de-framed and
  handed to lwIP via the thread-safe `tcpip_input`; outgoing IP packets are
  SLIP-encoded straight to the USB CDC.
- A custom lwIP **point-to-point netif** is the LAN side
  (`192.168.240.1/24` by default). The WiFi STA owns the default route, and
  **NAPT is enabled on the STA interface** so host traffic is masqueraded out.
- **Debug output goes to the hardware UART (`Serial0`)**, never the USB CDC —
  that channel must stay binary-clean for SLIP.

## Build & flash

Edit WiFi credentials in `include/config.h` (or pass `-DWIFI_SSID=...` /
`-DWIFI_PASS=...` in `platformio.ini`), then:

```bash
# Deployment build: TinyUSB CDC-ACM (use this with the DOS host)
pio run -e dos -t upload
#   Reflashing in this mode needs manual download mode:
#   hold the BOOT button while plugging the dongle in.

# Development build: hardware USB-Serial/JTAG, auto-flashes without the button dance
pio run -e dev -t upload
```

## Host (DOS) side — sketch

The dongle appears as a COM port via your USB CDC-ACM driver. Point a SLIP
packet driver at that COM port, give the host an address on the SLIP subnet,
and use the dongle as its gateway:

```
host IP : 192.168.240.2
gateway : 192.168.240.1   (the dongle)
netmask : 255.255.255.0
DNS     : any public resolver (e.g. 1.1.1.1) — NAT forwards it
```

Then run a TCP/IP stack (mTCP, WATTCP, Arachne, …) over the packet driver.

## Status

Working / verified to **build**:
- [x] PlatformIO project for the T-Dongle S3 (arduino-esp32 3.3.8 via pioarduino)
- [x] WiFi STA connect + NAPT on the STA interface
- [x] SLIP framing over USB CDC + custom lwIP netif

Not yet done (next):
- [ ] On-hardware bring-up (USB enumeration, CDC DTR behaviour, throughput)
- [ ] NVS-backed config + TCP `:7777` console (`show`/`set`/`save`/`portmap`)
- [ ] Port forwarding via `ip_portmap_add`
- [ ] Optional status on the 0.96" ST7735 TFT
- [ ] Optional Hayes/AT layer (kept off the data path)

> Heritage: ported from martin-ger's esp_slip_router (+ HeathenUK's Hayes work).
> The ESP8266 source remains in the repository root for reference.
