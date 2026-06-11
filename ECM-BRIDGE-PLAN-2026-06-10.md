# DOSONGLE CDC-ECM bridge — plan, 2026-06-10

Device half of the "WiFi for DOS" design. The host half lives in `~/CH375/ECM-CLASS-PLAN-2026-06-11.md` (CHUSB: ECM
class + INT 2Fh frame API only) and `~/FOSSLIP/ECM-TRANSPORT-PLAN-2026-06-11.md`
(the packet driver, dual SLIP/ECM). Nothing in THIS doc changes.
This doc is written to be self-contained for an agent working ONLY in this
repo — host-side context is summarized where it constrains firmware choices.

## Goal

Replace (or sit alongside) the SLIP-over-CDC-ACM transport with a USB
**CDC-ECM** network interface bridged to WiFi STA through the existing
lwIP/NAPT machinery, so the DOS host receives whole Ethernet frames at USB
full speed instead of SLIP bytes at 115200. The DOS machine's USB host
controller (CH375) tops out near 300 KB/s, so wire-rate targets are ~2 Mbit.

## What exists today (surveyed 2026-06-10, verify before relying)

- ESP-IDF (native, not Arduino) build at `tdongle-s3-idf/`, esp_tinyusb
  1.4.5 / TinyUSB >= 0.14.2. Flash auto-suspend is load-bearing for the
  CH375 host (`CONFIG_SPI_FLASH_AUTO_SUSPEND=y`).
- USB composite hand-rolled in `main/usb.c` (~lines 50-102): IAD-based,
  CDC-ACM (ITF 0/1, EP 0x81 notif + 0x02/0x82 bulk), MSC (ITF 2,
  0x03/0x83), HID keyboard (ITF 3, 0x84). 6 of the DWC2's 8 endpoint
  numbers used; numbers 5-7 free.
- SLIP transport: `main/slip.c` — RFC 1055 codec feeding a custom lwIP netif
  (`s_slip_nif`), NAPT enabled on the SLIP side via
  `ip_napt_enable_netif(&s_slip_nif)` (NOTE the repo's STACK.md documents the
  inverted-API pitfall — NAPT goes on the *client-side* netif). CDC RX
  arrives via `tud_cdc_rx_cb` in `main/modem.c`, gated by `slip_get_mode()`.
- **TinyUSB already ships the net class in this tree**:
  `managed_components/espressif__tinyusb/src/class/net/ecm_rndis_device.c`
  (+ `net_device.h`; reference: `examples/device/net_lwip_webserver/`).
  Nothing enables it today (no `CFG_TUD_ECM_RNDIS` in the active config).
- Heap is the tight resource: sdkconfig comments record min_free dipping to
  ~5 KB during concurrent bulk transfers; no PSRAM. CDC FIFOs are sized for
  SLIP throughput (RX 2048 / TX 4096) — that headroom is reclaimable once
  SLIP retires.
- TinyUSB DWC2 IRAM patches applied at build (`tools/apply_iram_patches.sh`,
  wired via CMakeLists) — keep them in mind when bumping component versions.

## USB contract (mirrored in the CHUSB plan — do not drift)

- CDC-ECM per CDC 1.2 + ECM subclass: IAD; comm interface class 02h subclass
  06h with the ECM functional descriptor; data interface class 0Ah with
  **alt 0 = no endpoints, alt 1 = bulk IN + bulk OUT (64-byte max)**. The
  DOS host issues SET_INTERFACE(alt 1) after SET_CONFIGURATION.
- **MAC address via iMACAddress**: a STRING descriptor of 12 UTF-16LE hex
  digits; the host parses it to 6 bytes. Use a stable per-unit MAC (derive
  from efuse base MAC, locally-administered variant for the USB side).
- One Ethernet frame per USB transfer; transfers end on a short packet, so
  exact-multiple-of-64 frames get a terminating ZLP — send it, tolerate it.
  Max frame 1514 bytes, no FCS.
- Notification endpoint may exist (NetworkConnection); the DOS host may
  ignore it. The full composite (HID + MSC + CDC-ACM + ECM) must keep the
  config descriptor under 512 bytes — the host's parse buffer.

## Design

1. **Descriptors** (`main/usb.c`): add the ECM IAD + interfaces; endpoints
   0x85 (notif), 0x05/0x86 (bulk OUT/IN) — confirm against the DWC2
   IN/OUT-per-number allocation. Keep CDC-ACM (the REPL/modem path and the
   existing FOSSIL users); `CFG_TUD_ECM_RNDIS=1` with ECM descriptors only
   (the combined TinyUSB driver speaks whichever the descriptor declares —
   declare ECM; RNDIS is not wanted).
2. **Netif glue**: implement `tud_network_recv_cb` / `tud_network_xmit_cb` /
   `tud_network_init_cb` against a new lwIP netif (`s_ecm_nif`) exactly where
   `s_slip_nif` sits today; NAPT moves to (or is duplicated on) the ECM netif
   — same inverted-API rule. The TinyUSB `net_lwip_webserver` example is the
   wiring reference.
3. **Addressing/DHCP**: run the lwIP DHCP **server** (the SoftAP `dhcps`
   component) on the ECM netif with a private subnet, default route +
   DNS pointing at the bridge — the DOS host's mTCP does plain DHCP and gets
   everything it needs. (SLIP today is static-configured; this is the UX
   upgrade that makes the DOS side zero-config.)
4. **Heap funding**: shrink `CFG_TUD_CDC_RX_BUFSIZE/TX_BUFSIZE` (2048/4096 →
   512/512 once SLIP is no longer the data path) to offset the ECM driver's
   frame buffers (~2× 1514 + bookkeeping). Measure min_free under
   simultaneous MSC + ECM load before calling it done.
5. **Mode interplay**: `modem.c`'s SLIP-mode gating stays for the ACM
   channel; ECM is always-on and independent. SLIP code is retired only
   after the DOS packet driver ships (keep both transports through the
   transition).

## Phases & gates

- **P1**: descriptors + class enabled; device enumerates as ECM against a
  Mac/Linux host (no DOS involved). Gate: host sees the interface, link up,
  DHCP lease served, `ping` through NAPT to the WiFi side works.
- **P2**: heap + throughput characterization (iperf-ish over the Mac host;
  MSC transfer running concurrently). Gate: min_free stays >4 KB; no MSC
  regression.
- **P3**: against the real DOS host once CHUSB's ECM class lands (the CHUSB
  plan's N2/N3). Gate: mTCP DHCP + FTP on the Pocket386.

## Risks / notes for the implementing agent

- Endpoint arithmetic on DWC2 is per-direction-per-number; double-check the
  notif EP choice against `tusb_config` FIFO sizing (the IRAM-patched DWC2
  driver has fixed FIFO carving).
- The CH375 host polls bulk IN; it never NAK-starves the device, but it IS
  slow — TinyUSB's ECM driver backpressures via `tud_network_can_xmit`;
  ensure WiFi→USB flooding drops at lwIP (pbuf exhaustion) rather than
  wedging the USB task. The existing CPU1-pinned TinyUSB task arrangement
  (see modem.c) applies.
- Keep `CONFIG_SPI_FLASH_AUTO_SUSPEND` and the IRAM patches — both exist
  because the CH375 host is timing-fragile; regressions there break the
  keyboard/MSC, not just networking.
- The DOS host enumerates strictly and was hardened against this device's
  earlier quirks (post-SET_CONFIG interface requests only, report-ID HID);
  don't reorder descriptor groups casually — the host parses the whole
  config blob with offset arithmetic.
