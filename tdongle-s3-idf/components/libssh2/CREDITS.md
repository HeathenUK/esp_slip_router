# Credits / attribution — components/libssh2

This component is **libssh2**, the SSH2 client library — https://libssh2.org
- Copyright (c) Daniel Stenberg and the libssh2 contributors.
- License: **BSD-3-Clause**. The original license text is preserved verbatim in the
  header of every source file (the "Redistribution and use … " block). Do not strip it.

It was obtained via the **Zimodem** firmware's ESP32 port of libssh2:
- Zimodem — Copyright (c) Bo Zimmerman — https://github.com/bozimmerman/Zimodem
- License: **Apache-2.0**.
- What we took from Zimodem: the mbedTLS-backed `libssh2_config.h` and the set of
  sources known to build/run on ESP32. The SSH glue (session/channel/PTY/shell
  driving) in our firmware is rewritten for ESP-IDF but follows the design of
  Zimodem's `WiFiSSHClient`.

Crypto backend is **mbedTLS** (Apache-2.0, ARM / TrustedFirmware), via `LIBSSH2_MBEDTLS`,
using ESP-IDF's bundled mbedTLS (we do not vendor a second copy).

Note for the production phase: this is the Zimodem-era libssh2 snapshot, used here to
measure feasibility quickly. Before shipping SSH, evaluate refreshing to a current
upstream libssh2 release for CVE fixes.
