# libssh2 as an ESP-IDF component.
#
# Vendored from the Zimodem firmware's libssh2 port (Bo Zimmerman, Apache-2.0),
# which is itself libssh2 (https://libssh2.org, BSD-3-Clause, Daniel Stenberg &
# contributors) configured for the mbedTLS crypto backend on ESP32. Original
# per-file BSD-3 headers are preserved verbatim; see CREDITS.md. We link against
# IDF's own mbedTLS (no second copy) and lwIP sockets.
idf_component_register(
    SRCS
        "channel.c" "comp.c" "crypt.c" "global.c" "hostkey.c" "keepalive.c"
        "kex.c" "mac.c" "mbedtls.c" "misc.c" "packet.c" "pem.c" "session.c"
        "transport.c" "userauth.c" "userauth_kbd_packet.c"
        "bcrypt_pbkdf.c" "blowfish.c"
        "sftp.c"   # vendored from libssh2-1.10.0 for AT$SFTP (BSD-3 header kept)
    INCLUDE_DIRS "."
    REQUIRES mbedtls lwip
)

# The vendored libssh2_config.h gates everything on `#if defined(ESP32)` (it came
# from an Arduino-ESP32 build, which defines ESP32). Pure IDF does not, so define
# it here for the component only.
target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP32 LIBSSH2_MBEDTLS)

# libssh2 is third-party C with warnings IDF's -Werror would reject; relax for
# this component only (do NOT relax project-wide).
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error -Wno-unused-parameter -Wno-sign-compare -Wno-unused-variable
    -Wno-unused-but-set-variable -Wno-implicit-fallthrough -Wno-type-limits)
