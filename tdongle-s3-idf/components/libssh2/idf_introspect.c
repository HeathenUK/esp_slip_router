/* Non-upstream IDF helper: expose sizeof(LIBSSH2_SESSION) (an opaque type to
 * public consumers) so the feasibility spike can report the contiguous-allocation
 * requirement. Not part of libssh2 proper; safe to delete with the spike. */
#include "libssh2_priv.h"
#include <stddef.h>

size_t libssh2_session_struct_size(void) {
    return sizeof(LIBSSH2_SESSION);
}
