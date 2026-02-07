#include <stdlib.h>
#include "utils.h"

uint32_t jitter_ms(uint32_t max_jitter) {
    return rand() % (max_jitter + 1);
}
