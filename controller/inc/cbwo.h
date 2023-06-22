#ifndef __CB_BYTE_H__
#define __CB_BYTE_H__

#include <stdint.h>

//* Ad-hoc circular buffer */
#define CB_BUFFER_SIZE 1024

typedef struct {
    uint8_t buffer[CB_BUFFER_SIZE];
    uint32_t start;
    uint32_t end;
    uint32_t size;
} cbwo_t;

void cbwo_init(cbwo_t *state);

void cbwo_write(cbwo_t *state, uint8_t value);

void cbwo_get_value(cbwo_t *state, uint8_t *value, int32_t offset);

#endif // !__CB_BYTE_H__

