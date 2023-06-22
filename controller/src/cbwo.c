#include "cbwo.h"

void cbwo_init(cbwo_t *state) {
    state->size = CB_BUFFER_SIZE;
    state->start = 0;
    state->end = state->size-1;
}

static void cbwo_increment_pointers(cbwo_t *state) {
    state->start = (state->start+1) % state->size;
    state->end = (state->end+1) % state->size;
}

void cbwo_write(cbwo_t *state, uint8_t value) {
    state->buffer[state->start] = value;
    cbwo_increment_pointers(state);
}

void cbwo_get_value(cbwo_t *state, uint8_t *value, int32_t offset) {
    int32_t index = state->end - offset;
    if(index < 0){
        index += state->size;
    }
    *value = state->buffer[index];
}
