#include <stdio.h>
#include <stdlib.h>

#include "ws2812_bcm2708.h"

/* TODO: this is a straightforward copy from my kernel debugging code, make cleaner */

/* Serializes o send '1', send 110; to send '0', send '100' */
#define OUTPUT_BIT(bit, write_command) \
  current_data |= (bit) << bit_shift; \
  bit_shift--; \
  if (bit_shift < 0) { /* flush word */ \
    write_command ; \
    bit_shift = 31; \
    current_data = 0; \
  }

#define OUTPUT_COLOR_LOOP(value, write_command) \
for (i = 7; i >= 0; i--) { \
      OUTPUT_BIT(1, write_command); \
      OUTPUT_BIT( ((value) >> i) & 1, write_command ); \
      OUTPUT_BIT(0, write_command); \
    }

static size_t write_ws2812_list_to_buffer(struct ws2812_color *list,
                                          size_t nens,
                                          uint32_t * buffer, size_t buffer_size)
{
  int bit_shift = 31;
  uint32_t current_data = 0;
  int i;
  size_t buf_pos = 0;
  size_t list_pos = 0;

#define TO_BUFFER \
  do { \
    buffer[buf_pos++] = current_data; \
    if(buf_pos >= buffer_size) goto done; \
  } while(0)

  for (list_pos = 0; list_pos < nens; list_pos++) {
    OUTPUT_COLOR_LOOP(list[list_pos].green, TO_BUFFER)
    OUTPUT_COLOR_LOOP(list[list_pos].red, TO_BUFFER)
    OUTPUT_COLOR_LOOP(list[list_pos].blue, TO_BUFFER)
  }
  
  /* Flush remaining */
  TO_BUFFER;
  
  /* Silence */
  current_data = 0;

  TO_BUFFER;
  
  /* There is a delay of >= 50 µs, so, say, 55µs, @ 2.4MHz, so 132 bits, i.e., 5 uint32_t's */
  for(i = 0; i < 5; i++)
    TO_BUFFER;
  
done:
  if (buf_pos >= buffer_size)
    printf("write_ws2812_list_to_buffer: Buffer was full!\n"); /* TODO printing from libraries is not done! */
  
  return buf_pos; /* Number of elements to be transfered with DMA */
}

#undef TO_BUFFER

#undef OUTPUT_COLOR_LOOP
#undef OUTPUT_BIT


/* TODO: don't hardcode buffersize to 4096 */
void ws28128_bcm2708_render(struct ws2812_array* array, struct ws28128_bcm2708* out) {
  if (out->buffer) {
    if (out->buffer_size != 4096) {
      free(out->buffer);
      out->buffer = malloc(4096);
      out->buffer_size = 4096;
    }
  } else {
    out->buffer = malloc(4096);
    out->buffer_size = 4096;
  }
  out->size = sizeof(uint32_t) * write_ws2812_list_to_buffer(array->colors, array->count, (uint32_t*)out->buffer, 4096);
}
