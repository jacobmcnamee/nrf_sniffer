#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stdbool.h>

uint32_t crc_compute(const uint8_t *data, uint32_t length);
uint32_t crc_continue(const uint8_t *data, uint32_t length, uint32_t crc);

#endif // CRC_H
