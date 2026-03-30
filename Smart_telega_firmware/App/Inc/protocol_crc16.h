#ifndef PROTOCOL_CRC16_H
#define PROTOCOL_CRC16_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC16-CCITT-FALSE:
 * poly = 0x1021
 * init = 0xFFFF
 * refin = false
 * refout = false
 * xorout = 0x0000
 */
uint16_t protocol_crc16_ccitt_false(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_CRC16_H */
