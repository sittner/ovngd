#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdint.h>
#include <assert.h>

#include "baro.h"
#include "ahrs.h"

typedef struct {
  BARO_EEPROM_T baro;
  AHRS_EEPROM_T ahrs;
} EEPROM_PAYLOAD_T;

typedef struct {
  EEPROM_PAYLOAD_T payload;
  uint32_t crc;
} EEPROM_DATA_T;

extern EEPROM_DATA_T eeprom_data;

int eeprom_init(const char *dev);
int eeprom_save(void);

#endif

