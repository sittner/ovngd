#include <unistd.h>
#include <string.h>
#include <syslog.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "eeprom.h"
#include "cfgfile.h"
#include "baro.h"
#include "ahrs.h"

EEPROM_DATA_T eeprom_data;

static char eeprom_dev[CFGFILE_PATH_LEN + 1];

static uint32_t crc32(uint8_t *data, int size) {
  int i;
  uint32_t crc;

  crc = ~0;
  for (; size > 0; data++, size--) {
    crc = crc ^ *data;
    for (i = 0; i < 8; i++) {
      crc = (crc >> 1) ^ (0xEDB88320 & (0 - (crc & 1)));
    }
  }
  return ~crc;
}

int eeprom_init(const char *dev) {
  int fd;
  ssize_t len;

  strncpy(eeprom_dev, dev, CFGFILE_PATH_LEN);
  if (eeprom_dev[0] == 0) {
    syslog(LOG_ERR, "EEPROM path not configured. Unable to load data.");
    goto fail0;
  }

  fd = open(eeprom_dev, O_RDONLY | O_SYNC);
  if (fd < 0) {
    syslog(LOG_ERR, "Failed to open EEPROM for reading (error %d).", errno);
    goto fail0;
  }

  len = read(fd, &eeprom_data, sizeof(EEPROM_DATA_T));
  close(fd);

  if (len < 0) {
    syslog(LOG_ERR, "Failed to read EEPROM (error %d).", errno);
    goto fail0;
  }

  if (len != sizeof(EEPROM_DATA_T)) {
    syslog(LOG_ERR, "Unexpected EOF on reading from EEPROM.");
    goto fail0;
  }

  if (eeprom_data.crc != crc32((uint8_t *) &eeprom_data.payload, sizeof(EEPROM_PAYLOAD_T))) {
    syslog(LOG_ERR, "EEPROM data CRC error. Not calibrated yet?");
    goto fail0;
  }

  return 0;

fail0:
  eeprom_data.payload.flags = 0;
  baro_eeprom_init();
  ahrs_eeprom_init();
  return 1;
}

int eeprom_save(void) {
  int fd;
  ssize_t len;

  if (eeprom_dev[0] == 0) {
    syslog(LOG_ERR, "EEPROM path not configured. Unable to save data.");
    return 1;
  }

  eeprom_data.crc = crc32((uint8_t *) &eeprom_data.payload, sizeof(EEPROM_PAYLOAD_T));

  fd = open(eeprom_dev, O_WRONLY);
  if (fd < 0) {
    syslog(LOG_ERR, "Failed to open EEPROM for writing (error %d).", errno);
    return 1;
  }

  len = write(fd, &eeprom_data, sizeof(EEPROM_DATA_T));
  close(fd);

  if (len != sizeof(EEPROM_DATA_T)) {
    syslog(LOG_ERR, "Failed to write EEPROM (error %d).", errno);
    return 1;
  }

  return 0;
}

