#ifndef _CFGFILE_H_
#define _CFGFILE_H_

#define CFGFILE_PATH_LEN 255
#define CFGFILE_OW_ID_LEN 32

typedef struct {
  int use_tek;
  double stat_filter_tau;
  double dyn_filter_tau;
  double tek_kalman_x_accel;
  double tek_kalman_z_abs;
} BARO_CONF_T;

typedef struct {
  int send_raw;
} AHRS_CONF_T;

typedef struct {
  char eeprom_path[CFGFILE_PATH_LEN + 1];
  char temp_id[CFGFILE_OW_ID_LEN + 1];
  BARO_CONF_T baro;
  AHRS_CONF_T ahrs;
} OVNGD_CONF_T;

void cfgfile_read(OVNGD_CONF_T *conf, const char *filename);

#endif
