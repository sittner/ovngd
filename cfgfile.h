#ifndef _CFGFILE_H_
#define _CFGFILE_H_

#define CFGFILE_PATH_LEN 255
#define CFGFILE_OW_ID_LEN 32

typedef struct {
  char eeprom_path[CFGFILE_PATH_LEN + 1];

  char temp_id[CFGFILE_OW_ID_LEN + 1];

  int baro_use_tek;
  double baro_stat_filter_tau;
  double baro_dyn_filter_tau;
  double baro_tek_kalman_x_accel;
  double baro_tek_kalman_z_abs;

  double ahrs_madg_beta;
  int ahrs_send_raw;
} OVNGD_CONF_T;

void cfgfile_read(OVNGD_CONF_T *conf, const char *filename);

#endif
