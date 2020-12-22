#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <string.h>

#include "cfgfile.h"

#include "baro.h"
#include "ahrs.h"

static const OVNGD_CONF_T default_conf = {
  .eeprom_path = { 0 },

  .temp_id = { 0 },

  .baro_use_tek = BARO_USE_TEK,
  .baro_stat_filter_tau = BARO_STAT_FILTER_TAU,
  .baro_dyn_filter_tau = BARO_DYN_FILTER_TAU,
  .baro_tek_kalman_x_accel = BARO_TEK_KALMAN_X_ACCEL,
  .baro_tek_kalman_z_abs = BARO_TEK_KALMAN_Z_ABS,

  .ahrs_madg_beta = MADG_BETA_DEFLT
};

static const char *trim_chars = " \t\r\n";

void cfgfile_read(OVNGD_CONF_T *conf, const char *filename) {
  FILE *fp;
  char *line = NULL;
  size_t len = 0;
  ssize_t read;
  char *p, *name, *value;

  // set defaults
  memcpy(conf, &default_conf, sizeof(OVNGD_CONF_T));

  // check for config file
  if (filename == NULL) {
    syslog(LOG_INFO, "Config file not given. Using defaults.");
    return;
  }

  fp = fopen(filename, "r");
  if (fp == NULL) {
    syslog(LOG_WARNING, "Failed to open config file %s. Using defaults.", filename);
    return;
  }

  while ((read = getline(&line, &len, fp)) >= 0) {
    // rtrim line
    for (p = line + read - 1; p >= line && strchr(trim_chars, *p); p--) {
      *p = 0;
    }

    // ltrim line
    for (name = line; *name != 0 && strchr(trim_chars, *name); name++);

    // skip empty lines and comments
    if (*name == 0 || *name == '#') {
      continue;
    }

    // try to split name and value
    p = strchr(name, '=');
    if (p == NULL) {
      continue;
    }
    *p = 0;

    // ltrim value
    for (value = p + 1; *value != 0 && strchr(trim_chars, *value); value++);

    // rtrim name
    for (p--; p >= name && strchr(trim_chars, *p); p--) {
      *p = 0;
    }

    // parse parameters
    if (strcmp(name, "EEPROM_PATH") == 0) {
      strncpy(conf->eeprom_path, value, CFGFILE_PATH_LEN);
      continue;
    }
    if (strcmp(name, "TEMP_OW_ID") == 0) {
      strncpy(conf->temp_id, value, CFGFILE_OW_ID_LEN);
      continue;
    }
    if (strcmp(name, "BARO_USE_TEK") == 0) {
      conf->baro_use_tek = !!atoi(value);
      continue;
    }
    if (strcmp(name, "BARO_STAT_FILTER_TAU") == 0) {
      conf->baro_stat_filter_tau = atof(value);
      continue;
    }
    if (strcmp(name, "BARO_DYN_FILTER_TAU") == 0) {
      conf->baro_dyn_filter_tau = atof(value);
      continue;
    }
    if (strcmp(name, "BARO_TEK_KALMAN_X_ACCEL") == 0) {
      conf->baro_tek_kalman_x_accel = atof(value);
      continue;
    }
    if (strcmp(name, "BARO_TEK_KALMAN_Z_ABS") == 0) {
      conf->baro_tek_kalman_z_abs = atof(value);
      continue;
    }
    if (strcmp(name, "AHRS_MADG_BETA") == 0) {
      conf->ahrs_madg_beta = atof(value);
      continue;
    }
  }

  if (line != NULL) {
    free(line);
  }

  fclose(fp);
}

