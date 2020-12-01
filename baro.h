#ifndef _BARO_H
#define _BARO_H

// TODO: move to EEPROM
#define BARO_STAT_OFFSET_HPA	28.26
#define BARO_TEK_OFFSET_HPA	10.91
#define BARO_DYN_OFFSET_PA	0.0

void baro_init(void);

void baro_stat_data(double data);
void baro_tek_data(double data);
void baro_dyn_data(double data);

#endif
