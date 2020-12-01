#ifndef _IIO_UTILS_H_
#define _IIO_UTILS_H_

/* IIO - useful set of util functionality
 *
 * Copyright (c) 2008 Jonathan Cameron
 * Copyright (c) 2019 Sascha Ittner <sascha.ittner@modusoft.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <stdint.h>
#include <string.h>

#include "vector.h"

/* Made up value to limit allocation sizes */
#define IIOUTILS_MAX_NAME_LENGTH 64
#define IIOUTILS_MAX_VAL_LENGTH 256

enum IIOUTILS_TYPE {
  IIO_TYPE_DEVICE = 0,
  IIO_TYPE_TRIGGER,
  _IIO_TYPES_COUNT
};

/**
 * struct iioutils_chaninfo - information about a given channel
 * @name: channel name
 * @generic_name: general name for channel type
 * @scale: scale factor to be applied for conversion to si units
 * @offset: offset to be applied for conversion to si units
 * @index: the channel index in the buffer output
 * @bytes: number of bytes occupied in buffer output
 * @bits_used: number of valid bits of data
 * @shift: amount of bits to shift right data before applying bit mask
 * @mask: a bit mask for the raw output
 * @be: flag if data is big endian
 * @is_signed: is the raw value stored signed
 * @location: data offset for this channel inside the buffer (in bytes)
 **/
struct iioutils_chaninfo {
	char *name;
	char *generic_name;
	double scale;
	double offset;
	unsigned index;
	unsigned bytes;
	unsigned bits_used;
	unsigned shift;
	uint64_t mask;
	unsigned be;
	unsigned is_signed;
	unsigned location;
};

struct iioutils_mount_matrix {
  vector3d_t x;
  vector3d_t y;
  vector3d_t z;
};

int iioutils_create_hrt_trigger(const char *name);
int iioutils_remove_hrt_trigger(const char *name);

int iioutils_find_by_name(const char *name, enum IIOUTILS_TYPE type, int match_of_name);

int iioutils_write_string(enum IIOUTILS_TYPE type, int index, const char *attr, const char *val);
int iioutils_write_int(enum IIOUTILS_TYPE type, int index, const char *attr, int val);
int iioutils_write_double(enum IIOUTILS_TYPE type, int index, const char *attr, double val);

int iioutils_read_string(enum IIOUTILS_TYPE type, int index, const char *attr, char *val);
int iioutils_read_int(enum IIOUTILS_TYPE type, int index, const char *attr, int *val);
int iioutils_read_double(enum IIOUTILS_TYPE type, int index, const char *attr, double *val);

int iioutils_read_mount_matrix(enum IIOUTILS_TYPE type, int index, struct iioutils_mount_matrix *mtx);
vector3d_t iioutils_transform_mount_matrix(const struct iioutils_mount_matrix *mtx, vector3d_t vect);

int iioutils_build_channel_array(enum IIOUTILS_TYPE type, int index,
			struct iioutils_chaninfo **ci_array, int *counter);
void iioutils_free_channel_array(struct iioutils_chaninfo *ci_array, int counter);

int iioutils_get_scan_size(struct iioutils_chaninfo *channels, int num_channels);

double iioutils_read_scan_double(const struct iioutils_chaninfo *channel, const char *buffer);

#endif /* _IIO_UTILS_H_ */
