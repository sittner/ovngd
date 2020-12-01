/* IIO - useful set of util functionality
 *
 * Copyright (c) 2008 Jonathan Cameron
 * Copyright (c) 2019 Sascha Ittner <sascha.ittner@modusoft.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <dirent.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <endian.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "iio_utils.h"

#define FORMAT_SCAN_ELEMENTS_DIR "%s/scan_elements"
#define FORMAT_TYPE_FILE "%s_type"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

const char *iio_dir = "/sys/bus/iio/devices/";
const char *hrt_trig_dir_fmt = "/sys/kernel/config/iio/triggers/hrtimer/%s";

const char *iio_types[_IIO_TYPES_COUNT] = {
  "iio:device",
  "trigger",
};

static char * const iio_direction[] = {
	"in",
	"out",
};

static int calc_digits(int num)
{
	int count = 0;

	while (num != 0) {
		num /= 10;
		count++;
	}

	return count;
}

/**
 * break_up_name() - extract generic name from full channel name
 * @full_name: the full channel name
 * @generic_name: the output generic channel name
 *
 * Returns 0 on success, or a negative error code if string extraction failed.
 **/
static int break_up_name(const char *full_name, char **generic_name)
{
	char *current;
	char *w, *r;
	char *working, *prefix = "";
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(iio_direction); i++)
		if (!strncmp(full_name, iio_direction[i],
			     strlen(iio_direction[i]))) {
			prefix = iio_direction[i];
			break;
		}

	current = strdup(full_name + strlen(prefix) + 1);
	if (!current)
		return -ENOMEM;

	working = strtok(current, "_\0");
	if (!working) {
		free(current);
		return -EINVAL;
	}

	w = working;
	r = working;

	while (*r != '\0') {
		if (!isdigit(*r)) {
			*w = *r;
			w++;
		}

		r++;
	}
	*w = '\0';
	ret = asprintf(generic_name, "%s_%s", prefix, working);
	free(current);

	return (ret == -1) ? -ENOMEM : 0;
}

/**
 * get_channel_type() - find and process _type attribute data
 * @is_signed: output whether channel is signed
 * @bytes: output how many bytes the channel storage occupies
 * @bits_used: output number of valid bits of data
 * @shift: output amount of bits to shift right data before applying bit mask
 * @mask: output a bit mask for the raw data
 * @be: output if data in big endian
 * @device_dir: the IIO device directory
 * @name: the channel name
 * @generic_name: the channel type name
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
static int get_channel_type(unsigned *is_signed, unsigned *bytes, unsigned *bits_used,
		      unsigned *shift, uint64_t *mask, unsigned *be,
		      const char *device_dir, const char *name,
		      const char *generic_name)
{
	FILE *sysfsfp;
	int ret;
	DIR *dp;
	char *scan_el_dir, *builtname, *builtname_generic, *filename = 0;
	char signchar, endianchar;
	unsigned padint;
	const struct dirent *ent;

	ret = asprintf(&scan_el_dir, FORMAT_SCAN_ELEMENTS_DIR, device_dir);
	if (ret < 0)
		return -ENOMEM;

	ret = asprintf(&builtname, FORMAT_TYPE_FILE, name);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_scan_el_dir;
	}
	ret = asprintf(&builtname_generic, FORMAT_TYPE_FILE, generic_name);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_builtname;
	}

	dp = opendir(scan_el_dir);
	if (!dp) {
		ret = -errno;
		goto error_free_builtname_generic;
	}

	ret = -ENOENT;
	while (ent = readdir(dp), ent)
		if ((strcmp(builtname, ent->d_name) == 0) ||
		    (strcmp(builtname_generic, ent->d_name) == 0)) {
			ret = asprintf(&filename,
				       "%s/%s", scan_el_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				goto error_closedir;
			}

			sysfsfp = fopen(filename, "r");
			if (!sysfsfp) {
				ret = -errno;
				fprintf(stderr, "failed to open %s\n",
					filename);
				goto error_free_filename;
			}

			ret = fscanf(sysfsfp,
				     "%ce:%c%u/%u>>%u",
				     &endianchar,
				     &signchar,
				     bits_used,
				     &padint, shift);
			if (ret < 0) {
				ret = -errno;
				fprintf(stderr,
					"failed to pass scan type description\n");
				goto error_close_sysfsfp;
			} else if (ret != 5) {
				ret = -EIO;
				fprintf(stderr,
					"scan type description didn't match\n");
				goto error_close_sysfsfp;
			}

			*be = (endianchar == 'b');
			*bytes = padint / 8;
			if (*bits_used == 64)
				*mask = ~0;
			else
				*mask = (1ULL << *bits_used) - 1;

			*is_signed = (signchar == 's');
			if (fclose(sysfsfp)) {
				ret = -errno;
				fprintf(stderr, "Failed to close %s\n",
					filename);
				goto error_free_filename;
			}

			sysfsfp = 0;
			free(filename);
			filename = 0;

			/*
			 * Avoid having a more generic entry overwriting
			 * the settings.
			 */
			if (strcmp(builtname, ent->d_name) == 0)
				break;
		}

error_close_sysfsfp:
	if (sysfsfp)
		if (fclose(sysfsfp))
			perror("get_channel_type(): Failed to close file");

error_free_filename:
	if (filename)
		free(filename);

error_closedir:
	if (closedir(dp) == -1)
		perror("get_channel_type(): Failed to close directory");

error_free_builtname_generic:
	free(builtname_generic);
error_free_builtname:
	free(builtname);
error_free_scan_el_dir:
	free(scan_el_dir);

	return ret;
}

/**
 * get_param_double() - read a double value from a channel parameter
 * @output: output the double value
 * @param_name: the parameter name to read
 * @device_dir: the IIO device directory in sysfs
 * @name: the channel name
 * @generic_name: the channel type name
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
int get_param_double(double *output, const char *param_name,
			     const char *device_dir, const char *name,
			     const char *generic_name)
{
	FILE *sysfsfp;
	int ret;
	DIR *dp;
	char *builtname, *builtname_generic;
	char *filename = NULL;
	const struct dirent *ent;

	ret = asprintf(&builtname, "%s_%s", name, param_name);
	if (ret < 0)
		return -ENOMEM;

	ret = asprintf(&builtname_generic,
		       "%s_%s", generic_name, param_name);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_builtname;
	}

	dp = opendir(device_dir);
	if (!dp) {
		ret = -errno;
		goto error_free_builtname_generic;
	}

	ret = -ENOENT;
	while (ent = readdir(dp), ent)
		if ((strcmp(builtname, ent->d_name) == 0) ||
		    (strcmp(builtname_generic, ent->d_name) == 0)) {
			ret = asprintf(&filename,
				       "%s/%s", device_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				goto error_closedir;
			}

			sysfsfp = fopen(filename, "r");
			if (!sysfsfp) {
				ret = -errno;
				goto error_free_filename;
			}

			errno = 0;
			if (fscanf(sysfsfp, "%lf", output) != 1)
				ret = errno ? -errno : -ENODATA;

			break;
		}
error_free_filename:
	if (filename)
		free(filename);

error_closedir:
	if (closedir(dp) == -1)
		perror("get_param_double(): Failed to close directory");

error_free_builtname_generic:
	free(builtname_generic);
error_free_builtname:
	free(builtname);

	return ret;
}

/**
 * bsort_channel_array_by_index() - sort the array in index order
 * @ci_array: the iioutils_chaninfo array to be sorted
 * @cnt: the amount of array elements
 **/

static void bsort_channel_array_by_index(struct iioutils_chaninfo *ci_array, int cnt)
{
	struct iioutils_chaninfo temp;
	int x, y;

	for (x = 0; x < cnt; x++)
		for (y = 0; y < (cnt - 1); y++)
			if (ci_array[y].index > ci_array[y + 1].index) {
				temp = ci_array[y + 1];
				ci_array[y + 1] = ci_array[y];
				ci_array[y] = temp;
			}
}

/**
 * iioutils_create_hrt_trigger() - create hr timer based software trigger
 * @name: name of the trigger
 *
 * Returns 0 on success, otherwise a negative error code.
 **/
int iioutils_create_hrt_trigger(const char *name)
{
	int ret = 0;
	char *dirname;

	if (asprintf(&dirname, hrt_trig_dir_fmt, name) < 0)
		return -ENOMEM;

	if (mkdir(dirname, 0755) && errno != EEXIST)
		ret = -errno;

	free(dirname);

	return ret;
}

/**
 * iioutils_remove_hrt_trigger() - delete hr timer based software trigger
 * @name: name of the trigger
 *
 * Returns 0 on success, otherwise a negative error code.
 **/
int iioutils_remove_hrt_trigger(const char *name)
{
	int ret = 0;
	char *dirname;

	if (asprintf(&dirname, hrt_trig_dir_fmt, name) < 0)
		return -ENOMEM;

	if (rmdir(dirname) && errno != ENOENT)
		ret = -errno;

	free(dirname);

	return ret;
}

/**
 * iioutils_find_by_name() - function to match top level types by name
 * @name: top level type instance name
 * @type: the type of top level instance being searched
 *
 * Returns the device number of a matched IIO device on success, otherwise a
 * negative error code.
 * Typical types this is used for are device and trigger.
 **/
int iioutils_find_by_name(const char *name, enum IIOUTILS_TYPE type, int match_of_name)
{
	const struct dirent *ent;
	int number, numstrlen, ret;

	FILE *namefp;
	DIR *dp;
	char thisname[IIOUTILS_MAX_NAME_LENGTH];
	char *filename;
	const char *typestr = iio_types[type];
	int typelen = strlen(typestr);

	dp = opendir(iio_dir);
	if (!dp) {
		fprintf(stderr, "No industrialio devices available\n");
		return -ENODEV;
	}

	while (ent = readdir(dp), ent) {
		if (strcmp(ent->d_name, ".") != 0 &&
		    strcmp(ent->d_name, "..") != 0 &&
		    strlen(ent->d_name) > typelen &&
		    strncmp(ent->d_name, typestr, typelen) == 0) {
			errno = 0;
			ret = sscanf(ent->d_name + typelen, "%d", &number);
			if (ret < 0) {
				ret = -errno;
				fprintf(stderr,
					"failed to read element number\n");
				goto error_close_dir;
			} else if (ret != 1) {
				ret = -EIO;
				fprintf(stderr,
					"failed to match element number\n");
				goto error_close_dir;
			}

			numstrlen = calc_digits(number);
			/* verify the next character is not a colon */
			if (strncmp(ent->d_name + typelen + numstrlen,
			    ":", 1) != 0) {
				ret = asprintf(&filename, "%s%s%d/%s",
					      iio_dir, typestr, number,
					      match_of_name ? "of_node/name" : "name");
				if (ret < 0) {
					ret = -ENOMEM;
					goto error_close_dir;
				}

				namefp = fopen(filename, "r");
				if (!namefp) {
					free(filename);
					continue;
				}

				free(filename);
				errno = 0;
				if (fscanf(namefp, "%s", thisname) != 1) {
					ret = errno ? -errno : -ENODATA;
					goto error_close_dir;
				}

				if (fclose(namefp)) {
					ret = -errno;
					goto error_close_dir;
				}

				if (strcmp(name, thisname) == 0) {
					if (closedir(dp) == -1)
						return -errno;

					return number;
				}
			}
		}
	}
	if (closedir(dp) == -1)
		return -errno;

	return -ENODEV;

error_close_dir:
	if (closedir(dp) == -1)
		perror("iioutils_find_by_name(): Failed to close directory");

	return ret;
}


/**
 * iioutils_write_string() - write string to a sysfs file
 * @filename: name of file to write to
 * @basedir: the sysfs directory in which the file is to be found
 * @val: the string to write
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
int iioutils_write_string(enum IIOUTILS_TYPE type, int index, const char *attr, const char *val)
{
	int ret = 0;
	FILE  *sysfsfp;
	char *temp;
	const char *typestr = iio_types[type];

	if (asprintf(&temp, "%s%s%d/%s", iio_dir, typestr, index, attr) < 0)
		return -ENOMEM;

	sysfsfp = fopen(temp, "w");
	if (!sysfsfp) {
		ret = -errno;
		fprintf(stderr, "Could not open %s\n", temp);
		goto error_free;
	}

	ret = fprintf(sysfsfp, "%s", val);
	if (ret < 0) {
		if (fclose(sysfsfp))
			perror("iioutils_write_string(): Failed to close dir");

		goto error_free;
	}

	if (fclose(sysfsfp)) {
		ret = -errno;
		goto error_free;
	}

error_free:
	free(temp);

	return ret;
}

/**
 * iioutils_write_int() - write an integer value to a sysfs file
 * @filename: name of the file to write to
 * @basedir: the sysfs directory in which the file is to be found
 * @val: integer value to write to file
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
int iioutils_write_int(enum IIOUTILS_TYPE type, int index, const char *attr, int val)
{
	char buf[IIOUTILS_MAX_VAL_LENGTH];

	snprintf(buf, IIOUTILS_MAX_VAL_LENGTH, "%d", val);
	return iioutils_write_string(type, index, attr, buf);
}

/**
 * iioutils_write_double() - write an double value to a sysfs file
 * @filename: name of the file to write to
 * @basedir: the sysfs directory in which the file is to be found
 * @val: double value to write to file
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
int iioutils_write_double(enum IIOUTILS_TYPE type, int index, const char *attr, double val)
{
	char buf[IIOUTILS_MAX_VAL_LENGTH];

	snprintf(buf, IIOUTILS_MAX_VAL_LENGTH, "%f", val);
	return iioutils_write_string(type, index, attr, buf);
}

/**
 * iioutils_read_string() - read a string from file
 * @filename: name of file to read from
 * @basedir: the sysfs directory in which the file is to be found
 * @str: output the read string
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
int iioutils_read_string(enum IIOUTILS_TYPE type, int index, const char *attr, char *val)
{
	int ret = 0;
	FILE  *sysfsfp;
	char *temp;
	const char *typestr = iio_types[type];

	if (asprintf(&temp, "%s%s%d/%s", iio_dir, typestr, index, attr) < 0)
		return -ENOMEM;

	sysfsfp = fopen(temp, "r");
	if (!sysfsfp) {
		ret = -errno;
		goto error_free;
	}

	errno = 0;
	ret = fread(val, 1, IIOUTILS_MAX_VAL_LENGTH, sysfsfp);
	if (ret < 0 || ret >= IIOUTILS_MAX_VAL_LENGTH) {
		ret = errno ? -errno : -ENODATA;
		if (fclose(sysfsfp))
			perror("iioutils_read_string(): Failed to close dir");

		goto error_free;
	}

	val[ret--] = 0;
	while (ret > 0 && val[ret] == '\n')
		val[ret--] = 0;

	if (fclose(sysfsfp))
		ret = -errno;

error_free:
	free(temp);

	return ret;
}

/**
 * iioutils_read_int() - read an integer value from file
 * @filename: name of file to read from
 * @basedir: the sysfs directory in which the file is to be found
 * @val: output the read integer value
 *
 * Returns the read integer value >= 0 on success, otherwise a negative error
 * code.
 **/
int iioutils_read_int(enum IIOUTILS_TYPE type, int index, const char *attr, int *val)
{
	int ret;
	char buf[IIOUTILS_MAX_VAL_LENGTH];

	ret = iioutils_read_string(type, index, attr, buf);
	if (ret < 0)
		return ret;

	errno = 0;
	if (sscanf(buf, "%d", val) != 1) {
		return errno ? -errno : -ENODATA;
	}

	return 0;
}

/**
 * iioutils_read_double() - read a double value from file
 * @filename: name of file to read from
 * @basedir: the sysfs directory in which the file is to be found
 * @val: output the read double value
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
int iioutils_read_double(enum IIOUTILS_TYPE type, int index, const char *attr, double *val)
{
	int ret;
	char buf[IIOUTILS_MAX_VAL_LENGTH];

	ret = iioutils_read_string(type, index, attr, buf);
	if (ret < 0)
		return ret;

	errno = 0;
	if (sscanf(buf, "%lf", val) != 1) {
		return errno ? -errno : -ENODATA;
	}

	return 0;
}

/**
 * iioutils_read_mount_matrix() - read a mount matrix from file
 * @filename: name of file to read from
 * @basedir: the sysfs directory in which the file is to be found
 * @val: output the read mount matrix value
 *
 * Returns a value >= 0 on success, otherwise a negative error code.
 **/
int iioutils_read_mount_matrix(enum IIOUTILS_TYPE type, int index, struct iioutils_mount_matrix *val)
{
	int ret;
	char buf[IIOUTILS_MAX_VAL_LENGTH];

	ret = iioutils_read_string(type, index, "in_mount_matrix", buf);
	if (ret < 0)
		goto fail;

	ret = sscanf(buf, "%lf,%lf,%lf;%lf,%lf,%lf;%lf,%lf,%lf",
		&val->x.x, &val->x.y, &val->x.z,
		&val->y.x, &val->y.y, &val->y.z,
		&val->z.x, &val->z.y, &val->z.z);
	if (ret != 9) {
		ret = -EINVAL;
		goto fail;
	}

	return 0;

fail:
	// init with 1:1 tranformation matrix
	val->x.x = 1.0; val->x.y = 0.0; val->x.z = 0.0;
	val->y.x = 0.0; val->y.y = 1.0; val->y.z = 0.0;
	val->z.x = 0.0; val->z.y = 0.0; val->z.z = 1.0;

	return ret;
}

/**
 * iioutils_transform_mount_matrix() - transform a 3d vector via mount matrix
 * @mtx: punter to matrix to be applied
 * @vect: vector to be processed
 **/
vector3d_t iioutils_transform_mount_matrix(const struct iioutils_mount_matrix *mtx, vector3d_t vect) {
	vector3d_t ret;

	ret.x = vector3d_dot_product(vect, mtx->x);
	ret.y = vector3d_dot_product(vect, mtx->y);
	ret.z = vector3d_dot_product(vect, mtx->z);

	return ret;
}

/**
 * iioutils_build_channel_array() - function to figure out what channels are present
 * @device_dir: the IIO device directory in sysfs
 * @ci_array: output the resulting array of iioutils_chaninfo
 * @counter: output the amount of array elements
 *
 * Returns 0 on success, otherwise a negative error code.
 **/
int iioutils_build_channel_array(enum IIOUTILS_TYPE type, int index,
			struct iioutils_chaninfo **ci_array, int *counter)
{
	char *device_dir;
	DIR *dp;
	FILE *sysfsfp;
	int count = 0, i;
	struct iioutils_chaninfo *current;
	int ret;
	const struct dirent *ent;
	char *scan_el_dir;
	char *filename;
	const char *typestr = iio_types[type];

	*counter = 0;

	if (asprintf(&device_dir, "%s%s%d", iio_dir, typestr, index) < 0)
		return -ENOMEM;

	ret = asprintf(&scan_el_dir, FORMAT_SCAN_ELEMENTS_DIR, device_dir);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_device_dir;
	}

	dp = opendir(scan_el_dir);
	if (!dp) {
		ret = -errno;
		goto error_free_name;
	}

	while (ent = readdir(dp), ent)
		if (strcmp(ent->d_name + strlen(ent->d_name) - strlen("_en"),
			   "_en") == 0) {
			ret = asprintf(&filename,
				       "%s/%s", scan_el_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				goto error_close_dir;
			}

			sysfsfp = fopen(filename, "r");
			if (!sysfsfp) {
				ret = -errno;
				free(filename);
				goto error_close_dir;
			}

			errno = 0;
			if (fscanf(sysfsfp, "%i", &ret) != 1) {
				ret = errno ? -errno : -ENODATA;
				if (fclose(sysfsfp))
					perror("iioutils_build_channel_array(): Failed to close file");

				free(filename);
				goto error_close_dir;
			}
			if (ret == 1)
				(*counter)++;

			if (fclose(sysfsfp)) {
				ret = -errno;
				free(filename);
				goto error_close_dir;
			}

			free(filename);
		}

	*ci_array = malloc(sizeof(**ci_array) * (*counter));
	if (!*ci_array) {
		ret = -ENOMEM;
		goto error_close_dir;
	}

	seekdir(dp, 0);
	while (ent = readdir(dp), ent) {
		if (strcmp(ent->d_name + strlen(ent->d_name) - strlen("_en"),
			   "_en") == 0) {
			int current_enabled = 0;

			current = &(*ci_array)[count++];
			ret = asprintf(&filename,
				       "%s/%s", scan_el_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				/* decrement count to avoid freeing name */
				count--;
				goto error_cleanup_array;
			}

			sysfsfp = fopen(filename, "r");
			if (!sysfsfp) {
				ret = -errno;
				free(filename);
				count--;
				goto error_cleanup_array;
			}

			errno = 0;
			if (fscanf(sysfsfp, "%i", &current_enabled) != 1) {
				ret = errno ? -errno : -ENODATA;
				free(filename);
				count--;
				goto error_cleanup_array;
			}

			if (fclose(sysfsfp)) {
				ret = -errno;
				free(filename);
				count--;
				goto error_cleanup_array;
			}

			if (!current_enabled) {
				free(filename);
				count--;
				continue;
			}

			current->scale = 1.0;
			current->offset = 0;
			current->name = strndup(ent->d_name,
						strlen(ent->d_name) -
						strlen("_en"));
			if (!current->name) {
				free(filename);
				ret = -ENOMEM;
				count--;
				goto error_cleanup_array;
			}

			/* Get the generic and specific name elements */
			ret = break_up_name(current->name,
						     &current->generic_name);
			if (ret) {
				free(filename);
				free(current->name);
				count--;
				goto error_cleanup_array;
			}

			ret = asprintf(&filename,
				       "%s/%s_index",
				       scan_el_dir,
				       current->name);
			if (ret < 0) {
				free(filename);
				ret = -ENOMEM;
				goto error_cleanup_array;
			}

			sysfsfp = fopen(filename, "r");
			if (!sysfsfp) {
				ret = -errno;
				fprintf(stderr, "failed to open %s\n",
					filename);
				free(filename);
				goto error_cleanup_array;
			}

			errno = 0;
			if (fscanf(sysfsfp, "%u", &current->index) != 1) {
				ret = errno ? -errno : -ENODATA;
				if (fclose(sysfsfp))
					perror("iioutils_build_channel_array(): Failed to close file");

				free(filename);
				goto error_cleanup_array;
			}

			if (fclose(sysfsfp)) {
				ret = -errno;
				free(filename);
				goto error_cleanup_array;
			}

			free(filename);
			/* Find the scale */
			ret = get_param_double(&current->scale,
						       "scale",
						       device_dir,
						       current->name,
						       current->generic_name);
			if ((ret < 0) && (ret != -ENOENT))
				goto error_cleanup_array;

			ret = get_param_double(&current->offset,
						       "offset",
						       device_dir,
						       current->name,
						       current->generic_name);
			if ((ret < 0) && (ret != -ENOENT))
				goto error_cleanup_array;

			ret = get_channel_type(&current->is_signed,
						&current->bytes,
						&current->bits_used,
						&current->shift,
						&current->mask,
						&current->be,
						device_dir,
						current->name,
						current->generic_name);
			if (ret < 0)
				goto error_cleanup_array;
		}
	}

	if (closedir(dp) == -1) {
		ret = -errno;
		goto error_cleanup_array;
	}

	free(scan_el_dir);
	/* reorder so that the array is in index order */
	bsort_channel_array_by_index(*ci_array, *counter);

	return 0;

error_cleanup_array:
	for (i = count - 1;  i >= 0; i--) {
		free((*ci_array)[i].name);
		free((*ci_array)[i].generic_name);
	}
	free(*ci_array);
	*ci_array = NULL;
	*counter = 0;
error_close_dir:
	if (dp)
		if (closedir(dp) == -1)
			perror("iioutils_build_channel_array(): Failed to close dir");

error_free_name:
	free(scan_el_dir);

error_free_device_dir:
	free(device_dir);

	return ret;
}

/**
 * iioutils_free_channel_array() - free array created by iioutils_build_channel_array
 * @ci_array: pointer to array of iioutils_chaninfo
 * @counter: amount of array elements
 **/
void iioutils_free_channel_array(struct iioutils_chaninfo *ci_array, int counter)
{
	struct iioutils_chaninfo *p;

	if (ci_array == NULL)
		return;

	for (p = ci_array; counter > 0; counter--, p++) {
		free(p->name);
		free(p->generic_name);
	}

	free(ci_array);
}

/**
 * iioutils_get_channel_size() - calculate the storage size of a scan
 * @channels:		the channel info array
 * @num_channels:	number of channels
 *
 * Has the side effect of filling the channels[i].location values used
 * in processing the buffer output.
 **/
int iioutils_get_scan_size(struct iioutils_chaninfo *channels, int num_channels)
{
	int bytes = 0;
	int i = 0;

	while (i < num_channels) {
		if (bytes % channels[i].bytes == 0)
			channels[i].location = bytes;
		else
			channels[i].location = bytes - bytes % channels[i].bytes
					       + channels[i].bytes;

		bytes = channels[i].location + channels[i].bytes;
		i++;
	}

	return bytes;
}

static inline uint32_t read_u8(const struct iioutils_chaninfo *channel, const char *buffer)
{
	uint8_t data = *((uint8_t *) (&buffer[channel->location]));
	return data;
}

static inline uint32_t read_u16(const struct iioutils_chaninfo *channel, const char *buffer)
{
	uint16_t data = *((uint16_t *) (&buffer[channel->location]));
	return (channel->be) ? be16toh(data) : le16toh(data);
}

static inline uint32_t read_u32(const struct iioutils_chaninfo *channel, const char *buffer)
{
	uint32_t data = *((uint32_t *) (&buffer[channel->location]));
	return (channel->be) ? be32toh(data) : le32toh(data);
}

double iioutils_read_scan_double(const struct iioutils_chaninfo *channel, const char *buffer)
{
	uint32_t raw;
	int ext;
	double val;

	// get raw data in correct byte order
	switch (channel->bytes) {
	case 1:
		raw = read_u8(channel, buffer);
		break;
	case 2:
		raw = read_u16(channel, buffer);
		break;
	case 4:
		raw = read_u32(channel, buffer);
		break;
	default:
		return NAN;
	}

	// do shift and mask
	raw >>= channel->shift;
	raw &= channel->mask;

	// expand sign if needed
	if (channel->is_signed) {
		ext = 32 - channel->bits_used;
		val = (((int32_t) raw) << ext) >> ext;
	} else {
		val = raw;
	}

	// scale to output value
	return val * channel->scale + channel->offset;
}

