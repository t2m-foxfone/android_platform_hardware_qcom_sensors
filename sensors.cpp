/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "Sensors"

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <linux/input.h>
#include <utils/Atomic.h>
#include <utils/Log.h>

#include "sensors.h"
#ifdef BMA222E_SENSOR
#include "BmaSensor.h"
#else
#include "AccelSensor.h"
#endif
#include "LightSensor.h"
#include "ProximitySensor.h"
#include "AkmSensor.h"
#include "GyroSensor.h"
#include "PressureSensor.h"
#include "ST480Sensor.h"

/*****************************************************************************/
static struct sensor_t sensor_list[MAX_SENSORS];
static char name[MAX_SENSORS][SYSFS_MAXLEN];
static char vendor[MAX_SENSORS][SYSFS_MAXLEN];
static int dynamic_sensor_number;

static int open_sensors(const struct hw_module_t* module, const char* id,
						struct hw_device_t** device);

static int get_node(char *buf, char *path) {
	char * fret;
	int len = 0;
	FILE * fd;

	if (NULL == buf || NULL == path)
		return -1;

	fd = fopen(path, "r");
	if (NULL == fd)
		return -1;

	fret = fgets(buf,SYSFS_MAXLEN,fd);
	if (NULL == fret) {
		fclose(fd);
		return -1;
	}

	len = strlen(buf);

	if (buf[len - 1] == '\n')
		buf[len - 1] = '\0';

	fclose(fd);
	return 0;
}

static int get_sensors_list() {
	int number = 0;
	int fd = -1;
	int err = -1;
	const char *dirname = SYSFS_CLASS;
	char devname[PATH_MAX];
	char *filename;
	char *nodename;
	DIR *dir;
	struct dirent *de;
	char tempname[SYSFS_MAXLEN];

	dir = opendir(dirname);
	if(dir == NULL) {
		dynamic_sensor_number = 0;
		return -1;
	}
	strlcpy(devname, dirname, PATH_MAX - SYSFS_MAXLEN * 2 - 2);
	filename = devname + strlen(devname);
	*filename++ = '/';

	while((de = readdir(dir))) {
		if(de->d_name[0] == '.' &&
			(de->d_name[1] == '\0' ||
				(de->d_name[1] == '.' && de->d_name[2] == '\0')))
			continue;

		strlcpy(filename, de->d_name, SYSFS_MAXLEN);
		nodename = filename + strlen(de->d_name);
		*nodename++ = '/';

		strlcpy(nodename, SYSFS_NAME, SYSFS_MAXLEN);
		err = get_node(name[number], devname);
		if(err < 0)
			goto error;
		sensor_list[number].name = name[number];

		strlcpy(nodename, SYSFS_VENDOR, SYSFS_MAXLEN);
		err = get_node(vendor[number], devname);
		if(err < 0)
			goto error;
		sensor_list[number].vendor = vendor[number];

		strlcpy(nodename, SYSFS_VERSION, SYSFS_MAXLEN);
		err = get_node(tempname, devname);
		if(err < 0)
			goto error;
		sensor_list[number].version = atoi(tempname);

		strlcpy(nodename, SYSFS_HANDLE, SYSFS_MAXLEN);
		err = get_node(tempname, devname);
		if(err < 0)
			goto error;
		sensor_list[number].handle = atoi(tempname);

		strlcpy(nodename, SYSFS_TYPE, SYSFS_MAXLEN);
		err = get_node(tempname, devname);
		if(err < 0)
			goto error;
		sensor_list[number].type = atoi(tempname);

		strlcpy(nodename, SYSFS_MAXRANGE, SYSFS_MAXLEN);
		err = get_node(tempname, devname);
		if(err < 0)
			goto error;
		sensor_list[number].maxRange = atof(tempname);

		strlcpy(nodename, SYSFS_RESOLUTION, SYSFS_MAXLEN);
		err = get_node(tempname, devname);
		if(err < 0)
			goto error;
		sensor_list[number].resolution = atof(tempname);

		strlcpy(nodename, SYSFS_POWER, SYSFS_MAXLEN);
		err = get_node(tempname, devname);
		if(err < 0)
			goto error;
		sensor_list[number].power = atof(tempname);

		strlcpy(nodename, SYSFS_MINDELAY, SYSFS_MAXLEN);
		err = get_node(tempname, devname);
		if(err < 0)
			goto error;
		sensor_list[number].minDelay = atoi(tempname);

		number++;
	}
	closedir(dir);
	dynamic_sensor_number = number;
	return number;

error:
	dynamic_sensor_number = 0;
	closedir(dir);
	ALOGE("get_sensors_list failed!");
	return -1;
}

static int sensors__get_sensors_list(struct sensors_module_t* module,
								 struct sensor_t const** list)
{
	if(dynamic_sensor_number > 0) {
		*list = sensor_list;
		return dynamic_sensor_number;
	} else { /* If we could not find any sensor folder, load the default.*/
		ALOGE("no list:get_sensors_list failed!");
		return 0;
	}
}

static struct hw_module_methods_t sensors_module_methods = {
		open: open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
		common: {
				tag: HARDWARE_MODULE_TAG,
				version_major: 1,
				version_minor: 0,
				id: SENSORS_HARDWARE_MODULE_ID,
				name: "Quic Sensor module",
				author: "Quic",
				methods: &sensors_module_methods,
		},
		get_sensors_list: sensors__get_sensors_list,
};

struct sensors_poll_context_t {
	struct sensors_poll_device_t device; // must be first

		sensors_poll_context_t();
		~sensors_poll_context_t();
	int activate(int handle, int enabled);
	int setDelay(int handle, int64_t ns);
	int pollEvents(sensors_event_t* data, int count);

private:
	int light;
	int proximity;
	int compass;
	int gyro;
	int accel;
	int pressure;
	static const size_t wake = MAX_SENSORS;
	static const char WAKE_MESSAGE = 'W';
	struct pollfd mPollFds[MAX_SENSORS+1];
	int mWritePipeFd;
	int device_id;
	SensorBase* mSensors[MAX_SENSORS];

	int handleToDriver(int handle) const {
		switch (handle) {
			case SENSORS_ACCELERATION_HANDLE:
				return accel;
			case SENSORS_MAGNETIC_FIELD_HANDLE:
			case SENSORS_ORIENTATION_HANDLE:
				return compass;
			case SENSORS_PROXIMITY_HANDLE:
				return proximity;
			case SENSORS_LIGHT_HANDLE:
				return light;
			case SENSORS_GYROSCOPE_HANDLE:
				return gyro;
			case SENSORS_PRESSURE_HANDLE:
				return pressure;
		}
		return -EINVAL;
	}
};

/*****************************************************************************/

sensors_poll_context_t::sensors_poll_context_t()
{
	int number;
	int handle;
	int compass_id = -1;
	light = -1;
	proximity = -1;
	compass = -1;
	gyro = -1;
	accel = -1;
	pressure = -1;
	device_id = 0;
	number = get_sensors_list();

	if(number <= 0){ /* use the static sensor list */
		light = 0;
		proximity = 1;
		compass = 2;
		gyro = 3;
		accel = 4;
		pressure = 5;
		device_id = 6;

		mSensors[light] = new LightSensor();
		mPollFds[light].fd = mSensors[light]->getFd();
		mPollFds[light].events = POLLIN;
		mPollFds[light].revents = 0;

		mSensors[proximity] = new ProximitySensor();
		mPollFds[proximity].fd = mSensors[proximity]->getFd();
		mPollFds[proximity].events = POLLIN;
		mPollFds[proximity].revents = 0;

#if 0
		mSensors[compass] = new AkmSensor();
		mPollFds[compass].fd = mSensors[compass]->getFd();
		mPollFds[compass].events = POLLIN;
		mPollFds[compass].revents = 0;
#endif

		mSensors[gyro] = new GyroSensor();
		mPollFds[gyro].fd = mSensors[gyro]->getFd();
		mPollFds[gyro].events = POLLIN;
		mPollFds[gyro].revents = 0;

#ifdef BMA222E_SENSOR
		mSensors[accel] = new BmaSensor("bma2x2", TOP_Y_FORWARD);
#else
		mSensors[accel] = new AccelSensor();
#endif
		mPollFds[accel].fd = mSensors[accel]->getFd();
		mPollFds[accel].events = POLLIN;
		mPollFds[accel].revents = 0;

		mSensors[compass] = new ST480Sensor((BmaSensor*)mSensors[accel]);
		mPollFds[compass].fd = mSensors[compass]->getFd();
		mPollFds[compass].events = POLLIN;
		mPollFds[compass].revents = 0;

		mSensors[pressure] = new PressureSensor();
		mPollFds[pressure].fd = mSensors[pressure]->getFd();
		mPollFds[pressure].events = POLLIN;
		mPollFds[pressure].revents = 0;

	} else { /* use the dynamic sensor list */
		for (handle = 0; handle < number; handle++) {
			switch (sensor_list[handle].handle) {
				case SENSORS_ACCELERATION_HANDLE:
#ifdef BMA222E_SENSOR
				mSensors[device_id] = new BmaSensor(name[handle], TOP_Y_FORWARD);
#else
				mSensors[device_id] = new AccelSensor(name[handle]);
#endif
				mPollFds[device_id].fd = mSensors[device_id]->getFd();
				mPollFds[device_id].events = POLLIN;
				mPollFds[device_id].revents = 0;
				accel = device_id;
				break;
#if 0
				case SENSORS_MAGNETIC_FIELD_HANDLE:
				mSensors[device_id] = new AkmSensor();
				mPollFds[device_id].fd = mSensors[device_id]->getFd();
				mPollFds[device_id].events = POLLIN;
				mPollFds[device_id].revents = 0;
				compass = device_id;
				break;
#endif

				case SENSORS_ORIENTATION_HANDLE:
				if (accel > 0) {
					mSensors[device_id] = new ST480Sensor((BmaSensor*)mSensors[accel]);
					mPollFds[device_id].fd = mSensors[device_id]->getFd();
					mPollFds[device_id].events = POLLIN;
					mPollFds[device_id].revents = 0;
					compass = device_id;
				}
				compass_id = device_id;
				break;

				case SENSORS_PROXIMITY_HANDLE:
				mSensors[device_id] = new ProximitySensor(name[handle]);
				mPollFds[device_id].fd = mSensors[device_id]->getFd();
				mPollFds[device_id].events = POLLIN;
				mPollFds[device_id].revents = 0;
				proximity = device_id;
				break;

				case SENSORS_LIGHT_HANDLE:
				mSensors[device_id] = new LightSensor(name[handle]);
				mPollFds[device_id].fd = mSensors[device_id]->getFd();
				mPollFds[device_id].events = POLLIN;
				mPollFds[device_id].revents = 0;
				light = device_id;
				break;

				case SENSORS_GYROSCOPE_HANDLE:
				mSensors[device_id] = new GyroSensor(name[handle]);
				mPollFds[device_id].fd = mSensors[device_id]->getFd();
				mPollFds[device_id].events = POLLIN;
				mPollFds[device_id].revents = 0;
				gyro = device_id;
				break;

				case SENSORS_PRESSURE_HANDLE:
				mSensors[device_id] = new PressureSensor(name[handle]);
				mPollFds[device_id].fd = mSensors[device_id]->getFd();
				mPollFds[device_id].events = POLLIN;
				mPollFds[device_id].revents = 0;
				pressure = device_id;
				break;

				default:
				ALOGE("No handle %d for this type sensor!",handle);
				device_id--;
			}
			device_id++;
		}

		if (compass < 0)
		{
			mSensors[compass_id] = new ST480Sensor((BmaSensor*)mSensors[accel]);
			mPollFds[compass_id].fd = mSensors[compass_id]->getFd();
			mPollFds[compass_id].events = POLLIN;
			mPollFds[compass_id].revents = 0;
			compass = compass_id;
		}
	}
	ALOGI("The avaliable sensor handle number is %d",device_id);
	int wakeFds[2];
	int result = pipe(wakeFds);
	ALOGE_IF(result<0, "error creating wake pipe (%s)", strerror(errno));
	fcntl(wakeFds[0], F_SETFL, O_NONBLOCK);
	fcntl(wakeFds[1], F_SETFL, O_NONBLOCK);
	mWritePipeFd = wakeFds[1];

	mPollFds[device_id].fd = wakeFds[0];
	mPollFds[device_id].events = POLLIN;
	mPollFds[device_id].revents = 0;
}

sensors_poll_context_t::~sensors_poll_context_t() {
	for (int i=0 ; i<device_id ; i++) {
		delete mSensors[i];
	}
	close(mPollFds[device_id].fd);
	close(mWritePipeFd);
}

int sensors_poll_context_t::activate(int handle, int enabled) {
	int index = handleToDriver(handle);
	if (index < 0) return index;
	int err =  mSensors[index]->enable(handle, enabled);
	if (enabled && !err) {
		const char wakeMessage(WAKE_MESSAGE);
		int result = write(mWritePipeFd, &wakeMessage, 1);
		ALOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
	}
	return err;
}

int sensors_poll_context_t::setDelay(int handle, int64_t ns) {

	int index = handleToDriver(handle);
	if (index < 0) return index;
	return mSensors[index]->setDelay(handle, ns);
}

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
	int nbEvents = 0;
	int n = 0;

	do {
		// see if we have some leftover from the last poll()
		for (int i=0 ; count && i<device_id ; i++) {
			SensorBase* const sensor(mSensors[i]);
			if ((mPollFds[i].revents & POLLIN) || (sensor->hasPendingEvents())) {
				int nb = sensor->readEvents(data, count);
				if (nb < count) {
					// no more data for this sensor
					mPollFds[i].revents = 0;
				}
				count -= nb;
				nbEvents += nb;
				data += nb;
			}
		}

		if (count) {
			// we still have some room, so try to see if we can get
			// some events immediately or just wait if we don't have
			// anything to return
			do {
				n = poll(mPollFds, device_id+1, nbEvents ? 0 : -1);
			} while (n < 0 && errno == EINTR);
			if (n<0) {
				ALOGE("poll() failed (%s)", strerror(errno));
				return -errno;
			}
			if (mPollFds[device_id].revents & POLLIN) {
				char msg;
				int result = read(mPollFds[device_id].fd, &msg, 1);
				ALOGE_IF(result<0, "error reading from wake pipe (%s)", strerror(errno));
				ALOGE_IF(msg != WAKE_MESSAGE, "unknown message on wake queue (0x%02x)", int(msg));
				mPollFds[device_id].revents = 0;
			}
		}
		// if we have events and space, go read them
	} while (n && count);

	return nbEvents;
}

/*****************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	if (ctx) {
		delete ctx;
	}
	return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
		int handle, int enabled) {
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
		int handle, int64_t ns) {
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	return ctx->setDelay(handle, ns);
}

static int poll__poll(struct sensors_poll_device_t *dev,
		sensors_event_t* data, int count) {
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	return ctx->pollEvents(data, count);
}

/*****************************************************************************/

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
						struct hw_device_t** device)
{
		int status = -EINVAL;
		sensors_poll_context_t *dev = new sensors_poll_context_t();

		memset(&dev->device, 0, sizeof(sensors_poll_device_t));

		dev->device.common.tag = HARDWARE_DEVICE_TAG;
		dev->device.common.version  = 0;
		dev->device.common.module   = const_cast<hw_module_t*>(module);
		dev->device.common.close	= poll__close;
		dev->device.activate		= poll__activate;
		dev->device.setDelay		= poll__setDelay;
		dev->device.poll			= poll__poll;

		*device = &dev->device.common;
		status = 0;

		return status;
}

