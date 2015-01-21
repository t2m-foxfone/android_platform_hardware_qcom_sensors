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

#ifndef ANDROID_ST480_SENSOR_H
#define ANDROID_ST480_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <linux/ioctl.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"
#include "RunAlgorithm.h"
#include "BmaSensor.h"
#define ST480_DEVICE_NAME     "/dev/st480"


struct input_event;
class BmaSensor;

class ST480Sensor : public SensorBase {
public:
		ST480Sensor(BmaSensor* as);
	virtual ~ST480Sensor();

	enum {
		MagneticField   = 0,
		Orientation	= 1,
		numSensors
	};

	virtual int setDelay(int32_t handle, int64_t ns);
	virtual int enable(int32_t handle, int enabled);
	virtual int readEvents(sensors_event_t* data, int count);

private:
	int update_delay();
	int mEnCount;
	uint32_t mEnabled;
	uint32_t mPendingMask;
	InputEventCircularReader mInputReader;
	sensors_event_t mPendingEvents[numSensors];
	uint64_t mDelays[numSensors];
	bool mMaEnabled;
	bool mOrEnabled;
	_st480 st480;
	BmaSensor *mAccSensor; //Tori
};

/*****************************************************************************/

#define SENODIAIO                   0xA1

/* IOCTLs for hal */
#define ECS_IOCTL_APP_SET_MFLAG         _IOW(SENODIAIO, 0x10, short)
#define ECS_IOCTL_APP_GET_MFLAG         _IOR(SENODIAIO, 0x11, short)
#define ECS_IOCTL_APP_SET_DELAY         _IOW(SENODIAIO, 0x12, short)
#define ECS_IOCTL_APP_GET_DELAY         _IOR(SENODIAIO, 0x13, short)
#define ECS_IOCTL_APP_SET_MVFLAG        _IOW(SENODIAIO, 0x14, short)
#define ECS_IOCTL_APP_GET_MVFLAG        _IOR(SENODIAIO, 0x15, short)

#endif
