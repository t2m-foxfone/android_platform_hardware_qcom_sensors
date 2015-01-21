/*
 * Copyright (C) 2008-2013 The Android Open Source Project
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

#ifndef ANDROID_BMA2x2_SENSOR_H
#define ANDROID_BMA2x2_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>


#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"

/*****************************************************************************/

struct input_event;

enum BmaLayout {
    /*the definition is from top view*/
    TOP_Y_FORWARD      = 0,
    TOP_Y_RIGHTWARD    = 1,
    TOP_Y_BACKWARD     = 2,
    TOP_Y_LEFTWARD     = 3,
    BOTTOM_Y_FORWARD   = 4,
    BOTTOM_Y_RIGHTWARD = 5,
    BOTTOM_Y_BACKWARD  = 6,
    BOTTOM_Y_LEFTWARD  = 7,

    BMA_LAYOUT_MAX
};

class BmaSensor : public SensorBase {
public:
            BmaSensor(const char* InputName, BmaLayout Layout);
    virtual ~BmaSensor();

    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);
    virtual int readEvents(sensors_event_t* data, int count);
    void processEvent(int code, int value);
    void getAccData(sensors_event_t* data){*data = mAccData;}
private:
	int sensor_get_class_path(char *class_path, const char* InputName);
	int is_sensor_enabled();
	int enable_sensor();
	int disable_sensor();
	int set_delay(int64_t ns);

	int update_delay();

	int readDisable();
	int writeDisable(int isDisable);
	int writeDelay(int64_t ns);

    enum {
        Primary = 0,
        Secondary,
        numChannels
    };

    uint32_t mEnabled;
    uint32_t mPendingMask;
    char mClassPath[PATH_MAX];
    InputEventCircularReader mInputReader;
    sensors_event_t mPendingEvent;
    int64_t mDelay;
    int mbitnum;
    BmaLayout mLayout;
    unsigned char get_chip_id();
    int get_sensor_bitnum();
    void RemapAxis(int SrcAxis, int SrcValue, sensors_vec_t* Vector);
    sensors_event_t mAccData;
};

/*****************************************************************************/

#endif  // ANDROID_BMA2x2_SENSOR_H
