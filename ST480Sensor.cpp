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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>

#include <cutils/log.h>

#define EVENT_TYPE_MAG_X                       ABS_HAT0X
#define EVENT_TYPE_MAG_Y                       ABS_HAT0Y
#define EVENT_TYPE_MAG_Z                       ABS_BRAKE

#define ID_A    SENSORS_ACCELERATION_HANDLE
#define ID_M    SENSORS_MAGNETIC_FIELD_HANDLE
#define ID_O    SENSORS_ORIENTATION_HANDLE



#include "Log.h"
#include "ST480Sensor.h"
//#include "BmaSensor.h"
/*****************************************************************************/

ST480Sensor::ST480Sensor(BmaSensor* as)
: SensorBase(ST480_DEVICE_NAME, "compass"),
	//mEnCount(0),
	mEnabled(0),
	mPendingMask(0),
	mInputReader(32),
	mMaEnabled(0),
	mAccSensor(as)  //Tori
{
	memset(mPendingEvents, 0, sizeof(mPendingEvents));

	mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
	mPendingEvents[MagneticField].sensor = ID_M;
	mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
	mPendingEvents[MagneticField].magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;

	mPendingEvents[Orientation  ].version = sizeof(sensors_event_t);
	mPendingEvents[Orientation  ].sensor = ID_O;
	mPendingEvents[Orientation  ].type = SENSOR_TYPE_ORIENTATION;
	mPendingEvents[Orientation  ].orientation.status = SENSOR_STATUS_ACCURACY_HIGH;

	for (int i=0 ; i<numSensors ; i++)
		mDelays[i] = 27000000; // 50 ms by default

	st480.acc_x = 0;
	st480.acc_y = 0;
	st480.acc_z = 0;
	st480.mag_x = 0;
	st480.mag_y = 0;
	st480.mag_z = 0;
}

ST480Sensor::~ST480Sensor() {
}


int ST480Sensor::enable(int32_t handle, int en)
{
	int what = -1;
	int AFlag = 1;

	LOGD("ID_M = %d, handle = %d, en = %d",ID_M,handle,en);
	switch (handle) {
		case ID_M:
			what = MagneticField;
			mMaEnabled = en;
			break;
		case ID_O:
			what = Orientation;
			mOrEnabled = en;
			break;
		default: 
			return -EINVAL;
	}

	if (uint32_t(what) >= numSensors)
		return -EINVAL;

	//Tori
	if((!mMaEnabled) && (!mOrEnabled))
        {
                AFlag = 0;
        }

	int newState  = en ? 1 : 0;
	int err = 0;

	/*int oldCount = mEnCount;
	if(newState)
		mEnCount++;
	else
		mEnCount--;
	
	if(mEnCount<0)
		mEnCount=0;
	
	int flags = -1;
	if(oldCount==0 && mEnCount>0)
		flags = 1;
	if(oldCount>0 && mEnCount==0)
		flags = 0;

	LOGD("flags = %d",flags);

	if (flags != -1)
	{*/
		if ((uint32_t(newState)<<what) != (mEnabled & (1<<what))) {
			if (!mEnabled) {
				open_device();
			}
			int cmd;
			switch (what) {
				case MagneticField: cmd = ECS_IOCTL_APP_SET_MFLAG; break;
				case Orientation:   cmd = ECS_IOCTL_APP_SET_MVFLAG;  break;
				default: return -EINVAL;
			}
			short flags = newState;
			err = ioctl(dev_fd, cmd, &flags);
			err = err<0 ? -errno : 0;
			LOGE_IF(err, "ECS_IOCTL_ST480_SET_XXX failed (%s)", strerror(-err));
			if (!err) {
				mEnabled &= ~(1<<what);
				mEnabled |= (uint32_t(flags)<<what);
				update_delay();
			}
			if (!mEnabled) {
				close_device();
			}
		}
	//}

	//Tori
        if(mAccSensor!=NULL)
        {
                err = mAccSensor->enable(ID_A, AFlag);
        }

	return err;
}

int ST480Sensor::setDelay(int32_t handle, int64_t ns)
{
	int what = -1;
	switch (handle) {
		case ID_M: what = MagneticField; break;
		case ID_O: what = Orientation;   break;
		default: return -EINVAL;
	}

	if (uint32_t(what) >= numSensors)
		return -EINVAL;

	if (ns < 0)
		return -EINVAL;

	 //Tori
        if(handle == ID_O || handle == ID_A)
        {
		if(mAccSensor!=NULL)
		{
            		mAccSensor->setDelay(ID_A, ns);
		}
        }

	mDelays[what] = ns;
	return update_delay();
}

int ST480Sensor::update_delay()
{
	if (mEnabled) {
		uint64_t wanted = -1LLU;
		for (int i=0 ; i<numSensors ; i++) {
			if (mEnabled & (1<<i)) {
				uint64_t ns = mDelays[i];
				wanted = wanted < ns ? wanted : ns;
			}
		}
		short delay = int64_t(wanted) / 1000000;
	
		delay = (int)(1000/38);
	
		if (ioctl(dev_fd, ECS_IOCTL_APP_SET_DELAY, &delay)) {
			return -errno;
		}
	}
	return 0;
}

int ST480Sensor::readEvents(sensors_event_t* data, int count)
{
	if (count < 1)
		return -EINVAL;

	ssize_t n = mInputReader.fill(data_fd);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;
	sensors_event_t accData;//Tori
	static int64_t prev_time;

	while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS) {
			float value = event->value;
                	switch (event->code) {
                        	case EVENT_TYPE_MAG_X:
					mPendingMask |= 1<<MagneticField;
                                	st480.mag_x = value;
                                	break;
                        	case EVENT_TYPE_MAG_Y:
					mPendingMask |= 1<<MagneticField;
                                	st480.mag_y = value;
                                	break;
                        	case EVENT_TYPE_MAG_Z:
					mPendingMask |= 1<<MagneticField;
                                	st480.mag_z = value;
                                	break;
                	}

                	mAccSensor->getAccData(&accData);
                	st480.acc_x = accData.acceleration.x;
                	st480.acc_y = accData.acceleration.y;
                	st480.acc_z = accData.acceleration.z;

 			mInputReader.next();
		} else if (type == EV_SYN) {
			int64_t time = timevalToNano(event->time);
			
			if(mMaEnabled || mOrEnabled)
            		{
                		int time_diff_ms = 0;
                		if(prev_time > 0)
                    			time_diff_ms = (int)((time - prev_time) / 1000);
                		prev_time = time;
#if 0
				LOGD("Mag_x = %f, Mag_y = %f, Mag_z = %f", st480.mag_x,st480.mag_y,st480.mag_z);
                        	LOGD("acc_x = %f, acc_y = %f, acc_z = %f", st480.acc_x,st480.acc_y,st480.acc_z);
				LOGD("time_diff_ms = %d",time_diff_ms);
                		//Run Algorithm
#endif
                		run_library(st480, time_diff_ms);
         
               		 	float angles[3];
				float values[3];

                		if(mMaEnabled)
				{	
		    			get_magnetic_values(values);
		    			mPendingEvents[MagneticField].magnetic.x = values[0];
		    			mPendingEvents[MagneticField].magnetic.y = values[1];
		    			mPendingEvents[MagneticField].magnetic.z = values[2];
				//	LOGD("x = %f, y = %f, z = %f", values[0],values[1],values[2]);
				}
		
				if(mOrEnabled)
				{
		    			get_orientation_angles(angles);
		    			mPendingEvents[Orientation].orientation.azimuth = angles[0];
		    			mPendingEvents[Orientation].orientation.pitch = angles[1];
		    			mPendingEvents[Orientation].orientation.roll = angles[2];
		    			mPendingMask |= 1<<Orientation;	
				//	LOGD("azimuth = %f, pitch = %f, roll = %f", angles[0],angles[1],angles[2]);
				}

            		}
			
			for (int j=0 ; count && mPendingMask && j<numSensors ; j++) {
				if (mPendingMask & (1<<j)) {
					mPendingMask &= ~(1<<j);
					mPendingEvents[j].timestamp = time;
					if (mEnabled & (1<<j)) {
						*data++ = mPendingEvents[j];
						count--;
						numEventReceived++;
					}
				}
			}
			if (!mPendingMask) {
				mInputReader.next();
			}
		} else {
			LOGE("ST480Sensor: unknown event (type=%d, code=%d)",type, event->code);
			mInputReader.next();
		}
	}

	return numEventReceived;
}
