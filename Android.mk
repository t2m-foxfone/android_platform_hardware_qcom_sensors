ifneq ($(filter msm8960 msm8610,$(TARGET_BOARD_PLATFORM)),)
ifneq ($(BUILD_TINY_ANDROID),true)
LOCAL_PATH := $(call my-dir)

# HAL module implemenation stored in
include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := libst480.a
include $(BUILD_MULTI_PREBUILT)
include $(CLEAR_VARS)

ifneq ($(filter msm8610,$(TARGET_BOARD_PLATFORM)),)
  LOCAL_MODULE := sensors.$(TARGET_BOARD_PLATFORM)
  LOCAL_CFLAGS := -DTARGET_8610
else
  LOCAL_MODULE := sensors.msm8930
endif

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS += -DLOG_TAG=\"Sensors\"
ifeq ($(call is-board-platform,msm8960),true)
  LOCAL_CFLAGS += -DTARGET_8930
endif

LOCAL_CFLAGS += -DBMA222E_SENSOR
LOCAL_SRC_FILES :=	\
		sensors.cpp 			\
		SensorBase.cpp			\
		LightSensor.cpp			\
		ProximitySensor.cpp		\
		Mpu3050.cpp				\
		Bmp180.cpp				\
		InputEventReader.cpp		\
		Accelerometer.cpp		\
		BmaSensor.cpp			\
		ST480Sensor.cpp



LOCAL_SHARED_LIBRARIES := liblog libcutils libdl
LOCAL_STATIC_LIBRARIES := libst480
include $(BUILD_SHARED_LIBRARY)

endif #BUILD_TINY_ANDROID
endif #TARGET_BOARD_PLATFORM
