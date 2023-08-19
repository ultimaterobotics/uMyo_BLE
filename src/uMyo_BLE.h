/*
  uMyo_BLE.h

  Copyright (c) 2022, Ultimate Robotics

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef UMYO_BLE_h
#define UMYO_BLE_h

#include <Arduino.h>

#include <ArduinoBLE.h>

#include "quat_math.h"
#define MAX_UMYO_DEVICES 12

typedef struct uMyo_data
{
	uint32_t id;
	int batt_mv;
	uint8_t last_data_id;
	int16_t cur_spectrum[4];
	uint16_t device_avg_muscle_level;
	uint32_t last_data_time;
	sQ Qsg;
	int operator=(uMyo_data d2)
	{
		id = d2.id;
		batt_mv = d2.batt_mv;
		last_data_id = d2.last_data_id;
		last_data_time = d2.last_data_time;
		device_avg_muscle_level = d2.device_avg_muscle_level;
		for(int x = 0; x < 4; x++) cur_spectrum[x] = d2.cur_spectrum[x];
		Qsg = d2.Qsg;
		return 1;
	}
}uMyo_data;

class uMyo_BLE_
{
private:
	uMyo_data devices[MAX_UMYO_DEVICES];
	sV nx, ny, nz;
	int device_count;
	uint8_t idToIdx(uint32_t id);
public:
	uMyo_BLE_(void);
	void begin();
	void run();
	uint8_t getDeviceCount();
	int getBattery(uint8_t devidx);
	uint32_t getID(uint8_t devidx);
	uint8_t getDataID(uint8_t devidx);
	float getMuscleLevel(uint8_t devidx);
	float getAverageMuscleLevel(uint8_t devidx);
	void getSpectrum(uint8_t devidx, float *spectrum);
	void getRawData(uint8_t devidx, int16_t *data);
	float getPitch(uint8_t devidx);
	float getRoll(uint8_t devidx);
	float getYaw(uint8_t devidx);
	uMyo_data *getDeviceByID(uint32_t id);
};
extern uMyo_BLE_ uMyo;

#endif
