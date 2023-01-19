/*
  uMyo_BLE.cpp

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
#include "uMyo_BLE.h"

int uMyo_BLE_::scanTime = 5;
BLEScan* uMyo_BLE_::pBLEScan = NULL;

class UMyoAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
	void onResult(BLEAdvertisedDevice advertisedDevice)
	{
		uint8_t *pack = advertisedDevice.getPayload();
		int len = advertisedDevice.getPayloadLength();
		int pp = 0;
		while(pp < len-5)
		{
			pp++;
			if(pack[pp] != 8) continue;
			if(pack[pp+1] != 'u') continue;
			if(pack[pp+2] != 'M') continue;
			if(pack[pp+3] != 'y') continue;
			if(pack[pp+4] != 'o') continue;
			while(pack[pp] != 255 && pp < len) pp++;
			if(pp >= len) return;
			pp++;

			int adc_id = pack[pp++];
			int batt_level = pack[pp++];
			int16_t sp0 = pack[pp++]<<8; //component 0 is sent at reduced precision
			int muscle_avg = pack[pp++];
			int16_t sp1 = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t sp2 = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t sp3 = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t qw = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t qx = (pack[pp++]<<8);
			int16_t qy = (pack[pp++]<<8);
			int16_t qz = (pack[pp++]<<8);
			sQ Qsg;
			Qsg.w = qw;
			Qsg.x = qx;
			Qsg.y = qy;
			Qsg.z = qz;
			q_renorm(&Qsg);

			uint8_t *mac = *advertisedDevice.getAddress().getNative();
			uint32_t unit_id = (mac[1]<<24) | (mac[2]<<16) | (mac[3]<<8) | mac[4];
			uMyo_data *device = uMyo.getDeviceByID(unit_id);
			device->last_data_id = adc_id;
			device->batt_mv = 2000 + batt_level*10;
			device->device_avg_muscle_level = (muscle_avg*muscle_avg)>>3;
			device->cur_spectrum[0] = sp0;
			device->cur_spectrum[1] = sp1;
			device->cur_spectrum[2] = sp2;
			device->cur_spectrum[3] = sp3;
			device->Qsg = Qsg;

		}
		return;
	}
};

//'void (* (*)(esp_gap_ble_cb_event_t, esp_ble_gap_cb_param_t*))(esp_gap_ble_cb_event_t, esp_ble_gap_cb_param_t*)' to 'esp_gap_ble_cb_t' {aka 'void (*)(esp_gap_ble_cb_event_t, esp_ble_gap_cb_param_t*)'} [-fpermissive]

void scan_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	Serial.println("evt");
	if(event == ESP_GAP_BLE_SCAN_RESULT_EVT)
	{
		Serial.println("scan res");
		esp_ble_gap_cb_param_t::ble_scan_result_evt_param *par = (esp_ble_gap_cb_param_t::ble_scan_result_evt_param *)param;
		uint8_t *pack = par->ble_adv;
		int len = par->adv_data_len;
		int pp = 0;
		while(pp < len-5)
		{
			pp++;
			if(pack[pp] != 8) continue;
			if(pack[pp+1] != 'u') continue;
			if(pack[pp+2] != 'M') continue;
			if(pack[pp+3] != 'y') continue;
			if(pack[pp+4] != 'o') continue;
			while(pack[pp] != 255 && pp < len) pp++;
			if(pp >= len) return;
			pp++;

			int adc_id = pack[pp++];
			int batt_level = pack[pp++];
			int16_t sp0 = pack[pp++]<<8; //component 0 is sent at reduced precision
			int muscle_avg = pack[pp++];
			int16_t sp1 = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t sp2 = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t sp3 = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t qw = (pack[pp]<<8) | pack[pp+1]; pp += 2;
			int16_t qx = (pack[pp++]<<8);
			int16_t qy = (pack[pp++]<<8);
			int16_t qz = (pack[pp++]<<8);
			sQ Qsg;
			Qsg.w = qw;
			Qsg.x = qx;
			Qsg.y = qy;
			Qsg.z = qz;
			q_renorm(&Qsg);

			uint8_t *mac = par->bda;
			uint32_t unit_id = (mac[1]<<24) | (mac[2]<<16) | (mac[3]<<8) | mac[4];
			uMyo_data *device = uMyo.getDeviceByID(unit_id);
			device->last_data_id = adc_id;
			device->batt_mv = 2000 + batt_level*10;
			device->device_avg_muscle_level = (muscle_avg*muscle_avg)>>3;
			device->cur_spectrum[0] = sp0;
			device->cur_spectrum[1] = sp1;
			device->cur_spectrum[2] = sp2;
			device->cur_spectrum[3] = sp3;
			device->Qsg = Qsg;

		}
		return;

	}
}

void uMyo_BLE_::rescan(BLEScanResults res)
{
	pBLEScan->clearResults();
	if(scanTime > 0) pBLEScan->start(scanTime, uMyo_BLE_::rescan, true);
}

uMyo_BLE_::uMyo_BLE_()
{
	nx.x = 1; ny.x = 0; nz.x = 0; //for angles calculation
	nx.y = 0; ny.y = 1; nz.y = 0;
	nx.z = 0; ny.z = 0; nz.z = 1;
	device_count = 0;
}
void uMyo_BLE_::begin()
{
/*	scanTime = 85; //seconds
	memset(&m_scan_params, 0, sizeof(m_scan_params)); // Initialize all params
	m_scan_params.scan_type          = BLE_SCAN_TYPE_ACTIVE; // Default is a passive scan.
	m_scan_params.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
	m_scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
	m_scan_params.scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE;
	m_scan_params.scan_interval = 16; //10 ms
	m_scan_params.scan_window = 16;
	esp_ble_gap_set_scan_params(&m_scan_params);
	esp_ble_gap_register_callback(&scan_cb);
	esp_ble_gap_start_scanning(scanTime);
	return;*/

	scanTime = 1; //seconds
	BLEDevice::init("");
	pBLEScan = BLEDevice::getScan(); //create new scan
	pBLEScan->setAdvertisedDeviceCallbacks(new UMyoAdvertisedDeviceCallbacks());
	pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
	pBLEScan->setInterval(32);
	pBLEScan->setWindow(31); // less or equal setInterval value

	pBLEScan->start(scanTime, this->rescan, false);
}
void uMyo_BLE_::pause()
{
	scanTime = 0; //stops rescan
}
void uMyo_BLE_::resume()
{
	scanTime = 5;
	pBLEScan->start(scanTime, this->rescan, true);
}

uint8_t uMyo_BLE_::idToIdx(uint32_t id)
{
	uint32_t ms = millis();
	for(uint8_t u = 0; u < device_count; u++)
	{
		if(ms - devices[u].last_data_time > 5000)
		{
			for(int u1 = u+1; u1 < device_count; u1++)
				devices[u1-1] = devices[u1];
			device_count--;
		}
	}
	for(uint8_t u = 0; u < device_count; u++)
		if(id == devices[u].id)
		{
			devices[u].last_data_time = ms;
			return u;
		}
	if(device_count < MAX_UMYO_DEVICES)
	{
		uint8_t u = device_count;
		devices[u].id = id;
		devices[u].last_data_time = ms;
		device_count++;
	}
	return device_count-1;
}
uMyo_data *uMyo_BLE_::getDeviceByID(uint32_t id)
{
	return devices + idToIdx(id);
}
uint8_t uMyo_BLE_::getDeviceCount()
{
	return device_count;
}
int uMyo_BLE_::getBattery(uint8_t devidx)
{
	if(devidx < device_count) return devices[devidx].batt_mv;
	return 0;
}
uint32_t uMyo_BLE_::getID(uint8_t devidx)
{
	if(devidx < device_count) return devices[devidx].id;
	return 0;
}
uint8_t uMyo_BLE_::getDataID(uint8_t devidx)
{
	if(devidx < device_count) return devices[devidx].last_data_id;
	return 0;
}
float uMyo_BLE_::getMuscleLevel(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	float lvl = devices[devidx].cur_spectrum[2] + 2*devices[devidx].cur_spectrum[3];
	return lvl;
}
float uMyo_BLE_::getAverageMuscleLevel(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	return devices[devidx].device_avg_muscle_level;
}
void uMyo_BLE_::getSpectrum(uint8_t devidx, float *spectrum)
{
	if(devidx >= device_count) return;
	for(uint8_t x = 0; x < 4; x++) spectrum[x] = devices[devidx].cur_spectrum[x];
}
float uMyo_BLE_::getPitch(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	sV nzg = nz;
	sQ Qgs;
	Qgs.w = devices[devidx].Qsg.w;
	Qgs.x = -devices[devidx].Qsg.x;
	Qgs.y = -devices[devidx].Qsg.y;
	Qgs.z = -devices[devidx].Qsg.z;
	rotate_v(&Qgs, &nzg);
	return acos_f(v_dot(&nzg, &ny));
}
float uMyo_BLE_::getRoll(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	sV nzg = nz;
	sQ Qgs;
	Qgs.w = devices[devidx].Qsg.w;
	Qgs.x = -devices[devidx].Qsg.x;
	Qgs.y = -devices[devidx].Qsg.y;
	Qgs.z = -devices[devidx].Qsg.z;
	rotate_v(&Qgs, &nzg);
	return atan2_f(nzg.z, nzg.x);
}
float uMyo_BLE_::getYaw(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	sV nyr = ny;
	rotate_v(&devices[devidx].Qsg, &nyr);
	return atan2_f(nyr.y, nyr.x);
}
uMyo_BLE_ uMyo;


