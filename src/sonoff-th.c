/*

 Copyright (C) 2018  Carsten Duvenhorst

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <sysparam.h>

#include "http_client_ota.h"

#include <dht/dht.h>
#include <etstimer.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include "led_status.h"
#include "button.h"
#include "debug.h"

// Generate new settings (and a new qrcode.png) with "make homekitSettings"
#include "homekit_settings.h"

#define SONOFF_FW_VERSION "1.1"

#define DEVICE_NAME "Sonoff"
#define DEVICE_MODEL "TH16"
#define DEVICE_SERIAL "12345678"

#define vTaskDelayMs(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

// The GPIO pin that is connected to the relay on the Sonoff TH.
const int relayGPIO = 12;

// The GPIO pin that is connected to the LED on the Sonoff TH.
const int ledGPIO = 13;

// The GPIO pin that is oconnected to the button on the Sonoff TH.
const int buttonGPIO = 0;

// GPIO pin used by the optional AM2301 Sensor
const int dhtSensorGPIO = 14;

led_status_t *blueLedStatus;

const float dhtSensorTemperatureBias = -1.4;
const float dhtSensorHumidityBias = -2.0;

static ETSTimer sensorTimer;

float movingAverageTemperature = 0.0;
float movingAverageHumidity = 0.0;

// Accessory Information Service Characteristics

homekit_characteristic_t accessoryInformationNameCharacteristic = {
	HOMEKIT_DECLARE_CHARACTERISTIC_NAME(DEVICE_NAME)
};

homekit_characteristic_t serialNumberCharacteristic = {
	HOMEKIT_DECLARE_CHARACTERISTIC_SERIAL_NUMBER(DEVICE_SERIAL)
};

homekit_characteristic_t characteristicIdentify = {
	HOMEKIT_DECLARE_CHARACTERISTIC_IDENTIFY(NULL)
};

/*
Required Characteristics for accessories:
 - NAME
 - MANUFACTURER
 - MODEL
 - SERIAL_NUMBER
 - FIRMWARE_REVISION
 - IDENTIFY
*/
homekit_characteristic_t *deviceCharacterisrics[] = {
	&accessoryInformationNameCharacteristic,
	HOMEKIT_CHARACTERISTIC(MANUFACTURER, "itead.cc"),
	HOMEKIT_CHARACTERISTIC(MODEL, DEVICE_MODEL),
	&serialNumberCharacteristic,
	HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, SONOFF_FW_VERSION),
	&characteristicIdentify,
	NULL
};

// Accessory Information Service
homekit_service_t accessoryInformationService = {
	.type = HOMEKIT_SERVICE_ACCESSORY_INFORMATION,
	.characteristics = deviceCharacterisrics
};

/*
Switch service
Switch Service Characteristics
*/
homekit_characteristic_t switchCharacteristicName =
    { HOMEKIT_DECLARE_CHARACTERISTIC_NAME(DEVICE_NAME) };
homekit_characteristic_t switchCharacteristicOn =
    { HOMEKIT_DECLARE_CHARACTERISTIC_ON(false) };
homekit_characteristic_t *switchServiceCharacterisrics[] = {
	&switchCharacteristicName,
	&switchCharacteristicOn,
	NULL
};

// Switch service service
homekit_service_t switchService = {
	.type = HOMEKIT_SERVICE_SWITCH,
	.primary = true,
	.characteristics = switchServiceCharacterisrics
};

// MARK: Temperature Sensor
/*
Temperature Sensor Service Characteristics
Required: CURRENT_TEMPERATURE characteristic
*/
homekit_characteristic_t temperatureSensorCharacteristicCurrentTemperature =
    { HOMEKIT_DECLARE_CHARACTERISTIC_CURRENT_TEMPERATURE(0.0) };

/*
Optional characteristic: Name
*/
homekit_characteristic_t temperatureSensorCharacteristicName =
    { HOMEKIT_DECLARE_CHARACTERISTIC_NAME("Temperature") };

// Optional characteristic: STATUS_ACTIVE
homekit_characteristic_t temperatureSensorCharacteristicStatusActive =
    { HOMEKIT_DECLARE_CHARACTERISTIC_STATUS_ACTIVE(false) };

// Array of characteristics for the temperature sensor service
homekit_characteristic_t *temperatureSensorServiceCharacterisrics[] = {
	&temperatureSensorCharacteristicCurrentTemperature,
	&temperatureSensorCharacteristicName,
	&temperatureSensorCharacteristicStatusActive,
	NULL
};

// Temperature Sensor Service
homekit_service_t temperatureSensorService = {
	.type = HOMEKIT_SERVICE_TEMPERATURE_SENSOR,
	.characteristics = temperatureSensorServiceCharacterisrics
};

// MARK: Humidity Sensor
//

// humidity sensor service: Required characteristic
homekit_characteristic_t humiditySensorCharacteristicCurrentRelativeHumidity =
    { HOMEKIT_DECLARE_CHARACTERISTIC_CURRENT_RELATIVE_HUMIDITY(0.0) };

// optional characteristic: Name
homekit_characteristic_t humiditySensorCharacteristicName =
    { HOMEKIT_DECLARE_CHARACTERISTIC_NAME("Humidity") };

// Optional characteristic: STATUS_ACTIVE
homekit_characteristic_t humiditySensorCharacteristicStatusActive =
    { HOMEKIT_DECLARE_CHARACTERISTIC_STATUS_ACTIVE(false) };

// Array of characteristics for the humidity sensor service
homekit_characteristic_t *humiditySensorServiceCharacterisrics[] = {
	&humiditySensorCharacteristicCurrentRelativeHumidity,
	&humiditySensorCharacteristicName,
	&humiditySensorCharacteristicStatusActive,
	NULL
};

// humidity sensor service
homekit_service_t humiditySensorService = {
	.type = HOMEKIT_SERVICE_HUMIDITY_SENSOR,
	.characteristics = humiditySensorServiceCharacterisrics
};

//
// Accessory with services

// Services of the accessory with sensor services
homekit_service_t *accessoryServicesWithSensor[] = {
	&accessoryInformationService,
	&switchService,
	&temperatureSensorService,
	&humiditySensorService,
	NULL
};

// Services of the accessory without sensor services
homekit_service_t *accessoryServices[] = {
	&accessoryInformationService,
	&switchService,
	NULL
};

// Accessory definition
homekit_accessory_t accessorySonoffTH = {
	.id = 1,
	.category = homekit_accessory_category_switch,
	.config_number = 1,
	.services = accessoryServices
};

// Array of managed accessories
homekit_accessory_t *accessories[] = {
	&accessorySonoffTH,
	NULL
};

// MARK: Homekit server configuration
homekit_server_config_t config = {
	.accessories = accessories,
	.setupCode = SETUP_CODE,
	.setupIdentifier = SETUP_ID
};

void relay_write(bool on)
{
	DEBUG("Switching relay %d", on);
	gpio_write(relayGPIO, on ? 1 : 0);
}

void led_write(bool on)
{
	gpio_write(ledGPIO, on ? 0 : 1);
}

// set the startOTA system start parameter to true and restart.
void prepare_ota_update_task()
{
	sdk_os_timer_disarm(&sensorTimer);

	sysparam_set_bool("startOTA", true);

	sdk_system_restart();
	vTaskDelete(NULL);
}

void prepare_ota_update()
{
	DEBUG("Preparing OTA-Update");
	xTaskCreate(prepare_ota_update_task, "OTA Update", 256, NULL, 2, NULL);
}

void
switch_on_callback(homekit_characteristic_t * _ch, homekit_value_t on,
		   void *context)
{
	relay_write(on.bool_value);
}

void button_callback(uint8_t gpio, button_event_t event)
{
	switch (event) {

	case button_event_single_press:
		DEBUG("Toggling relay");

		switchCharacteristicOn.value.bool_value =
		    !switchCharacteristicOn.value.bool_value;

		relay_write(switchCharacteristicOn.value.bool_value);

		homekit_characteristic_notify(&switchCharacteristicOn,
					      switchCharacteristicOn.value);
		break;

	case button_event_long_press:
		prepare_ota_update();
		break;

	default:
		DEBUG("Unknown button event: %d", event);
	}

}

void ota_button_callback(uint8_t gpio, button_event_t event)
{

	static bool on = false;

	switch (event) {

	case button_event_single_press:
		DEBUG("Toggling relay");
		on = !on;
		relay_write(on);
		break;

	case button_event_long_press:
		sdk_system_restart();
		break;

	default:
		DEBUG("Unknown button event: %d", event);
	}

}

void gpio_init()
{

	gpio_enable(ledGPIO, GPIO_OUTPUT);
	led_write(false);

	gpio_enable(relayGPIO, GPIO_OUTPUT);
	relay_write(switchCharacteristicOn.value.bool_value);

}

bool sensor_init()
{

	gpio_set_pullup(dhtSensorGPIO, false, false);

	if (dht_read_float_data
	    (DHT_TYPE_DHT22, dhtSensorGPIO, &movingAverageHumidity,
	     &movingAverageTemperature)) {

		movingAverageHumidity += dhtSensorHumidityBias;
		movingAverageTemperature += dhtSensorTemperatureBias;

		return true;

	}

	return false;
}

void identify_task(void *_args)
{

	blueLedStatus = led_status_init(ledGPIO);
	led_status_set(blueLedStatus, &identifyPattern);

	vTaskDelayMs(5000);

	led_status_done(blueLedStatus);

	vTaskDelete(NULL);

}

void identify(homekit_value_t _value)
{

	DEBUG("Switch identify");
	xTaskCreate(identify_task, "Switch identify", 128, NULL, 2, NULL);

}

void dhtMeasurementTask(void *pvParameters)
{
	DEBUG("Measuring.");

	float temperature, humidity;
	float smoothedTemperature, smoothedHumidity;

	float eta = 0.90;

	if (dht_read_float_data
	    (DHT_TYPE_DHT22, dhtSensorGPIO, &humidity, &temperature)) {

		temperature += dhtSensorTemperatureBias;
		humidity += dhtSensorHumidityBias;

		smoothedTemperature =
		    (eta * temperature) +
		    ((1 - eta) * movingAverageTemperature);
		smoothedHumidity =
		    (eta * humidity) + ((1 - eta) * movingAverageHumidity);

		if (smoothedTemperature != movingAverageTemperature) {
			movingAverageTemperature = smoothedTemperature;

			temperatureSensorCharacteristicCurrentTemperature.value
			    = HOMEKIT_FLOAT(movingAverageTemperature);

			homekit_characteristic_notify
			    (&temperatureSensorCharacteristicCurrentTemperature,
			     temperatureSensorCharacteristicCurrentTemperature.
			     value);
		}

		if (smoothedHumidity != movingAverageHumidity) {
			movingAverageHumidity = smoothedHumidity;

			humiditySensorCharacteristicCurrentRelativeHumidity.
			    value = HOMEKIT_FLOAT(movingAverageHumidity);

			homekit_characteristic_notify
			    (&humiditySensorCharacteristicCurrentRelativeHumidity,
			     humiditySensorCharacteristicCurrentRelativeHumidity.
			     value);
		}

	} else {

		DEBUG("DHT22: Sensor Error.");

		temperatureSensorCharacteristicStatusActive.value =
		    HOMEKIT_BOOL(false);
		homekit_characteristic_notify
		    (&temperatureSensorCharacteristicStatusActive,
		     temperatureSensorCharacteristicStatusActive.value);

		blueLedStatus = led_status_init(ledGPIO);
		led_status_set(blueLedStatus, &sensorErrorPattern);
		vTaskDelayMs(5000);
		led_status_done(blueLedStatus);
		blueLedStatus = NULL;
	}

}

void createHAPAccessory(bool dhtSensorAvailable)
{

	uint8_t macaddr[6];
	sdk_wifi_get_macaddr(STATION_IF, macaddr);

	int name_len = snprintf(NULL, 0, "%s-%02X%02X%02X",
				DEVICE_NAME,
				macaddr[3],
				macaddr[4],
				macaddr[5]);

	if (name_len > 63) {
		name_len = 63;
	}

	char *name_value = malloc(name_len + 1);

	snprintf(name_value, name_len + 1, "%s-%02X%02X%02X",
		 DEVICE_NAME, macaddr[3], macaddr[4], macaddr[5]);

	accessoryInformationNameCharacteristic.value =
	    HOMEKIT_STRING(name_value);
	switchCharacteristicName.value = HOMEKIT_STRING(name_value);

	int serialLength = snprintf(NULL, 0, "%d", sdk_system_get_chip_id());

	char *serialNumberValue = malloc(serialLength + 1);
	snprintf(serialNumberValue, serialLength + 1, "%d",
		 sdk_system_get_chip_id());

	serialNumberCharacteristic.value = HOMEKIT_STRING(serialNumberValue);

	characteristicIdentify.setter = identify;
	homekit_characteristic_add_notify_callback(&switchCharacteristicOn,
						   switch_on_callback, NULL);

	if (dhtSensorAvailable) {
		accessorySonoffTH.services = accessoryServicesWithSensor;
	}

}

void setupAccessory()
{

	bool dhtSensorAvailable = sensor_init();
	createHAPAccessory(dhtSensorAvailable);

	homekit_server_init(&config);

	if (dhtSensorAvailable) {

		sdk_os_timer_setfn(&sensorTimer, dhtMeasurementTask, NULL);
		sdk_os_timer_arm(&sensorTimer, 60000, 1);

		humiditySensorCharacteristicCurrentRelativeHumidity.value =
		    HOMEKIT_FLOAT(movingAverageHumidity);
		temperatureSensorCharacteristicCurrentTemperature.value =
		    HOMEKIT_FLOAT(movingAverageTemperature);

		homekit_characteristic_notify
		    (&humiditySensorCharacteristicCurrentRelativeHumidity,
		     humiditySensorCharacteristicCurrentRelativeHumidity.value);

		homekit_characteristic_notify
		    (&temperatureSensorCharacteristicCurrentTemperature,
		     temperatureSensorCharacteristicCurrentTemperature.value);
	}

}

static ota_info ota_update_info = {
	.server = OTA_UPDATE_SERVER,
	.port = OTA_UPDATE_PORT,
	.path = OTA_UPDATE_PATH,
	.basename = OTA_UPDATE_FIRMWARE_NAME,
	.checkSHA256 = true,
};

static void firmwareDownloadTask(void *PvParameter)
{

	blueLedStatus = led_status_init(ledGPIO);
	led_status_set(blueLedStatus, &otaUpdatePattern);

	while (1) {
		OTA_err err;
		// Remake this task until ota work
		err = ota_update((ota_info *) PvParameter);

		if (err != OTA_UPDATE_DONE) {
			vTaskDelayMs(1000);
			continue;
		}

		vTaskDelayMs(3000);
		led_status_done(blueLedStatus);
		DEBUG("Booting updated firmware.\n");
		sdk_system_restart();
	}
}

void wifiReadyCallback()
{

	led_status_done(blueLedStatus);

	bool ota = false;

	sysparam_status_t status = sysparam_get_bool("startOTA", &ota);
	if (status == SYSPARAM_OK) {
		if (ota) {
			sysparam_set_bool("startOTA", false);

			button_delete(buttonGPIO);

			if (button_create
			    (buttonGPIO, 0, 4000, ota_button_callback)) {
				DEBUG("Failed to initialize button.");
			}

			DEBUG("Looking for updates.\n");
			xTaskCreate(firmwareDownloadTask, "download_task", 4096,
				    &ota_update_info, 2, NULL);
			return;
		}
	}

	setupAccessory();

}

void user_init(void)
{

	uart_set_baud(0, 115200);

	gpio_init();

	blueLedStatus = led_status_init(ledGPIO);
	led_status_set(blueLedStatus, &waitingWifiPattern);

	wifi_config_init("sonoff-th16", NULL, wifiReadyCallback);

	if (button_create(buttonGPIO, 0, 4000, button_callback)) {
		DEBUG("Failed to initialize button.");
	}

}
