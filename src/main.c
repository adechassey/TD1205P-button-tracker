/******************************************************************************
 * @file
 * @brief Simple GPS periodic fix application for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

#include <efm32.h>

#include <td_core.h>
#include <td_uart.h>
#include <td_printf.h>
#include <td_rtc.h>

#include <td_tools_switch.h>

#include <td_stream.h>
#include <td_flash.h>
#include <td_scheduler.h>
#include <td_watchdog.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_measure.h>
#include <td_sensor.h>
#include <sensor_data_geoloc.h>
#include <td_tools_led.h>

#include <td_accelero.h>
#include <td_geoloc.h>
#include <td_sigfox.h>

#include <td_config.h>

/*****************************************************************************
 *****************************     DEFINES    ********************************
 *****************************************************************************/

/** Flash variable version ID */
#define VARIABLES_VERSION 0

#define LED_PORT	gpioPortD			/**< LED port */
#define LED_BIT		5			    	/**< LED bit */

#define BUTTON_PORT	DB2_PORT			/**< Button port */
#define BUTTON_BIT	DB2_BIT				/**< Button bit */
#define BUTTON_MASK	DB2_MASK			/**< Button mask */

/** GPS */
#define TD_GEOLOC_USE_CODE 	1
#define FIX_MAX_HDOP 		500 			// Acceptable minimum horizontal accuracy, 800 to be very accurate
#define GPS_SLEEP_MODE 		TD_GEOLOC_OFF 		// Because time is long between 2 measures
#define GPS_PRECISION_TYPE 	TD_GEOLOC_2D_FIX

/** Stop trying to fix after timeout (in seconds) */
#define FIX_TIMEOUT 90

/** Boot monitoring, 1 to enable */
#define BOOT_MONITORING 0

/** Keepalive monitoring interval in hours, 0 to disable */
#define KEEPALIVE_INTERVAL 0

#define DEBUG 1 //trace on serial
#if DEBUG
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__);
#define BLINK_LED(...) TD_TOOLS_LED_StartBlink(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#define BLINK_LED(...)
#endif

bool encrypt = false;

static uint8_t SwitchId;

/******************************************************************************
 ******************************  GLOBAL FUNCTIONS  ****************************
 ******************************************************************************/

void encryption(uint8_t * bytes, uint8_t * cryptMessage, int size) {
	int temp = 42;
	int i = 0;

	for (i = 0; i < size; i++) {
		if (bytes[i] + temp < 0xFF) {
			cryptMessage[i] = bytes[i] + temp;
		} else {
			cryptMessage[i] = bytes[i] + temp - 255;
		}
		temp = bytes[i];
	}
}

/***************************************************************************//**
 * @brief
 *  GPS fix callback
 *
 * @param[in] fix
 *   The GPS fix data structure.
 *
 * @param[in] timeout
 *   Flag that indicates whether a timeout occurred if set to true.
 ******************************************************************************/
static void GPSFix(TD_GEOLOC_Fix_t *fix, bool timeout) {
	// Toggle the LED pin on/off
	GPIO_PinOutToggle(LED_PORT, LED_BIT);
	DEBUG_PRINTF("[START] GPSFix\r\n");
	int i;
	uint8_t bytes[12];
	uint8_t cryptMessage[12];
	uint8_t timeoutMessage[2];

	unsigned long latitude, longitude;
	char latitude_direction, longitude_direction;
	int hdop, nbSat, acqTime, speed, size;

	// Message init - Set to zero not to get random unwanted values
	for (i = 0; i < 12; i++) {
		bytes[i] = 0x00;
	}

	// Message init - Set to zero not to get random unwanted values
	for (i = 0; i < 2; i++) {
		timeoutMessage[i] = 0x00;
	}

	//	if (fix->type >= TD_GEOLOC_TIME_FIX && fix->hard.rtc_calibrated) {
	//		DEBUG_PRINTF("Time: %d %d %d\r\n", fix->datetime.hours,
	//				fix->datetime.minutes, fix->datetime.seconds);
	//		int hours = fix->datetime.hours;
	//		int minutes = fix->datetime.minutes;
	//		int seconds = fix->datetime.seconds;
	//
	//		//if(fix->type >= TD_GEOLOC_DATE_FIX && fix->hard.rtc_calibrated){
	//		//	DEBUG_PRINTF("Date: %d %d %d\r\n", fix->datetime.year, fix->datetime.month, fix->datetime.day);
	//		//}
	//	}

	if (fix->type >= GPS_PRECISION_TYPE && fix->hard.rtc_calibrated
			&& fix->quality.hdop <= FIX_MAX_HDOP) {
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);
		DEBUG_PRINTF("Voltage: %d\r\n", mv);
		DEBUG_PRINTF("Lat: %d - %08lX\r\n", fix->position.latitude,
				fix->position.latitude);
		DEBUG_PRINTF("Long: %d - %08lX\r\n", fix->position.longitude,
				fix->position.longitude);
		DEBUG_PRINTF("Hdop: %d \r\n", fix->quality.hdop);
		DEBUG_PRINTF("nb sat: %d \r\n", fix->quality.sat);
		DEBUG_PRINTF("acquisition time: %d \r\n", fix->duration);
		DEBUG_PRINTF("speed: %d \r\n", fix->speed.speed_kmh);

		// Latitude
		if (fix->position.latitude < 0) {
			latitude = (int32_t) (-1) * ((int32_t) fix->position.latitude / 10);
			latitude_direction = 'S';
			latitude = latitude | 0x80000000;
		} else {
			latitude = fix->position.latitude / 10;
			latitude_direction = 'N';
		}
		bytes[0] = (latitude >> 24) & 0xFF;
		bytes[1] = (latitude >> 16) & 0xFF;
		bytes[2] = (latitude >> 8) & 0xFF;
		bytes[3] = latitude & 0xFF;

		// Longitude
		if (fix->position.longitude < 0) {
			longitude = (int32_t) (-1)
					* ((int32_t) fix->position.longitude / 10);
			longitude_direction = 'W';
			longitude = longitude | 0x80000000;
		} else {
			longitude = fix->position.longitude / 10;
			longitude_direction = 'E';
		}
		bytes[4] = (longitude >> 24) & 0xFF;
		bytes[5] = (longitude >> 16) & 0xFF;
		bytes[6] = (longitude >> 8) & 0xFF;
		bytes[7] = longitude & 0xFF;

		// Hdop
		/*
		 * On the byte 8, we will encode both quality (hdop) and number of satellites
		 * 		  h h s s           (h for hdop and s for the number of satellites)
		 * Byte : _ _ _ _   _ _ _ _
		 */
		hdop = fix->quality.hdop / 100;
		if (hdop > 5) {
			//Set the 2 first bits to 3
			bytes[8] = 0xC0;
		} else if (hdop >= 2 && hdop <= 5) {
			//Set the 2 first bits to 2
			bytes[8] = 0x80;
		} else if (hdop >= 1 && hdop < 2) {
			//Set the 2 first bits to 1
			bytes[8] = 0x04;
		}

		// Satellites number
		nbSat = fix->quality.sat;
		if (nbSat >= 8) {
			bytes[8] = bytes[8] | 0x30;
		} else if (nbSat >= 6 && nbSat <= 8) {
			bytes[8] = bytes[8] | 0x20;
		} else if (nbSat >= 4 && nbSat <= 6) {
			bytes[8] = bytes[8] | 0x10;
		}

		// Acquisition time
		/*
		 * On the byte 9, we will encode both acquisition (a) time and the speed (s)
		 * 		  a a a a   s s s s
		 * Byte : _ _ _ _   _ _ _ _
		 */
		uint8_t mask;
		acqTime = fix->duration;
		for (i = 0; i < 15; i++) {
			if (acqTime >= i * 5 && acqTime <= (i + 1) * 5) {
				mask = i << 4;
				bytes[9] = bytes[9] | mask;
			} else if (acqTime > 75) {
				bytes[9] = bytes[9] | 0xF0;
			}
		}

		// Speed
		speed = fix->speed.speed_kmh;
		for (i = 0; i < 15; i++) {
			if (speed >= i * 5 && speed <= (i + 1) * 5) {
				bytes[9] = bytes[9] | i;
			} else if (speed > 75) {
				bytes[9] = bytes[9] | 0x0F;
			}
		}

		// Battery
		for (i = 0; i < 255; i++) {
			if (mv >= i * 15 && mv <= (i + 1) * 15) {
				bytes[10] = bytes[10] | i;
			} else if (mv > 3825) {
				bytes[10] = bytes[10] | 0xFF;
			}
		}

		// DEBUG
		size = sizeof(bytes) / sizeof(bytes[0]);
		for (i = 0; i < size; i++) {
			DEBUG_PRINTF("bytes[%d]: %x\r\n", i, bytes[i]);
		}
		//TD_GEOLOC_PrintfFix(fix);
		encryption(&bytes, &cryptMessage, size);
		for (i = 0; i < size; i++) {
			DEBUG_PRINTF("crypt[%d]: %x\r\n", i, cryptMessage[i]);
		}
		
		// Stop GPS
		TD_GEOLOC_StopFix(GPS_SLEEP_MODE);
		
		// Send Sigfox message
		TD_SIGFOX_Send(encrypt ? cryptMessage : bytes, size, 2);
		
		// GPIO_PinModeSet(gpioPortC, 0, gpioModeDisabled, 0);
		// Reactivate interrupt
		TD_TOOLS_SWITCH_Start(SwitchId);

	} else if (timeout) {
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);
		DEBUG_PRINTF("Voltage: %d\r\n", mv);
		for (i = 0; i < 255; i++) {
			if (mv >= i * 15 && mv <= (i + 1) * 15) {
				timeoutMessage[0] = timeoutMessage[0] | i;
			} else if (mv > 3825) {
				timeoutMessage[0] = timeoutMessage[0] | 0xFF;
			}
		}
		DEBUG_PRINTF("Fix Timeout\r\n");
		DEBUG_PRINTF("Time: %d \r\n", fix->duration);

		TD_GEOLOC_PrintfFix(fix);
		size = sizeof(timeoutMessage)/sizeof(timeoutMessage[0]);
		
		// Stop GPS
		TD_GEOLOC_StopFix(GPS_SLEEP_MODE);
		
		// Send Sigfox message
		TD_SIGFOX_Send((uint8_t *) timeoutMessage, size, 2);
		
		// GPIO_PinModeSet(gpioPortC, 0, gpioModeDisabled, 0);
		// Reactivate interrupt
		TD_TOOLS_SWITCH_Start(SwitchId);
	}

	DEBUG_PRINTF("[END] GPSFix\r\n");
	// Toggle the LED pin on/off
	GPIO_PinOutToggle(LED_PORT, LED_BIT);
}

/***************************************************************************//**
 * @brief
 *   Button interrupt handler.
 ******************************************************************************/
static void ButtonInterrupt(uint32_t mask) {
	DEBUG_PRINTF("Button pressed!\r\n");
	// Deactivate interrupt (will be reactivated when GPSFix finishes)
	TD_TOOLS_SWITCH_Stop(SwitchId);
	// Fix GPS and send payload
	TD_GEOLOC_TryToFix(TD_GEOLOC_NAVIGATION, FIX_TIMEOUT, GPSFix);
}

/***************************************************************************//**
 * @brief
 *  User Setup function.
 ******************************************************************************/
void TD_USER_Setup(void) {
	DEBUG_PRINTF("[START] TD_USER_Setup\r\n");
	TD_UART_Options_t options = { LEUART_DEVICE, LEUART_LOCATION, 9600, 8, 'N',
			1, false };
	// Open an I/O stream using LEUART0
	TD_UART_Open(&options, TD_STREAM_RDWR);

	// Set the device as a Sensor transmitter
	DEBUG_PRINTF("TD_SENSOR_Init...");
	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);
	DEBUG_PRINTF("OK\r\n");

	// Geoloc and accelerometer initialization
	DEBUG_PRINTF("TD_GEOLOC_Init...");
	TD_GEOLOC_Init();
	DEBUG_PRINTF("OK\r\n");

	// Define the LED pin as an output in push-pull mode
	GPIO_PinModeSet(LED_PORT, LED_BIT, gpioModePushPull, 1);
	// Toggle the LED pin on/off
	GPIO_PinOutToggle(LED_PORT, LED_BIT);

	// If using DB2/DB3
	GPIO_DbgSWDIOEnable(false);
	GPIO_DbgSWDClkEnable(false);

	// Push button initialization
	SwitchId = TD_TOOLS_SWITCH_Init(BUTTON_PORT, 1, gpioModeInputPull, true,
			true, 2, 5, 8);

	// Interrupt handling
	TD_TOOLS_SWITCH_Start(SwitchId);

	// Set flash variables version
	//	TD_FLASH_DeleteVariables();

	// Start fixing right now
	TD_GEOLOC_TryToFix(TD_GEOLOC_NAVIGATION, FIX_TIMEOUT, GPSFix);

#if BOOT_MONITORING

// Will only send a boot monitor frame on NEXT reboot
	TD_SENSOR_MonitorBoot(true, 0);
#endif

#if KEEPALIVE_INTERVAL > 0

// Send a keep-alive frame immediately, then at given interval
	TD_SENSOR_MonitorKeepAlive(true, KEEPALIVE_INTERVAL);
#endif

	// Use a 64 s automatic watchdog
	TD_WATCHDOG_Init(64);
	TD_WATCHDOG_Enable(true, true);

	// Toggle the LED pin on/off
	GPIO_PinOutToggle(LED_PORT, LED_BIT);

	DEBUG_PRINTF("[END] TD_USER_Setup\r\n");
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void) {

	// Process Geoloc events
	TD_GEOLOC_Process();

	// Print event
	switch (TD_TOOLS_SWITCH_Process(SwitchId)) {
	case TD_TOOLS_SWITCH_ON:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_ON\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_OFF:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_OFF\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_STARTUP_PUSHED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_STARTUP_PUSH\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_SHORT_PUSHED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_SHORT_PUSHED\r\n")
		;
		ButtonInterrupt(BUTTON_MASK);
		break;

	case TD_TOOLS_SWITCH_SHORT_RELEASED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_SHORT_RELEASED\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_LONG_PUSHED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_LONG_PUSHED\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_LONG_RELEASED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_LONG_RELEASED\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_VERY_LONG_PUSHED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_VERY_LONG_PUSHED\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_VERY_LONG_RELEASED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_VERY_LONG_RELEASED\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_EXTRA_LONG_PUSHED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_EXTRA_LONG_PUSHED\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_EXTRA_LONG_RELEASED:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_EXTRA_LONG_RELEASED\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_DOUBLE_CLICK:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_DOUBLE_CLICK\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_TRIPLE_CLICK:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_TRIPLE_CLICK\r\n")
		;
		break;

	case TD_TOOLS_SWITCH_MULTIPLE_CLICK_PENDING:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_MULTIPLE_CLICK_PENDING %d\r\n",
				TD_TOOLS_SWITCH_GetMultipleClickCount(SwitchId))
		;
		break;

	case TD_TOOLS_SWITCH_MULTIPLE_CLICK_DONE:
		DEBUG_PRINTF("TD_TOOLS_SWITCH_MULTIPLE_CLICK_DONE %d\r\n",
				TD_TOOLS_SWITCH_GetMultipleClickCount(SwitchId))
		;
		break;

	default:
		break;
	}
}
