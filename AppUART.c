/*
 * AppUART.c
 *
 *  Created on: Feb 11, 2019
 *      Author: ikhwani
 */

#include "XdkAppInfo.h"

#undef BCDS_MODULE_ID
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_APP_UART

#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "AppController.h"
#include "AppUART.h"
#include "BSP_ExtensionPort_2.h"
#include "BCDS_UARTTransceiver.h"
#include "BCDS_MCU_UART.h"

#define MAX_UART_RX_BUFFERSIZE  UINT32_C(1024)
#define UART_RX_ERROR           UINT32_C(0xFFFF)
#define UART_RX_SUCCESS         UINT32_C(0)

double GpsLatitude = 0;
double GpsLongitude = 0;
time_t GpsTime = 0;

static uint8_t UartRingBuffer[MAX_UART_RX_BUFFERSIZE];
static UARTTransceiver_T UartTransceiverInstance;

static uint8_t Uart2RingBuffer[MAX_UART_RX_BUFFERSIZE];
static UARTTransceiver_T Uart2TransceiverInstance;

//static uint8_t TxBuffer[MAX_UART_RX_BUFFERSIZE];
static uint8_t LineBuffer[512];
static uint32_t LineBufferIdx = 0;
static uint8_t PrevRxCmd = 0xFF;
static uint8_t Buffer[256];
static uint8_t BufferIdx = 0;
static uint8_t PrevRxCommand = 0xFF;
static uint8_t PitchBuffer[20], RollBuffer[20], HeadingBuffer[20];
static uint8_t RawBuffer[BUFFSIZE];
static uint8_t RawBufferLength = 0;

static const char HEX[] = "0123456789ABCDEF";

static int GpsChecksumOk(const char *gpsMessage) {
	int8_t max = 90; // NMEA says 82, but there could have longer proprietary messages
	if (*gpsMessage != '$')
		return 0;
	char v = 0;
	for (;;) {
		if (--max <= 0)
			return false; // Protect from run away if no *
		char b = *++gpsMessage;
		if (b == '*')
			break;
		v ^= b;
	}
	unsigned char digit0 = HEX[(v & 0xf0) >> 4];
	unsigned char digit1 = HEX[(v & 0x0f)];
	if (gpsMessage[1] != digit0 || gpsMessage[2] != digit1) {
		return 0;
	}
	return 1;
}

//SEMENTARA EXTERN UNTUK DEBUG PROGRAM TANPA GPS
void SetInitEpoch(struct tm* datetime, char* date_str, char* time_str) {
	char dom[2], month[2], year[2];
	char hour[2], min[2], sec[2];

	strncpy(hour, time_str, 2);
	strncpy(min, time_str + 2, 2);
	strncpy(sec, time_str + 4, 2);

	strncpy(dom, date_str, 2);
	strncpy(month, date_str + 2, 2);
	strncpy(year, date_str + 4, 2);

	datetime->tm_year = 2000 + atoi(year) - 1900;
	datetime->tm_mon = atoi(month) - 1;
	datetime->tm_mday = atoi(dom);
	datetime->tm_hour = atoi(hour);
	datetime->tm_min = atoi(min);
	datetime->tm_sec = atoi(sec);

	if (StartTime == 0 && datetime->tm_year > 100 && datetime->tm_mon > 0
			&& datetime->tm_mday > 0) {
		time_t currentTime = mktime(datetime);
		TickType_t currentTicks = xTaskGetTickCount();
		StartTime = currentTime - (currentTicks / 1000);
		printf("Start Time initialized: %lu\n", StartTime);
		// printf("%02d-%02d-%04d %02d:%02d:%02d UTC\n",
		// datetime->tm_mday, datetime->tm_mon, 1900 + datetime->tm_year,
		// datetime->tm_hour, datetime->tm_min, datetime->tm_sec);
	}
}

static double NmeaPositionParse(char* pos, char* cardinal) {
	char* cursor;
	int degrees;
	double minutes;

	if (NULL == (cursor = strchr(pos, '.'))) {
		return 0;
	}

	cursor -= 2;
	minutes = atof(cursor);
	*cursor = '\0';

	cursor = pos;
	degrees = atoi(cursor);
	// printf("%i %.5f\n", degrees, minutes);

	double coord = degrees + (minutes / 60);
	if (strcmp(cardinal, "S") == 0 || strcmp(cardinal, "W") == 0) {
		coord = coord * -1;
	}
	return coord;
}

static void SetGPSData(struct tm* datetime, char* lat, char* latNS, char* lng,
		char* lngWE) {
	if (datetime->tm_year > 100 && datetime->tm_mon > 0
			&& datetime->tm_mday > 0) {
		time_t prevTime = GpsTime;
		time_t currentTime = mktime(datetime);
		time_t timeDiff = currentTime - prevTime;

		GpsLatitude = NmeaPositionParse(lat, latNS);
		GpsLongitude = NmeaPositionParse(lng, lngWE);

		if (prevTime == 0 || timeDiff > 30) {
			GpsTime = currentTime;
			printf("Save GPS data: %.7f,%.7f at %lu\n", GpsLatitude,
					GpsLongitude, GpsTime);
		}

//		printf("%02d-%02d-%04d %02d:%02d:%02d UTC, %.7f,%.7f\n",
//			datetime->tm_mday, datetime->tm_mon, 1900 + datetime->tm_year,
//			datetime->tm_hour, datetime->tm_min, datetime->tm_sec,
//			gLatitude, gLongitude);
	}
}

/* --------------------------------------- */

static Retcode_T UartDataWrite(uint8_t *writeBuffer, uint8_t writeLength) {
	Retcode_T retVal = RETCODE_OK;
	uint32_t writeTimeout = UINT32_MAX;
	if (NULL == writeBuffer) {
		retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
	} else {
		retVal = UARTTransceiver_WriteData(&UartTransceiverInstance,
				writeBuffer, writeLength, writeTimeout);
	}
	return retVal;
}
static Retcode_T UartDataRead(UARTTransceiver_T *transceiver,
		uint8_t *readBuffer, uint8_t readlength, uint32_t *actualLength) {
	Retcode_T retVal = RETCODE_OK;
	uint32_t readTimeout = UINT32_MAX;
	if (NULL == readBuffer) {
		retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
	} else {
		retVal = UARTTransceiver_ReadData(transceiver, readBuffer, readlength,
				actualLength, readTimeout);
	}
	return retVal;
}

static void ProcessGXRMC(char *buffer, struct tm *datetime) {
	const char s[4] = ",";

	printf("%s\n", LineBuffer);
	if (GpsChecksumOk(buffer) == 1) {
		memset(datetime, 0, sizeof(struct tm));

		strtok(buffer, s);
		char *time_str = strtok(NULL, s);

		char *status = strtok(NULL, s);
		if (strcmp(status, "A") == 0) {
			char *lat = strtok(NULL, s);
			char *latNS = strtok(NULL, s);
			char *lng = strtok(NULL, s);
			char *lngWE = strtok(NULL, s);
			strtok(NULL, s); // speed
			char *angle = strtok(NULL, s);
			char *date_str;
			if (strstr(angle, ".") != NULL) {
				date_str = strtok(NULL, s);
			} else {
				date_str = angle;
				angle = "";
			}
			// printf("speed: %s, angle: %s\n", speed, angle);
			SetInitEpoch(datetime, date_str, time_str);
			SetGPSData(datetime, lat, latNS, lng, lngWE);

		} else {
			char *date_str = strtok(NULL, s);
			SetInitEpoch(datetime, date_str, time_str);
			printf("%02d-%02d-%04d %02d:%02d:%02d UTC (no GPS data)\n",
					datetime->tm_mday, datetime->tm_mon + 1,
					1900 + datetime->tm_year, datetime->tm_hour,
					datetime->tm_min, datetime->tm_sec);
		}
	} else {
		// printf("Invalid %s\n", LineBuffer);
	}
}

static void ProcessRxXDKData(void *param1, uint32_t param2) {

	BCDS_UNUSED(param1);

	UARTTransceiver_T *transceiver;
	uint8_t *ringBuffer;
	uint8_t ii, bb;

	transceiver = &UartTransceiverInstance;
	ringBuffer = UartRingBuffer;

	Retcode_T retVal = RETCODE_OK;
	uint8_t RxCommand = 0xff;
	uint8_t RxCommandIndex = 0xFF;

	uint32_t actualLength = 0;
	uint8_t readlength = sizeof(readlength) / sizeof(uint8_t);
	uint8_t xx = 0;

	uint8_t Tx_datasend[] = "Send";
	uint8_t Tx_dataUnsend[] = "nSend";

	if (gpiolevel == 0)
	{
		UartDataWrite(Tx_datasend, strlen(Tx_datasend));
	}
	else if (gpiolevel == 1)
	{
		UartDataWrite(Tx_dataUnsend, strlen(Tx_dataUnsend));
	}


	if (UART_RX_ERROR != param2) {
		retVal = UartDataRead(transceiver, ringBuffer, readlength,
				&actualLength);
		if ((RETCODE_OK == retVal) && (readlength == actualLength)) {
			RxCommandIndex = readlength - actualLength;
			RxCommand = ringBuffer[RxCommandIndex];

			Buffer[BufferIdx] = RxCommand;

			if (PrevRxCommand == '~' && RxCommand == '#') {
//				printf("data string : %s \n", Buffer);

				for (ii = 0; ii < BufferIdx; ii++) {
					if (Buffer[ii] == '#') {
						xx++;
						if (xx == 1) {
							for (bb = 0; bb < 20; bb++) {
								if (Buffer[ii + bb + 1] == '#'
										|| Buffer[ii + bb + 1] == '~') {
									PitchBuffer[bb] = 0;
									break;
								}
								PitchBuffer[bb] = Buffer[ii + bb + 1];
							}

						} else if (xx == 2) {
							for (bb = 0; bb < 20; bb++) {
								if (Buffer[ii + bb + 1] == '#'
										|| Buffer[ii + bb + 1] == '~') {
									RollBuffer[bb] = 0;
									break;
								}
								RollBuffer[bb] = Buffer[ii + bb + 1];
							}

						} else if (xx == 3) {
							for (bb = 0; bb < 20; bb++) {
								if (Buffer[ii + bb + 1] == '#'
										|| Buffer[ii + bb + 1] == '~') {
									HeadingBuffer[bb] = 0;
									break;
								}
								HeadingBuffer[bb] = Buffer[ii + bb + 1];
							}
						}
						if (Buffer[ii] == '~') {
							if (Buffer[ii + 1] == '#')
								break;
						}
					}
				}

				OrientationValues_LA.pitch = atof(PitchBuffer);
				OrientationValues_LA.roll = atof(RollBuffer);
				OrientationValues_LA.heading = atof(HeadingBuffer);

//				printf("data konversi= pitch %f roll %f heading %f \r\n", OrientationValues_LA.pitch, OrientationValues_LA.roll,
//						OrientationValues_LA.heading);

				memset(Buffer, 0, sizeof(Buffer));
				BufferIdx = 0;
			} else {
				BufferIdx++;
			}

			PrevRxCommand = RxCommand;

		}
	}
}

static void ProcessRxGpsData(void *param1, uint32_t param2) {

	BCDS_UNUSED(param1);

	UARTTransceiver_T *transceiver;
	uint8_t *ringBuffer;
	transceiver = &Uart2TransceiverInstance;
	ringBuffer = Uart2RingBuffer;

	Retcode_T retVal = RETCODE_OK;
	uint8_t RxCommand = 0xFF;
	uint8_t RxCommandIndex = 0xFF;

	uint32_t actualLength = 0;
	uint8_t readlength = 1;
	struct tm datetime;

	if (UART_RX_ERROR != param2) {
		retVal = UartDataRead(transceiver, ringBuffer, readlength,
				&actualLength);
		if ((RETCODE_OK == retVal) && (readlength == actualLength)) {
			RxCommandIndex = readlength - actualLength;
			RxCommand = ringBuffer[RxCommandIndex];

			if (PrevRxCmd == 0x24 && RxCommand == 0x47) { // $G
				if (RawBufferLength > 0) {
					AddGpsRawData(RawBuffer, RawBufferLength - 1);
					RawBufferLength = 0;
				}

				memset(LineBuffer, 0, sizeof(LineBuffer));
				LineBuffer[0] = PrevRxCmd;
				LineBuffer[1] = RxCommand;
				LineBufferIdx = 2;

			} else if (PrevRxCmd == 0xb5 && RxCommand == 0x62) { // µb
				if (RawBufferLength > 0) {
					AddGpsRawData(RawBuffer, RawBufferLength - 1);
				}

				RawBuffer[0] = PrevRxCmd;
				RawBuffer[1] = RxCommand;
				RawBufferLength = 2;

			} else {
				if (LineBufferIdx >= 2) {
					if (RxCommand == 0x0A) { // newline
						LineBuffer[LineBufferIdx] = 0;
						// printf("%s\n", LineBuffer);
						if (memcmp(LineBuffer, "$GNRMC", 5) == 0) {
							ProcessGXRMC((char *) LineBuffer, &datetime);
						}
						LineBufferIdx = 0;
					} else {
						LineBuffer[LineBufferIdx] = RxCommand;
						LineBufferIdx++;
					}
				} else if (RawBufferLength >= 2) {
					RawBuffer[RawBufferLength] = RxCommand;
					RawBufferLength++;
				}
			}

//			if (RxCommand == 0xb5) {
//				printf("\n");
//			}
//			printf("%02x", RxCommand);

			PrevRxCmd = RxCommand;
		}
	}
}

static bool UartFrameEndCheck(uint8_t lastByte) {
	BCDS_UNUSED(lastByte);
	return true;
}

static void UartTxRxCallback(struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE_OK;
	bool flag = true;
	if ((event.RxComplete) && (NULL != AppCmdProcessor)) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor, ProcessRxXDKData,
				&flag, UART_RX_SUCCESS);
	} else if (event.RxError) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor, ProcessRxXDKData,
				&flag, UART_RX_ERROR);
	}
	if (RETCODE_OK != retVal) {
		Retcode_RaiseErrorFromIsr(retVal);
	}
}

static void Uart2TxRxCallback(struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE_OK;
	bool flag = false;
	if ((event.RxComplete) && (NULL != AppCmdProcessor)) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor, ProcessRxGpsData,
				&flag, UART_RX_SUCCESS);
	} else if (event.RxError) {
		retVal = CmdProcessor_EnqueueFromIsr(AppCmdProcessor, ProcessRxGpsData,
				&flag, UART_RX_ERROR);
	}
	if (RETCODE_OK != retVal) {
		Retcode_RaiseErrorFromIsr(retVal);
	}
}

static void UartDriverCallBack(UART_T uart, struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
	if (UartTransceiverInstance.handle == uart) {
		UARTTransceiver_LoopCallback(&UartTransceiverInstance, event);
	} else {
		Retcode_RaiseError(retVal);
	}
}

static void Uart2DriverCallBack(UART_T uart, struct MCU_UART_Event_S event) {
	Retcode_T retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
	if (Uart2TransceiverInstance.handle == uart) {
		UARTTransceiver_LoopCallback(&Uart2TransceiverInstance, event);
	} else {
		Retcode_RaiseError(retVal);
	}
}

Retcode_T Uart_Init(uint32_t baudRate, uint32_t parity, uint32_t stopbit,
bool nr1) {
	Retcode_T retVal = RETCODE_OK;
	enum UARTTransceiver_UartType_E type;
	HWHandle_T UartHandle = NULL;
	uint32_t rxBuffSize;

	bool flag;
	if (nr1) {
		flag = true;
	} else {
		flag = false;
	}

	retVal = BSP_ExtensionPort_ConnectUart(flag);
	if (RETCODE_OK == retVal) {
		retVal = BSP_ExtensionPort_SetUartConfig(BSP_EXTENSIONPORT_UART_PARITY,
				parity, NULL, flag);
	}
	if (RETCODE_OK == retVal) {
		retVal = BSP_ExtensionPort_SetUartConfig(
				BSP_EXTENSIONPORT_UART_BAUDRATE, baudRate, NULL, flag);
	}
	if (RETCODE_OK == retVal) {
		retVal = BSP_ExtensionPort_SetUartConfig(
				BSP_EXTENSIONPORT_UART_STOPBITS, stopbit, NULL, flag);
	}

	if (RETCODE_OK == retVal) {
		/*Get the serial uart  handle */
		UartHandle = BSP_ExtensionPort_GetUartHandle(flag);
	}
	if (NULL == UartHandle) {
		retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
	} else {
		if (flag == true) {
			retVal = MCU_UART_Initialize(UartHandle, UartDriverCallBack);
			if (RETCODE_OK == retVal) {
				type = UART_TRANSCEIVER_UART_TYPE_UART;
				rxBuffSize = (MAX_UART_RX_BUFFERSIZE - 1);
				retVal = UARTTransceiver_Initialize(&UartTransceiverInstance,
						UartHandle, UartRingBuffer, rxBuffSize, type);
			}
			// printf("retVal after init UART 1: %04ld\n", retVal);
		} else {
			retVal = MCU_UART_Initialize(UartHandle, Uart2DriverCallBack);
			if (RETCODE_OK == retVal) {
				type = UART_TRANSCEIVER_UART_TYPE_UART;
				rxBuffSize = (MAX_UART_RX_BUFFERSIZE - 1);
				retVal = UARTTransceiver_Initialize(&Uart2TransceiverInstance,
						UartHandle, Uart2RingBuffer, rxBuffSize, type);
			}
			// printf("retVal after init UART 2: %04ld\n", retVal);
		}
	}

	return retVal;
}

Retcode_T Uart_Enable(bool nr1) {
	Retcode_T retVal = RETCODE_OK;
	if (nr1) {
		retVal = BSP_ExtensionPort_EnableUart(true);
		if (RETCODE_OK == retVal) {
			retVal = UARTTransceiver_StartInAsyncMode(&UartTransceiverInstance,
					UartFrameEndCheck, UartTxRxCallback);
		}
		// printf("retVal after enable UART 1: %04ld\n", retVal);
	} else {
		retVal = BSP_ExtensionPort_EnableUart(false);
		if (RETCODE_OK == retVal) {
			retVal = UARTTransceiver_StartInAsyncMode(&Uart2TransceiverInstance,
					UartFrameEndCheck, Uart2TxRxCallback);
		}
		// printf("retVal after enable UART 2: %04ld\n", retVal);
	}
	return retVal;
}
