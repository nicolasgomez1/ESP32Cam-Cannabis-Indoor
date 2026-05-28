//         ___________________________________________________________________________
//        /                                                                          /\
//       /  _   __   ____   ______   ____     __       ___      _____    ______     / /\
//      /  / | / /  /  _/  / ____/  / __ \   / /      /   |    / ___/   / ____/  __/ /
//     /  /  |/ /   / /   / /      / / / /  / /      / /| |    \__ \   / / __   /\_\/
//    /  / /|  /  _/ /   / /___   / /_/ /  / /___   / ___ |   ___/ /  / /_/ /  /_/
//   /  /_/ |_/  /___/   \____/   \____/  /_____/  /_/  |_|  /____/   \____/    /\
//  /                             Version 1 (2026)                             / /
// /__________________________________________________________________________/ /
// \__________________________________________________________________________\/
//  \    \    \    \    \    \    \    \    \    \    \    \    \    \    \    \

#define FIRMWAREVERSION "V5_0528_1753WiP"	// Subfix d (DEBUG), r (RELEASE) & WiP (Work in process)

#include <WiFi.h>
#include <SD_MMC.h>
#include <Update.h>
#include <Secrets.h>
#include <HTTPClient.h>
#include <esp_camera.h>
#include <ESPAsyncWebServer.h>

/* NOTES:
	- Free pins: 0, 1, 3, 12, 13 & 16
*/

// Definitions
#define LOG_QUEUE_SIZE				20
#define LOG_QUEUE_MAX_MSG_LEN	256

#define WIFI_MAX_RETRYS			5			// Max WiFi reconnection attempts
#define WIFI_RETRY_INTERVAL	1000	// 1 second

#define TIMEZONE "ART3"	// POSIX Format

#define TIME_SAVE_INTERVAL 10000	// 10 seconds

#define FLASH_LED_FREQUENCY		20000	// 20kHz
#define FLASH_LED_RESOLUTION	8

// Pins (Using an NodeMCU ESP32-CAM (OV3660))
#define PWDN_GPIO_NUM		32
#define RESET_GPIO_NUM	-1
#define XCLK_GPIO_NUM		0
#define SIOD_GPIO_NUM		26
#define SIOC_GPIO_NUM		27

#define Y9_GPIO_NUM			35
#define Y8_GPIO_NUM			34
#define Y7_GPIO_NUM			39
#define Y6_GPIO_NUM			36
#define Y5_GPIO_NUM			21
#define Y4_GPIO_NUM			19
#define Y3_GPIO_NUM			18
#define Y2_GPIO_NUM			5
#define VSYNC_GPIO_NUM	25
#define HREF_GPIO_NUM		23
#define PCLK_GPIO_NUM		22

#define LED_GPIO_NUM		4

// Global Variables
enum SETTINGS_CODES {
	IDX_TIME,
	IDX_WIFI_SSID,
	IDX_WIFI_PWD,
	IDX_WIFI_STA,
	IDX_WIFI_RETRY,
	IDX_WIFI_SLEEP,
	IDX_WIFI_POWER,
	IDX_SENSOR_SHUTDOWN_INTERVAL,
	IDX_TL_START,
	IDX_TL_STOP,
	IDX_TL_INTERVAL,
	IDX_TL_COUNTER,
	IDX_LED_BRIGHT_TIMELAPSE,
	IDX_LED_BRIGHT_MONITORING,
	IDX_XCLK,
	IDX_PIXFORMAT,
	IDX_IFS_RESOLUTION,
	IDX_JPEG_QUALITY,
	IDX_FB_COUNT,
	IDX_FB_LOCATION,
	IDX_GRAB_MODE,
	IDX_MFS_RESOLUTION,
	IDX_BRIGHTNESS,
	IDX_CONTRAST,
	IDX_SATURATION,
	IDX_SHARPNESS,
	IDX_DENOISE,
	IDX_SPECIAL_EFFECT,
	IDX_WB_MODE,
	IDX_AWB_ENABLE,
	IDX_AWB_GAIN_LEVEL,
	IDX_AEC_ENABLE,
	IDX_AEC2_NIGHT,
	IDX_AE_LEVEL,
	IDX_AEC_VALUE,
	IDX_AGC_ENABLE,
	IDX_AGC_GAIN_LEVEL,
	IDX_GAIN_CEILING,
	IDX_BPC,
	IDX_WPC,
	IDX_RAW_GAMMA,
	IDX_LENS_CORR,
	IDX_H_FLIP,
	IDX_V_FLIP,
	IDX_DCW_ENABLE,
	IDX_COLORBAR_ENABLE
};

// DO NOT TOUCH IT!
enum ERR_TYPE { INFO, WARN, ERROR };

struct LogMessage {
	char cBuffer[LOG_QUEUE_MAX_MSG_LEN];
	char cFileName[29];
};

// Settings Variables
char g_cSSID[32];
char g_cSSIDPWD[32];
uint64_t g_nWiFiRetryConnectInterval = 0;
bool g_bWiFiSleep = true;
wifi_power_t g_pWiFiPower;
uint64_t g_nSensorShutdownInterval = 0;
uint8_t g_nEffectiveStartTimelapse = 0;
uint8_t g_nEffectiveStopTimelapse = 0;
uint64_t g_nTimelapseInterval = 0;
uint16_t g_nTimelapseCounter = 0;
uint8_t g_nTimelapseLedBrightness = 0;
uint8_t g_nMonitoringLedBrightness = 0;
camera_config_t g_pCameraConfig;
camera_status_t g_pSensorStatus;

// Internal Variables
bool bRestart = false;
bool bForceTryConnectWiFi = true;
uint64_t g_nLastCameraActivity = 0;
uint8_t g_nCurrentLedBrightness = 0;
framesize_t g_CurrentFrameSize;
bool g_bTakingSnapshot = false;
bool g_bTakingTimelapse = false;
uint8_t g_nOTAProgress = 0;

// Global Handles, Interface & Instances
AsyncWebServer g_pWebServer(SECRET_WEBSERVER_PORT);	// Asynchronous web server instance listening on SECRET_WEBSERVER_PORT
TaskHandle_t g_pWiFiReconnect;											// Task handle for WiFi reconnect logic running on core 0
SemaphoreHandle_t g_pSDMutex;												// Mutex to synchronize concurrent access to the SD card across tasks
QueueHandle_t g_pLogQueue;													// Queue handle for asynchronous logging to decouple SD writes from main logic
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static inline void SET_BIT_TO_MASK(uint64_t &nMask, uint8_t nBit) { nMask |= (1ULL << nBit); }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static inline uint64_t millis64() { return esp_timer_get_time() / 1000ULL; }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sets the system time and timezone based on a given Unix timestamp.
// Updates the system's internal clock to the provided timestamp (seconds since epoch).
static void SetCurrentDatetime(time_t nTimestamp) {
	struct timeval tv;
	tv.tv_sec = nTimestamp;
	tv.tv_usec = 0;

	settimeofday(&tv, nullptr);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Retrieves the current local time and stores it in the provided tm struct.
// Uses the system time (UTC) and converts it to local time based on the configured timezone.
// The result is stored in the pTimeInfo pointer passed by the caller.
static void GetLocalTimeNow(struct tm* pTimeInfo) {
	time_t pTimeNow = time(nullptr);
	localtime_r(&pTimeNow, pTimeInfo);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reads a line from the given file stream into the provided buffer.
// Ensures the buffer is null-terminated and trims trailing whitespace (e.g., spaces, tabs, newlines).
// - pFile: reference to the open File object.
// - cBuffer: pointer to the destination character buffer (must be large enough to hold the line).
// - nBufferSize: size of the destination buffer (including null terminator).
static void ReadFromStream(File& pFile, char* cBuffer, size_t nBufferSize) {
	size_t nLength = pFile.readBytesUntil('\n', cBuffer, nBufferSize - 1);

	cBuffer[nLength] = '\0';

	while (nLength > 0 && isspace(cBuffer[nLength - 1]))
		cBuffer[--nLength] = '\0';
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Provides utility functions to convert between ticks (milliseconds) and human-readable time units.
// Ticks are assumed to be in milliseconds, as returned by the millis64() function.
static inline uint64_t TicksToSeconds(uint64_t nTicks) { return nTicks / 1000; }
static inline uint64_t TicksToMinutes(uint64_t nTicks) { return nTicks / (1000 * 60); }

static inline uint64_t SecondsToTicks(uint64_t nSeconds) { return nSeconds * 1000; }
static inline uint64_t MinutesToTicks(uint64_t nMinutes) { return nMinutes * 1000 * 60; }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Executes the provided function (`fn`) with safe, exclusive access to the SD card using the SDMMC peripheral.
// - Tries to acquire the SD card mutex within 250 ms to ensure thread-safe access across concurrent tasks (e.g., Web Server and Background Logging).
// - Once the mutex is acquired, it is automatically released when the function scope ends using RAII via the ScopedMutexUnlock helper.
// - Manages internal initialization state via a persistent static flag (`bIsSDInit`).
// - If the SD card is not yet initialized, attempts initialization via SD_MMC.begin() in 1-bit mode (mode1bit = true).
// Note: 1-bit mode is essential to avoid conflicts with the camera data bus and the onboard high-power flash LED (GPIO 4).
// - If initialization fails, returns false without executing `fn`.
// - After initialization, checks hardware status (cardType != CARD_NONE). If no card is detected, resets the internal state, calls SD_MMC.end(), and returns false.
// - Validates the FAT filesystem by attempting to open the root directory. If inaccessible, resets the state, calls SD_MMC.end(), and returns false.
// - Upon successful validation, executes `fn()` while holding the mutex and returns true.
bool SafeSDAccess(std::function<void()> fn) {
	static bool bIsSDInit = false;

	if (!xSemaphoreTake(g_pSDMutex, pdMS_TO_TICKS(250 /*ms*/)))	// NOTE: while more shitty is the SD card, more higher this value should be
		return false;

	struct ScopedMutexUnlock {
		SemaphoreHandle_t& pMutex;
		~ScopedMutexUnlock() { xSemaphoreGive(pMutex); }
	} unlocker{g_pSDMutex};

	if (!bIsSDInit) {
		bIsSDInit = SD_MMC.begin("/sdcard", true);

		if (!bIsSDInit)
			return false;
	}

	if (SD_MMC.cardType() == CARD_NONE) {
		bIsSDInit = false;

		SD_MMC.end();

		return false;
	}

	File pRoot = SD_MMC.open("/");
	if (!pRoot || !pRoot.isDirectory()) {
		bIsSDInit = false;

		SD_MMC.end();

		return false;
	} else {
		pRoot.close();
	}

	fn();

	return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Writes a line of text to a file on the SD card, always wrapped in SafeSDAccess for thread-safe access.
// - szFileName: Path of the file to write to.
// - szBuffer: Text string to be written.
// - bAppend: If true, appends to the file; otherwise, overwrites it.
static void WriteToSD(const char* szFileName, const char* szBuffer, bool bAppend) {
	SafeSDAccess([&]() {
		File pFile = SD_MMC.open(szFileName, bAppend ? FILE_APPEND : FILE_WRITE);
		if (pFile) {
			pFile.println(szBuffer);

			pFile.close();
		}
	});
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Writes a text string to a file on the SD card using an atomic write pattern, always wrapped in SafeSDAccess for thread-safe access.
// Writes to a temporary file first, verifies the content matches the input, then replaces the target file.
// If verification fails, the temporary file is removed and the target file is left unchanged.
// - szFileName: Path of the file to write to.
// - szBuffer: Text string to be written.
static void WriteToSDAtomic(const char* szFileName, const char* szBuffer) {
	SafeSDAccess([&]() {
		char cTempFileName[64];
		snprintf(cTempFileName, sizeof(cTempFileName), "%s.atomic", szFileName);

		File pTempFile = SD_MMC.open(cTempFileName, FILE_WRITE);
		if (!pTempFile)
			return;

		pTempFile.print(szBuffer);
		pTempFile.close();

		File pFile = SD_MMC.open(cTempFileName, FILE_READ);
		if (!pFile) {
			SD_MMC.remove(cTempFileName);
			return;
		}

		char cContent[strlen(szBuffer) + 1] = {};
		pFile.readBytes(cContent, sizeof(cContent) - 1);

		pFile.close();

		if (strcmp(cContent, szBuffer) != 0) {
			SD_MMC.remove(cTempFileName);
			return;
		}

		SD_MMC.remove(szFileName);
		SD_MMC.rename(cTempFileName, szFileName);
	});
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Logs a formatted message with severity and timestamp to a daily SD log file via an async queue.
// Parameters:
// - nType: Log severity (INFO, WARN, ERROR).
// - szFormat: printf-style format string with optional arguments.
// Behavior:
// - Prepends timestamp (DD/MM/YYYY HH:MM:SS) and severity tag ([INFO], [WARN], [ERROR]).
// - Enqueues the formatted message for writing to a daily log file (/logs/logging_DD_MM_YYYY.txt).
// - Non-blocking: if the queue is full, the message is silently dropped.
static void LOGGER(ERR_TYPE nType, const char* szFormat, ...) {
	LogMessage pMSG;
	char cPrintType[9];
	struct tm currentTime;

	GetLocalTimeNow(&currentTime);

	switch (nType) {
		case INFO:	snprintf(cPrintType, sizeof(cPrintType), "[INFO] ");	break;
		case WARN:	snprintf(cPrintType, sizeof(cPrintType), "[WARN] ");	break;
		case ERROR:	snprintf(cPrintType, sizeof(cPrintType), "[ERROR] ");	break;
	}

	int nOffset = snprintf(pMSG.cBuffer, sizeof(pMSG.cBuffer), "%02d/%02d/%04d %02d:%02d:%02d %s", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900, currentTime.tm_hour, currentTime.tm_min, currentTime.tm_sec, cPrintType);

	va_list args;
	va_start(args, szFormat);
	vsnprintf(pMSG.cBuffer + nOffset, sizeof(pMSG.cBuffer) - nOffset, szFormat, args);
	va_end(args);

	snprintf(pMSG.cFileName, sizeof(pMSG.cFileName), "/logs/logging_%02d_%02d_%04d.txt", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900);

	xQueueSend(g_pLogQueue, &pMSG, 0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Saves current global configuration settings to the "/settings" file on the SD card using SafeSDAccess.
// Writes one setting per line, including Wi-Fi credentials, Flash LED, Camera & Sensor settings.
// If the SD card is not initialized or the file fails to open, the function does nothing.
// Logs a success message if the settings file is successfully written.
void SaveSettings() {
	SafeSDAccess([&]() {
		File pSettingsFile = SD_MMC.open("/settings", FILE_WRITE);
		if (pSettingsFile) {
			pSettingsFile.println(g_cSSID);
			pSettingsFile.println(g_cSSIDPWD);
			pSettingsFile.println(g_nWiFiRetryConnectInterval);
			pSettingsFile.println(g_bWiFiSleep);
			pSettingsFile.println(g_pWiFiPower);

			pSettingsFile.println(g_nSensorShutdownInterval);

			pSettingsFile.println(g_nEffectiveStartTimelapse);
			pSettingsFile.println(g_nEffectiveStopTimelapse);
			pSettingsFile.println(g_nTimelapseInterval);
			pSettingsFile.println(g_nTimelapseCounter);
			pSettingsFile.println(g_nTimelapseLedBrightness);

			pSettingsFile.println(g_nMonitoringLedBrightness);

			pSettingsFile.println(g_pCameraConfig.xclk_freq_hz);
			pSettingsFile.println(g_pCameraConfig.pixel_format);
			pSettingsFile.println(g_pCameraConfig.frame_size);	// Initial & Timelapse Frame Size
			pSettingsFile.println(g_pCameraConfig.jpeg_quality);
			pSettingsFile.println(g_pCameraConfig.fb_count);
			pSettingsFile.println(g_pCameraConfig.fb_location);
			pSettingsFile.println(g_pCameraConfig.grab_mode);
			pSettingsFile.println(g_pSensorStatus.framesize);	// Monitoring Frame Size
			pSettingsFile.println(g_pSensorStatus.brightness);
			pSettingsFile.println(g_pSensorStatus.contrast);
			pSettingsFile.println(g_pSensorStatus.saturation);
			pSettingsFile.println(g_pSensorStatus.sharpness);
			pSettingsFile.println(g_pSensorStatus.denoise);
			pSettingsFile.println(g_pSensorStatus.special_effect);
			pSettingsFile.println(g_pSensorStatus.wb_mode);
			pSettingsFile.println(g_pSensorStatus.awb);
			pSettingsFile.println(g_pSensorStatus.awb_gain);
			pSettingsFile.println(g_pSensorStatus.aec);
			pSettingsFile.println(g_pSensorStatus.aec2);
			pSettingsFile.println(g_pSensorStatus.ae_level);
			pSettingsFile.println(g_pSensorStatus.aec_value);
			pSettingsFile.println(g_pSensorStatus.agc);
			pSettingsFile.println(g_pSensorStatus.agc_gain);
			pSettingsFile.println(g_pSensorStatus.gainceiling);
			pSettingsFile.println(g_pSensorStatus.bpc);
			pSettingsFile.println(g_pSensorStatus.wpc);
			pSettingsFile.println(g_pSensorStatus.raw_gma);
			pSettingsFile.println(g_pSensorStatus.lenc);
			pSettingsFile.println(g_pSensorStatus.hmirror);
			pSettingsFile.println(g_pSensorStatus.vflip);
			pSettingsFile.println(g_pSensorStatus.dcw);
			pSettingsFile.println(g_pSensorStatus.colorbar);

			pSettingsFile.close();

			LOGGER(INFO, "Settings file updated successfully.");
		}
	});
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handles automatic WiFi reconnection using the global SSID and password credentials.
// - Starts a temporary Access Point (SECRET_ACCESSPOINT_NAME) to allow reconfiguration during reconnection attempts.
// - Sets execution parameters like transmission power and sleep mode upon connection attempt.
// - Tries to reconnect up to WIFI_MAX_RETRYS times, with a delay of WIFI_RETRY_INTERVAL between each attempt.
// - Once successfully connected, the Access Point is shut down and the mode is switched to station-only (WIFI_STA).
// - Logs a success message with the assigned IP address upon connection, or an error message if all attempts fail.
// - After completion (regardless of success), the task is suspended until explicitly resumed elsewhere.
void Thread_WiFiReconnect(void*) {
	for (;;) {
		if (!(WiFi.getMode() & WIFI_AP)) {
			LOGGER(INFO, "Starting Access Point (SSID: %s) mode for reconfiguration...", SECRET_ACCESSPOINT_NAME);

			WiFi.mode(WIFI_AP_STA);	// Set dual mode, Access Point & Station

			vTaskDelay(100 / portTICK_PERIOD_MS);	// Delay to stabilize AP

			WiFi.softAP(SECRET_ACCESSPOINT_NAME);	// Start Access Point, while try to connect to WiFi
		}

		WiFi.disconnect(true);

		LOGGER(INFO, "Trying to reconnect WiFi...");

		if (g_cSSID[0] != '\0') {
			WiFi.begin(g_cSSID, g_cSSIDPWD);
			WiFi.setTxPower(g_pWiFiPower);
			WiFi.setSleep(g_bWiFiSleep);

			uint8_t nConnectTrysCount = 0;

			while (nConnectTrysCount < WIFI_MAX_RETRYS && WiFi.status() != WL_CONNECTED) {
				nConnectTrysCount++;

				vTaskDelay(WIFI_RETRY_INTERVAL / portTICK_PERIOD_MS);	// Wait before trying again
			}

			if (WiFi.status() == WL_CONNECTED) {
				LOGGER(INFO, "Connected to WiFi SSID: %s PASSWORD: %s. IP: %d.%d.%d.%d.", g_cSSID, g_cSSIDPWD, WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

				WiFi.softAPdisconnect(true);
				WiFi.mode(WIFI_STA);

				LOGGER(INFO, "Access Point disconnected.");
			} else {
				LOGGER(ERROR, "Max WiFi reconnect attempts reached.");
			}
		}

		vTaskSuspend(NULL);	// Suspends the task until needed again
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Asynchronously processes log messages from a global queue and writes them to the SD card.
// Behavior:
// - Continuously waits for new LogMessage structures in the g_pLogQueue.
// - Uses portMAX_DELAY to stay in a blocked state without consuming CPU cycles until a message arrives.
// - Once a message is received, it invokes WriteToSD to persist the data.
// - This task decouples log generation from file system I/O, preventing the main application logic from stalling during slow SD card write operations or mutex contention.
void Thread_LogProcessor(void*) {
	LogMessage pMSG;

	for (;;) {
		if (xQueueReceive(g_pLogQueue, &pMSG, portMAX_DELAY))
			WriteToSD(pMSG.cFileName, pMSG.cBuffer, true);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Synchronizes the physical camera sensor registers with the current values stored in the global g_pSensorStatus structure.
// Behavior:
// - Retrieves the current sensor pointer using esp_camera_sensor_get().
// - Applies the resolution specified by the 'FrameSize' parameter, allowing the caller to switch between Monitoring or Timelapse resolutions dynamically.
// - Sequentially applies all remaining image processing parameters (exposure, gain, white balance, etc.) from the global status structure.
// - This function is essential to ensure hardware-software consistency, especially after the sensor wakes up from a Power Down state (PWDN HIGH to LOW), as the OV3660 volatile registers are reset to factory defaults upon hardware reactivation.
// - By passing 'FrameSize' as an argument, the function decouples the resolution intent from the rest of the sensor's image state.
static void SetSensorConfig(framesize_t FrameSize) {
	sensor_t *pSensorConfig = esp_camera_sensor_get();

	pSensorConfig->set_framesize(pSensorConfig, FrameSize);
	pSensorConfig->set_brightness(pSensorConfig, g_pSensorStatus.brightness);
	pSensorConfig->set_contrast(pSensorConfig, g_pSensorStatus.contrast);
	pSensorConfig->set_saturation(pSensorConfig, g_pSensorStatus.saturation);
	pSensorConfig->set_sharpness(pSensorConfig, g_pSensorStatus.sharpness);
	pSensorConfig->set_denoise(pSensorConfig, g_pSensorStatus.denoise);
	pSensorConfig->set_special_effect(pSensorConfig, g_pSensorStatus.special_effect);
	pSensorConfig->set_wb_mode(pSensorConfig, g_pSensorStatus.wb_mode);
	pSensorConfig->set_whitebal(pSensorConfig, g_pSensorStatus.awb);
	pSensorConfig->set_awb_gain(pSensorConfig, g_pSensorStatus.awb_gain);
	pSensorConfig->set_exposure_ctrl(pSensorConfig, g_pSensorStatus.aec);
	pSensorConfig->set_aec2(pSensorConfig, g_pSensorStatus.aec2);
	pSensorConfig->set_ae_level(pSensorConfig, g_pSensorStatus.ae_level);
	pSensorConfig->set_aec_value(pSensorConfig, g_pSensorStatus.aec_value);
	pSensorConfig->set_gain_ctrl(pSensorConfig, g_pSensorStatus.agc);
	pSensorConfig->set_agc_gain(pSensorConfig, g_pSensorStatus.agc_gain);
	pSensorConfig->set_gainceiling(pSensorConfig, (gainceiling_t)g_pSensorStatus.gainceiling);
	pSensorConfig->set_bpc(pSensorConfig, g_pSensorStatus.bpc);
	pSensorConfig->set_wpc(pSensorConfig, g_pSensorStatus.wpc);
	pSensorConfig->set_raw_gma(pSensorConfig, g_pSensorStatus.raw_gma);
	pSensorConfig->set_lenc(pSensorConfig, g_pSensorStatus.lenc);
	pSensorConfig->set_hmirror(pSensorConfig, g_pSensorStatus.hmirror);
	pSensorConfig->set_vflip(pSensorConfig, g_pSensorStatus.vflip);
	pSensorConfig->set_dcw(pSensorConfig, g_pSensorStatus.dcw);
	pSensorConfig->set_colorbar(pSensorConfig, g_pSensorStatus.colorbar);
}

void setup() {
	esp_reset_reason_t pReason = esp_reset_reason();
	g_pSDMutex = xSemaphoreCreateMutex();
	g_pLogQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));

	LOGGER(INFO, "========== Indoor Camera Controller Started ==========");
	LOGGER(INFO, "Firmware Version: %s", FIRMWAREVERSION);

	if (pReason != ESP_RST_POWERON && pReason != ESP_RST_SW) {
		const char* szReasons[] = { "Unknown", "Power on", "External pin", "Software", "Panic/Exception", "Interrupt watchdog", "Task watchdog", "Other watchdog", "Deepsleep", "Brownout", "SDIO" };

		LOGGER(WARN, "Reset reason: %s", szReasons[pReason]);
	}

	LOGGER(INFO, "Initializing Pins...");

	g_pCameraConfig.pin_pwdn = PWDN_GPIO_NUM;
	g_pCameraConfig.pin_reset = RESET_GPIO_NUM;

	g_pCameraConfig.pin_xclk = XCLK_GPIO_NUM;

	g_pCameraConfig.pin_sccb_sda = SIOD_GPIO_NUM;
	g_pCameraConfig.pin_sccb_scl = SIOC_GPIO_NUM;

	g_pCameraConfig.pin_d0 = Y2_GPIO_NUM;
	g_pCameraConfig.pin_d1 = Y3_GPIO_NUM;
	g_pCameraConfig.pin_d2 = Y4_GPIO_NUM;
	g_pCameraConfig.pin_d3 = Y5_GPIO_NUM;
	g_pCameraConfig.pin_d4 = Y6_GPIO_NUM;
	g_pCameraConfig.pin_d5 = Y7_GPIO_NUM;
	g_pCameraConfig.pin_d6 = Y8_GPIO_NUM;
	g_pCameraConfig.pin_d7 = Y9_GPIO_NUM;

	g_pCameraConfig.pin_vsync = VSYNC_GPIO_NUM;

	g_pCameraConfig.pin_href = HREF_GPIO_NUM;

	g_pCameraConfig.pin_pclk = PCLK_GPIO_NUM;

	LOGGER(INFO, "Camera Pins Done!");

	ledcAttach(LED_GPIO_NUM, FLASH_LED_FREQUENCY, FLASH_LED_RESOLUTION);
	ledcWrite(LED_GPIO_NUM, 0);

	LOGGER(INFO, "Flash LED Pin Done!");

	LOGGER(INFO, "Loading Settings & Time...");

	SafeSDAccess([&]() {
		File pSettingsFile = SD_MMC.open("/settings", FILE_READ); // Read Settings File
		if (pSettingsFile) {
			char cBuffer[64];
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// SSID

			strncpy(g_cSSID, cBuffer, sizeof(g_cSSID));
			g_cSSID[sizeof(g_cSSID) - 1] = '\0';
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// SSID PASSWORD

			strncpy(g_cSSIDPWD, cBuffer, sizeof(g_cSSIDPWD));
			g_cSSIDPWD[sizeof(g_cSSIDPWD) - 1] = '\0';
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// WIFI RETRY CONNECT INTERVAL
			g_nWiFiRetryConnectInterval = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// WIFI SLEEP
			g_bWiFiSleep = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// WIFI TRANSMIT POWER
			g_pWiFiPower = (wifi_power_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// INTERVAL FROM LAST WEB CONNECTION TO SHUTDOWN THE SENSOR & FLASH LED
			g_nSensorShutdownInterval = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// TIMELAPSE START HOUR
			uint8_t nValue = atoi(cBuffer);

			g_nEffectiveStartTimelapse = (nValue == 24) ? 0 : nValue;	// Stores the effective timelapse start hour, converting hour 24 to 0 (midnight)
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// TIMELAPSE STOP HOUR
			nValue = atoi(cBuffer);

			g_nEffectiveStopTimelapse = (nValue == 24) ? 0 : nValue;	// Stores the effective timelapse stop hour, converting hour 24 to 0 (midnight)
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// TIMELAPSE CAPTURE INTERVAL
			g_nTimelapseInterval = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// TIMELAPSE CAPTURES COUNTER
			g_nTimelapseCounter = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// TIMELAPSE FLASH LED BRIGHTNESS
			g_nTimelapseLedBrightness = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// MONITORING FLASH LED BRIGHTNESS
			g_nMonitoringLedBrightness = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// CAMERA MASTER CLOCK (XCLK)
			g_pCameraConfig.xclk_freq_hz = atoi(cBuffer);
			///////////////////////////////////////////////////
			g_pCameraConfig.ledc_timer = LEDC_TIMER_0;								// XCLK GENERATOR SETUP (WARNING: Hardcode value)
			g_pCameraConfig.ledc_channel = LEDC_CHANNEL_0;						// XCLK GENERATOR SETUP (WARNING: Hardcode value)
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// PIXEL FORMAT
			g_pCameraConfig.pixel_format = (pixformat_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// IMAGE RESOLUTION	(Initial & Timelapse Frame Size)
			g_pCameraConfig.frame_size = (framesize_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// IMAGE COMPRESSION LEVEL
			g_pCameraConfig.jpeg_quality = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// FRAME BUFFERS COUNT
			g_pCameraConfig.fb_count = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// STORE FRAME IN
			g_pCameraConfig.fb_location = (camera_fb_location_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// FRAME TO GRAB
			g_pCameraConfig.grab_mode = (camera_grab_mode_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			//conv_mode																								// CONVERT COLOR FORMAT
			///////////////////////////////////////////////////
			//scale																										// RESIZING
			///////////////////////////////////////////////////
			//binning																									// PIXEL GROUPING
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// IMAGE RESOLUTION	(Monitoring Frame Size)
			g_pSensorStatus.framesize = (framesize_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// BRIGHTNESS LEVEL
			g_pSensorStatus.brightness = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// CONTRAST LEVEL
			g_pSensorStatus.contrast = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// SATURATION LEVEL
			g_pSensorStatus.saturation = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// SHARPNESS LEVEL
			g_pSensorStatus.sharpness = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// NOISE REDUCTION LEVEL
			g_pSensorStatus.denoise = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// SPECIAL EFFECTS
			g_pSensorStatus.special_effect = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC WHITE BALANCE PROFILE
			g_pSensorStatus.wb_mode = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC WHITE BALANCE
			g_pSensorStatus.awb = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC WHITE BALANCE GAIN
			g_pSensorStatus.awb_gain = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC EXPOSURE
			g_pSensorStatus.aec = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC EXPOSURE (NIGHT MODE)
			g_pSensorStatus.aec2 = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC EXPOSURE LEVEL
			g_pSensorStatus.ae_level = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// MANUAL EXPOSURE LEVEL
			g_pSensorStatus.aec_value = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC GAIN
			g_pSensorStatus.agc = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC GAIN LEVEL
			g_pSensorStatus.agc_gain = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// MAX AUTOMATIC GAIN LEVEL
			g_pSensorStatus.gainceiling = (gainceiling_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// BLACK PIXEL CANCELLATION
			g_pSensorStatus.bpc = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// WHITE PIXEL CANCELLATION
			g_pSensorStatus.wpc = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// RAW GAMMA CORRECTION
			g_pSensorStatus.raw_gma = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// VIGNETTE CORRECTION
			g_pSensorStatus.lenc = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// HORIZONTAL MIRRORING
			g_pSensorStatus.hmirror = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// VERTICAL FLIP
			g_pSensorStatus.vflip = atoi(cBuffer);
			//////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// DIGITAL DOWNSAMPLE
			g_pSensorStatus.dcw = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// COLOR BARS (TEST MODE)
			g_pSensorStatus.colorbar = atoi(cBuffer);
			///////////////////////////////////////////////////
			pSettingsFile.close();
		} else {
			LOGGER(ERROR, "Failed to open Settings file.");
		}
		///////////////////////////////////////////////////
		struct tm currentTime;

		LOGGER(INFO, "Getting Datetime from SD Card...");

		File pTimeFile = SD_MMC.open("/time", FILE_READ);	// Read Time file

		if (!pTimeFile) {
			if (SD_MMC.exists("/time.atomic"))
				SD_MMC.rename("/time.atomic", "/time");

			pTimeFile = SD_MMC.open("/time", FILE_READ);
		}

		if (pTimeFile) {
			setenv("TZ", TIMEZONE, 1);
			tzset();

			LOGGER(INFO, "Timezone setted.");

			char cBuffer[11];

			ReadFromStream(pTimeFile, cBuffer, sizeof(cBuffer));

			size_t nBytesRead = strlen(cBuffer);

			LOGGER(INFO, "Time file raw content: '%s', length: %d.", cBuffer, nBytesRead);

			time_t nTime = strtol(cBuffer, nullptr, 10);

			if (nTime > 1770000000) {	// WARNING: Hardcode check
				SetCurrentDatetime(nTime);

				LOGGER(INFO, "Current datetime setted.");

				GetLocalTimeNow(&currentTime);

				LOGGER(INFO, "Current Datetime: %02d/%02d/%04d %02d:%02d:%02d.", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900, currentTime.tm_hour, currentTime.tm_min, currentTime.tm_sec);
			} else {
				LOGGER(ERROR, "Time file contains invalid timestamp: %ld.", nTime);
			}

			pTimeFile.close();
		}
	});

	LOGGER(INFO, "Initializing Camera...");

	esp_err_t pCameraError = esp_camera_init(&g_pCameraConfig);
	if (pCameraError != ESP_OK) {
		LOGGER(ERROR, "Camera init failed. Error: 0x%x", pCameraError);
	} else {
		digitalWrite(PWDN_GPIO_NUM, HIGH);	// Pos esp_camera_init call

		LOGGER(INFO, "Turning Off Camera Done!");
	}

	LOGGER(INFO, "Initializing WiFi...");

	WiFi.mode(WIFI_STA);

	LOGGER(INFO, "Creating WiFi reconnect task thread...");

	xTaskCreatePinnedToCore(Thread_WiFiReconnect, "WiFiReconnectTask", 4096, NULL, 1, &g_pWiFiReconnect, 0);
	vTaskSuspend(g_pWiFiReconnect);	// Suspend the task as it's not needed right now

	LOGGER(INFO, "Creating Logging task thread...");

	xTaskCreate(Thread_LogProcessor, "LoggingTask", 4096, NULL, 1, NULL);

	LOGGER(INFO, "Setting up Web Server...");

	// Global headers to prevent CORS problems.
	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");

	// Static files server
	// Static serving of the logs, snapshots & timelapse folders and all the files inside of them
	g_pWebServer.serveStatic("/logs", SD_MMC, "/logs").setCacheControl("max-age=2592000, immutable");	// Cache it by 1 month
	g_pWebServer.serveStatic("/snapshots", SD_MMC, "/snapshots").setCacheControl("max-age=2592000, immutable");	// Cache it by 1 month
	g_pWebServer.serveStatic("/timelapse", SD_MMC, "/timelapse").setCacheControl("max-age=2592000, immutable");	// Cache it by 1 month

	// Request handler
	g_pWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest* pRequest) {
		const AsyncWebParameter* pParamAction = pRequest->getParam("action");
		if (pParamAction) {
			if (pParamAction->value() == "refresh") { // This is for refresh Panel values
				char cBuffer[278];
				time_t pTimeNow = time(nullptr);

				snprintf(cBuffer, sizeof(cBuffer), "REFRESH%lu:%s:%u:%s:%s:%llu:%u:%d:%llu:%u:%u:%llu:%d:%u:%u:%d:%d:%d:%d:%lu:%d:%d:%d:%d:%d:%d:%d:%u:%u:%u:%u:%u:%u:%u:%d:%u:%u:%u:%u:%u:%u:%u:%u:%u:%u:%u:%u",
					(unsigned long)pTimeNow, FIRMWAREVERSION, g_nOTAProgress, g_cSSID, g_cSSIDPWD, TicksToMinutes(g_nWiFiRetryConnectInterval), (unsigned)g_bWiFiSleep, (int)g_pWiFiPower, TicksToSeconds(g_nSensorShutdownInterval),
					(g_nEffectiveStartTimelapse == 0) ? 24 : g_nEffectiveStartTimelapse, (g_nEffectiveStopTimelapse == 0) ? 24 : g_nEffectiveStopTimelapse, TicksToMinutes(g_nTimelapseInterval), g_nTimelapseCounter,
					g_nTimelapseLedBrightness, g_nMonitoringLedBrightness, g_pCameraConfig.xclk_freq_hz, (int)g_pCameraConfig.pixel_format, (int)g_pCameraConfig.frame_size,	g_pCameraConfig.jpeg_quality,
					(unsigned long)g_pCameraConfig.fb_count, (int)g_pCameraConfig.fb_location, (int)g_pCameraConfig.grab_mode, (int)g_pSensorStatus.framesize, g_pSensorStatus.brightness, g_pSensorStatus.contrast, g_pSensorStatus.saturation,
					g_pSensorStatus.sharpness, g_pSensorStatus.denoise, g_pSensorStatus.special_effect, g_pSensorStatus.wb_mode, g_pSensorStatus.awb, g_pSensorStatus.awb_gain, g_pSensorStatus.aec, g_pSensorStatus.aec2,
					g_pSensorStatus.ae_level, g_pSensorStatus.aec_value, g_pSensorStatus.agc, g_pSensorStatus.agc_gain, g_pSensorStatus.gainceiling, g_pSensorStatus.bpc, g_pSensorStatus.wpc, g_pSensorStatus.raw_gma, g_pSensorStatus.lenc,
					g_pSensorStatus.hmirror, g_pSensorStatus.vflip, g_pSensorStatus.dcw, g_pSensorStatus.colorbar
				);
				// ========================================================================================================================= //
				/*
					Response structure example: each data[X] is divided by ':'
					data[0] → Current Timestamp
					data[1] → Firmware Version
					data[2] → OTA Update Progress
					data[3] → SSID
					data[4] → SSID PWD
					data[5] → Try to reconnect interval
					data[6] → WiFi Power Save Mode (Sleep)
					data[7] → Wifi Transmit Power
					data[8] → Interval to Turn Off Camera Sensor & Flash LED
					data[9] → Timelapse Start Hour
					data[10] → Timelapse Stop Hour
					data[11] → Timelapse Interval
					data[12] → Timelapse Captures Counter
					data[13] → Timelapse Flash LED Brightness
					data[14] → Monitoring Flash LED Brightness
					data[15] → Camera Master Clock (XCLK)
					data[16] → Camera Pixel Format
					data[17] → Camera Initial & Timelapse Frame Size
					data[18] → Camera Image Compression Level
					data[19] → Camera Frame Buffers Count
					data[20] → Camera Frame Buffer Location
					data[21] → Camera Frame To Grab
					data[22] → Sensor Monitoring Frame Size
					data[23] → Sensor Brightness
					data[24] → Sensor Contrast
					data[25] → Sensor Saturation
					data[26] → Sensor Sharpness
					data[27] → Sensor Noise Reduction Level
					data[28] → Sensor Special Effect
					data[29] → Sensor Automatic White Balance Profile
					data[30] → Sensor Automatic White Balance Enable
					data[31] → Sensor Automatic White Balance Gain
					data[32] → Sensor Automatic Exposure Enable
					data[33] → Sensor Automatic Exposure (Night Mode) Enable
					data[34] → Sensor Auto Exposure Compensation Level
					data[35] → Sensor Manual Exposure Level
					data[36] → Sensor Automatic Gain Enable
					data[37] → Sensor Manual Gain Level
					data[38] → Sensor Gain Ceiling Level
					data[39] → Sensor Black Pixel Cancellation Enable
					data[40] → Sensor White Pixel Cancellation Enable
					data[41] → Sensor Raw Gamma Correction Level
					data[42] → Sensor Vignette Correction Enable
					data[43] → Sensor Horizontal Mirroring
					data[44] → Sensor Vertical Flip
					data[45] → Sensor Digital Downsample Enable
					data[46] → Sensor Color Bars (Test Mode) Enable
				*/

				pRequest->send(200, "text/plain", cBuffer);
				return;
			} else if (pParamAction->value() == "update") {	// This is for update Settings
				uint64_t nNewValue;
				uint64_t nSuccessCodeMask = 0;
				bool bWiFiChanges = false;
				sensor_t *pSensorConfig = esp_camera_sensor_get();

				// =============== Current DateTime =============== //
				const AsyncWebParameter* pParamTime = pRequest->getParam("time");
				if (pParamTime) {
					SetCurrentDatetime(pParamTime->value().toInt());

					struct tm currentTime;
					GetLocalTimeNow(&currentTime);
					LOGGER(INFO, "New Datetime: %d/%d/%04d %02d:%02d:%02d.", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900, currentTime.tm_hour, currentTime.tm_min, currentTime.tm_sec);

					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TIME);
				}
				// =============== WiFi =============== //
				const AsyncWebParameter* pParamSSID = pRequest->getParam("ssid");
				if (pParamSSID && strcmp(pParamSSID->value().c_str(), g_cSSID) != 0) {
					bWiFiChanges = true;

					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_SSID);
				}

				const AsyncWebParameter* pParamSSIDPWD = pRequest->getParam("ssidpwd");
				if (pParamSSIDPWD && strcmp(pParamSSIDPWD->value().c_str(), g_cSSIDPWD) != 0) {
					bWiFiChanges = true;

					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_PWD);
				}

				if (bWiFiChanges)
					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_STA);

				if (const AsyncWebParameter* pParam = pRequest->getParam("wfrci")) {
					nNewValue = MinutesToTicks(pParam->value().toInt());

					if (nNewValue != g_nWiFiRetryConnectInterval) {
						g_nWiFiRetryConnectInterval = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_RETRY);
					}
				}

				if (const AsyncWebParameter* pParam = pRequest->getParam("ws")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_bWiFiSleep) {
						g_bWiFiSleep = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_SLEEP);
					}
				}

				if (const AsyncWebParameter* pParam = pRequest->getParam("wp")) {
					nNewValue = pParam->value().toInt();

					if ((wifi_power_t)nNewValue != g_pWiFiPower) {
						g_pWiFiPower = (wifi_power_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_POWER);
					}
				}
				// =============== SENSOR SHUTDOWN INTERVAL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("ssi")) {
					nNewValue = SecondsToTicks(pParam->value().toInt());

					if (nNewValue != g_nSensorShutdownInterval) {
						g_nSensorShutdownInterval = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SENSOR_SHUTDOWN_INTERVAL);
					}
				}
				// =============== TIMELAPSE START HOUR =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("timelapsestart")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_nEffectiveStartTimelapse) {
						g_nEffectiveStartTimelapse = (nNewValue == 24) ? 0 : nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TL_START);
					}
				}
				// =============== TIMELAPSE STOP HOUR =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("timelapsestop")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_nEffectiveStopTimelapse) {
						g_nEffectiveStopTimelapse = (nNewValue == 24) ? 0 : nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TL_STOP);
					}
				}
				// =============== TIMELAPSE CAPTURE INTERVAL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("timelapseinterval")) {
					nNewValue = MinutesToTicks(pParam->value().toInt());

					if (nNewValue != g_nTimelapseInterval) {
						g_nTimelapseInterval = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TL_INTERVAL);
					}
				}
				// =============== TIMELAPSE CAPTURES COUNTER =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("timelapsecount")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_nTimelapseCounter) {
						g_nTimelapseCounter = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TL_COUNTER);
					}
				}
				// =============== TIMELAPSE FLASH LED BRIGHTNESS =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("lbt")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_nTimelapseLedBrightness) {
						if (g_nCurrentLedBrightness == g_nTimelapseLedBrightness) {	// If is currently use Flash, update it brightness in real time
							g_nCurrentLedBrightness = nNewValue;

							ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);
						}

						g_nTimelapseLedBrightness = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_LED_BRIGHT_TIMELAPSE);
					}
				}
				// =============== MONITORING FLASH LED BRIGHTNESS =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("lbm")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_nMonitoringLedBrightness) {
						if (g_nCurrentLedBrightness == g_nMonitoringLedBrightness) {	// If is currently use Flash, update it brightness in real time
							g_nCurrentLedBrightness = nNewValue;

							ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);
						}

						g_nMonitoringLedBrightness = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_LED_BRIGHT_MONITORING);
					}
				}
				// =============== CAMERA MASTER CLOCK (XCLK) =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("xclk")) {
					nNewValue = pParam->value().toInt();

					if ((nNewValue * 1000000U) != g_pCameraConfig.xclk_freq_hz) {
						g_pCameraConfig.xclk_freq_hz = (nNewValue * 1000000U);

						if (pSensorConfig->set_xclk(pSensorConfig, LEDC_TIMER_0, nNewValue) == 0)
							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_XCLK);
					}
				}
				// =============== PIXEL FORMAT =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("pf")) {
					nNewValue = pParam->value().toInt();

					if ((pixformat_t)nNewValue != g_pCameraConfig.pixel_format) {
						g_pCameraConfig.pixel_format = (pixformat_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_PIXFORMAT);
					}
				}
				// =============== IMAGE RESOLUTION (Initial & Timelapse) =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("ifs")) {
					nNewValue = pParam->value().toInt();

					if ((framesize_t)nNewValue != g_pCameraConfig.frame_size) {
						g_pCameraConfig.frame_size = (framesize_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_IFS_RESOLUTION);
					}
				}
				// =============== IMAGE COMPRESSION LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("jpegq")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pCameraConfig.jpeg_quality) {
						g_pCameraConfig.jpeg_quality = nNewValue;

						if (pSensorConfig->set_quality(pSensorConfig, nNewValue) == 0)
							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_JPEG_QUALITY);
					}
				}
				// =============== FRAME BUFFERS COUNT =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("fbc")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pCameraConfig.fb_count) {
						g_pCameraConfig.fb_count = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_FB_COUNT);
					}
				}
				// =============== STORE FRAME IN =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("fbl")) {
					nNewValue = pParam->value().toInt();

					if ((camera_fb_location_t)nNewValue != g_pCameraConfig.fb_location) {
						g_pCameraConfig.fb_location = (camera_fb_location_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_FB_LOCATION);
					}
				}
				// =============== FRAME TO GRAB =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("gm")) {
					nNewValue = pParam->value().toInt();

					if ((camera_grab_mode_t)nNewValue != g_pCameraConfig.grab_mode) {
						g_pCameraConfig.grab_mode = (camera_grab_mode_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_GRAB_MODE);
					}
				}
				// =============== IMAGE RESOLUTION (Monitoring) =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("mfs")) {
					nNewValue = pParam->value().toInt();

					if ((framesize_t)nNewValue != g_pSensorStatus.framesize) {
						if (pSensorConfig->set_framesize(pSensorConfig, (framesize_t)nNewValue) == 0) {
							g_pSensorStatus.framesize = (framesize_t)nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_MFS_RESOLUTION);
						}
					}
				}
				// =============== BRIGHTNESS LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("bnl")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.brightness) {
						if (pSensorConfig->set_brightness(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.brightness = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_BRIGHTNESS);
						}
					}
				}
				// =============== CONTRAST LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("cl")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.contrast) {
						if (pSensorConfig->set_contrast(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.contrast = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_CONTRAST);
						}
					}
				}
				// =============== SATURATION LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("sl")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.saturation) {
						if (pSensorConfig->set_saturation(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.saturation = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SATURATION);
						}
					}
				}
				// =============== SHARPNESS LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("snl")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.sharpness) {
						if (pSensorConfig->set_sharpness(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.sharpness = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SHARPNESS);
						}
					}
				}
				// =============== NOISE REDUCTION LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("nrl")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.denoise) {
						if (pSensorConfig->set_denoise(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.denoise = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_DENOISE);
						}
					}
				}
				// =============== SPECIAL EFFECTS =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("se")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.special_effect) {
						if (pSensorConfig->set_special_effect(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.special_effect = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SPECIAL_EFFECT);
						}
					}
				}
				// =============== AUTOMATIC WHITE BALANCE PROFILE =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("wbp")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.wb_mode) {
						if (pSensorConfig->set_wb_mode(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.wb_mode = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WB_MODE);
						}
					}
				}
				// =============== AUTOMATIC WHITE BALANCE =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("awb")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.awb) {
						if (pSensorConfig->set_whitebal(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.awb = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AWB_ENABLE);
						}
					}
				}
				// =============== AUTOMATIC WHITE BALANCE GAIN =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("awbg")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.awb_gain) {
						if (pSensorConfig->set_awb_gain(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.awb_gain = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AWB_GAIN_LEVEL);
						}
					}
				}
				// =============== AUTOMATIC EXPOSURE =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("aec")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.aec) {
						if (pSensorConfig->set_exposure_ctrl(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.aec = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AEC_ENABLE);
						}
					}
				}
				// =============== AUTOMATIC EXPOSURE (NIGHT MODE) =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("aec2")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.aec2) {
						if (pSensorConfig->set_aec2(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.aec2 = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AEC2_NIGHT);
						}
					}
				}
				// =============== AUTOMATIC EXPOSURE LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("ael")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.ae_level) {
						if (pSensorConfig->set_ae_level(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.ae_level = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AE_LEVEL);
						}
					}
				}
				// =============== MANUAL EXPOSURE LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("aev")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.aec_value) {
						if (pSensorConfig->set_aec_value(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.aec_value = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AEC_VALUE);
						}
					}
				}
				// =============== AUTOMATIC GAIN =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("agc")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.agc) {
						if (pSensorConfig->set_gain_ctrl(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.agc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AGC_ENABLE);
						}
					}
				}
				// =============== AUTOMATIC GAIN LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("agcl")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.agc_gain) {
						if (pSensorConfig->set_agc_gain(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.agc_gain = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AGC_GAIN_LEVEL);
						}
					}
				}
				// =============== MAX AUTOMATIC GAIN LEVEL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("gc")) {
					nNewValue = pParam->value().toInt();

					if ((gainceiling_t)nNewValue != g_pSensorStatus.gainceiling) {
						if (pSensorConfig->set_gainceiling(pSensorConfig, (gainceiling_t)nNewValue) == 0) {
							g_pSensorStatus.gainceiling = (gainceiling_t)nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_GAIN_CEILING);
						}
					}
				}
				// =============== BLACK PIXEL CANCELLATION =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("bpc")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.bpc) {
						if (pSensorConfig->set_bpc(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.bpc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_BPC);
						}
					}
				}
				// =============== WHITE PIXEL CANCELLATION =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("wpc")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.wpc) {
						if (pSensorConfig->set_wpc(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.wpc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WPC);
						}
					}
				}
				// =============== RAW GAMMA CORRECTION =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("rgc")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.raw_gma) {
						if (pSensorConfig->set_raw_gma(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.raw_gma = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_RAW_GAMMA);
						}
					}
				}
				// =============== VIGNETTE CORRECTION =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("lenc")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.lenc) {
						if (pSensorConfig->set_lenc(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.lenc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_LENS_CORR);
						}
					}
				}
				// =============== HORIZONTAL MIRRORING =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("hflip")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.hmirror) {
						if (pSensorConfig->set_hmirror(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.hmirror = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_H_FLIP);
						}
					}
				}
				// =============== VERTICAL FLIP =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("vflip")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.vflip) {
						if (pSensorConfig->set_vflip(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.vflip = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_V_FLIP);
						}
					}
				}
				// =============== DIGITAL DOWNSAMPLE =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("dcw")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.dcw) {
						if (pSensorConfig->set_dcw(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.dcw = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_DCW_ENABLE);
						}
					}
				}
				// =============== COLOR BARS (TEST MODE) =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("cb")) {
					nNewValue = pParam->value().toInt();

					if (nNewValue != g_pSensorStatus.colorbar) {
						if (pSensorConfig->set_colorbar(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.colorbar = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_COLORBAR_ENABLE);
						}
					}
				}
				// =============== FLASH LED CONTROL =============== //
				if (const AsyncWebParameter* pParam = pRequest->getParam("fls")) {
					nNewValue = 0;

					if (g_nCurrentLedBrightness == 0) {
						if ( pParam->value() == "0")	// Monitoring
							nNewValue = g_nMonitoringLedBrightness;
						else															// Timelapse
							nNewValue = g_nTimelapseLedBrightness;
					}

					g_nCurrentLedBrightness = nNewValue;

					ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);
				}
				//////////////////////////////////////////////////
				if (nSuccessCodeMask != 0) {	// If have some change, send response to web client and finally save new settings values
					char cBuffer[24];
					snprintf(cBuffer, sizeof(cBuffer), "MSG%llu", nSuccessCodeMask);

					pRequest->send(200, "text/plain", cBuffer);

					if (bWiFiChanges) { // Update WiFi values after response the request. in otherwise the message is not sended.
						strncpy(g_cSSID, pParamSSID->value().c_str(), sizeof(g_cSSID) - 1);
						g_cSSID[sizeof(g_cSSID) - 1] = '\0';

						strncpy(g_cSSIDPWD, pParamSSIDPWD->value().c_str(), sizeof(g_cSSIDPWD) - 1);
						g_cSSIDPWD[sizeof(g_cSSIDPWD) - 1] = '\0';
					}

					SaveSettings();

					if (bWiFiChanges) { // After send response to web client, Try reconnect to WiFi if is required
						LOGGER(INFO, "Disconnecting WiFi to start connection to new SSID...");

						WiFi.disconnect(false); // First disconnect from current Network (Arg false to just disconnect the Station, not the AP)

						if (eTaskGetState(g_pWiFiReconnect) != eSuspended)
							vTaskSuspend(g_pWiFiReconnect);

						bForceTryConnectWiFi = true;
					}

					return;
				}
			} else if (pParamAction->value() == "restart") {
				LOGGER(INFO, "Restarting Controller by Web command.");

				bRestart = true;
			} else if (pParamAction->value() == "list") {	// This returns file list from any directory in the SD Card
				if (!SafeSDAccess([&]() {
					AsyncResponseStream *pResponseStream = pRequest->beginResponseStream("text/plain");
					bool bFirst = true;
					char cPath[64];

					snprintf(cPath, sizeof(cPath), "/%s", pRequest->getParam("folder")->value().c_str());

					File pWorkingDirectory = SD_MMC.open(cPath);
					File pFile = pWorkingDirectory.openNextFile();

					while (pFile) {
						if (!pFile.isDirectory()) {
							if (!bFirst)
								pResponseStream->print(":");
							else
								bFirst = false;

							pResponseStream->print(pFile.name());
							pResponseStream->print("|");
							pResponseStream->print(pFile.size());
						}

						pFile.close();

						pFile = pWorkingDirectory.openNextFile();
					}

					pWorkingDirectory.close();
					pRequest->send(pResponseStream);
				})) {
					pRequest->send(500, "text/plain", "NO_SD");
				}

				return;
			} else if (pParamAction->value() == "cleanfolder") {
				if (!SafeSDAccess([&]() {
					if (const AsyncWebParameter* pParam = pRequest->getParam("folder")) {
						uint32_t nDeleteFilesCount = 0;
						char cPath[64];
						char cFilePath[64];

						snprintf(cPath, sizeof(cPath), "/%s", pRequest->getParam("folder")->value().c_str());

						File pWorkingDirectory = SD_MMC.open(cPath);
						File pFile = pWorkingDirectory.openNextFile();

						while (pFile) {
							if (!pFile.isDirectory()) {
								snprintf(cFilePath, sizeof(cFilePath), "%s", pFile.path());

								pFile.close();

								SD_MMC.remove(cFilePath);

								nDeleteFilesCount++;
							} else {
								pFile.close();
							}

							pFile = pWorkingDirectory.openNextFile();
						}

						pWorkingDirectory.close();

						char cDeletedFileCount[12];
						snprintf(cDeletedFileCount, sizeof(cDeletedFileCount), "%u", nDeleteFilesCount);

						pRequest->send(200, "text/plain", cDeletedFileCount);
					}
				})) {
					pRequest->send(500, "text/plain", "NO_SD");
				}

				return;
			} else if (pParamAction->value() == "deletefile") {	// Delete files from any SD folder
				if (!SafeSDAccess([&]() {
					bool bSuccess = true;
					char cPath[64];

					snprintf(cPath, sizeof(cPath), "/%s/%s", pRequest->getParam("folder")->value().c_str(), pRequest->getParam("file")->value().c_str());

					if (SD_MMC.exists(cPath)) {
						if (!SD_MMC.remove(cPath))
							bSuccess = false;
					} else {
						bSuccess = false;
					}

					pRequest->send(bSuccess ? 200 : 500, "text/plain", bSuccess ? "DELETE_FILE_SCS" : "DELETE_FILE_ERROR");
				})) {
					pRequest->send(500, "text/plain", "NO_SD");
				}

				return;
			} else if (pParamAction->value() == "capture") {
				if (g_nOTAProgress > 0) {
					if (digitalRead(PWDN_GPIO_NUM) == LOW)	// If is working
						digitalWrite(PWDN_GPIO_NUM, HIGH);	// turn it off

					pRequest->send(500, "text/plain", "OTA_IN_PROGRESS");
					return;
				}

				if (g_bTakingSnapshot) {
					pRequest->send(500, "text/plain", "TAKING_SNAPSHOT");
					return;
				}

				if (g_bTakingTimelapse) {
					pRequest->send(500, "text/plain", "TAKING_TIMELAPSE");
					return;
				}

				g_nLastCameraActivity = millis64();

				if (digitalRead(PWDN_GPIO_NUM) == HIGH)	// If is off
					digitalWrite(PWDN_GPIO_NUM, LOW);	// turn it on

				sensor_t *pSensorConfig = esp_camera_sensor_get();
				if (pSensorConfig->status.framesize != g_pSensorStatus.framesize) {
					SetSensorConfig(g_pSensorStatus.framesize);	// Monitoring Frame Size

					pSensorConfig->set_dcw(pSensorConfig, 1);
				}

				camera_fb_t *pCameraFrameBuffer = esp_camera_fb_get();
				if (!pCameraFrameBuffer) {
					pRequest->send(500, "text/plain", "FRAME_BUFFER");
					return;
				}

				AsyncWebServerResponse *pResponse = pRequest->beginResponse_P(200, "image/jpeg", pCameraFrameBuffer->buf, pCameraFrameBuffer->len);
				pResponse->addHeader("Content-Disposition", "inline; filename=capture.jpg");
				pResponse->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
				pResponse->addHeader("Access-Control-Expose-Headers", "X-Flash-Status, X-Timestamp");
				pResponse->addHeader("X-Flash-Status", (g_nCurrentLedBrightness > 0) ? "1" : "0");

				char cBuffer[11];
				snprintf(cBuffer, sizeof(cBuffer), "%lu", (unsigned long)time(nullptr));

				pResponse->addHeader("X-Timestamp", cBuffer);

				pRequest->onDisconnect([pCameraFrameBuffer]() {
					esp_camera_fb_return(pCameraFrameBuffer);
				});

				pRequest->send(pResponse);
				return;
			} else if (pParamAction->value() == "tss") {	// Take a snapshot
				if (g_nOTAProgress > 0) {
					pRequest->send(500, "text/plain", "OTA_IN_PROGRESS");
					return;
				}

				if (g_bTakingTimelapse) {
					pRequest->send(500, "text/plain", "TAKING_TIMELAPSE");
					return;
				}

				if (!SafeSDAccess([&]() {
					g_bTakingSnapshot = true;

					if (digitalRead(PWDN_GPIO_NUM) == HIGH)	// If is off
						digitalWrite(PWDN_GPIO_NUM, LOW);	// turn it on

					sensor_t *pSensorConfig = esp_camera_sensor_get();
					if (pSensorConfig->status.framesize != g_pCameraConfig.frame_size) {
						SetSensorConfig(g_pCameraConfig.frame_size);	// Initial & Timelapse Frame Size

						pSensorConfig->set_dcw(pSensorConfig, 0);
					}

					if (const AsyncWebParameter* pParam = pRequest->getParam("flash")) {
						if (pParam->value() == "1") {
							g_nCurrentLedBrightness = g_nTimelapseLedBrightness;

							ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);

							vTaskDelay(2000 / portTICK_PERIOD_MS);	// 2000ms
						}
					}

					camera_fb_t *pCameraFrameBuffer = esp_camera_fb_get();
					if (!pCameraFrameBuffer) {
						pRequest->send(500, "text/plain", "FRAME_BUFFER");
						return;
					}

					char cFilename[35];
					struct tm currentTime;

					GetLocalTimeNow(&currentTime);

					snprintf(cFilename, sizeof(cFilename), "/snapshots/%02d_%02d_%04d-%02d_%02d_%02d.jpg", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900, currentTime.tm_hour, currentTime.tm_min, currentTime.tm_sec);

					File pFile = SD_MMC.open(cFilename, FILE_WRITE);
					if (pFile) {
						bool bSaved = false;

						if (pFile.write(pCameraFrameBuffer->buf, pCameraFrameBuffer->len) == pCameraFrameBuffer->len)
							bSaved = true;

						pFile.close();

						if (bSaved)
							LOGGER(INFO, "Snapshot save to: %s", cFilename);
					}

					AsyncWebServerResponse *pResponse = pRequest->beginResponse_P(200, "image/jpeg", pCameraFrameBuffer->buf, pCameraFrameBuffer->len);
					pResponse->addHeader("Content-Disposition", "inline; filename=capture.jpg");
					pResponse->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");

					pRequest->onDisconnect([pCameraFrameBuffer]() {
						esp_camera_fb_return(pCameraFrameBuffer);
					});

					g_bTakingSnapshot = false;

					pRequest->send(pResponse);
				})) {
					pRequest->send(500, "text/plain", "NO_SD");
				}

				return;
			}
		}

		pRequest->send(501, "text/plain", "HTTP 501");
	});

	g_pWebServer.onNotFound([](AsyncWebServerRequest *pRequest) {
		if (pRequest->method() == HTTP_OPTIONS)
			pRequest->send(200, "text/plain", "HTTP 200");
		else
			pRequest->send(404, "text/plain", "HTTP 404");
	});

	g_pWebServer.on("/ota", HTTP_POST, [](AsyncWebServerRequest* pRequest) {
		bool bUpdate = !Update.hasError();

		if (bUpdate) {
			LOGGER(INFO, "Restarting Controller to do a Firmware Update.");

			pRequest->send(200, "text/plain", "OTA_SCS");

			bRestart = true;
		} else {
			LOGGER(ERROR, "Final OTA check failed.");

			pRequest->send(500, "text/plain", "OTA_ERR");
		}
	}, [](AsyncWebServerRequest* pRequest, String strFileName, size_t nIndex, uint8_t* nData, size_t nLength, bool bFinal) {
		static bool bUpdateError = false;

		if (!nIndex) {
			bUpdateError = false;

			Update.abort();

			LOGGER(INFO, "Updating Firmware. File: %s", strFileName.c_str());

			if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
				g_nOTAProgress = 0;
				bUpdateError = true;

				LOGGER(ERROR, "Firmware update failed. Error: %s", Update.errorString());
			}
		}

		if (!bUpdateError && Update.write(nData, nLength) != nLength) {
			g_nOTAProgress = 0;
			bUpdateError = true;

			LOGGER(ERROR, "Firmware update failed. Error: %s", Update.errorString());
		} else {
			uint8_t nPercent = (Update.progress() * 100) / Update.size();

			if (nPercent != g_nOTAProgress) {
				g_nOTAProgress = nPercent;

				LOGGER(INFO, "Firmware update written: %d%%", nPercent);
			}
		}

		if (bFinal) {
			if (!bUpdateError && Update.end(true)) {
				LOGGER(INFO, "Firmware Update successfully.");
			} else {
				g_nOTAProgress = 0;

				LOGGER(ERROR, "Firmware update failed. Error: %s", Update.errorString());
			}
		}
	});

	g_pWebServer.begin();

	LOGGER(INFO, "Web Server Started at Port: %u.", SECRET_WEBSERVER_PORT);
}

void loop() {
	static uint64_t nLastSecondTick = 0;
	uint64_t nCurrentMillis = millis64();
	// ================================================== Code execution with 1 second interval Section ================================================== //
	if ((nCurrentMillis - nLastSecondTick) >= 1000) {	// Check if 1 second has passed since the last tick to perform once-per-second tasks
		nLastSecondTick = nCurrentMillis;

		time_t pTimeNow = time(nullptr);
		struct tm currentTime;
		localtime_r(&pTimeNow, &currentTime);
		// ================================================== OTA Section ================================================== //
		if (bRestart) {
			delay(2000);

			ESP.restart();
		}
		// ================================================== WiFi Section ================================================== //
		{
			static uint64_t nLastReconnectAttemptInterval = 0;

			if (eTaskGetState(g_pWiFiReconnect) == eSuspended && WiFi.status() != WL_CONNECTED && (bForceTryConnectWiFi || (nCurrentMillis - nLastReconnectAttemptInterval) >= g_nWiFiRetryConnectInterval)) {
				nLastReconnectAttemptInterval = nCurrentMillis;

				vTaskResume(g_pWiFiReconnect);
			}
		}
		// ================================================== Time Section ================================================== //
		{
			static uint64_t nTimestampSaveInterval = 0;

			if ((nCurrentMillis - nTimestampSaveInterval) >= TIME_SAVE_INTERVAL) {
				nTimestampSaveInterval = nCurrentMillis;

				char cBuffer[11];
				snprintf(cBuffer, sizeof(cBuffer), "%lu", (unsigned long)pTimeNow);
				WriteToSDAtomic("/time", cBuffer);	// Write current time to SD Card
			}
		}
		// ================================================== Timelapse Section ================================================== //
		{
			static uint64_t nTimelapseInterval = 0;

			if ((nCurrentMillis - nTimelapseInterval) >= g_nTimelapseInterval) {
				if ((g_nEffectiveStartTimelapse != g_nEffectiveStopTimelapse) &&
					(g_nEffectiveStartTimelapse < g_nEffectiveStopTimelapse && currentTime.tm_hour >= g_nEffectiveStartTimelapse && currentTime.tm_hour < g_nEffectiveStopTimelapse) || // Normal case: timelapse start time is before stop time (e.g., from 7 AM to 7 PM)
					(g_nEffectiveStartTimelapse >= g_nEffectiveStopTimelapse && (currentTime.tm_hour >= g_nEffectiveStartTimelapse || currentTime.tm_hour < g_nEffectiveStopTimelapse))) {	// Special case: timelapse schedule crosses midnight (e.g., from 8 PM to 6 AM)
					nTimelapseInterval = nCurrentMillis;

					if (g_nOTAProgress > 0) {
						LOGGER(WARN, "Cannot take Snapshot for Timelapse. OTA Update in progress.");
					} else {
						if (g_bTakingSnapshot) {
							LOGGER(WARN, "Cannot take Snapshot for Timelapse. Is Taking Snapshot.");
						} else {
							g_bTakingTimelapse = true;

							if (digitalRead(PWDN_GPIO_NUM) == HIGH)	// If is off
								digitalWrite(PWDN_GPIO_NUM, LOW);	// turn it on

							sensor_t *pSensorConfig = esp_camera_sensor_get();
							if (pSensorConfig->status.framesize != g_pCameraConfig.frame_size) {
								SetSensorConfig(g_pCameraConfig.frame_size);	// Initial & Timelapse Frame Size

								pSensorConfig->set_dcw(pSensorConfig, 0);
							}

							if (g_nTimelapseLedBrightness > 0) {
								g_nCurrentLedBrightness = g_nTimelapseLedBrightness;

								ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);

								vTaskDelay(2000 / portTICK_PERIOD_MS);	// 2000ms
							}

							camera_fb_t *pCameraFrameBuffer = esp_camera_fb_get();
							if (!pCameraFrameBuffer) {
								LOGGER(ERROR, "Frame Buffer Error. Cannot take Snapshot for Timelapse.");
							} else {
								SafeSDAccess([&]() {
									char cFilename[28];
									snprintf(cFilename, sizeof(cFilename), "/timelapse/capture%05d.jpg", g_nTimelapseCounter);

									File pFile = SD_MMC.open(cFilename, FILE_WRITE);
									if (pFile) {
										bool bSaved = false;

										if (pFile.write(pCameraFrameBuffer->buf, pCameraFrameBuffer->len) == pCameraFrameBuffer->len)
											bSaved = true;

										pFile.close();

										if (bSaved) {
											g_nTimelapseCounter++;

											SaveSettings();

											LOGGER(INFO, "Snapshot for Timelapse save to: %s", cFilename);
										}
									}
								});
							}

							esp_camera_fb_return(pCameraFrameBuffer);

							if (g_nTimelapseLedBrightness > 0) {
								g_nCurrentLedBrightness = 0;

								ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);
							}

							g_bTakingTimelapse = false;
						}
					}
				}
			}
		}
		// ================================================== Auto Sensor Shutdown Section ================================================== //
		{
			if ((nCurrentMillis - g_nLastCameraActivity) >= g_nSensorShutdownInterval && !g_bTakingSnapshot && !g_bTakingTimelapse) {
				if (digitalRead(PWDN_GPIO_NUM) == LOW /*If is working*/)
					digitalWrite(PWDN_GPIO_NUM, HIGH);	// turn it off

				if (g_nCurrentLedBrightness > 0) {
					g_nCurrentLedBrightness = 0;

					ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);
				}
			}
		}
	}
}
