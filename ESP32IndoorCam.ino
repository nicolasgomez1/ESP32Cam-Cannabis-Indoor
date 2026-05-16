//          _______________________________________________________________________
//         /                                                                      /\
//        /  _   __    ____   ______   ____     __     ___    _____   ______     / /\
//       /  / | / /   /  _/  / ____/  / __ \   / /    /   |  / ___/  / ____/  __/ /
//      /  /  |/ /    / /   / /      / / / /  / /    / /| |  \__ \  / / __   /\_\/
//     /  / /|  /   _/ /   / /___   / /_/ /  / /___ / ___ | ___/ / / /_/ /  /_/
//    /  /_/ |_/   /___/   \____/   \____/  /_____//_/  |_|/____/  \____/    /\
//   /                           Version 1 (2026)                           / /
//  /______________________________________________________________________/ /
//  \______________________________________________________________________\/
//   \    \    \    \    \    \    \    \    \    \    \    \    \    \     \

#define FIRMWAREVERSION "V1_0515_2259WiP"	// Subfix d (DEBUG), r (RELEASE) & WiP (Work in process)

// TODO: Poner una varible bool para que el código de la función loop no apague el sensor durante un timelapse...
// TODO: Hacer el código del timelapse capturer (los archivos se tienen que llamar capture%05.jpg)

#include <WiFi.h>
#include <SD_MMC.h>
#include <Update.h>
#include <Secrets.h>
#include <HTTPClient.h>
#include <esp_camera.h>
#include <ESPAsyncWebServer.h>

// NOTE: Free pins: 0, 1, 3, 12, 13 & 16

// Definitions
//#define ENABLE_AP_ALWAYS	// Use this to enable always the Access Point. Else it just enable when have no internet connection

#define LOG_QUEUE_SIZE 20
#define LOG_QUEUE_MAX_MSG_LEN 256

#define WIFI_MAX_RETRYS 5	// Max WiFi reconnection attempts
#define WIFI_RETRY_INTERVAL 1000	// 1 second

#define TIMEZONE "ART3"	// POSIX Format

#define TIME_SAVE_INTERVAL 10000	// 10 seconds

#define FLASH_LED_FREQUENCY 20000	// 20kHz
#define FLASH_LED_RESOLUTION 8

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
	IDX_AWB_GAIN,
	IDX_AEC_ENABLE,
	IDX_AEC2_NIGHT,
	IDX_AE_LEVEL,
	IDX_AEC_VALUE,
	IDX_AGC_ENABLE,
	IDX_AGC_GAIN,
	IDX_GAIN_CEILING,
	IDX_BPC,
	IDX_WPC,
	IDX_RAW_GAMMA,
	IDX_LENS_CORR,
	IDX_H_FLIP,
	IDX_V_FLIP,
	IDX_DCW,
	IDX_COLORBAR,
	IDX_COUNT
};

// DO NOT TOUCH IT!
enum ERR_TYPE { INFO, WARN, ERROR };

struct LogMessage {
	char cBuffer[LOG_QUEUE_MAX_MSG_LEN];
	char cFileName[29];
};

// Settings Variables
char g_cSSID[16];
char g_cSSIDPWD[16];
uint64_t g_nWiFiRetryConnectInterval = 0;
bool g_bWiFiSleep = true;
wifi_power_t g_pWiFiPower;
uint64_t g_nSensorShutdownInterval = 0;
uint8_t g_nEffectiveStartTimelapse = 0;
uint8_t g_nEffectiveStopTimelapse = 0;
uint64_t g_nTimelapseInterval = 0;
uint8_t g_nTimelapseLedBrightness = 0;
uint8_t g_nMonitoringLedBrightness = 0;
camera_config_t g_pCameraConfig;
camera_status_t g_pSensorStatus;

// Internal Variables
uint64_t g_nLastCameraActivity = 0;
uint8_t g_nCurrentLedBrightness = 0;
framesize_t g_CurrentFrameSize;
bool bTakingSnapshot = false;
uint8_t g_nOTAProgress = 0;

// Global Handles, Interface & Instances
AsyncWebServer g_pWebServer(SECRET_WEBSERVER_PORT);	// Asynchronous web server instance listening on SECRET_WEBSERVER_PORT
TaskHandle_t g_pWiFiReconnect;											// Task handle for WiFi reconnect logic running on core 0
SemaphoreHandle_t g_pSDMutex;												// Mutex to synchronize concurrent access to the SD card across tasks
QueueHandle_t g_pLogQueue;													// Queue handle for asynchronous logging to decouple SD writes from main logic
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SET_BIT_TO_MASK(uint64_t &nMask, uint8_t nBit) { nMask |= (1ULL << nBit); }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline uint64_t millis64() { return esp_timer_get_time() / 1000ULL; }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sets the system time and timezone based on a given Unix timestamp.
// Updates the system's internal clock to the provided timestamp (seconds since epoch).
void SetCurrentDatetime(time_t nTimestamp) {
	struct timeval tv;
	tv.tv_sec = nTimestamp;
	tv.tv_usec = 0;

	settimeofday(&tv, nullptr);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Retrieves the current local time and stores it in the provided tm struct.
// Uses the system time (UTC) and converts it to local time based on the configured timezone.
// The result is stored in the pTimeInfo pointer passed by the caller.
void GetLocalTimeNow(struct tm* pTimeInfo) {
	time_t pTimeNow = time(nullptr);
	localtime_r(&pTimeNow, pTimeInfo);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reads a line from the given file stream into the provided buffer.
// Ensures the buffer is null-terminated and trims trailing whitespace (e.g., spaces, tabs, newlines).
// - pFile: reference to the open File object.
// - cBuffer: pointer to the destination character buffer (must be large enough to hold the line).
// - nBufferSize: size of the destination buffer (including null terminator).
void ReadFromStream(File& pFile, char* cBuffer, size_t nBufferSize) {
	size_t nLength = pFile.readBytesUntil('\n', cBuffer, nBufferSize - 1);

	cBuffer[nLength] = '\0';

	while (nLength > 0 && isspace(cBuffer[nLength - 1]))
		cBuffer[--nLength] = '\0';
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Provides utility functions to convert between ticks (milliseconds) and human-readable time units.
// Ticks are assumed to be in milliseconds, as returned by the millis64() function.
inline uint64_t TicksToSeconds(uint64_t nTicks) { return nTicks / 1000; }
inline uint64_t TicksToMinutes(uint64_t nTicks) { return nTicks / (1000 * 60); }

inline uint32_t SecondsToTicks(uint32_t nSeconds) { return nSeconds * 1000; }
inline uint32_t MinutesToTicks(uint32_t nMinutes) { return nMinutes * 1000 * 60; }
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
// - cFileName: Path of the file to write to.
// - cBuffer: Text string to be written.
// - bAppend: If true, appends to the file; otherwise, overwrites it.
void WriteToSD(const char* cFileName, const char* cBuffer, bool bAppend) {
	SafeSDAccess([&]() {
		File pFile = SD_MMC.open(cFileName, bAppend ? FILE_APPEND : FILE_WRITE);
		if (pFile) {
			pFile.println(cBuffer);

			pFile.close();
		}
	});
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Writes a text string to a file on the SD card using an atomic write pattern, always wrapped in SafeSDAccess for thread-safe access.
// Writes to a temporary file first, verifies the content matches the input, then replaces the target file.
// If verification fails, the temporary file is removed and the target file is left unchanged.
// - cFileName: Path of the file to write to.
// - cBuffer: Text string to be written.
void WriteToSDAtomic(const char* cFileName, const char* cBuffer) {
	SafeSDAccess([&]() {
		char cTempFileName[64];
		snprintf(cTempFileName, sizeof(cTempFileName), "%s.atomic", cFileName);

		File pTempFile = SD_MMC.open(cTempFileName, FILE_WRITE);
		if (!pTempFile)
			return;

		pTempFile.print(cBuffer);
		pTempFile.close();

		File pFile = SD_MMC.open(cTempFileName, FILE_READ);
		if (!pFile) {
			SD_MMC.remove(cTempFileName);
			return;
		}

		char cContent[strlen(cBuffer) + 1] = {};
		pFile.readBytes(cContent, sizeof(cContent) - 1);

		pFile.close();

		if (strcmp(cContent, cBuffer) != 0) {
			SD_MMC.remove(cTempFileName);
			return;
		}

		SD_MMC.remove(cFileName);
		SD_MMC.rename(cTempFileName, cFileName);
	});
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Logs a formatted message with severity and timestamp to a daily SD log file via an async queue.
// Parameters:
// - nType: Log severity (INFO, WARN, ERROR).
// - cFormat: printf-style format string with optional arguments.
// Behavior:
// - Prepends timestamp (DD/MM/YYYY HH:MM:SS) and severity tag ([INFO], [WARN], [ERROR]).
// - Enqueues the formatted message for writing to a daily log file (/logs/logging_DD_MM_YYYY.txt).
// - Non-blocking: if the queue is full, the message is silently dropped.
void LOGGER(ERR_TYPE nType, const char* cFormat, ...) {
	LogMessage pMSG;
	char cPrintType[9];
	struct tm currentTime;

	GetLocalTimeNow(&currentTime);

	switch (nType) {
		case INFO:	snprintf(cPrintType, sizeof(cPrintType), "[INFO] ");	break;
		case WARN:	snprintf(cPrintType, sizeof(cPrintType), "[WARN] ");	break;
		case ERROR:	snprintf(cPrintType, sizeof(cPrintType), "[ERROR] ");	break;
	}

	uint8_t nOffset = snprintf(pMSG.cBuffer, sizeof(pMSG.cBuffer), "%02d/%02d/%04d %02d:%02d:%02d %s", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900, currentTime.tm_hour, currentTime.tm_min, currentTime.tm_sec, cPrintType);

	va_list args;
	va_start(args, cFormat);
	vsnprintf(pMSG.cBuffer + nOffset, sizeof(pMSG.cBuffer) - nOffset, cFormat, args);
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
			pSettingsFile.println(g_pSensorStatus.sharpness);		// Line 20
			pSettingsFile.println(g_pSensorStatus.denoise);
			pSettingsFile.println(g_pSensorStatus.special_effect);
			pSettingsFile.println(g_pSensorStatus.wb_mode);
			pSettingsFile.println(g_pSensorStatus.awb);
			pSettingsFile.println(g_pSensorStatus.awb_gain);
			pSettingsFile.println(g_pSensorStatus.aec);
			pSettingsFile.println(g_pSensorStatus.aec2);
			pSettingsFile.println(g_pSensorStatus.ae_level);
			pSettingsFile.println(g_pSensorStatus.aec_value);
			pSettingsFile.println(g_pSensorStatus.agc);					// Line 30
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
// If ENABLE_AP_ALWAYS is not defined:
// - Starts a temporary Access Point (SECRET_ACCESSPOINT_NAME) to allow reconfiguration during reconnection attempts.
// - Once connected, the Access Point is shut down and the mode is switched to station-only.
// Tries to reconnect up to WIFI_MAX_RETRYS times, with a 1-second delay between attempts.
// Logs a success message with IP address upon connection, or an error message if all attempts fail.
// After completion (regardless of success), the task is suspended until explicitly resumed elsewhere.
void Thread_WiFiReconnect(void*) {
	for (;;) {
#if !defined(ENABLE_AP_ALWAYS)
		if (!(WiFi.getMode() & WIFI_AP)) {
			LOGGER(INFO, "Starting Access Point (SSID: %s) mode for reconfiguration...", SECRET_ACCESSPOINT_NAME);

			WiFi.mode(WIFI_AP_STA);	// Set dual mode, Access Point & Station

			vTaskDelay(100 / portTICK_PERIOD_MS);	// Delay to stabilize AP

			WiFi.softAP(SECRET_ACCESSPOINT_NAME);	// Start Access Point, while try to connect to WiFi
		}
#endif

		WiFi.disconnect(true);

		LOGGER(INFO, "Trying to reconnect WiFi...");

		WiFi.begin(g_cSSID, g_cSSIDPWD);
		WiFi.setTxPower(g_pWiFiPower);
		WiFi.setSleep(g_bWiFiSleep);

		uint8_t nConnectTrysCount = 0;

		while (nConnectTrysCount < WIFI_MAX_RETRYS && WiFi.status() != WL_CONNECTED) {
			nConnectTrysCount++;

			vTaskDelay(WIFI_RETRY_INTERVAL / portTICK_PERIOD_MS);	// Wait before trying again
		}

		if (WiFi.status() == WL_CONNECTED) {
			LOGGER(INFO, "Connected to WiFi SSID: %s PASSWORD: %s. IP: %s.", g_cSSID, g_cSSIDPWD, WiFi.localIP().toString().c_str());

#if !defined(ENABLE_AP_ALWAYS)
			WiFi.softAPdisconnect(true);
			WiFi.mode(WIFI_STA);

			LOGGER(INFO, "Access Point disconnected.");
#endif
		} else {
			LOGGER(ERROR, "Max WiFi reconnect attempts reached.");
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
void SetSensorConfig(framesize_t FrameSize) {
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
		const char* cReasons[] = { "Unknown", "Power on", "External pin", "Software", "Panic/Exception", "Interrupt watchdog", "Task watchdog", "Other watchdog", "Deepsleep", "Brownout", "SDIO" };

		LOGGER(WARN, "Reset reason: %s", cReasons[pReason]);
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

	LOGGER(INFO, "Pins for Camera Done!");

	ledcAttach(LED_GPIO_NUM, FLASH_LED_FREQUENCY, FLASH_LED_RESOLUTION);
	ledcWrite(LED_GPIO_NUM, 0);

	LOGGER(INFO, "Pin for Flash LED Done!");

	LOGGER(INFO, "Loading Settings & Time...");

	if (!SafeSDAccess([&]() {
		File pSettingsFile = SD_MMC.open("/settings", FILE_READ); // Read Settings File
		if (pSettingsFile) {
			char cBuffer[64];
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));  // SSID

			strncpy(g_cSSID, cBuffer, sizeof(g_cSSID));
			g_cSSID[sizeof(g_cSSID) - 1] = '\0';
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));  // SSID PASSWORD

			strncpy(g_cSSIDPWD, cBuffer, sizeof(g_cSSIDPWD));
			g_cSSIDPWD[sizeof(g_cSSIDPWD) - 1] = '\0';
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));  // WIFI RETRY CONNECT INTERVAL
			g_nWiFiRetryConnectInterval = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));  // WIFI SLEEP
			g_bWiFiSleep = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));  // WIFI TRANSMIT POWER
			g_pWiFiPower = (wifi_power_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// INTERVAL FROM LAST WEB CONNECTION TO SHUTDOWN THE SENSOR & FLASH LED
			g_nSensorShutdownInterval = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// START TIMELAPSE TIME
			uint8_t nValue = atoi(cBuffer);

			g_nEffectiveStartTimelapse = (nValue == 24) ? 0 : nValue;	// Stores the effective timelapse start hour, converting hour 24 to 0 (midnight)
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// STOP TIMELAPSE TIME
			nValue = atoi(cBuffer);

			g_nEffectiveStopTimelapse = (nValue == 24) ? 0 : nValue;	// Stores the effective timelapse stop hour, converting hour 24 to 0 (midnight)
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// TIMELAPSE CAPTURE INTERVAL
			g_nTimelapseInterval = atoi(cBuffer);
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
			///////////////////////////////////////////////////				// PIXEL FORMAT
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));
			g_pCameraConfig.pixel_format = (pixformat_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// IMAGE RESOLUTION	(Initial & Timelapse Frame Size)
			g_pCameraConfig.frame_size = (framesize_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// COMPRESSION LEVEL
			g_pCameraConfig.jpeg_quality = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// FRAME BUFFERS COUNT
			g_pCameraConfig.fb_count = atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// STORE FRAME IN
			g_pCameraConfig.fb_location = (camera_fb_location_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// FRAME TO CAPTURE
			g_pCameraConfig.grab_mode = (camera_grab_mode_t)atoi(cBuffer);
			///////////////////////////////////////////////////
			//conv_mode																								// CONVERT COLOR FORMAT
			///////////////////////////////////////////////////
			//quality																									// COMPRESSION LEVEL
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
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// AUTOMATIC EXPOSURE ALGORITHM (NIGHT MODE)
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
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// MANUAL GAIN LEVEL
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
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// VERTICAL MIRRORING
			g_pSensorStatus.vflip = atoi(cBuffer);
			//////////////////////////////////////////////////
			ReadFromStream(pSettingsFile, cBuffer, sizeof(cBuffer));	// DIGITAL SOFT WHITE BALANCE
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

			String strRawTime = pTimeFile.readStringUntil('\n');

			LOGGER(INFO, "Time file raw content: '%s', length: %d.", strRawTime.c_str(), strRawTime.length());

			time_t nTime = strRawTime.toInt();

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
	})) {
		LOGGER(ERROR, "SD initialization failed. Settings & Time will not be loaded, but the system will not restart to avoid unexpected relay behavior.");
	}

	LOGGER(INFO, "Initializing Camera...");

	esp_err_t pCameraError = esp_camera_init(&g_pCameraConfig);
	if (pCameraError != ESP_OK) {
		LOGGER(ERROR, "Camera init failed. Error: 0x%x", pCameraError);
	} else {
		digitalWrite(PWDN_GPIO_NUM, HIGH);	// Pos esp_camera_init call

		LOGGER(INFO, "Turning Off Camera Done!");
	}

	LOGGER(INFO, "Initializing WiFi...");

#ifdef ENABLE_AP_ALWAYS
	WiFi.mode(WIFI_AP_STA);	// Set dual mode, Access Point & Station

	WiFi.softAP(SECRET_ACCESSPOINT_NAME);

	LOGGER(INFO, "Access Point active. AP IP: %s", WiFi.softAPIP().toString().c_str());
#else
	WiFi.mode(WIFI_STA);	// Only Station mode
#endif

	if (g_cSSID[0] != '\0') {
		WiFi.begin(g_cSSID, g_cSSIDPWD);
		WiFi.setTxPower(g_pWiFiPower);
		WiFi.setSleep(g_bWiFiSleep);

		uint8_t nConnectTrysCount = 0;

		while (nConnectTrysCount < WIFI_MAX_RETRYS && WiFi.status() != WL_CONNECTED) {
			nConnectTrysCount++;

			delay(WIFI_RETRY_INTERVAL);	// Wait before trying again
		}

		if (WiFi.status() == WL_CONNECTED)
			LOGGER(INFO, "Connected to WiFi SSID: %s PASSWORD: %s. IP: %s.", g_cSSID, g_cSSIDPWD, WiFi.localIP().toString().c_str());
		else
			LOGGER(ERROR, "Max WiFi reconnect attempts reached.");
	}

	LOGGER(INFO, "Creating WiFi reconnect task thread...");

	xTaskCreatePinnedToCore(Thread_WiFiReconnect, "WiFiReconnectTask", 4096, NULL, 1, &g_pWiFiReconnect, 0);
	vTaskSuspend(g_pWiFiReconnect);	// Suspend the task as it's not needed right now

	LOGGER(INFO, "Creating Logging task thread...");

	xTaskCreate(Thread_LogProcessor, "LoggingTask", 4096, NULL, 1, NULL);

	LOGGER(INFO, "Setting up Web Server...");

	// Static files server
	// Static serving of the logs folders and all the files inside of them
	g_pWebServer.serveStatic("/logs", SD_MMC, "/logs").setCacheControl("max-age=2592000, immutable");	// Cache it by 1 month

	// Request handler
	g_pWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest* pRequest) {
		if (pRequest->hasArg("action")) {	// Process the request
			if (pRequest->arg("action") == "restart") {
				LOGGER(INFO, "Restarting Controller by Web command.");

				delay(1000);

				ESP.restart();
			} else if (pRequest->arg("action") == "update") {	// This is for update Settings
				uint64_t nNewValue;
				uint64_t nSuccessCodeMask = 0;
				uint64_t nErrorCodeMask = 0;
				bool bWiFiChanges = false;
				sensor_t *pSensorConfig = esp_camera_sensor_get();

				// =============== Current DateTime =============== //
				if (pRequest->hasArg("time")) {
					SetCurrentDatetime(pRequest->arg("time").toInt());

					struct tm currentTime;
					GetLocalTimeNow(&currentTime);
					LOGGER(INFO, "New Datetime: %d/%d/%04d %02d:%02d:%02d.", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900, currentTime.tm_hour, currentTime.tm_min, currentTime.tm_sec);

					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TIME);
				}
				// =============== WiFi =============== //
				if (pRequest->hasArg("ssid") && strcmp(pRequest->arg("ssid").c_str(), g_cSSID) != 0) {
					bWiFiChanges = true;

					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_SSID);
				}

				if (pRequest->hasArg("ssidpwd") && strcmp(pRequest->arg("ssidpwd").c_str(), g_cSSIDPWD) != 0) {
					bWiFiChanges = true;

					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_PWD);
				}

				if (bWiFiChanges)
					SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_STA);

				if (pRequest->hasArg("wfrci")) {
					nNewValue = MinutesToTicks(pRequest->arg("wfrci").toInt());

					if (nNewValue != g_nWiFiRetryConnectInterval) {
						g_nWiFiRetryConnectInterval = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_RETRY);
					}
				}

				if (pRequest->hasArg("ws")) {
					nNewValue = pRequest->arg("ws").toInt();

					if (nNewValue != g_bWiFiSleep) {
						g_bWiFiSleep = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_SLEEP);
					}
				}

				if (pRequest->hasArg("wp")) {
					nNewValue = pRequest->arg("wp").toInt();

					if ((wifi_power_t)nNewValue != g_pWiFiPower) {
						g_pWiFiPower = (wifi_power_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WIFI_POWER);
					}
				}
				// =============== SENSOR SHUTDOWN INTERVAL =============== //
				if (pRequest->hasArg("si")) {
					nNewValue = SecondsToTicks(pRequest->arg("si").toInt());

					if (nNewValue != g_nSensorShutdownInterval) {
						g_nSensorShutdownInterval = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SENSOR_SHUTDOWN_INTERVAL);
					}
				}
				// =============== START TIMELAPSE TIME =============== //
				if (pRequest->hasArg("timelapsestart")) {
					nNewValue = pRequest->arg("timelapsestart").toInt();

					if (nNewValue != g_nEffectiveStartTimelapse) {
						g_nEffectiveStartTimelapse = (nNewValue == 24) ? 0 : nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TL_START);
					}
				}
				// =============== STOP TIMELAPSE TIME =============== //
				if (pRequest->hasArg("timelapsestop")) {
					nNewValue = pRequest->arg("timelapsestop").toInt();

					if (nNewValue != g_nEffectiveStopTimelapse) {
						g_nEffectiveStopTimelapse = (nNewValue == 24) ? 0 : nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TL_STOP);
					}
				}
				// =============== TIMELAPSE CAPTURE INTERVAL =============== //
				if (pRequest->hasArg("timelapseinterval")) {
					nNewValue = MinutesToTicks(pRequest->arg("timelapseinterval").toInt());

					if (nNewValue != g_nTimelapseInterval) {
						g_nTimelapseInterval = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_TL_INTERVAL);
					}
				}
				// =============== TIMELAPSE FLASH LED BRIGHTNESS =============== //
				if (pRequest->hasArg("lbt")) {
					nNewValue = pRequest->arg("lbt").toInt();

					if (nNewValue != g_nTimelapseLedBrightness) {
						if (g_nCurrentLedBrightness == g_nTimelapseLedBrightness) {	// If is currently use Flash, update it brightness in real time
							g_nCurrentLedBrightness = nNewValue;

							ledcWrite(LED_GPIO_NUM, nNewValue);
						}

						g_nTimelapseLedBrightness = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_LED_BRIGHT_TIMELAPSE);
					}
				}
				// =============== MONITORING FLASH LED BRIGHTNESS =============== //
				if (pRequest->hasArg("lbm")) {
					nNewValue = pRequest->arg("lbm").toInt();

					if (nNewValue != g_nMonitoringLedBrightness) {
						if (g_nCurrentLedBrightness == g_nMonitoringLedBrightness) {	// If is currently use Flash, update it brightness in real time
							g_nCurrentLedBrightness = nNewValue;

							ledcWrite(LED_GPIO_NUM, nNewValue);
						}

						g_nMonitoringLedBrightness = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_LED_BRIGHT_MONITORING);
					}
				}
				// =============== CAMERA MASTER CLOCK (XCLK) =============== //
				if (pRequest->hasArg("xclk")) {
					nNewValue = pRequest->arg("xclk").toInt();

					if ((nNewValue * 1000000U) != g_pCameraConfig.xclk_freq_hz) {
						g_pCameraConfig.xclk_freq_hz = (nNewValue * 1000000U);

						if (pSensorConfig->set_xclk(pSensorConfig, LEDC_TIMER_0, nNewValue) == 0)
							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_XCLK);
						else
							SET_BIT_TO_MASK(nErrorCodeMask, IDX_XCLK);
					}
				}
				// =============== PIXEL FORMAT =============== //
				if (pRequest->hasArg("pf")) {
					nNewValue = pRequest->arg("pf").toInt();

					if ((pixformat_t)nNewValue != g_pCameraConfig.pixel_format) {
						g_pCameraConfig.pixel_format = (pixformat_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_PIXFORMAT);
					}
				}
				// =============== IMAGE RESOLUTION (Inicial & Timelapse) =============== //
				if (pRequest->hasArg("ifs")) {
					nNewValue = pRequest->arg("ifs").toInt();

					if ((framesize_t)nNewValue != g_pCameraConfig.frame_size) {
						g_pCameraConfig.frame_size = (framesize_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_IFS_RESOLUTION);
					}
				}
				// =============== COMPRESSION LEVEL =============== //
				if (pRequest->hasArg("jpegq")) {
					nNewValue = pRequest->arg("jpegq").toInt();

					if (nNewValue != g_pCameraConfig.jpeg_quality) {
						g_pCameraConfig.jpeg_quality = nNewValue;

						if (pSensorConfig->set_quality(pSensorConfig, nNewValue) == 0)
							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_JPEG_QUALITY);
						else
							SET_BIT_TO_MASK(nErrorCodeMask, IDX_JPEG_QUALITY);
					}
				}
				// =============== FRAME BUFFERS COUNT =============== //
				if (pRequest->hasArg("fbc")) {
					nNewValue = pRequest->arg("fbc").toInt();

					if (nNewValue != g_pCameraConfig.fb_count) {
						g_pCameraConfig.fb_count = nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_FB_COUNT);
					}
				}
				// =============== STORE FRAME IN =============== //
				if (pRequest->hasArg("fbl")) {
					nNewValue = pRequest->arg("fbl").toInt();

					if ((camera_fb_location_t)nNewValue != g_pCameraConfig.fb_location) {
						g_pCameraConfig.fb_location = (camera_fb_location_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_FB_LOCATION);
					}
				}
				// =============== FRAME TO CAPTURE =============== //
				if (pRequest->hasArg("gm")) {
					nNewValue = pRequest->arg("gm").toInt();

					if ((camera_grab_mode_t)nNewValue != g_pCameraConfig.grab_mode) {
						g_pCameraConfig.grab_mode = (camera_grab_mode_t)nNewValue;

						SET_BIT_TO_MASK(nSuccessCodeMask, IDX_GRAB_MODE);
					}
				}
				// =============== IMAGE RESOLUTION (Monitoring) =============== //
				if (pRequest->hasArg("mfs")) {
					nNewValue = pRequest->arg("mfs").toInt();

					if ((framesize_t)nNewValue != g_pSensorStatus.framesize) {
						if (pSensorConfig->set_framesize(pSensorConfig, (framesize_t)nNewValue) == 0) {
							g_pSensorStatus.framesize = (framesize_t)nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_MFS_RESOLUTION);
						} else {
							SET_BIT_TO_MASK(nErrorCodeMask, IDX_MFS_RESOLUTION);
						}
					}
				}
				// =============== BRIGHTNESS LEVEL =============== //
				if (pRequest->hasArg("bnl")) {
					nNewValue = pRequest->arg("bnl").toInt();

					if (nNewValue != g_pSensorStatus.brightness) {
						if (pSensorConfig->set_brightness(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.brightness = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_BRIGHTNESS);
						}
					}
				}
				// =============== CONTRAST LEVEL =============== //
				if (pRequest->hasArg("cl")) {
					nNewValue = pRequest->arg("cl").toInt();

					if (nNewValue != g_pSensorStatus.contrast) {
						if (pSensorConfig->set_contrast(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.contrast = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_CONTRAST);
						}
					}
				}
				// =============== SATURATION LEVEL =============== //
				if (pRequest->hasArg("sl")) {
					nNewValue = pRequest->arg("sl").toInt();

					if (nNewValue != g_pSensorStatus.saturation) {
						if (pSensorConfig->set_saturation(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.saturation = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SATURATION);
						}
					}
				}
				// =============== SHARPNESS LEVEL =============== //
				if (pRequest->hasArg("snl")) {
					nNewValue = pRequest->arg("snl").toInt();

					if (nNewValue != g_pSensorStatus.sharpness) {
						if (pSensorConfig->set_sharpness(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.sharpness = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SHARPNESS);
						}
					}
				}
				// =============== NOISE REDUCTION LEVEL =============== //
				if (pRequest->hasArg("nrl")) {
					nNewValue = pRequest->arg("nrl").toInt();

					if (nNewValue != g_pSensorStatus.denoise) {
						if (pSensorConfig->set_denoise(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.denoise = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_DENOISE);
						}
					}
				}
				// =============== SPECIAL EFFECTS =============== //
				if (pRequest->hasArg("se")) {
					nNewValue = pRequest->arg("se").toInt();

					if (nNewValue != g_pSensorStatus.special_effect) {
						if (pSensorConfig->set_special_effect(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.special_effect = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_SPECIAL_EFFECT);
						}
					}
				}
				// =============== AUTOMATIC WHITE BALANCE PROFILE =============== //
				if (pRequest->hasArg("wbp")) {
					nNewValue = pRequest->arg("wbp").toInt();

					if (nNewValue != g_pSensorStatus.wb_mode) {
						if (pSensorConfig->set_wb_mode(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.wb_mode = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WB_MODE);
						}
					}
				}
				// =============== AUTOMATIC WHITE BALANCE =============== //
				if (pRequest->hasArg("awb")) {
					nNewValue = pRequest->arg("awb").toInt();

					if (nNewValue != g_pSensorStatus.awb) {
						if (pSensorConfig->set_whitebal(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.awb = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AWB_ENABLE);
						}
					}
				}
				// =============== AUTOMATIC WHITE BALANCE GAIN =============== //
				if (pRequest->hasArg("awbg")) {
					nNewValue = pRequest->arg("awbg").toInt();

					if (nNewValue != g_pSensorStatus.awb_gain) {
						if (pSensorConfig->set_awb_gain(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.awb_gain = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AWB_GAIN);
						}
					}
				}
				// =============== AUTOMATIC EXPOSURE =============== //
				if (pRequest->hasArg("ae")) {
					nNewValue = pRequest->arg("ae").toInt();

					if (nNewValue != g_pSensorStatus.aec) {
						if (pSensorConfig->set_exposure_ctrl(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.aec = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AEC_ENABLE);
						}
					}
				}
				// =============== AUTOMATIC EXPOSURE ALGORITHM (NIGHT MODE) =============== //
				if (pRequest->hasArg("ae2")) {
					nNewValue = pRequest->arg("ae2").toInt();

					if (nNewValue != g_pSensorStatus.aec2) {
						if (pSensorConfig->set_aec2(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.aec2 = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AEC2_NIGHT);
						}
					}
				}
				// =============== AUTOMATIC EXPOSURE LEVEL =============== //
				if (pRequest->hasArg("ael")) {
					nNewValue = pRequest->arg("ael").toInt();

					if (nNewValue != g_pSensorStatus.ae_level) {
						if (pSensorConfig->set_ae_level(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.ae_level = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AE_LEVEL);
						}
					}
				}
				// =============== MANUAL EXPOSURE LEVEL =============== //
				if (pRequest->hasArg("aev")) {
					nNewValue = pRequest->arg("aev").toInt();

					if (nNewValue != g_pSensorStatus.aec_value) {
						if (pSensorConfig->set_aec_value(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.aec_value = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AEC_VALUE);
						}
					}
				}
				// =============== AUTOMATIC GAIN =============== //
				if (pRequest->hasArg("agc")) {
					nNewValue = pRequest->arg("agc").toInt();

					if (nNewValue != g_pSensorStatus.agc) {
						if (pSensorConfig->set_gain_ctrl(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.agc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AGC_ENABLE);
						}
					}
				}
				// =============== MANUAL GAIN LEVEL =============== //
				if (pRequest->hasArg("agcl")) {
					nNewValue = pRequest->arg("agcl").toInt();

					if (nNewValue != g_pSensorStatus.agc_gain) {
						if (pSensorConfig->set_agc_gain(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.agc_gain = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_AGC_GAIN);
						}
					}
				}
				// =============== MAX AUTOMATIC GAIN LEVEL =============== //
				if (pRequest->hasArg("gc")) {
					nNewValue = pRequest->arg("gc").toInt();

					if ((gainceiling_t)nNewValue != g_pSensorStatus.gainceiling) {
						if (pSensorConfig->set_gainceiling(pSensorConfig, (gainceiling_t)nNewValue) == 0) {
							g_pSensorStatus.gainceiling = (gainceiling_t)nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_GAIN_CEILING);
						}
					}
				}
				// =============== BLACK PIXEL CANCELLATION =============== //
				if (pRequest->hasArg("bpc")) {
					nNewValue = pRequest->arg("bpc").toInt();

					if (nNewValue != g_pSensorStatus.bpc) {
						if (pSensorConfig->set_bpc(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.bpc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_BPC);
						}
					}
				}
				// =============== WHITE PIXEL CANCELLATION =============== //
				if (pRequest->hasArg("wpc")) {
					nNewValue = pRequest->arg("wpc").toInt();

					if (nNewValue != g_pSensorStatus.wpc) {
						if (pSensorConfig->set_wpc(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.wpc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_WPC);
						}
					}
				}
				// =============== RAW GAMMA CORRECTION =============== //
				if (pRequest->hasArg("rgc")) {
					nNewValue = pRequest->arg("rgc").toInt();

					if (nNewValue != g_pSensorStatus.raw_gma) {
						if (pSensorConfig->set_raw_gma(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.raw_gma = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_RAW_GAMMA);
						}
					}
				}
				// =============== VIGNETTE CORRECTION =============== //
				if (pRequest->hasArg("lenc")) {
					nNewValue = pRequest->arg("lenc").toInt();

					if (nNewValue != g_pSensorStatus.lenc) {
						if (pSensorConfig->set_lenc(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.lenc = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_LENS_CORR);
						}
					}
				}
				// =============== HORIZONTAL MIRRORING =============== //
				if (pRequest->hasArg("hflip")) {
					nNewValue = pRequest->arg("hflip").toInt();

					if (nNewValue != g_pSensorStatus.hmirror) {
						if (pSensorConfig->set_hmirror(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.hmirror = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_H_FLIP);
						}
					}
				}
				// =============== VERTICAL MIRRORING =============== //
				if (pRequest->hasArg("vflip")) {
					nNewValue = pRequest->arg("vflip").toInt();

					if (nNewValue != g_pSensorStatus.vflip) {
						if (pSensorConfig->set_vflip(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.vflip = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_V_FLIP);
						}
					}
				}
				// =============== DIGITAL SOFT WHITE BALANCE =============== //
				if (pRequest->hasArg("dcw")) {
					nNewValue = pRequest->arg("dcw").toInt();

					if (nNewValue != g_pSensorStatus.dcw) {
						if (pSensorConfig->set_dcw(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.dcw = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_DCW);
						}
					}
				}
				// =============== COLOR BARS (TEST MODE) =============== //
				if (pRequest->hasArg("cb")) {
					nNewValue = pRequest->arg("cb").toInt();

					if (nNewValue != g_pSensorStatus.colorbar) {
						if (pSensorConfig->set_colorbar(pSensorConfig, nNewValue) == 0) {
							g_pSensorStatus.colorbar = nNewValue;

							SET_BIT_TO_MASK(nSuccessCodeMask, IDX_COLORBAR);
						}
					}
				}
				// =============== FLASH LED CONTROL =============== //
				if (pRequest->hasArg("fls")) {
					nNewValue = 0;

					if (g_nCurrentLedBrightness == 0) {
						nNewValue = pRequest->arg("fls").toInt();

						if (nNewValue == 0)	// Timelapse
							nNewValue = g_nTimelapseLedBrightness;
						else								// Monitoring
							nNewValue = g_nMonitoringLedBrightness;
					}

					g_nCurrentLedBrightness = nNewValue;

					ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);
				}
				//////////////////////////////////////////////////
				if (nSuccessCodeMask != 0 || nErrorCodeMask != 0) {	// If have some change, send response to web client and finally save new settings values
					pRequest->send(200, F("text/plain"), "R" + String(nSuccessCodeMask) + "," + String(nErrorCodeMask));

					if (bWiFiChanges) { // Update WiFi values after response the request. in otherwise the message is not sended.
						strncpy(g_cSSID, pRequest->arg("ssid").c_str(), sizeof(g_cSSID) - 1);
						g_cSSID[sizeof(g_cSSID) - 1] = '\0';

						strncpy(g_cSSIDPWD, pRequest->arg("ssidpwd").c_str(), sizeof(g_cSSIDPWD) - 1);
						g_cSSIDPWD[sizeof(g_cSSIDPWD) - 1] = '\0';
					}

					SaveSettings();

					if (bWiFiChanges) { // After send response to web client, Try reconnect to WiFi if is required
						LOGGER(INFO, "Disconnecting WiFi to start connection to new SSID...");

						WiFi.disconnect(false); // First disconnect from current Network (Arg false to just disconnect the Station, not the AP)

						if (eTaskGetState(g_pWiFiReconnect) != eSuspended)
							vTaskSuspend(g_pWiFiReconnect);
					}

					return;
				}
			} else if (pRequest->arg("action") == "refresh") { // This is for refresh Panel values constantly
				// ================================================== Current Time Section ================================================== //
				time_t pTimeNow = time(nullptr);

				String strResponse = ":" + String(pTimeNow);
				// ================================================== Firmware Versioning & OTA Update Progress Section ================================================== //
				strResponse += ":" + String(FIRMWAREVERSION) + ":" + String(g_nOTAProgress);
				// ================================================== WiFi Section ================================================== //
				strResponse += ":" + String(g_cSSID) + ":" + String(g_cSSIDPWD) + ":" + String(g_nWiFiRetryConnectInterval) + ":" + String(g_bWiFiSleep) + ":" + String(g_pWiFiPower);
				// ================================================== Shutdown Camera & Flash LED Section ================================================== //
				strResponse += ":" + String(TicksToSeconds(g_nSensorShutdownInterval));
				// TODO: Acá hay que retornar toda la configuración de la SD
				// ================================================== Timelapse Section ================================================== //
				strResponse += ":" + String(((g_nEffectiveStartTimelapse == 0) ? 24 : g_nEffectiveStartTimelapse)) + ":" + String(((g_nEffectiveStopTimelapse == 0) ? 24 : g_nEffectiveStopTimelapse)) + ":" + String(TicksToMinutes(g_nTimelapseInterval)) + ":" + String(g_nTimelapseLedBrightness);
				// ================================================== TODO: .... Section ================================================== //
/*
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
			pSettingsFile.println(g_pSensorStatus.sharpness);		// Line 20
			pSettingsFile.println(g_pSensorStatus.denoise);
			pSettingsFile.println(g_pSensorStatus.special_effect);
			pSettingsFile.println(g_pSensorStatus.wb_mode);
			pSettingsFile.println(g_pSensorStatus.awb);
			pSettingsFile.println(g_pSensorStatus.awb_gain);
			pSettingsFile.println(g_pSensorStatus.aec);
			pSettingsFile.println(g_pSensorStatus.aec2);
			pSettingsFile.println(g_pSensorStatus.ae_level);
			pSettingsFile.println(g_pSensorStatus.aec_value);
			pSettingsFile.println(g_pSensorStatus.agc);					// Line 30
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

				también enviar el estado actual del flash led ((g_nCurrentLedBrightness > 0) ? "1" : "0")
				*/

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
					data[12] → Timelap Flash LED Brightness
				*/
				pRequest->send(200, F("text/plain"), "REFRESH" + strResponse);	// TODO: Retornar una respuesta con header, que el ESP32Cam siempre sea con headers; Eventualmente convertir el ESP32 Controller a headers...
				return;
			} else if (pRequest->arg("action") == "list") {	// This returns file list from any directory in the SD Card
				if (pRequest->hasArg("folder")) {
					if (!SafeSDAccess([&]() {
						bool bFirst = true;
						String strFileName, strResponse;
						File pWorkingDirectory = SD_MMC.open("/" + pRequest->arg("folder"));
						File pFile = pWorkingDirectory.openNextFile();

						while (pFile) {
							strFileName = String(pFile.name());

							if (!pFile.isDirectory()) {
								if (!bFirst)
									strResponse += ":";
								else
									bFirst = false;

								int nLastSlash = strFileName.lastIndexOf('/');
								if (nLastSlash >= 0)
									strResponse += strFileName.substring(nLastSlash + 1);
								else
									strResponse += strFileName;
							}

							pFile.close();

							pFile = pWorkingDirectory.openNextFile();
						}

						pWorkingDirectory.close();

						pRequest->send(200, F("text/plain"), strResponse);
					})) {
						pRequest->send(500, F("text/plain"), F("No hay una Tarjeta SD conectada."));
					}
				}

				return;
			} else if (pRequest->arg("action") == "capture") {
				if (g_nOTAProgress > 0) {
					if (digitalRead(PWDN_GPIO_NUM) == LOW)	// If is working
						digitalWrite(PWDN_GPIO_NUM, HIGH);	// turn it off

					pRequest->send(503, F("text/plain"), F("Error: Actualización de Firmware en curso."));
					return;
				}

				if (bTakingSnapshot) {
					pRequest->send(503, F("text/plain"), F("Error: Capturando instantánea."));
					return;
				}

				g_nLastCameraActivity = millis64();

				if (digitalRead(PWDN_GPIO_NUM) == HIGH)	// If is off
					digitalWrite(PWDN_GPIO_NUM, LOW);	// turn it on

				// Monitoring Frame Size
				sensor_t *pSensorConfig = esp_camera_sensor_get();
				if (pSensorConfig->status.framesize != g_pSensorStatus.framesize)
					SetSensorConfig(g_pSensorStatus.framesize);

				camera_fb_t *pCameraFrameBuffer = esp_camera_fb_get();
				if (!pCameraFrameBuffer) {
					pRequest->send(503, F("text/plain"), F("Error en el Frame Buffer de la Cámara."));
					return;
				}

				AsyncWebServerResponse *pResponse = pRequest->beginResponse_P(200, "image/jpeg", pCameraFrameBuffer->buf, pCameraFrameBuffer->len);
				pResponse->addHeader(F("Access-Control-Allow-Origin"), F("*"));
				pResponse->addHeader(F("Content-Disposition"), F("inline; filename=capture.jpg"));
				pResponse->addHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
				pResponse->addHeader(F("Access-Control-Expose-Headers"), F("X-Flash-Status"));
				pResponse->addHeader(F("X-Flash-Status"), (g_nCurrentLedBrightness > 0) ? "1" : "0");

				pRequest->onDisconnect([pCameraFrameBuffer]() {
					esp_camera_fb_return(pCameraFrameBuffer);
				});

				pRequest->send(pResponse);
				return;
			} else if (pRequest->arg("action") == "tss") {	// Take a snapshot
				if (g_nOTAProgress > 0) {
					pRequest->send(503, F("text/plain"), F("Error: Actualización en curso."));
					return;
				}

				g_nLastCameraActivity = millis64();
				bTakingSnapshot = true;

				if (digitalRead(PWDN_GPIO_NUM) == HIGH)	// If is off
					digitalWrite(PWDN_GPIO_NUM, LOW);	// turn it on

				// Initial & Timelapse Frame Size
				sensor_t *pSensorConfig = esp_camera_sensor_get();
				if (pSensorConfig->status.framesize != g_pCameraConfig.frame_size)
					SetSensorConfig(g_pCameraConfig.frame_size);

				if (pRequest->arg("flash") == "1") {
					g_nCurrentLedBrightness = g_nTimelapseLedBrightness;

					ledcWrite(LED_GPIO_NUM, g_nCurrentLedBrightness);

					vTaskDelay(2000 / portTICK_PERIOD_MS);	// 2000ms
				}

				camera_fb_t *pCameraFrameBuffer = esp_camera_fb_get();
				if (!pCameraFrameBuffer) {
					pRequest->send(503, F("text/plain"), F("Error en el Frame Buffer de la Cámara."));
					return;
				}

				SafeSDAccess([&]() {
					bool bSaved = false;
					char cFilename[35];
					struct tm currentTime;

					GetLocalTimeNow(&currentTime);

					snprintf(cFilename, sizeof(cFilename), "/snapshots/%02d_%02d_%04d-%02d_%02d_%02d.jpg", currentTime.tm_mday, currentTime.tm_mon + 1, currentTime.tm_year + 1900, currentTime.tm_hour, currentTime.tm_min, currentTime.tm_sec);

					File pFile = SD_MMC.open(cFilename, FILE_WRITE);
					if (pFile) {
						if (pFile.write(pCameraFrameBuffer->buf, pCameraFrameBuffer->len) == pCameraFrameBuffer->len)
							bSaved = true;

						pFile.close();

						LOGGER(INFO, "Snapshot save to: %s", cFilename);
					} else {
						Serial.println("Error al abrir la SD");
					}
				});

				AsyncWebServerResponse *pResponse = pRequest->beginResponse_P(200, "image/jpeg", pCameraFrameBuffer->buf, pCameraFrameBuffer->len);
				pResponse->addHeader(F("Access-Control-Allow-Origin"), F("*"));
				pResponse->addHeader(F("Content-Disposition"), F("inline; filename=capture.jpg"));
				pResponse->addHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
				pResponse->addHeader(F("Access-Control-Expose-Headers"), F("X-Return"));
				pResponse->addHeader(F("X-Return"), "TSS");

				pRequest->onDisconnect([pCameraFrameBuffer]() {
					esp_camera_fb_return(pCameraFrameBuffer);
				});

				bTakingSnapshot = false;

				pRequest->send(pResponse);
				return;
			}
		}

		pRequest->send(501, F("text/plain"), F("HTTP 501"));
	});

	g_pWebServer.onNotFound([](AsyncWebServerRequest* pRequest) { pRequest->send(404, F("text/plain"), F("HTTP 404")); });

	g_pWebServer.on("/ota", HTTP_POST, [](AsyncWebServerRequest* pRequest) {
		bool bUpdate = !Update.hasError();

		if (bUpdate) {
			time_t pTimeNow = time(nullptr);
			char cBuffer[11];
			snprintf(cBuffer, sizeof(cBuffer), "%lu", (long)pTimeNow);
			WriteToSDAtomic("/time", cBuffer);	// Write current time to SD Card

			LOGGER(INFO, "Restarting Controller to do a Firmware Update.");

			delay(1000);

			ESP.restart();
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

	LOGGER(INFO, "Web Server Started at Port: %d.", SECRET_WEBSERVER_PORT);
}

void loop() {
	static uint64_t nLastSecondTick = 0;
	uint64_t nCurrentMillis = millis64();
	// ================================================== Code execution with 1 second interval Section ================================================== //
	if ((nCurrentMillis - nLastSecondTick) >= 1000) {	// Check if 1 second has passed since the last tick to perform once-per-second tasks
		nLastSecondTick = nCurrentMillis;

		time_t pTimeNow = time(nullptr);
		// ================================================== WiFi Section ================================================== //
		{
			static uint64_t nLastReconnectAttemptInterval = 0;

			if (eTaskGetState(g_pWiFiReconnect) == eSuspended && WiFi.status() != WL_CONNECTED && (nCurrentMillis - nLastReconnectAttemptInterval) >= g_nWiFiRetryConnectInterval) {
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
				snprintf(cBuffer, sizeof(cBuffer), "%lu", (long)pTimeNow);
				WriteToSDAtomic("/time", cBuffer);	// Write current time to SD Card
			}
		}
		// ================================================== Auto Sensor Shutdown Section ================================================== //
		{
			if ((nCurrentMillis - g_nLastCameraActivity) >= g_nSensorShutdownInterval) {
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
