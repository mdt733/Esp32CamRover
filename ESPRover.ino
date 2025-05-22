// =================================================================================================
// ESP32-CAM Servo Robot with Video Stream, OTA, Basic Authentication, and NVS Calibration
//
// Version 4.0.0 - Definitive Edition (2025-05-22)
// - Final review and polish of all code and comments for clarity and long-term maintainability.
// - Comprehensive documentation embedded within the code.
// - Ensured all features up to V3.7.3 are stable and well-commented.
// - Removed all non-essential Serial.print statements.
// - This version is intended as a feature-complete, stable, and shareable baseline.
//
// Key Version History (Recent Major Changes):
//   V3.7.3: Stream/AP Mode Fixes & Battery Critical Handling.
//   V3.7.0: WebSockets, Advanced Sleep, AP Mode, UI Enhancements.
//   V3.6.x: Light Sleep Mode implementation and refinements.
//   V3.5.0: Snapshot feature removed, Serial logging cleaned.
//   V3.4.x: Snapshot feature attempts, RSSI display.
//   V3.0.0 - V3.3.x: Initial NVS, UI enhancements, multi-client stream, compiler fixes.
//
// Project Overview:
// This firmware transforms an ESP32-CAM module into a versatile Wi-Fi controlled robot.
// It's designed for hobbyists and enthusiasts looking to build a feature-rich remote-controlled
// device with live video feedback.
//
// Core Features:
// - Live MJPEG Video Streaming: Real-time video feed via HTTP.
// - Dual Servo Control: For differential drive movement.
// - Web-Based User Interface (UI):
//   - Responsive design for desktop and mobile.
//   - HTML/CSS/JS stored in PROGMEM to conserve RAM.
//   - Joystick and Slider control modes.
//   - Real-time status display (Connection, LiPo, Wi-Fi RSSI, Sleep State, FPS).
// - Onboard LED Control: Adjustable brightness with gamma correction.
// - Battery Monitoring: LiPo voltage display and visual bar, with critical level handling.
// - Wi-Fi Management:
//   - Connects to a configured STA network.
//   - Falls back to Access Point (AP) mode ("ESP32-CAM-Robot-Setup") if STA connection fails,
//     allowing Wi-Fi credentials to be configured via the web UI.
// - Over-The-Air (OTA) Firmware Updates: Wireless updates via Arduino IDE or web browser.
// - Basic HTTP Authentication: Secures access to the web interface and stream.
// - Non-Volatile Storage (NVS): Persistently stores:
//   - Servo calibration (stop pulse, deadzone).
//   - LiPo ADC calibration (voltage divider ratio).
//   - Camera settings (resolution, quality, brightness, contrast, saturation).
//   - Light sleep mode enable/disable preference.
//   - Wi-Fi STA credentials.
// - Dual-Core Processing (ESP32):
//   - Core 0: Wi-Fi, WebServer, WebSockets, OTA, Application Logic.
//   - Core 1: Video Streaming Task.
// - WebSockets: Used for most real-time control commands and status updates, reducing HTTP overhead.
// - Light Sleep Mode:
//   - Reduces power consumption after a configurable period of inactivity.
//   - Wakes on Wi-Fi activity or a periodic timer.
//   - UI toggle to enable/disable this feature.
//
// Origins:
// Based on concepts from the "esp32-caretaker" project by positron48 and significantly
// extended and refactored throughout this development process.
//
// -------------------------------------------------------------------------------------------------
// GPIO Pin Assignments (AI-Thinker ESP32-CAM Model assumed):
// -------------------------------------------------------------------------------------------------
// Internal Camera Connections (Standard for ESP32-CAM AI-Thinker - Do Not Change):
// - GPIO 0:  XCLK_GPIO_NUM (Camera Clock Input)
// - GPIO 5:  Y2_GPIO_NUM   (Camera Data Line 0)
// - GPIO 18: Y3_GPIO_NUM   (Camera Data Line 1)
// - GPIO 19: Y4_GPIO_NUM   (Camera Data Line 2)
// - GPIO 21: Y5_GPIO_NUM   (Camera Data Line 3)
// - GPIO 36: Y6_GPIO_NUM   (Camera Data Line 4) (Often VP on schematics)
// - GPIO 39: Y7_GPIO_NUM   (Camera Data Line 5) (Often VN on schematics)
// - GPIO 34: Y8_GPIO_NUM   (Camera Data Line 6)
// - GPIO 35: Y9_GPIO_NUM   (Camera Data Line 7)
// - GPIO 22: PCLK_GPIO_NUM (Camera Pixel Clock Output)
// - GPIO 23: HREF_GPIO_NUM (Camera Horizontal Reference Output)
// - GPIO 25: VSYNC_GPIO_NUM(Camera Vertical Sync Output)
// - GPIO 26: SIOD_GPIO_NUM (Camera I2C Data - SCCB Data)
// - GPIO 27: SIOC_GPIO_NUM (Camera I2C Clock - SCCB Clock)
// - GPIO 32: PWDN_GPIO_NUM (Camera Power Down Control)
//
// External Connections / User-Defined Pins:
// - GPIO 4:  LED_PIN         (Onboard Flash LED control)
// - GPIO 12: SERVO_LEFT_PIN  (Signal for Left Servo)
// - GPIO 13: SERVO_RIGHT_PIN (Signal for Right Servo)
// - GPIO 33: LIPO_ADC_PIN    (Analog input for LiPo voltage divider. Often connected to onboard RED status LED trace)
// - GPIO 1 (U0TXD), GPIO 3 (U0RXD): Used for Serial Programming/Debugging (if accessible).
//
// -------------------------------------------------------------------------------------------------
// User Configuration Notes:
// -------------------------------------------------------------------------------------------------
// 1. Initial Setup: On first boot or if STA connection fails, the robot starts in AP Mode.
//    Connect to "ESP32-CAM-Robot-Setup" Wi-Fi (default password: "password123").
//    Open 192.168.4.1 in a browser. Use the Settings panel (gear icon) to navigate to
//    "Wi-Fi Configuration (AP Mode)" section. Enter your home Wi-Fi SSID & Password.
//    Click "Connect & Restart Robot". The robot will save these and restart to attempt
//    connection to your specified network.
// 2. Default Wi-Fi Credentials: `nvs_wifi_ssid` and `nvs_wifi_password` in code are defaults
//    if NVS is empty. They are overwritten by AP mode configuration.
// 3. HTTP Authentication: CRITICAL - CHANGE `HTTP_AUTH_USERNAME` and `HTTP_AUTH_PASSWORD`
//    constants below for security.
// 4. OTA Password: CRITICAL - CHANGE `OTA_PASSWORD` constant for security.
// 5. Servo Calibration: Access the web UI (Settings -> Calibration). Fine-tune "Stop Pulse"
//    (typically around 1500µs) and "Pulse Deadzone" (e.g., 20-50µs) for each servo.
//    Save settings to NVS.
// 6. LiPo Voltage Divider Calibration: In UI (Settings -> Calibration), set the
//    "Voltage Divider Ratio" based on your resistor values (Ratio = (R1+R2)/R2).
//    Save to NVS.
// 7. Camera Settings: Resolution, JPEG quality, brightness, contrast, and saturation
//    are adjustable in the UI (Settings -> Video) and saved to NVS.
// 8. Sleep Mode: Can be enabled/disabled in UI settings (Settings -> Power Saving).
// =================================================================================================

// =================================================================================================
// 1. INCLUDES
// =================================================================================================
// ESP32 Core Libraries
#include <WiFi.h>             // For Wi-Fi connectivity (connect, manage network, RSSI)
#include <WiFiUdp.h>          // For UDP communication, used by ArduinoOTA
#include <ESPmDNS.h>          // For mDNS (Multicast DNS) service discovery, used by ArduinoOTA
#include <ArduinoOTA.h>       // For Over-The-Air firmware updates
#include <WebServer.h>        // For creating the HTTP web server (ESP8266/ESP32 WebServer library)
#include <WebSocketsServer.h> // For WebSocket communication (e.g., by Markus Sattler)
#include <esp_camera.h>       // For ESP32 camera functions and configurations (ESP32 Arduino Core component)
#include <Preferences.h>      // For NVS (Non-Volatile Storage) to save settings persistently (ESP32 Arduino Core component)
#include <esp_wifi.h>         // For advanced Wi-Fi control like esp_wifi_set_ps()
#include <esp_sleep.h>        // For light sleep (esp_light_sleep_start()) and wake-up source configuration

// ESP-IDF Specific Headers (from the ESP32 Arduino Core's underlying ESP-IDF)
#include "soc/soc.h"           // For direct System-on-Chip register access
#include "soc/rtc_cntl_reg.h"  // For Real-Time Clock control registers (used with brownout)
#include "driver/ledc.h"       // For ESP-IDF LEDC (PWM) peripheral control, used for flash LED brightness

// Commonly used Third-Party Libraries (Install via Arduino IDE Library Manager)
#include <ESP32Servo.h>       // For controlling servo motors on ESP32.
                              // (Typically by Kevin Harrington/John K. Bennett et al.)
#include <ArduinoJson.h>      // For efficient JSON parsing and generation.
                              // (By Benoit Blanchon)

// Standard C/C++ Libraries (Part of the compiler toolchain)
#include <math.h>             // For mathematical functions like pow() (used in gamma correction)


// =================================================================================================
// 2. CONFIGURATION & GLOBAL DEFINITIONS
// =================================================================================================

// --- Rolling average filter type definition ---
// Used for calculating an average FPS for the video stream.
typedef struct {
    size_t size;    // Number of samples in the filter's circular buffer
    size_t index;   // Current index in the values array (for circular buffer)
    size_t count;   // Number of values currently stored in the filter (up to 'size')
    int sum;        // Sum of current values in the filter (used to calculate average)
    int* values;    // Pointer to the array storing the sample values
} ra_filter_t;

// --- Network & Server Configuration ---
const int HTTP_PORT = 80;       // Port for the HTTP web server
const int WEBSOCKET_PORT = 81;  // Port for the WebSocket server
char nvs_wifi_ssid[64] = "YOUR_DEFAULT_SSID"; // Default SSID if NVS is empty; overwritten by NVS or AP mode config
char nvs_wifi_password[64] = "YOUR_DEFAULT_PASSWORD"; // Default PSK if NVS is empty
const char* AP_SSID = "ESP32-CAM-Robot-Setup"; // SSID for Access Point mode
const char* AP_PASSWORD = "password123";      // Password for Access Point mode (consider changing for security)
bool isInAPMode = false; // Flag to indicate if the robot is currently running in AP mode

// --- Basic Authentication Credentials ---
const char* HTTP_AUTH_USERNAME = "admin"; 
const char* HTTP_AUTH_PASSWORD = "authpassword"; // <<< CRITICAL: CHANGE THIS FOR SECURITY!
const char* AUTH_REALM = "ESP32-CAM Robot Access"; // Realm message displayed in auth prompt

// --- OTA Configuration ---
const char* OTA_HOSTNAME = "esp32-cam-robot"; // Hostname for OTA updates (e.g., esp32-cam-robot.local)
const char* OTA_PASSWORD = "otapassword"; // <<< CRITICAL: CHANGE THIS FOR SECURITY!
bool otaIsActive = false; // Flag to prevent sleep and other operations during OTA update

// --- Camera Pin Definitions (Standard for AI-Thinker ESP32-CAM) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1 // -1 indicates not used
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26 // SCCB Data (I2C Data)
#define SIOC_GPIO_NUM     27 // SCCB Clock (I2C Clock)
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- Camera Configuration Constants ---
const int CAM_XCLK_FREQ = 20000000; // Camera clock frequency (20MHz typical for OV2640/OV3660)
const pixformat_t CAM_PIXEL_FORMAT = PIXFORMAT_JPEG; // Output format for streaming (JPEG is efficient)
const int CAM_FB_COUNT = 2;         // Number of frame buffers (2 recommended with PSRAM for smoother streaming)
framesize_t nvs_cam_framesize = FRAMESIZE_QVGA; // Default streaming resolution, loaded from/saved to NVS
int nvs_cam_quality = 12;                     // Default streaming JPEG quality (10-63, lower is better quality), NVS stored
int nvs_cam_brightness = 0; // Default sensor brightness (-2 to +2), NVS stored
int nvs_cam_contrast = 0;   // Default sensor contrast (-2 to +2), NVS stored
int nvs_cam_saturation = 0; // Default sensor saturation (-2 to +2), NVS stored

// --- LED Configuration (Onboard Flash LED on GPIO4) ---
const int LED_PIN = 4; 
const ledc_channel_t LED_LEDC_CHANNEL_NUM = LEDC_CHANNEL_2; // ESP32 LEDC PWM channel for LED
const ledc_timer_t LED_LEDC_TIMER_NUM = LEDC_TIMER_2;       // ESP32 LEDC PWM timer for LED
const int LED_RESOLUTION_BITS = 8;     // PWM resolution (8-bit => 0-255 duty cycle range)
const int LED_FREQUENCY = 5000;      // PWM frequency in Hz for LED control
const float LED_GAMMA = 2.2;         // Gamma correction factor for more linear perceived brightness
int g_current_led_brightness_percent = 0; // Global variable to track current UI-set LED brightness

// --- Servo Configuration ---
const int SERVO_LEFT_PIN = 12;  // GPIO for the left servo signal
const int SERVO_RIGHT_PIN = 13; // GPIO for the right servo signal
Servo servoLeft;
Servo servoRight;
const int DEFAULT_SERVO_STOP_US = 1500;   // Default servo stop pulse width (microseconds)
const int SERVO_PULSE_SPAN_US = 500;      // Servo movement range: STOP_US +/- SPAN_US
const int DEFAULT_SERVO_PULSE_DEADZONE_US = 50; // Default deadzone around stop pulse where servo doesn't move
// NVS: Global variables to hold current servo calibration values, loaded from NVS
int servo_left_stop_us    = DEFAULT_SERVO_STOP_US;
int servo_left_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;
int servo_right_stop_us   = DEFAULT_SERVO_STOP_US;
int servo_right_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;

// --- LiPo ADC Configuration ---
const int LIPO_ADC_PIN = 33; // ADC pin for LiPo voltage measurement
const float DEFAULT_VOLTAGE_DIVIDER_RATIO = 3.2; // Default ratio for voltage divider (R1+R2)/R2
const float DEFAULT_ADC_REF_VOLTAGE = 3.3;     // ESP32 ADC reference voltage (can vary, calibrate if needed)
const int ADC_RESOLUTION_BITS = 12;            // ESP32 ADC resolution (0-4095)
float lipo_calib_voltage_divider_ratio = DEFAULT_VOLTAGE_DIVIDER_RATIO; // Loaded from NVS
const float BATTERY_LOW_WARN_VOLTAGE = 3.5f;    // Voltage threshold for low battery warning
const float BATTERY_CRITICAL_VOLTAGE = 3.2f;  // Voltage threshold for critical battery (limit operations)
bool isBatteryCritical = false; // Flag indicating if battery is critically low

// --- NVS (Non-Volatile Storage) ---
Preferences preferences; // Object for NVS access

// --- WebServer & WebSockets ---
WebServer server(HTTP_PORT); // HTTP Web server object
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT); // WebSocket server object

// --- Stream Task Configuration & Globals ---
const size_t HDR_BUF_LEN = 64;       // Buffer size for MJPEG stream part headers
const size_t MAX_FRAME_SIZE = 128 * 1024; // Max expected JPEG frame size (generous for various resolutions)
const int STREAM_TASK_STACK_SIZE = 8192; // Stack size for the video streaming FreeRTOS task
const UBaseType_t STREAM_TASK_PRIORITY = 2; // Priority for the streaming task
const BaseType_t STREAM_TASK_CORE = 1;      // Core to pin the streaming task to (Core 1 for video)
const uint32_t STREAM_DELAY_MS = 20;        // Small delay in stream loop to yield CPU & control FPS

// MJPEG stream constants
const char* PART_BOUNDARY = "123456789000000000000987654321"; // Boundary for multipart stream
const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
const char* STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

bool isStreaming = false;        // Flag: Is the MJPEG video stream currently active?
WiFiClient streamClient;         // WiFi client object for the active HTTP MJPEG stream
TaskHandle_t streamTaskHandle = NULL; // Handle for the video streaming FreeRTOS task
uint8_t* streamBuffer = nullptr; // Buffer for non-JPEG to JPEG conversion (currently not used as CAM_PIXEL_FORMAT is JPEG)
size_t streamBufferSize = 0;

// Rolling Average Filter for FPS Calculation
const int RA_FILTER_SAMPLE_SIZE = 20; // Number of samples for FPS rolling average
static int ra_values_array[RA_FILTER_SAMPLE_SIZE] = {0}; // Static array for filter values
ra_filter_t ra_filter; // Global instance of the filter struct

// Robot Control Mode
enum ControlMode { JOYSTICK, SLIDERS };
ControlMode currentControlMode = JOYSTICK; 

// --- Light Sleep Management ---
unsigned long lastClientActivityTimestamp = 0; // Timestamp of the last client interaction (HTTP or WebSocket)
const unsigned long lightSleepTimeoutMs = 5 * 60 * 1000; // 5 minutes: Time of inactivity before entering light sleep
const uint64_t lightSleepPeriodicWakeupUs = 60 * 1000 * 1000ULL; // Periodic timer wakeup from light sleep (60 seconds in microseconds)
TaskHandle_t sleepManagementTaskHandle = NULL; // Handle for the sleep management FreeRTOS task
bool isLightSleeping = false; // Flag: Is the ESP32 currently in light sleep?
bool g_light_sleep_enabled = true; // NVS stored preference for enabling/disabling light sleep mode
unsigned long lastStatusBroadcastTime = 0; // Timestamp for periodic WebSocket status pushes
const unsigned long statusBroadcastInterval = 750; // Interval for WebSocket status pushes (milliseconds)


// =================================================================================================
// 3. HTML CONTENT (Stored in PROGMEM to save RAM)
// =================================================================================================
static const char htmlContentFlash[] PROGMEM = R"RAW_HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>
    <meta name="mobile-web-app-capable" content="yes">
    <meta name="apple-mobile-web-app-capable" content="yes">
    <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
    <meta name="format-detection" content="telephone=no">
    <meta name="msapplication-tap-highlight" content="no">
    <meta name="HandheldFriendly" content="true">
    <title>ESP32-CAM ServoBot V4.0.0</title>
    <style>
        :root {
            --primary-color: #007bff; 
            --secondary-color: #28a745;
            --info-color: #17a2b8; 
            --danger-color: #dc3545;
            --warning-color: #ffc107;
            --background-color: #212529;
            --card-background-color: #343a40; 
            --text-color: #f8f9fa; 
            --text-muted-color: #adb5bd;
            --control-bg: rgba(52, 58, 64, 0.85); 
            --input-bg: #495057;
            --input-border-color: #5a6268;
            --battery-bar-bg: #555;
            --battery-good: #28a745;
            --battery-medium: #ffc107;
            --battery-low: #dc3545;
            --rssi-bar-color: var(--primary-color); 
            --conn-status-good: var(--secondary-color);
            --conn-status-bad: var(--danger-color);
            --toast-info-bg: rgba(23, 162, 184, 0.9); /* Info color for toasts */
            --toast-warn-bg: rgba(255, 193, 7, 0.9); /* Warning color for toasts */
            --toast-error-bg: rgba(220, 53, 69, 0.9);/* Error color for toasts */
            --toast-text-color: #f1f1f1;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            background-color: var(--background-color);
            min-height: 100vh; overflow: hidden; color: var(--text-color);
            touch-action: none; -ms-touch-action: none;
            overscroll-behavior: none !important;
            position: fixed; width: 100%; height: 100%;
            display: flex; flex-direction: column;
        }
        .container {
            position: relative; width: 100%; height: 100%;
            display: flex; align-items: center; justify-content: center;
            flex-grow: 1; 
        }
        .video-container {
            position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 1;
            background: #000; display: flex; align-items: center; justify-content: center;
            overflow: hidden; 
        }
        .stream-feedback { 
            color: var(--text-muted-color); font-size: 1.2em; text-align: center; 
            padding: 20px; position: absolute; z-index: 2; 
            display: flex; flex-direction: column; align-items: center; justify-content: center;
            width: 100%; height: 100%; 
            background-color: rgba(0,0,0,0.5); 
        }
        .stream-feedback.hidden { display: none; }
        .spinner {
            border: 4px solid rgba(255, 255, 255, 0.2);
            border-left-color: var(--primary-color);
            border-radius: 50%;
            width: 30px;
            height: 30px;
            animation: spin 1s linear infinite;
            margin-bottom: 10px;
        }
        @keyframes spin { to { transform: rotate(360deg); } }

        #stream { 
            width: 100%; height: 100%; object-fit: contain; 
            display: none; transform-origin: center center; transform: rotate(-90deg); 
        }
        
        .controls-overlay { position: absolute; z-index: 2; width: 100%; height: 100%; pointer-events: none; }
        .controls-top-left { 
            position: absolute; top: 15px; left: 15px; display: flex; align-items: center;
            gap: 10px; pointer-events: all; background: var(--control-bg);
            padding: 10px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3);
            backdrop-filter: blur(5px);
        }
        .controls-top-right-status {
            position: absolute; top: 15px; right: 15px;
            background: var(--control-bg); padding: 8px 12px; border-radius: 8px;
            font-size: 0.85em; z-index: 5; color: var(--text-muted-color);
            box-shadow: 0 2px 5px rgba(0,0,0,0.3); backdrop-filter: blur(5px);
            display: flex; flex-direction: column; 
            align-items: flex-end; 
            gap: 5px; 
            pointer-events: all;
        }
        .status-item { display: flex; align-items: center; gap: 8px; }
        #lipo-voltage-display, #wifi-rssi-display { font-weight: bold; }
        #lipo-voltage-display { color: var(--warning-color); }
        #wifi-rssi-display { color: var(--info-color); }
        .connection-status-dot {
            width: 10px; height: 10px; border-radius: 50%;
            background-color: var(--conn-status-good); 
            margin-left: 5px; 
        }
        .connection-status-dot.bad { background-color: var(--conn-status-bad); }
        .bar-container {
            width: 50px; height: 10px; background-color: var(--battery-bar-bg); 
            border-radius: 3px; overflow: hidden; border: 1px solid #444;
        }
        .bar-level {
            height: 100%; width: 0%; 
            transition: width 0.3s ease-out, background-color 0.3s ease-out;
        }
        #battery-bar-level { background-color: var(--battery-good); }
        #wifi-bar-level { background-color: var(--rssi-bar-color); }
        .stream-status-fps { 
            background: transparent; 
            color: white; padding: 0; font-size: 0.8em; 
            text-align: right; 
        }
        #sleep-status-indicator { font-size: 0.8em; color: var(--text-muted-color); text-align: right; }


        .controls-joystick-panel { 
            position: absolute; left: 15px; top: 50%; transform: translateY(-50%);
            pointer-events: all; display: flex; flex-direction: column; gap: 15px;
            background: var(--control-bg); padding: 15px; border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.3); backdrop-filter: blur(5px);
        }
        .joystick-container {
            width: 160px; height: 160px; background: rgba(255,255,255,0.05);
            border-radius: 8px; padding: 15px; display: none; 
        }
        .joystick-container.active { display: block; }
        #joystick {
            width: 100%; height: 100%; background: rgba(255,255,255,0.1);
            border-radius: 50%; position: relative; touch-action: none;
        }
        #stick {
            width: 40%; height: 40%; background: var(--primary-color);
            border-radius: 50%; position: absolute; top: 50%; left: 50%;
            transform: translate(-50%, -50%); cursor: pointer;
            box-shadow: 0 0 10px rgba(0,123,255,0.5);
        }
        .sliders-control-area { 
            position: absolute; top: 0; left: 0; width: 100%; height: 100%;
            pointer-events: none; display: none; 
        }
        .sliders-control-area.active { display: block; }
        .slider-vertical-area { 
            position: fixed; top: 50%; transform: translateY(-50%);
            width: 80px; height: 200px; background: var(--control-bg);
            border-radius: 10px; padding: 10px;
            display: flex; justify-content: center; align-items: center;
            pointer-events: all; box-shadow: 0 4px 8px rgba(0,0,0,0.3); backdrop-filter: blur(5px);
        }
        .slider-left-area { left: 15px; }
        .slider-right-area { right: 15px; }
        .slider-track { 
            width: 12px; height: 100%; background: rgba(255,255,255,0.1);
            border-radius: 6px; position: relative;
        }
        .slider-thumb {
            width: 44px; height: 44px; background: var(--primary-color);
            border-radius: 50%; cursor: pointer; position: absolute; left: 50%;
            transform: translateX(-50%); top: 50%; margin-top: -22px; 
            box-shadow: 0 0 10px rgba(0,123,255,0.5);
        }
        .button {
            padding: 10px 15px; border: none; border-radius: 6px; cursor: pointer;
            font-size: 1em; background: var(--input-bg); color: var(--text-color);
            transition: background-color 0.2s, box-shadow 0.2s;
            white-space: nowrap; text-align: center; touch-action: manipulation;
            user-select: none; -webkit-user-select: none; display: flex;
            align-items: center; justify-content: center; line-height: 1;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }
        .button:hover { background-color: #495057; box-shadow: 0 3px 6px rgba(0,0,0,0.3); }
        .button:active, .button.active { background-color: var(--primary-color); color: white; box-shadow: 0 1px 3px rgba(0,0,0,0.2) inset; }
        .button.btn-danger { background-color: var(--danger-color); color: white; }
        .button.btn-danger:hover { background-color: #c82333; }
        .button.btn-info { background-color: var(--info-color); color: white; }
        .button.btn-info:hover { background-color: #138496; }

        .icon-button .material-icons { font-size: 20px; }
        
        .settings-panel {
            position: fixed; top: 0; left: -320px; 
            width: 320px; height: 100%;
            background: var(--card-background-color); z-index: 1000;
            transition: left 0.3s ease; padding: 20px; overflow-y: auto;
            box-shadow: 5px 0 15px rgba(0,0,0,0.5); touch-action: pan-y !important;
        }
        .settings-panel.active { left: 0; pointer-events: auto; }
        .settings-close {
            position: absolute; top: 15px; right: 15px; font-size: 24px; cursor: pointer;
            color: var(--text-muted-color); opacity: 0.7; z-index: 1001;
        }
        .settings-close:hover { opacity: 1; color: var(--text-color); }
        .settings-title { font-size: 1.4em; margin-bottom: 20px; font-weight: 500; color: var(--text-color); border-bottom: 1px solid #495057; padding-bottom: 10px; }
        .settings-section { margin-bottom: 25px; }
        .settings-section h4 { font-size: 1.1em; color: var(--text-color); margin-bottom: 15px; border-bottom: 1px solid #495057; padding-bottom: 8px;}
        .settings-subsection h5 { font-size: 0.9em; color: var(--text-muted-color); margin-top: 10px; margin-bottom: 5px; font-weight: normal; }
        .select-wrapper { position: relative; width: 100%; margin-bottom: 10px; }
        .settings-select, .settings-input {
            width: 100%; padding: 10px; appearance: none; -webkit-appearance: none; -moz-appearance: none;
            background-color: var(--input-bg); color: var(--text-color); border: 1px solid var(--input-border-color);
            border-radius: 5px; cursor: pointer; outline: none; font-size: 1em; margin-bottom: 8px;
        }
        .settings-input[type="number"] { -moz-appearance: textfield; } 
        .settings-input::-webkit-outer-spin-button,
        .settings-input::-webkit-inner-spin-button { -webkit-appearance: none; margin: 0; }
        .select-wrapper::after {
            content: '\\25BC'; position: absolute; right: 12px; top: 50%;
            transform: translateY(-50%); pointer-events: none; color: var(--text-muted-color);
        }
        .settings-slider-container { width: 100%; margin: 10px 0; }
        .settings-slider {
            width: 100%; -webkit-appearance: none; height: 8px; border-radius: 4px;
            background: #5a6268; outline: none; cursor: pointer;
        }
        .settings-slider::-webkit-slider-thumb {
            -webkit-appearance: none; width: 20px; height: 20px; border-radius: 50%;
            background: var(--primary-color); cursor: pointer; box-shadow: 0 0 5px rgba(0,123,255,0.4);
        }
        .settings-slider::-moz-range-thumb {
            width: 20px; height: 20px; border-radius: 50%; background: var(--primary-color);
            cursor: pointer; border: none; box-shadow: 0 0 5px rgba(0,123,255,0.4);
        }
        .settings-slider-value { text-align: right; margin-top: 5px; font-size: 0.9em; color: var(--text-muted-color); }
        .control-mode-tabs { display: flex; width: 100%; margin-bottom: 15px; }
        .control-mode-tab {
            flex: 1; text-align: center; padding: 10px 0; border: 1px solid #495057;
            cursor: pointer; opacity: 0.7; background: var(--input-bg); color: var(--text-muted-color);
            transition: background-color 0.2s, opacity 0.2s;
        }
        .control-mode-tab:first-child { border-radius: 5px 0 0 5px; border-right: none;}
        .control-mode-tab:last-child { border-radius: 0 5px 5px 0; }
        .control-mode-tab.active { background: var(--primary-color); opacity: 1; color: white; }
        .overlay {
            position: fixed; top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(0,0,0,0.6); z-index: 999; display: none;
        }
        .overlay.active { display: block; }
        .calibration-buttons button { margin-top: 10px; width: 100%; }
        .calibration-buttons button#btn-save-calib { background-color: var(--secondary-color); }
        .calibration-buttons button#btn-reset-calib { background-color: var(--danger-color); }
        .calibration-buttons button#btn-load-calib { background-color: var(--info-color); } 
        .calibration-input-group { margin-bottom: 15px; }
        .calibration-input-group label { display: block; font-size: 0.85em; color: var(--text-muted-color); margin-bottom: 3px; }
        
        .settings-toggle-switch { display: flex; justify-content: space-between; align-items: center; width: 100%; padding: 8px 0; }
        .settings-toggle-switch label { color: var(--text-muted-color); font-size: 0.9em; }
        .settings-toggle-switch .switch { position: relative; display: inline-block; width: 44px; height: 24px; }
        .settings-toggle-switch .switch input { opacity: 0; width: 0; height: 0; }
        .settings-toggle-switch .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #5a6268; transition: .4s; border-radius: 24px; }
        .settings-toggle-switch .slider:before { position: absolute; content: ""; height: 18px; width: 18px; left: 3px; bottom: 3px; background-color: white; transition: .4s; border-radius: 50%; }
        .settings-toggle-switch input:checked + .slider { background-color: var(--primary-color); }
        .settings-toggle-switch input:checked + .slider:before { transform: translateX(20px); }

        #toast-container {
            position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%);
            z-index: 2000; display: flex; flex-direction: column; align-items: center; gap: 10px;
        }
        .toast {
            background-color: var(--toast-info-bg); color: var(--toast-text-color);
            padding: 12px 20px; border-radius: 6px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);
            opacity: 0; transition: opacity 0.5s, transform 0.5s; transform: translateY(20px);
            font-size: 0.9em; max-width: 90vw; text-align: center;
        }
        .toast.show { opacity: 1; transform: translateY(0); }
        .toast.error { background-color: var(--toast-error-bg); }
        .toast.warning { background-color: var(--toast-warn-bg); }
        
        #ap-mode-config { display: none; padding: 20px; background-color: var(--card-background-color); border-radius: 8px; margin: 20px; text-align: center; z-index: 1001; position: relative;}
        #ap-mode-config h3 { margin-bottom: 15px; }
        #ap-mode-config input { margin-bottom: 10px; }


        @media (max-width: 768px) { 
            .controls-joystick-panel {
                left: 50%; top: auto; bottom: 45px; 
                transform: translateX(-50%); 
                background: rgba(33, 37, 41, 0.7); padding: 10px;
                max-width: 180px; 
            }
            .joystick-container { width: 130px; height: 130px; background: rgba(255,255,255,0.03); }
            .slider-vertical-area { width: 60px; height: 150px; padding: 8px; }
            .slider-left-area { left: 10px; }
            .slider-right-area { right: 10px; }
            .slider-thumb { width: 36px; height: 36px; margin-top: -18px; }
            
            .controls-top-right-status { 
                font-size: 0.8em; padding: 6px 10px; 
            }
            .stream-status-fps { 
                font-size: 0.75em; 
                padding: 0; 
            }
            .bar-container { width: 40px; } 
        }

        @font-face {
          font-family: 'Material Icons'; font-style: normal; font-weight: 400;
          src: url(https://fonts.gstatic.com/s/materialicons/v141/flUhRq6tzZclQEJ-Vdg-IuiaDsNc.woff2) format('woff2');
        }
        .material-icons {
          font-family: 'Material Icons'; font-weight: normal; font-style: normal;
          font-size: 24px; line-height: 1; letter-spacing: normal; text-transform: none;
          display: inline-block; white-space: nowrap; word-wrap: normal; direction: ltr;
          -webkit-font-smoothing: antialiased; text-rendering: optimizeLegibility;
          -moz-osx-font-smoothing: grayscale; font-feature-settings: 'liga';
        }
    </style>
</head>
<body>
    <div id="toast-container"></div>
    <div class="container">
        <div class="video-container">
            <div class="stream-feedback" id="stream-feedback-placeholder">
                <span>Video Stream Offline</span>
            </div>
            <div class="stream-feedback hidden" id="stream-feedback-loading">
                <div class="spinner"></div>
                <span>Connecting to stream...</span>
            </div>
            <img id="stream">
        </div>

        <div id="ap-mode-config">
            <h3>Robot in Setup Mode</h3>
            <p>Connect to Wi-Fi network:</p>
            <input type="text" id="ap-mode-ssid" class="settings-input" placeholder="Network Name (SSID)">
            <input type="password" id="ap-mode-password" class="settings-input" placeholder="Password">
            <button class="button" id="ap-mode-connect-button">Connect & Restart</button>
        </div>
        
        <div class="controls-overlay" id="main-controls-overlay">
            <div class="controls-top-left">
                <button class="button icon-button" id="stream-toggle" title="Start/Stop Stream">
                    <span class="material-icons">play_arrow</span>
                </button>
                <button class="button icon-button" id="settings-toggle" title="Settings">
                    <span class="material-icons">settings</span>
                </button>
            </div>

            <div class="controls-top-right-status">
                <div class="status-item">
                    <span>Conn:</span> <span class="connection-status-dot" id="connection-status-dot"></span>
                </div>
                <div class="status-item">
                    <span>LiPo: <span id="lipo-voltage-display">N/A</span> V</span>
                    <div class="bar-container">
                        <div class="bar-level" id="battery-bar-level"></div>
                    </div>
                </div>
                <div class="status-item">
                    <span>WiFi: <span id="wifi-rssi-display">N/A</span></span>
                    <div class="bar-container">
                        <div class="bar-level" id="wifi-bar-level"></div>
                    </div>
                </div>
                <div class="stream-status-fps" id="fps-status" style="display: none;">FPS: ...</div>
                <div id="sleep-status-indicator">Awake</div>
            </div>
            
            <div class="controls-joystick-panel" id="joystick-panel">
                <div class="joystick-container" id="joystick-control">
                    <div id="joystick"><div id="stick"></div></div>
                </div>
            </div>

            <div class="sliders-control-area" id="sliders-panel">
                <div class="slider-vertical-area slider-left-area" id="left-slider-area">
                    <div class="slider-track">
                        <div class="slider-thumb" id="left-thumb"></div>
                    </div>
                </div>
                <div class="slider-vertical-area slider-right-area" id="right-slider-area">
                     <div class="slider-track">
                        <div class="slider-thumb" id="right-thumb"></div>
                    </div>
                </div>
            </div>
            
        </div>
        
        <div class="overlay" id="settings-overlay"></div>
        <div class="settings-panel" id="settings-panel">
            <div class="settings-close" id="settings-close">&times;</div>
            <div class="settings-title">Settings</div>
            
            <div class="settings-section">
                <h4>Video</h4>
                <div class="select-wrapper">
                    <select id="resolution-select" class="settings-select">
                        <option value="QQVGA">QQVGA (160x120)</option>
                        <option value="QCIF">QCIF (176x144)</option>
                        <option value="HQVGA">HQVGA (240x176)</option>
                        <option value="240X240">240X240</option>
                        <option value="QVGA" selected>QVGA (320x240)</option>
                        <option value="CIF">CIF (400x296)</option>
                        <option value="HVGA">HVGA (480x320)</option>
                        <option value="VGA">VGA (640x480)</option>
                        <option value="SVGA">SVGA (800x600)</option>
                        <option value="XGA">XGA (1024x768)</option>
                        <option value="HD">HD (1280x720)</option>
                        <option value="SXGA">SXGA (1280x1024)</option>
                        <option value="UXGA">UXGA (1600x1200)</option>
                    </select>
                </div>
                 <div class="settings-slider-container">
                    <label for="quality-slider" style="font-size:0.9em; color: var(--text-muted-color);">Quality (10-63, lower is better)</label>
                    <input type="range" min="10" max="63" value="12" class="settings-slider" id="quality-slider">
                    <div class="settings-slider-value" id="quality-value">12</div>
                </div>
                <div class="settings-slider-container">
                    <label for="brightness-slider" style="font-size:0.9em; color: var(--text-muted-color);">Brightness (-2 to +2)</label>
                    <input type="range" min="-2" max="2" value="0" step="1" class="settings-slider" id="brightness-slider">
                    <div class="settings-slider-value" id="brightness-value">0</div>
                </div>
                <div class="settings-slider-container">
                    <label for="contrast-slider" style="font-size:0.9em; color: var(--text-muted-color);">Contrast (-2 to +2)</label>
                    <input type="range" min="-2" max="2" value="0" step="1" class="settings-slider" id="contrast-slider">
                    <div class="settings-slider-value" id="contrast-value">0</div>
                </div>
                <div class="settings-slider-container">
                    <label for="saturation-slider" style="font-size:0.9em; color: var(--text-muted-color);">Saturation (-2 to +2)</label>
                    <input type="range" min="-2" max="2" value="0" step="1" class="settings-slider" id="saturation-slider">
                    <div class="settings-slider-value" id="saturation-value">0</div>
                </div>
            </div>
            
            <div class="settings-section">
                <h4>Controls</h4>
                <div class="control-mode-tabs">
                    <div class="control-mode-tab active" data-mode="joystick" id="joystick-tab">Joystick</div>
                    <div class="control-mode-tab" data-mode="sliders" id="sliders-tab">Sliders</div>
                </div>
            </div>
            
            <div class="settings-section">
                <h4>LED</h4>
                <div class="settings-slider-container">
                     <label for="led-slider" style="font-size:0.9em; color: var(--text-muted-color);">Brightness (0-100%)</label>
                    <input type="range" min="0" max="100" value="0" class="settings-slider" id="led-slider">
                    <div class="settings-slider-value" id="led-value">Off</div>
                </div>
            </div>

             <div class="settings-section">
                <h4>Display</h4>
                <button class="button" id="show-fps-toggle" style="width:100%; justify-content: space-between;">
                    <span>Show FPS</span> <span class="material-icons" id="fps-toggle-icon">visibility_off</span>
                </button>
            </div>

            <div class="settings-section">
                <h4>Power Saving</h4>
                <div class="settings-toggle-switch">
                    <label for="light-sleep-toggle">Enable Light Sleep Mode</label>
                    <label class="switch">
                        <input type="checkbox" id="light-sleep-toggle">
                        <span class="slider"></span>
                    </label>
                </div>
            </div>

            <div class="settings-section" id="wifi-config-section" style="display:none;">
                <h4>Wi-Fi Configuration (AP Mode)</h4>
                <input type="text" id="ap-mode-ssid" class="settings-input" placeholder="New Network Name (SSID)">
                <input type="password" id="ap-mode-password" class="settings-input" placeholder="New Password">
                <button class="button" id="ap-mode-connect-button">Connect & Restart Robot</button>
            </div>


            <div class="settings-section">
                <h4>Calibration</h4>
                <div class="settings-subsection">
                    <h5>Left Servo (µs)</h5>
                    <div class="calibration-input-group">
                        <label for="cal-servo-l-stop">Stop Pulse:</label>
                        <input type="number" class="settings-input" id="cal-servo-l-stop" min="1000" max="2000" step="10">
                    </div>
                    <div class="calibration-input-group">
                        <label for="cal-servo-l-dz">Pulse Deadzone:</label>
                        <input type="number" class="settings-input" id="cal-servo-l-dz" min="0" max="200" step="5">
                    </div>
                </div>
                <div class="settings-subsection">
                    <h5>Right Servo (µs)</h5>
                    <div class="calibration-input-group">
                        <label for="cal-servo-r-stop">Stop Pulse:</label>
                        <input type="number" class="settings-input" id="cal-servo-r-stop" min="1000" max="2000" step="10">
                    </div>
                     <div class="calibration-input-group">
                        <label for="cal-servo-r-dz">Pulse Deadzone:</label>
                        <input type="number" class="settings-input" id="cal-servo-r-dz" min="0" max="200" step="5">
                    </div>
                </div>
                <div class="settings-subsection">
                    <h5>LiPo ADC</h5>
                    <div class="calibration-input-group">
                        <label for="cal-lipo-ratio">Voltage Divider Ratio:</label>
                        <input type="number" class="settings-input" id="cal-lipo-ratio" step="0.01" min="1.0" max="10.0">
                    </div>
                </div>
                <div class="calibration-buttons">
                    <button class="button btn-info" id="btn-load-calib">Load Current Values</button>
                    <button class="button" id="btn-save-calib">Save to Robot</button>
                    <button class="button" id="btn-reset-calib">Reset to Defaults</button>
                </div>
            </div>

        </div>
    </div>

    <script>
        // DOM Elements
        const streamImg = document.getElementById('stream');
        const streamFeedbackPlaceholder = document.getElementById('stream-feedback-placeholder');
        const streamFeedbackLoading = document.getElementById('stream-feedback-loading');
        const streamToggle = document.getElementById('stream-toggle');
        
        const joystickPanel = document.getElementById('joystick-panel'); 
        const joystickControl = document.getElementById('joystick-control'); 
        const joystick = document.getElementById('joystick');
        const stick = document.getElementById('stick');
        
        const slidersPanel = document.getElementById('sliders-panel'); 
        const leftSliderArea = document.getElementById('left-slider-area');
        const rightSliderArea = document.getElementById('right-slider-area');
        const leftThumb = document.getElementById('left-thumb');
        const rightThumb = document.getElementById('right-thumb');

        const lipoVoltageDisplay = document.getElementById('lipo-voltage-display');
        const batteryBarLevel = document.getElementById('battery-bar-level');
        const wifiRssiDisplay = document.getElementById('wifi-rssi-display');
        const wifiBarLevel = document.getElementById('wifi-bar-level');
        const connectionStatusDot = document.getElementById('connection-status-dot');
        const fpsStatusDisplay = document.getElementById('fps-status');
        const sleepStatusIndicator = document.getElementById('sleep-status-indicator');

        const settingsToggle = document.getElementById('settings-toggle');
        const settingsPanel = document.getElementById('settings-panel');
        const settingsClose = document.getElementById('settings-close');
        const settingsOverlay = document.getElementById('settings-overlay');
        
        const resolutionSelect = document.getElementById('resolution-select');
        const qualitySlider = document.getElementById('quality-slider');
        const qualityValueDisplay = document.getElementById('quality-value');
        const brightnessSlider = document.getElementById('brightness-slider');
        const brightnessValueDisplay = document.getElementById('brightness-value');
        const contrastSlider = document.getElementById('contrast-slider');
        const contrastValueDisplay = document.getElementById('contrast-value');
        const saturationSlider = document.getElementById('saturation-slider');
        const saturationValueDisplay = document.getElementById('saturation-value');

        const ledSlider = document.getElementById('led-slider');
        const ledValueDisplay = document.getElementById('led-value');
        const lightSleepToggle = document.getElementById('light-sleep-toggle'); 
        const joystickTab = document.getElementById('joystick-tab');
        const slidersTab = document.getElementById('sliders-tab');
        const showFpsToggle = document.getElementById('show-fps-toggle');
        const fpsToggleIcon = document.getElementById('fps-toggle-icon');

        const calServoLstop = document.getElementById('cal-servo-l-stop');
        const calServoLdz = document.getElementById('cal-servo-l-dz');
        const calServoRstop = document.getElementById('cal-servo-r-stop');
        const calServoRdz = document.getElementById('cal-servo-r-dz');
        const calLipoRatio = document.getElementById('cal-lipo-ratio');
        const btnLoadCalib = document.getElementById('btn-load-calib');
        const btnSaveCalib = document.getElementById('btn-save-calib');
        const btnResetCalib = document.getElementById('btn-reset-calib');

        const apModeConfigSection = document.getElementById('ap-mode-config');
        const apModeSsidInput = document.getElementById('ap-mode-ssid');
        const apModePasswordInput = document.getElementById('ap-mode-password');
        const apModeConnectButton = document.getElementById('ap-mode-connect-button');
        const mainControlsOverlay = document.getElementById('main-controls-overlay');


        // State Variables
        let isStreamActive = false;
        let currentUiControlMode = 'joystick'; 
        let joystickIsDragging = false;
        let joystickCurrentX = 0, joystickCurrentY = 0;
        let lastControlSendTime = 0; 
        const CONTROL_THROTTLE_MS = 100; 
        let showFps = false;
        let isServerConnected = true; 
        let clientKeepAliveIntervalId = null;
        let webSocket;
        let isRobotInAPMode = false; 
        let apModeToastShown = false; 
        let attemptingStreamStart = false; 
        const STREAM_START_GRACE_PERIOD = 2500; 
        let streamStartAttemptTime = 0;


        const streamUrlBase = ''; 

        // --- Toast Notification Function ---
        function showToast(message, type = 'info', duration = 3000) {
            const toastContainer = document.getElementById('toast-container');
            const toast = document.createElement('div');
            toast.className = `toast ${type}`; 
            toast.textContent = message;
            toastContainer.appendChild(toast);
            
            setTimeout(() => {
                toast.classList.add('show');
            }, 100);

            setTimeout(() => {
                toast.classList.remove('show');
                setTimeout(() => {
                    if (toast.parentNode === toastContainer) { 
                         toastContainer.removeChild(toast);
                    }
                }, 500); 
            }, duration);
        }
        
        // --- WebSocket Functions ---
        function connectWebSocket() {
            const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${wsProtocol}//${window.location.hostname}:81`; 
            
            webSocket = new WebSocket(wsUrl);

            webSocket.onopen = (event) => {
                console.log('WebSocket connection opened.');
                showToast('Connected to Robot!', 'info');
                updateConnectionStatusIndicator(true);
                if (webSocket.readyState === WebSocket.OPEN) {
                    webSocket.send(JSON.stringify({ command: "get_full_status" }));
                }
                if (isRobotInAPMode) { 
                    handleAPModeUI(true, true); 
                }
            };

            webSocket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    lastClientActivityTimestamp = Date.now(); 
                    updateConnectionStatusIndicator(true);

                    if (data.type === 'status_update') {
                        if (data.lipo_v !== undefined) updateLipoVoltageUI(data.lipo_v);
                        if (data.rssi !== undefined && data.wifi_connected !== undefined) updateWifiStatusUI(data.rssi, data.wifi_connected);
                        if (data.fps !== undefined && showFps && isStreamActive) updateFpsDisplayUI(data.fps);
                        if (data.is_sleeping !== undefined) updateSleepStatusIndicatorUI(data.is_sleeping);
                        
                        if (data.is_ap_mode !== undefined) {
                            const wasInAPMode = isRobotInAPMode;
                            isRobotInAPMode = data.is_ap_mode; 
                            handleAPModeUI(isRobotInAPMode, webSocket.readyState === WebSocket.OPEN);
                            if (isRobotInAPMode && !wasInAPMode && !apModeToastShown) {
                                showToast('Robot in AP Mode. Configure Wi-Fi via Settings.', 'warning', 10000);
                                apModeToastShown = true;
                            } else if (!isRobotInAPMode) {
                                apModeToastShown = false; 
                            }
                        }

                        if (data.battery_status === 'low') showToast('Battery Low!', 'warning', 5000);
                        if (data.battery_status === 'critical') {
                            showToast('BATTERY CRITICAL! Operations limited.', 'error', 10000);
                        }
                        // Update UI based on stream_active from server
                        if (data.stream_active !== undefined) {
                            if (data.stream_active && !isStreamActive) {
                                startStreamUI(); // Server confirms stream is active
                            } else if (!data.stream_active && isStreamActive) {
                                stopStreamUI(); // Server says stream stopped
                            }
                            isStreamActive = data.stream_active; // Sync local state
                        }
                         if (isStreamActive) attemptingStreamStart = false; 


                    } else if (data.type === 'initial_settings') {
                        if (data.resolution) resolutionSelect.value = data.resolution;
                        if (data.quality) { qualitySlider.value = data.quality; qualityValueDisplay.textContent = data.quality; }
                        if (data.brightness !== undefined) { brightnessSlider.value = data.brightness; brightnessValueDisplay.textContent = data.brightness; }
                        if (data.contrast !== undefined) { contrastSlider.value = data.contrast; contrastValueDisplay.textContent = data.contrast; }
                        if (data.saturation !== undefined) { saturationSlider.value = data.saturation; saturationValueDisplay.textContent = data.saturation; }
                        if (data.led_brightness !== undefined) { ledSlider.value = data.led_brightness; ledValueDisplay.textContent = (data.led_brightness == 0 ? "Off" : data.led_brightness + "%");}
                        if (data.sleep_enabled !== undefined) lightSleepToggle.checked = data.sleep_enabled;
                        showToast('Initial settings loaded.', 'info');
                    } else if (data.type === 'calibration_data') {
                        calServoLstop.value = data.sL_stop || ''; calServoLdz.value = data.sL_dz || '';
                        calServoRstop.value = data.sR_stop || ''; calServoRdz.value = data.sR_dz || '';
                        calLipoRatio.value = data.lipo_ratio || '';
                        showToast("Calibration values loaded.", 'info');
                    } else if (data.type === 'sleep_mode_pref') {
                         if (typeof data.enabled === 'boolean') lightSleepToggle.checked = data.enabled;
                    } else if (data.type === 'command_ack') {
                        showToast(data.status || `Command '${data.command}' successful.`, 'info');
                        if (data.command === 'reset_calibration') { 
                            loadCalibrationValuesToUI(); 
                            sendWebSocketCommand({ command: "get_full_status" }); 
                        }
                    } else if (data.type === 'error') {
                        showToast(`Error: ${data.message}`, 'error');
                    }

                } catch (e) {
                    console.error('Error parsing WebSocket message:', e, event.data);
                }
            };

            webSocket.onclose = (event) => {
                console.log('WebSocket connection closed.');
                showToast('Disconnected from Robot.', 'error');
                updateConnectionStatusIndicator(false);
                if (!isRobotInAPMode) { 
                    setTimeout(connectWebSocket, 3000); 
                }
            };

            webSocket.onerror = (error) => {
                console.error('WebSocket error:', error);
                showToast('WebSocket connection error.', 'error');
                updateConnectionStatusIndicator(false);
            };
        }

        function sendWebSocketCommand(command) {
            if (webSocket && webSocket.readyState === WebSocket.OPEN) {
                webSocket.send(JSON.stringify(command));
                lastClientActivityTimestamp = Date.now(); 
            } else {
                showToast('Not connected to robot. Command not sent.', 'error');
                console.error('WebSocket not open. State:', webSocket ? webSocket.readyState : 'null');
            }
        }


        function updateConnectionStatusIndicator(connected) {
            isServerConnected = connected;
            if (connectionStatusDot) {
                connectionStatusDot.classList.toggle('bad', !isServerConnected);
            }
        }
        
        function updateLipoVoltageUI(voltage) {
            lipoVoltageDisplay.textContent = voltage.toFixed(2);
            const minV = 3.2, maxV = 4.2; let perc = ((voltage - minV) / (maxV - minV)) * 100;
            perc = Math.max(0, Math.min(100, perc)); batteryBarLevel.style.width = perc + '%';
            if (voltage > 3.8) batteryBarLevel.style.backgroundColor = 'var(--battery-good)';
            else if (voltage > 3.5) batteryBarLevel.style.backgroundColor = 'var(--battery-medium)';
            else batteryBarLevel.style.backgroundColor = 'var(--battery-low)';
        }

        function updateWifiStatusUI(rssi, connected) {
            if (connected) {
                wifiRssiDisplay.textContent = rssi + ' dBm';
                const minRssi = -90; const maxRssi = -30; 
                let rssiPerc = ((rssi - minRssi) / (maxRssi - minRssi)) * 100;
                rssiPerc = Math.max(0, Math.min(100, rssiPerc)); 
                wifiBarLevel.style.width = rssiPerc + '%';
            } else {
                wifiRssiDisplay.textContent = 'N/A';
                wifiBarLevel.style.width = '0%';
            }
        }
        
        function updateFpsDisplayUI(fps) {
            if (showFps && isStreamActive) {
                fpsStatusDisplay.textContent = `FPS: ${fps.toFixed(1)}`;
                fpsStatusDisplay.style.display = 'block';
            } else {
                fpsStatusDisplay.style.display = 'none';
            }
        }

        function updateSleepStatusIndicatorUI(isSleeping) {
            if (sleepStatusIndicator) {
                sleepStatusIndicator.textContent = isSleeping ? 'Sleeping...' : 'Awake';
            }
        }
        
        function handleAPModeUI(inAPMode, wsConnected = false) {
            isRobotInAPMode = inAPMode;
            if (inAPMode) {
                mainControlsOverlay.style.display = 'block'; 
                if (!wsConnected) { 
                    apModeConfigSection.style.display = 'block';
                    mainControlsOverlay.style.display = 'none'; 
                } else {
                    apModeConfigSection.style.display = 'none';
                }
                if (settingsPanel.classList.contains('active') && document.getElementById('wifi-config-section')) {
                     document.getElementById('wifi-config-section').style.display = 'block';
                }
            } else {
                apModeConfigSection.style.display = 'none';
                mainControlsOverlay.style.display = 'block';
                 if (document.getElementById('wifi-config-section')) {
                    document.getElementById('wifi-config-section').style.display = 'none';
                }
            }
        }


        if (apModeConnectButton) {
            apModeConnectButton.addEventListener('click', () => {
                const ssid = apModeSsidInput.value;
                const password = apModePasswordInput.value;
                if (!ssid) {
                    showToast('Please enter Wi-Fi SSID.', 'error');
                    return;
                }
                sendWebSocketCommand({ command: 'set_wifi_credentials', ssid: ssid, password: password });
                showToast('Credentials sent. Robot will restart. Reconnect to your normal Wi-Fi and refresh this page after a minute.', 'info', 10000);
            });
        }


        function toggleStream() { 
            if (isStreamActive) {
                sendWebSocketCommand({ command: 'stream_control', action: 'stop' });
                // UI update (button to play, hide image) will be handled by WebSocket status_update
            } else {
                attemptingStreamStart = true; 
                streamStartAttemptTime = Date.now();
                sendWebSocketCommand({ command: 'stream_control', action: 'start' });
                
                // Optimistically show loading and try to set img.src
                streamFeedbackPlaceholder.classList.add('hidden');
                streamFeedbackLoading.classList.remove('hidden');
                streamImg.style.display = 'none'; 

                // This timeout is a fallback if WS messages don't update the state
                const streamTimeout = setTimeout(() => {
                    if (attemptingStreamStart || (isStreamActive && streamImg.style.display === 'none')) { 
                        showToast('Stream timed out.', 'error'); 
                        stopStreamUI(); // Update UI to reflect stopped state
                        updateConnectionStatusIndicator(false); 
                    }
                    attemptingStreamStart = false;
                }, 10000); 

                streamImg.onload = () => {
                    clearTimeout(streamTimeout);
                    attemptingStreamStart = false;
                    // Server will confirm via WebSocket if stream is truly active by sending stream_active:true
                    // This just handles the image successfully loading.
                    streamImg.style.display = 'block';
                    streamFeedbackLoading.classList.add('hidden');
                    updateConnectionStatusIndicator(true);
                    // Button state is best handled by WebSocket status_update
                };
                streamImg.onerror = () => { 
                    clearTimeout(streamTimeout);
                    // Only show error if not rapidly after start attempt OR if server still thinks it's streaming
                    if (attemptingStreamStart || isStreamActive) { 
                        if (Date.now() - streamStartAttemptTime > STREAM_START_GRACE_PERIOD) {
                            showToast('Stream error or disconnected.', 'error'); 
                        } else {
                             console.log("Stream onerror fired very quickly, possibly transient during startup.");
                        }
                    }
                    stopStreamUI(); // Ensure UI is reset
                    updateConnectionStatusIndicator(false);
                    attemptingStreamStart = false;
                };
                streamImg.src = streamUrlBase + '/stream?' + new Date().getTime(); 
            }
        }

        function startStreamUI() { 
            isStreamActive = true;
            attemptingStreamStart = false;
            streamFeedbackPlaceholder.classList.add('hidden');
            streamFeedbackLoading.classList.add('hidden'); 
            streamImg.style.display = 'block'; 
            streamToggle.innerHTML = '<span class="material-icons">stop</span>';
            streamToggle.classList.add('active');
            if (showFps) fpsStatusDisplay.style.display = 'block';
        }

        function stopStreamUI() { 
            isStreamActive = false;
            attemptingStreamStart = false; 
            streamImg.style.display = 'none'; streamImg.src = ''; 
            streamFeedbackLoading.classList.add('hidden');
            streamFeedbackPlaceholder.classList.remove('hidden');
            streamToggle.innerHTML = '<span class="material-icons">play_arrow</span>';
            streamToggle.classList.remove('active');
            fpsStatusDisplay.style.display = 'none';
        }
        
        function toggleSettingsPanel() {
            const isActive = settingsPanel.classList.toggle('active');
            settingsOverlay.classList.toggle('active');
            if (isActive) { 
                sendWebSocketCommand({ command: 'get_calibration' }); 
                sendWebSocketCommand({ command: 'get_sleep_mode_pref' });
                // Also show/hide AP config section within settings panel
                if (document.getElementById('wifi-config-section')) {
                    document.getElementById('wifi-config-section').style.display = isRobotInAPMode ? 'block' : 'none';
                }
            }
        }
        settingsToggle.addEventListener('click', toggleSettingsPanel);
        settingsClose.addEventListener('click', toggleSettingsPanel);
        settingsOverlay.addEventListener('click', toggleSettingsPanel);

        function sendCameraSettings() {
            sendWebSocketCommand({
                command: 'set_camera_settings',
                resolution: resolutionSelect.value,
                quality: parseInt(qualitySlider.value),
                brightness: parseInt(brightnessSlider.value),
                contrast: parseInt(contrastSlider.value),
                saturation: parseInt(saturationSlider.value)
            });
        }
        resolutionSelect.addEventListener('change', sendCameraSettings);
        qualitySlider.addEventListener('input', () => qualityValueDisplay.textContent = qualitySlider.value);
        qualitySlider.addEventListener('change', sendCameraSettings);
        brightnessSlider.addEventListener('input', () => brightnessValueDisplay.textContent = brightnessSlider.value);
        brightnessSlider.addEventListener('change', sendCameraSettings);
        contrastSlider.addEventListener('input', () => contrastValueDisplay.textContent = contrastSlider.value);
        contrastSlider.addEventListener('change', sendCameraSettings);
        saturationSlider.addEventListener('input', () => saturationValueDisplay.textContent = saturationSlider.value);
        saturationSlider.addEventListener('change', sendCameraSettings);


        function sendLedBrightness() {
            sendWebSocketCommand({ command: 'set_led', brightness: parseInt(ledSlider.value) });
        }
        ledSlider.addEventListener('input', () => ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : ledSlider.value + "%"));
        ledSlider.addEventListener('change', sendLedBrightness);

        function setLightSleepPreference() {
            sendWebSocketCommand({ command: 'set_sleep_mode', enabled: lightSleepToggle.checked });
        }
        lightSleepToggle.addEventListener('change', setLightSleepPreference);
        
        function setUiControlMode(mode) {
            currentUiControlMode = mode;
            joystickTab.classList.toggle('active', mode === 'joystick'); slidersTab.classList.toggle('active', mode === 'sliders');
            joystickPanel.style.display = (mode === 'joystick') ? 'flex' : 'none'; joystickControl.classList.toggle('active', mode === 'joystick'); 
            slidersPanel.style.display = (mode === 'sliders') ? 'block' : 'none'; slidersPanel.classList.toggle('active', mode === 'sliders');
            sendWebSocketCommand({ command: 'set_control_mode', mode: currentUiControlMode, x:0, y:0, left:0, right:0 });
            resetJoystickVisuals(); resetSlidersVisuals();
        }
        joystickTab.addEventListener('click', () => setUiControlMode('joystick')); slidersTab.addEventListener('click', () => setUiControlMode('sliders'));

        function sendJoystickData(x, y, force = false) {
            const now = Date.now();
            if (force || now - lastControlSendTime > CONTROL_THROTTLE_MS) {
                sendWebSocketCommand({ command: 'control', mode: 'joystick', x: x, y: y });
                lastControlSendTime = now;
            }
        }
        function handleJoystickMove(event) {
            if (!joystickIsDragging) return; const rect = joystick.getBoundingClientRect();
            const centerX = rect.width / 2; const centerY = rect.height / 2;
            const clientX = event.clientX || (event.touches && event.touches[0].clientX); const clientY = event.clientY || (event.touches && event.touches[0].clientY);
            let x = clientX - rect.left - centerX; let y = clientY - rect.top - centerY;
            const maxOffset = rect.width / 2 - stick.offsetWidth / 2;
            x = Math.max(-maxOffset, Math.min(maxOffset, x)); y = Math.max(-maxOffset, Math.min(maxOffset, y));
            stick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
            joystickCurrentX = x / maxOffset; joystickCurrentY = -y / maxOffset; 
            sendJoystickData(joystickCurrentX, joystickCurrentY);
        }
        function resetJoystickVisuals() { stick.style.transform = 'translate(-50%, -50%)'; }
        function onJoystickEnd() { if (!joystickIsDragging) return; joystickIsDragging = false; resetJoystickVisuals(); joystickCurrentX = 0; joystickCurrentY = 0; sendJoystickData(0, 0, true); }
        stick.addEventListener('mousedown', e => { if(currentUiControlMode === 'joystick') { joystickIsDragging = true; e.preventDefault(); }});
        document.addEventListener('mousemove', e => { if(currentUiControlMode === 'joystick' && joystickIsDragging) handleJoystickMove(e); });
        document.addEventListener('mouseup', () => { if(currentUiControlMode === 'joystick') onJoystickEnd(); });
        stick.addEventListener('touchstart', e => { if(currentUiControlMode === 'joystick') { joystickIsDragging = true; e.preventDefault(); handleJoystickMove(e.touches[0]); }}, { passive: false });
        document.addEventListener('touchmove', e => { if(currentUiControlMode === 'joystick' && joystickIsDragging && e.touches.length > 0) {handleJoystickMove(e.touches[0]); e.preventDefault();}}, { passive: false });
        document.addEventListener('touchend', () => { if(currentUiControlMode === 'joystick') onJoystickEnd(); });
        document.addEventListener('touchcancel', () => { if(currentUiControlMode === 'joystick') onJoystickEnd(); });

        function sendSlidersData(left, right, force = false) {
            const now = Date.now();
            if (force || now - lastControlSendTime > CONTROL_THROTTLE_MS) { 
                 sendWebSocketCommand({ command: 'control', mode: 'sliders', left: left, right: right });
                lastControlSendTime = now;
            }
        }
        function setupSlider(thumbElement, trackElement, areaElement) {
            let isDraggingSlider = false;
            function updateThumbPosition(clientY) {
                const trackRect = trackElement.getBoundingClientRect(); let relativeY = clientY - trackRect.top;
                const thumbHeight = thumbElement.offsetHeight; relativeY = Math.max(thumbHeight / 2, Math.min(trackRect.height - thumbHeight / 2, relativeY));
                thumbElement.style.top = `calc(${((relativeY - thumbHeight/2) / (trackRect.height - thumbHeight)) * 100}%)`;
                let value = -(((relativeY - thumbHeight/2) / (trackRect.height - thumbHeight)) * 2 - 1); return Math.max(-1, Math.min(1, value));
            }
            function onSliderDragStart(e) { if (currentUiControlMode !== 'sliders') return; isDraggingSlider = true; e.preventDefault(); }
            function onSliderDragMove(e) {
                if (!isDraggingSlider || currentUiControlMode !== 'sliders') return; const clientY = e.clientY || (e.touches && e.touches[0].clientY);
                if (clientY === undefined) return; let value = updateThumbPosition(clientY);
                let leftVal = (thumbElement === leftThumb) ? value : getSliderCurrentValue(leftThumb, leftSliderArea.querySelector('.slider-track'));
                let rightVal = (thumbElement === rightThumb) ? value : getSliderCurrentValue(rightThumb, rightSliderArea.querySelector('.slider-track'));
                sendSlidersData(leftVal, rightVal); e.preventDefault();
            }
            function onSliderDragEnd() {
                if (!isDraggingSlider || currentUiControlMode !== 'sliders') return; isDraggingSlider = false; thumbElement.style.top = '50%'; 
                let leftVal = (thumbElement === leftThumb) ? 0 : getSliderCurrentValue(leftThumb, leftSliderArea.querySelector('.slider-track'));
                let rightVal = (thumbElement === rightThumb) ? 0 : getSliderCurrentValue(rightThumb, rightSliderArea.querySelector('.slider-track'));
                sendSlidersData(leftVal, rightVal, true);
            }
            areaElement.addEventListener('mousedown', onSliderDragStart); areaElement.addEventListener('touchstart', onSliderDragStart, { passive: false });
            document.addEventListener('mousemove', onSliderDragMove); document.addEventListener('touchmove', e => { if (isDraggingSlider) onSliderDragMove(e); }, { passive: false });
            document.addEventListener('mouseup', onSliderDragEnd); document.addEventListener('touchend', onSliderDragEnd); document.addEventListener('touchcancel', onSliderDragEnd);
        }
        function getSliderCurrentValue(thumbElement, trackElement) {
            if (!trackElement) return 0; const trackRect = trackElement.getBoundingClientRect(); const thumbHeight = thumbElement.offsetHeight;
            const thumbStyleTopPercent = parseFloat(thumbElement.style.top) || 50;
            const currentThumbPixelTopInTrack = (thumbStyleTopPercent / 100) * (trackRect.height - thumbHeight) + thumbHeight/2;
            let value = -((currentThumbPixelTopInTrack - thumbHeight/2) / (trackRect.height - thumbHeight) * 2 - 1); return Math.max(-1, Math.min(1, value));
        }
        function resetSlidersVisuals() { leftThumb.style.top = '50%'; rightThumb.style.top = '50%'; }
        setupSlider(leftThumb, leftSliderArea.querySelector('.slider-track'), leftSliderArea); setupSlider(rightThumb, rightSliderArea.querySelector('.slider-track'), rightSliderArea);

        function startStatusUpdates() { /* Now handled by WebSocket pushes from server */ }
        function stopStatusUpdates() { /* Now handled by WebSocket pushes from server */ }
        
        showFpsToggle.addEventListener('click', () => {
            showFps = !showFps; 
            fpsToggleIcon.textContent = showFps ? 'visibility' : 'visibility_off';
            if (showFps && isStreamActive) {
                fpsStatusDisplay.style.display = 'block'; 
            } else {
                fpsStatusDisplay.style.display = 'none';
            }
        });

        function loadCalibrationValuesToUI() { 
            sendWebSocketCommand({ command: 'get_calibration' });
            sendWebSocketCommand({ command: 'get_sleep_mode_pref' });
        }
        function saveCalibrationValuesFromUI() { 
            const calData = { sL_stop: parseInt(calServoLstop.value) || 0, sL_dz: parseInt(calServoLdz.value) || 0, sR_stop: parseInt(calServoRstop.value) || 0, sR_dz: parseInt(calServoRdz.value) || 0, lipo_ratio: parseFloat(calLipoRatio.value) || 0 };
            sendWebSocketCommand({ command: 'set_calibration', data: calData });
        }
        function resetCalibrationOnRobot() { 
            if (!confirm("Are you sure you want to reset ALL calibration values (Servos, LiPo, Camera, Sleep Mode) to firmware defaults?")) return;
            sendWebSocketCommand({ command: 'reset_calibration' });
        }
        btnLoadCalib.addEventListener('click', loadCalibrationValuesToUI); 
        btnSaveCalib.addEventListener('click', saveCalibrationValuesFromUI); 
        btnResetCalib.addEventListener('click', resetCalibrationOnRobot);

        function loadInitialSettings() { 
            ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : ledSlider.value + "%"); 
            setUiControlMode('joystick'); 
            updateConnectionStatusIndicator(true); 
            streamFeedbackPlaceholder.classList.remove('hidden'); 
            streamFeedbackLoading.classList.add('hidden');

            if (clientKeepAliveIntervalId) clearInterval(clientKeepAliveIntervalId);
            clientKeepAliveIntervalId = setInterval(() => {
                if (webSocket && webSocket.readyState === WebSocket.OPEN) {
                    webSocket.send(JSON.stringify({ command: 'ping' }));
                } else if (webSocket && webSocket.readyState !== WebSocket.CONNECTING && !isRobotInAPMode) {
                    console.log("Keep-alive: WebSocket not open, attempting reconnect.");
                    connectWebSocket(); 
                }
            }, 5000); 
        }
        document.body.addEventListener('touchstart', function(e) { if (e.target === document.body || e.target === document.querySelector('.container') || e.target === document.querySelector('.video-container')) e.preventDefault(); }, { passive: false });
        document.body.addEventListener('touchmove', function(e) { if (e.target === document.body || e.target === document.querySelector('.container') || e.target === document.querySelector('.video-container')) e.preventDefault(); }, { passive: false });
        
        streamToggle.addEventListener('click', toggleStream); 
        document.addEventListener('DOMContentLoaded', () => {
            connectWebSocket(); 
            loadInitialSettings(); 
        });
    </script>
</body>
</html>
)RAW_HTML";

// =================================================================================================
// 4. FUNCTION PROTOTYPES 
// =================================================================================================
bool checkAuthentication(); 
void initCamera(); 
void initWiFi(bool forceAP = false); 
void setupAPMode();
void setupLed(); 
void applyLedBrightness(int percent); 
float gammaCorrection(float brightnessPercent, float gamma); 
void handleLedControl(uint8_t clientId, JsonDocument& doc); 
void initServos(); 
void TaskHttpServer(void *pvParameters); 
void TaskOtaHandler(void *pvParameters); 
void sleepManagementTask(void *pvParameters); 
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length); 
void broadcastStatusUpdate(); 
void setupWebServerRoutes(); 
void setupOTA(); 
void loadCalibrationFromNVS(); 
void saveCalibrationToNVS(); 
void saveCameraSettingsToNVS(); 
void saveSleepModePrefToNVS(); 
void saveWiFiCredentialsToNVS(const char* ssid, const char* password);
void handleGetCalibration(uint8_t clientId); 
void handleSetCalibration(uint8_t clientId, JsonDocument& doc); 
void handleResetCalibration(uint8_t clientId); 
// void handleWifiStatus(); // Replaced by WebSocket
void handleGetSleepModePref(uint8_t clientId); 
void handleSetSleepModePref(uint8_t clientId, JsonDocument& doc); 
void handleSetWiFiCredentials(uint8_t clientId, JsonDocument& doc);
int getCalibratedServoPulse(float controlValue, int stop_us, int pulse_deadzone_us, float input_deadzone_threshold = 0.05f); 
void processJoystickControlServos(float x, float y); 
void processSlidersControlServos(float leftSlider, float rightSlider); 
float readLipoVoltage(); 
// void handleLipoVoltage(); // Replaced by WebSocket
ra_filter_t* ra_filter_init(ra_filter_t* filter, size_t sample_size); 
int ra_filter_run(ra_filter_t* filter, int value); 
void handleStartStreamForHttpRequest(); // For HTTP GET /stream
// void handleStopStream(uint8_t clientId); // Now only via WebSocket
// void handleStreamStatus(); // Replaced by WebSocket
void streamTask(void* parameter); 
bool sendMJPEGFrame(const uint8_t* buf, size_t len); 
const char* framesizeToString(framesize_t fs); 
framesize_t stringToFramesize(const String& fsStr); 
void handleCameraSettings(uint8_t clientId, JsonDocument& doc); 

// =================================================================================================
// 5. SETUP FUNCTION
// =================================================================================================
void setup() {
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

    Serial.begin(115200); 
    Serial.println("\n\nESP32-CAM Servo Robot V3.7.3 (Stream/AP Fix & Battery Critical) Initializing..."); 

    lastClientActivityTimestamp = millis(); 
    loadCalibrationFromNVS(); 

    if(!psramFound()){
        Serial.println("PSRAM not found! Camera performance will be limited.");
    } else {
        Serial.println("PSRAM found.");
    }

    initCamera(); 
    initWiFi(); 

    setupLed(); 
    initServos();

    ra_filter.values = ra_values_array; 
    ra_filter_init(&ra_filter, RA_FILTER_SAMPLE_SIZE); 

    setupWebServerRoutes(); 
    webSocket.begin(); 
    webSocket.onEvent(webSocketEvent); 

    setupOTA(); 

    xTaskCreatePinnedToCore(TaskHttpServer, "HttpServerTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskOtaHandler, "OTATask", 4096, NULL, 1, NULL, 0);  
    xTaskCreatePinnedToCore(sleepManagementTask, "SleepMgmtTask", 4096, NULL, 0, &sleepManagementTaskHandle, 0); 

    Serial.println("Setup complete. Main tasks started.");
    if (!isInAPMode) {
        Serial.printf("HTTP server on port %d. WebSocket on port %d. Stream on /stream.\n", HTTP_PORT, WEBSOCKET_PORT);
    }
    Serial.println("Calibrated settings loaded from NVS or defaults applied.");
    Serial.printf("HTTP Auth User: %s (Password: ***) <<< CHANGE THESE!\n", HTTP_AUTH_USERNAME); 
    Serial.printf("OTA Hostname: %s (Password: ***) <<< CHANGE THIS!\n", OTA_HOSTNAME);       
}

// =================================================================================================
// 6. LOOP FUNCTION 
// =================================================================================================
void loop() {
    vTaskDelete(NULL); 
}

// =================================================================================================
// 7. FUNCTION IMPLEMENTATIONS
// =================================================================================================

// --- Light Sleep Management Task ---
void sleepManagementTask(void *pvParameters) {
    (void)pvParameters;
    const TickType_t checkInterval = pdMS_TO_TICKS(15000); 

    for (;;) {
        vTaskDelay(checkInterval);

        if (g_light_sleep_enabled && !isLightSleeping && !isStreaming && !otaIsActive && !isInAPMode &&
            (millis() - lastClientActivityTimestamp > lightSleepTimeoutMs)) {
            
            processJoystickControlServos(0,0); 
            applyLedBrightness(0); 
            
            esp_wifi_set_ps(WIFI_PS_MIN_MODEM); 
            esp_sleep_enable_wifi_wakeup();
            esp_sleep_enable_timer_wakeup(lightSleepPeriodicWakeupUs); 
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO); 
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_AUTO);
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_AUTO);

            isLightSleeping = true;
            
            esp_light_sleep_start(); 

            // ---- Execution resumes here after wake-up ----
            isLightSleeping = false;
            esp_wifi_set_ps(WIFI_PS_NONE); 
            
            esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
            bool clientActivityJustOccurred = (millis() - lastClientActivityTimestamp < 2000); 

            if (WiFi.status() != WL_CONNECTED) {
                WiFi.reconnect();
                vTaskDelay(pdMS_TO_TICKS(1000)); 
            }
            
            applyLedBrightness(20); 
            vTaskDelay(pdMS_TO_TICKS(500)); 
            applyLedBrightness(g_current_led_brightness_percent); 

            if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER && !clientActivityJustOccurred) {
                lastClientActivityTimestamp = millis() - lightSleepTimeoutMs + (30 * 1000); 
            } else { 
                lastClientActivityTimestamp = millis(); 
            }
        } else if (isLightSleeping) {
            isLightSleeping = false; 
            esp_wifi_set_ps(WIFI_PS_NONE);
        }
    }
}


// --- Authentication Helper ---
bool checkAuthentication() {
    if (!server.authenticate(HTTP_AUTH_USERNAME, HTTP_AUTH_PASSWORD)) {
        server.requestAuthentication(BASIC_AUTH, AUTH_REALM, "Authentication failed or required. Please login.");
        return false; 
    }
    // lastClientActivityTimestamp = millis(); // Updated by individual handlers
    return true; 
}

// --- NVS Calibration Functions ---
void loadCalibrationFromNVS() {
    preferences.begin("robotCalib", true); 
    servo_left_stop_us    = preferences.getInt("sL_stop", DEFAULT_SERVO_STOP_US);
    servo_left_pulse_deadzone_us = preferences.getInt("sL_dz", DEFAULT_SERVO_PULSE_DEADZONE_US);
    servo_right_stop_us   = preferences.getInt("sR_stop", DEFAULT_SERVO_STOP_US);
    servo_right_pulse_deadzone_us = preferences.getInt("sR_dz", DEFAULT_SERVO_PULSE_DEADZONE_US);
    lipo_calib_voltage_divider_ratio = preferences.getFloat("lipo_ratio", DEFAULT_VOLTAGE_DIVIDER_RATIO);
    nvs_cam_framesize = static_cast<framesize_t>(preferences.getInt("cam_res_idx", static_cast<int>(FRAMESIZE_QVGA))); 
    nvs_cam_quality = preferences.getInt("cam_qual", 12); 
    nvs_cam_brightness = preferences.getInt("cam_brght", 0);
    nvs_cam_contrast   = preferences.getInt("cam_contr", 0);
    nvs_cam_saturation = preferences.getInt("cam_sat", 0);
    g_light_sleep_enabled = preferences.getBool("sleep_en", true); 
    preferences.getString("sta_ssid", nvs_wifi_ssid, sizeof(nvs_wifi_ssid));
    preferences.getString("sta_psk", nvs_wifi_password, sizeof(nvs_wifi_password));
    preferences.end();
}

void saveCalibrationToNVS() { 
    preferences.begin("robotCalib", false); 
    preferences.putInt("sL_stop", servo_left_stop_us); 
    preferences.putInt("sL_dz", servo_left_pulse_deadzone_us);
    preferences.putInt("sR_stop", servo_right_stop_us); 
    preferences.putInt("sR_dz", servo_right_pulse_deadzone_us);
    preferences.putFloat("lipo_ratio", lipo_calib_voltage_divider_ratio);
    preferences.end(); 
}

void saveCameraSettingsToNVS() { 
    preferences.begin("robotCalib", false); 
    preferences.putInt("cam_res_idx", static_cast<int>(nvs_cam_framesize)); 
    preferences.putInt("cam_qual", nvs_cam_quality);
    preferences.putInt("cam_brght", nvs_cam_brightness);
    preferences.putInt("cam_contr", nvs_cam_contrast);
    preferences.putInt("cam_sat", nvs_cam_saturation);
    preferences.end(); 
}

void saveSleepModePrefToNVS(){
    preferences.begin("robotCalib", false);
    preferences.putBool("sleep_en", g_light_sleep_enabled);
    preferences.end();
}

void saveWiFiCredentialsToNVS(const char* ssid, const char* password) {
    preferences.begin("robotCalib", false);
    preferences.putString("sta_ssid", ssid);
    preferences.putString("sta_psk", password);
    preferences.end();
    strlcpy(nvs_wifi_ssid, ssid, sizeof(nvs_wifi_ssid));
    strlcpy(nvs_wifi_password, password, sizeof(nvs_wifi_password));
}


void handleGetCalibration(uint8_t clientId) { 
    StaticJsonDocument<300> doc; 
    doc["type"] = "calibration_data";
    doc["sL_stop"] = servo_left_stop_us; doc["sL_dz"] = servo_left_pulse_deadzone_us;
    doc["sR_stop"] = servo_right_stop_us; doc["sR_dz"] = servo_right_pulse_deadzone_us;
    doc["lipo_ratio"] = lipo_calib_voltage_divider_ratio;
    String response; serializeJson(doc, response); 
    webSocket.sendTXT(clientId, response);
}

void handleSetCalibration(uint8_t clientId, JsonDocument& doc) { 
    JsonObject data = doc["data"];
    servo_left_stop_us    = data["sL_stop"] | DEFAULT_SERVO_STOP_US; 
    servo_left_pulse_deadzone_us = data["sL_dz"] | DEFAULT_SERVO_PULSE_DEADZONE_US;
    servo_right_stop_us   = data["sR_stop"] | DEFAULT_SERVO_STOP_US; 
    servo_right_pulse_deadzone_us = data["sR_dz"] | DEFAULT_SERVO_PULSE_DEADZONE_US;
    lipo_calib_voltage_divider_ratio = data["lipo_ratio"] | DEFAULT_VOLTAGE_DIVIDER_RATIO;
    saveCalibrationToNVS(); 
    
    StaticJsonDocument<100> ackDoc;
    ackDoc["type"] = "command_ack";
    ackDoc["command"] = "set_calibration";
    ackDoc["status"] = "Servo/LiPo Calibration Saved";
    String response; serializeJson(ackDoc, response);
    webSocket.sendTXT(clientId, response);
}

void handleResetCalibration(uint8_t clientId) { 
    servo_left_stop_us    = DEFAULT_SERVO_STOP_US; servo_left_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;
    servo_right_stop_us   = DEFAULT_SERVO_STOP_US; servo_right_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;
    lipo_calib_voltage_divider_ratio = DEFAULT_VOLTAGE_DIVIDER_RATIO; 
    saveCalibrationToNVS(); 
    
    nvs_cam_framesize = FRAMESIZE_QVGA; nvs_cam_quality = 12; 
    nvs_cam_brightness = 0; nvs_cam_contrast = 0; nvs_cam_saturation = 0;
    saveCameraSettingsToNVS();          
    
    g_light_sleep_enabled = true; 
    saveSleepModePrefToNVS();

    sensor_t * s = esp_camera_sensor_get(); 
    if (s) { 
        s->set_framesize(s, nvs_cam_framesize); 
        s->set_quality(s, nvs_cam_quality);
        s->set_brightness(s, nvs_cam_brightness);
        s->set_contrast(s, nvs_cam_contrast);
        s->set_saturation(s, nvs_cam_saturation);
    }
    
    StaticJsonDocument<100> ackDoc;
    ackDoc["type"] = "command_ack";
    ackDoc["command"] = "reset_calibration";
    ackDoc["status"] = "All Calibration Reset to Defaults";
    String response; serializeJson(ackDoc, response);
    webSocket.sendTXT(clientId, response);
}

// --- OTA Setup and Task ---
void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME); 
    ArduinoOTA.setPassword(OTA_PASSWORD); 
    ArduinoOTA
        .onStart([]() { 
            otaIsActive = true; 
            String type; if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch"; else type = "filesystem"; 
            Serial.println("OTA: Start updating " + type); 
            if (isStreaming) { 
                isStreaming = false; 
                if (streamTaskHandle != NULL) {
                    if (streamClient && streamClient.connected()) streamClient.stop();
                    vTaskDelay(pdMS_TO_TICKS(200)); 
                    if (eTaskGetState(streamTaskHandle) != eDeleted) {
                        vTaskDelete(streamTaskHandle);
                    }
                    streamTaskHandle = NULL;
                }
            }
        })
        .onEnd([]() { otaIsActive = false; Serial.println("\nOTA: End. Rebooting..."); }) 
        .onProgress([](unsigned int progress, unsigned int total) { Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100))); }) 
        .onError([](ota_error_t error) { 
            otaIsActive = false; 
            Serial.printf("OTA Error[%u]: ", error); 
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
    ArduinoOTA.begin(); 
    Serial.println("OTA Service Ready."); 
}

void TaskOtaHandler(void *pvParameters) { 
    (void)pvParameters; 
    for (;;) { ArduinoOTA.handle(); vTaskDelay(pdMS_TO_TICKS(10)); } 
}

// --- Camera Initialization ---
void initCamera() { 
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; 
    config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; 
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM; 
    config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM; 
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM; 
    config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = CAM_XCLK_FREQ; 
    config.pixel_format = CAM_PIXEL_FORMAT; 
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; 
    config.fb_location = CAMERA_FB_IN_PSRAM;   

    config.frame_size = nvs_cam_framesize; 
    config.jpeg_quality = nvs_cam_quality;        
    
    if (psramFound()) {
        config.fb_count = CAM_FB_COUNT;   
    } else { 
        Serial.println("Warning: PSRAM not found. Overriding camera settings to QQVGA.");
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 15; 
        config.fb_count = 1;
        nvs_cam_framesize = FRAMESIZE_QQVGA;
        nvs_cam_quality = 15;
        nvs_cam_brightness = 0; nvs_cam_contrast = 0; nvs_cam_saturation = 0;
        saveCameraSettingsToNVS(); 
    }

    esp_err_t err = esp_camera_init(&config); 
    if (err != ESP_OK) { 
        Serial.printf("Camera init failed with error 0x%x: %s\n", err, esp_err_to_name(err)); 
        ESP.restart(); return; 
    }
    Serial.println("Camera initialized successfully.");

    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, nvs_cam_brightness);
        s->set_contrast(s, nvs_cam_contrast);
        s->set_saturation(s, nvs_cam_saturation);
        if (s->id.PID == OV3660_PID) { 
            s->set_vflip(s, 1);       
        }
    }
}


// --- WiFi Initialization ---
void initWiFi(bool forceAP) { 
    WiFi.mode(WIFI_STA); 
    if (forceAP) {
        setupAPMode();
        return;
    }

    if (strlen(nvs_wifi_ssid) > 0 && strcmp(nvs_wifi_ssid, "YOUR_DEFAULT_SSID") != 0 ) { 
        WiFi.setHostname(OTA_HOSTNAME); 
        WiFi.begin(nvs_wifi_ssid, nvs_wifi_password);
        Serial.print("Connecting to STA: "); Serial.print(nvs_wifi_ssid);
        unsigned long startTime = millis();
        int retries = 0;
        const int maxRetries = 20; 

        while (WiFi.status() != WL_CONNECTED && retries < maxRetries) { 
            Serial.print("."); 
            delay(500); 
            retries++;
        }
        if (WiFi.status() == WL_CONNECTED) { 
            Serial.println("\nConnected to WiFi.");
            Serial.print("IP address: "); Serial.println(WiFi.localIP()); 
            lastClientActivityTimestamp = millis(); 
            isInAPMode = false;
            return; 
        } else { 
            Serial.println("\nFailed to connect to STA. Starting AP mode.");
        }
    } else {
        Serial.println("No valid STA credentials stored or using defaults. Starting AP mode.");
    }
    setupAPMode(); 
}

void setupAPMode() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP Mode Enabled. SSID: "); Serial.print(AP_SSID);
    // Serial.print(" | Password: "); Serial.println(AP_PASSWORD); // Don't log AP password
    Serial.print("AP IP address: "); Serial.println(IP);
    isInAPMode = true;
    lastClientActivityTimestamp = millis(); 
}


// --- LED Control Functions ---
void setupLed() { 
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution  = (ledc_timer_bit_t)LED_RESOLUTION_BITS, .timer_num = LED_LEDC_TIMER_NUM, .freq_hz = LED_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK }; 
    if (ledc_timer_config(&ledc_timer) != ESP_OK) { return; }
    ledc_channel_config_t ledc_channel_conf = { .gpio_num   = LED_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LED_LEDC_CHANNEL_NUM, .intr_type  = LEDC_INTR_DISABLE, .timer_sel  = LED_LEDC_TIMER_NUM, .duty = 0, .hpoint = 0 }; 
    if (ledc_channel_config(&ledc_channel_conf) != ESP_OK) { return; }
    applyLedBrightness(0); 
}

float gammaCorrection(float brightnessPercent, float gamma) { 
    float normalizedBrightness = constrain(brightnessPercent / 100.0f, 0.0f, 1.0f);
    return pow(normalizedBrightness, gamma);
}

void applyLedBrightness(int percent) {
    if (isBatteryCritical && percent > 0) { 
        percent = 0;
    }
    g_current_led_brightness_percent = constrain(percent, 0, 100);
    float correctedBrightnessNormalized = gammaCorrection(static_cast<float>(g_current_led_brightness_percent), LED_GAMMA);
    uint32_t max_duty = (1 << LED_RESOLUTION_BITS) - 1;
    uint32_t pwmValue = constrain(static_cast<uint32_t>(correctedBrightnessNormalized * max_duty), static_cast<uint32_t>(0), max_duty);
    
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM, pwmValue) == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM);
    }
}

void handleLedControl(uint8_t clientId, JsonDocument& doc) { 
    lastClientActivityTimestamp = millis();
    if (doc.containsKey("brightness")) { 
        int brightnessPercent = doc["brightness"];
        if (!isBatteryCritical || brightnessPercent == 0) { 
             applyLedBrightness(brightnessPercent);
        }
    } 
}

// --- Servo Control Functions ---
void initServos() { 
    ESP32PWM::allocateTimer(0); 
    ESP32PWM::allocateTimer(1); 
    servoLeft.attach(SERVO_LEFT_PIN); 
    servoRight.attach(SERVO_RIGHT_PIN);
    servoLeft.writeMicroseconds(servo_left_stop_us);    
    servoRight.writeMicroseconds(servo_right_stop_us);  
}

int getCalibratedServoPulse(float controlValue, int stop_us, int pulse_deadzone_us, float input_deadzone_threshold) { 
    controlValue = constrain(controlValue, -1.0f, 1.0f);
    if (abs(controlValue) < input_deadzone_threshold) return stop_us; 
    int max_cw_us = stop_us + SERVO_PULSE_SPAN_US; 
    int max_ccw_us = stop_us - SERVO_PULSE_SPAN_US; 
    if (controlValue > 0) { 
        return static_cast<int>(map(static_cast<long>(controlValue * 1000), static_cast<long>(input_deadzone_threshold * 1000), 1000L, stop_us + pulse_deadzone_us, max_cw_us));
    } else { 
        return static_cast<int>(map(static_cast<long>(controlValue * 1000), -1000L, -static_cast<long>(input_deadzone_threshold * 1000), max_ccw_us, stop_us - pulse_deadzone_us));
    } 
}

void processJoystickControlServos(float x, float y) { 
    if (isBatteryCritical) { 
        servoLeft.writeMicroseconds(servo_left_stop_us);
        servoRight.writeMicroseconds(servo_right_stop_us);
        return;
    }
    float leftSpeed = y; float rightSpeed = y; 
    if (x > 0.05f) { 
        rightSpeed = y * (1.0f - (x * 1.5f)); 
        if (fabs(y) < 0.15f) { leftSpeed = x; rightSpeed = -x; } 
    } else if (x < -0.05f) { 
        leftSpeed = y * (1.0f + (x * 1.5f)); 
        if (fabs(y) < 0.15f) { leftSpeed = x; rightSpeed = -x; } 
    } 
    leftSpeed = constrain(leftSpeed, -1.0f, 1.0f); 
    rightSpeed = constrain(rightSpeed, -1.0f, 1.0f);
    int leftPulse = getCalibratedServoPulse(leftSpeed, servo_left_stop_us, servo_left_pulse_deadzone_us);
    int rightPulse = getCalibratedServoPulse(-rightSpeed, servo_right_stop_us, servo_right_pulse_deadzone_us); 
    servoLeft.writeMicroseconds(leftPulse); 
    servoRight.writeMicroseconds(rightPulse); 
}

void processSlidersControlServos(float leftSlider, float rightSlider) { 
    if (isBatteryCritical) { 
        servoLeft.writeMicroseconds(servo_left_stop_us);
        servoRight.writeMicroseconds(servo_right_stop_us);
        return;
    }
    int leftPulse = getCalibratedServoPulse(constrain(leftSlider, -1.0f, 1.0f), servo_left_stop_us, servo_left_pulse_deadzone_us);
    int rightPulse = getCalibratedServoPulse(constrain(-rightSlider, -1.0f, 1.0f), servo_right_stop_us, servo_right_pulse_deadzone_us); 
    servoLeft.writeMicroseconds(leftPulse); 
    servoRight.writeMicroseconds(rightPulse);
}

// --- LiPo ADC Reading ---
float readLipoVoltage() { 
    uint32_t adc_reading = 0; 
    for (int i = 0; i < 16; i++) { adc_reading += analogRead(LIPO_ADC_PIN); } 
    adc_reading /= 16;
    float voltageAtPin = adc_reading * (DEFAULT_ADC_REF_VOLTAGE / (static_cast<float>((1 << ADC_RESOLUTION_BITS) -1.0)) );
    return voltageAtPin * lipo_calib_voltage_divider_ratio; 
}

// --- Stream Handling Functions ---
ra_filter_t* ra_filter_init(ra_filter_t* filter, size_t sample_size) { 
    memset(filter, 0, sizeof(ra_filter_t)); 
    filter->values = ra_values_array;       
    filter->size = sample_size;             
    if (!filter->values) { return nullptr; } 
    return filter; 
}

int ra_filter_run(ra_filter_t* filter, int value) { 
    if (!filter->values || filter->size == 0) return value; 
    filter->sum -= filter->values[filter->index]; 
    filter->values[filter->index] = value; 
    filter->sum += filter->values[filter->index]; 
    filter->index = (filter->index + 1) % filter->size; 
    if (filter->count < filter->size) filter->count++; 
    if (filter->count == 0) return value; 
    return filter->sum / filter->count; 
}

// This function is called by the HTTP GET /stream route
void handleStartStreamForHttpRequest() {
    lastClientActivityTimestamp = millis();
    if (isBatteryCritical) {
        server.send(503, "text/plain", "Battery critical, stream disabled.");
        return;
    }

    if (!isStreaming) { 
        // This means the /stream HTTP endpoint was hit without a prior WebSocket "start_stream" command.
        // For robustness, we can allow it, but it's not the primary intended flow.
        // Serial.println("HTTP /stream hit without WS pre-signal. Starting stream."); // No Serial
        isStreaming = true; 
        broadcastStatusUpdate(); // Inform WS clients that stream is now active
    }
    
    // If another HTTP stream client is already active, stop it.
    if (streamTaskHandle != NULL && streamClient && streamClient.connected() && streamClient != server.client()) { 
        // This logic might be redundant if streamTaskHandle correctly nullifies on client disconnect
        // but added as a safeguard for direct HTTP /stream requests from multiple sources.
        bool oldIsStreaming = isStreaming; // Store current state
        isStreaming = false; // Signal old task to stop
        streamClient.stop(); 
        vTaskDelay(pdMS_TO_TICKS(250)); 
        if (eTaskGetState(streamTaskHandle) != eDeleted) {
             vTaskDelete(streamTaskHandle);
        }
        streamTaskHandle = NULL;
        isStreaming = oldIsStreaming; // Restore intended streaming state for the new client
    }
    
    streamClient = server.client(); 
    if (!streamClient || !streamClient.connected()) { 
        isStreaming = false; 
        broadcastStatusUpdate();
        return; 
    }

    String response = "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: " + String(STREAM_CONTENT_TYPE) + "\r\nConnection: keep-alive\r\nCache-Control: no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0\r\nPragma: no-cache\r\nExpires: -1\r\n\r\n"; 
    if (streamClient.print(response) != response.length()) { 
        streamClient.stop(); 
        isStreaming = false; 
        broadcastStatusUpdate();
        return; 
    }
    
    if (streamTaskHandle == NULL) { // Only create task if not already running
        xTaskCreatePinnedToCore( streamTask, "StreamTask", STREAM_TASK_STACK_SIZE, NULL, STREAM_TASK_PRIORITY, &streamTaskHandle, STREAM_TASK_CORE );
    }
}


void handleStopStream(uint8_t clientId) { // clientId can be 0 if from HTTP
    lastClientActivityTimestamp = millis();
    if (isStreaming) { // Only act if currently streaming
        isStreaming = false; // Signal streamTask to stop
        // The streamTask will handle closing streamClient and deleting itself.
    }
    
    broadcastStatusUpdate(); // Update all WebSocket clients

    if (clientId == 0 && server.uri() == "/stopstream") { // If called from HTTP
         server.sendHeader("Connection", "close");
         server.send(200, "text/plain", "Stream stop requested.");
    }
}

bool sendMJPEGFrame(const uint8_t* buf, size_t len) { 
    if (!streamClient || !streamClient.connected()) { isStreaming = false; return false; } 
    if (streamClient.print(STREAM_BOUNDARY) != strlen(STREAM_BOUNDARY)) { isStreaming = false; return false; } 
    char hdrBuf[HDR_BUF_LEN]; 
    snprintf(hdrBuf, HDR_BUF_LEN, STREAM_PART, len); 
    if (streamClient.print(hdrBuf) != strlen(hdrBuf)) { isStreaming = false; return false; } 
    if (streamClient.write(buf, len) != len) { isStreaming = false; return false; } 
    if (streamClient.print("\r\n") != 2) { isStreaming = false; return false; } 
    return true; 
}

void streamTask(void* parameter) { 
    (void)parameter; 
    camera_fb_t *fb = NULL; 
    int64_t last_frame_us = esp_timer_get_time();
    
    if (!streamBuffer && CAM_PIXEL_FORMAT != PIXFORMAT_JPEG) { 
        streamBuffer = static_cast<uint8_t*>(ps_malloc(MAX_FRAME_SIZE)); 
        if (!streamBuffer) { 
            isStreaming = false; 
            streamTaskHandle = NULL; 
            broadcastStatusUpdate();
            vTaskDelete(NULL); 
            return;
        } 
    }
    
    while (isStreaming) { 
        if (isBatteryCritical) { 
            isStreaming = false; 
            break;
        }
        lastClientActivityTimestamp = millis(); 
        if (!streamClient || !streamClient.connected()) { 
            break; 
        }
        
        fb = esp_camera_fb_get(); 
        if (!fb) { 
            vTaskDelay(pdMS_TO_TICKS(50)); 
            continue; 
        }
        
        bool success = false;
        if (fb->format == PIXFORMAT_JPEG) { 
            success = sendMJPEGFrame(fb->buf, fb->len); 
        } else { 
            success = false; 
        }
        
        esp_camera_fb_return(fb); 
        fb = NULL; 

        if (!success) { 
            break; 
        }
        
        int64_t fr_end_us = esp_timer_get_time(); 
        int64_t frame_time_us = fr_end_us - last_frame_us;
        last_frame_us = fr_end_us; 
        ra_filter_run(&ra_filter, static_cast<int>(frame_time_us)); 
        
        vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS)); 
    }

    if (streamClient && streamClient.connected()) {
        streamClient.stop(); 
    }
    
    if (xTaskGetCurrentTaskHandle() == streamTaskHandle) {
        streamTaskHandle = NULL;
    }
    isStreaming = false; // Ensure this is false when task ends
    broadcastStatusUpdate(); // Update clients that stream is no longer active
    vTaskDelete(NULL); 
}

// --- Sleep Mode Preference Handlers (via WebSocket) ---
void handleGetSleepModePref(uint8_t clientId) {
    StaticJsonDocument<100> doc;
    doc["type"] = "sleep_mode_pref";
    doc["enabled"] = g_light_sleep_enabled;
    String response; serializeJson(doc, response);
    webSocket.sendTXT(clientId, response);
}

void handleSetSleepModePref(uint8_t clientId, JsonDocument& doc) {
    if (doc.containsKey("enabled")) {
        g_light_sleep_enabled = doc["enabled"];
        saveSleepModePrefToNVS(); 
        
        StaticJsonDocument<100> ackDoc;
        ackDoc["type"] = "command_ack";
        ackDoc["command"] = "set_sleep_mode";
        ackDoc["status"] = g_light_sleep_enabled ? "Light sleep enabled." : "Light sleep disabled.";
        String response; serializeJson(ackDoc, response);
        webSocket.sendTXT(clientId, response);
    }
}

// --- Set New Wi-Fi Credentials (via WebSocket, typically in AP Mode) ---
void handleSetWiFiCredentials(uint8_t clientId, JsonDocument& doc) {
    lastClientActivityTimestamp = millis(); 
    if (doc.containsKey("ssid") && doc.containsKey("password")) {
        String ssid = doc["ssid"];
        String password = doc["password"];
        saveWiFiCredentialsToNVS(ssid.c_str(), password.c_str());

        StaticJsonDocument<100> ackDoc;
        ackDoc["type"] = "command_ack";
        ackDoc["command"] = "set_wifi_credentials";
        ackDoc["status"] = "Wi-Fi credentials saved. Robot will restart.";
        String response; serializeJson(ackDoc, response);
        webSocket.sendTXT(clientId, response);
        
        // Visual indication of restart
        for (int i=0; i<10; ++i) { // Blink for ~2 seconds
            applyLedBrightness(50); delay(100);
            applyLedBrightness(0); delay(100);
        }
        delay(1000); 
        ESP.restart();
    }
}

// --- Camera Settings Handler (via WebSocket) ---
void handleCameraSettings(uint8_t clientId, JsonDocument& doc) {
    lastClientActivityTimestamp = millis();
    sensor_t * s = esp_camera_sensor_get();
    if (!s) { 
        StaticJsonDocument<100> errDoc; errDoc["type"] = "error"; errDoc["message"] = "Camera sensor error";
        String response; serializeJson(errDoc, response); webSocket.sendTXT(clientId, response);
        return; 
    }
    bool settings_changed_applied = false;
    bool success = true;

    if (doc.containsKey("resolution")) {
        framesize_t newSize = stringToFramesize(doc["resolution"].as<String>());
        if (s->status.framesize != newSize) { 
            if(s->set_framesize(s, newSize) == ESP_OK) { nvs_cam_framesize = newSize; settings_changed_applied = true; }
            else { success = false; }
        }
    }
    if (doc.containsKey("quality")) {
        int quality = constrain(doc["quality"].as<int>(), 10, 63);
        if (s->status.quality != quality) { 
            if (s->set_quality(s, quality) == ESP_OK) { nvs_cam_quality = quality; settings_changed_applied = true; }
            else { success = false; }
        }
    }
    if (doc.containsKey("brightness")) {
        int val = constrain(doc["brightness"].as<int>(), -2, 2);
        if (s->set_brightness(s, val) == ESP_OK) { nvs_cam_brightness = val; settings_changed_applied = true;}
        else { success = false; }
    }
    if (doc.containsKey("contrast")) {
        int val = constrain(doc["contrast"].as<int>(), -2, 2);
        if (s->set_contrast(s, val) == ESP_OK) { nvs_cam_contrast = val; settings_changed_applied = true;}
        else { success = false; }
    }
    if (doc.containsKey("saturation")) {
        int val = constrain(doc["saturation"].as<int>(), -2, 2);
        if (s->set_saturation(s, val) == ESP_OK) { nvs_cam_saturation = val; settings_changed_applied = true;}
        else { success = false; }
    }

    if (settings_changed_applied && success) { saveCameraSettingsToNVS(); }
    
    StaticJsonDocument<150> ackDoc;
    ackDoc["type"] = "command_ack";
    ackDoc["command"] = "set_camera_settings";
    if (!success) { ackDoc["status"] = "Failed to apply one or more camera settings."; }
    else { ackDoc["status"] = settings_changed_applied ? "Camera Settings Saved" : "NoChange_Camera"; }
    String response; serializeJson(ackDoc, response);
    webSocket.sendTXT(clientId, response);
}


// --- WebSocket Event Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    lastClientActivityTimestamp = millis(); 
    switch(type) {
        case WStype_DISCONNECTED:
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            
            StaticJsonDocument<512> statusDoc; 
            statusDoc["type"] = "status_update"; 
            statusDoc["lipo_v"] = readLipoVoltage();
            statusDoc["rssi"] = WiFi.RSSI();
            statusDoc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
            statusDoc["stream_active"] = isStreaming;
            statusDoc["is_sleeping"] = isLightSleeping;
            statusDoc["is_ap_mode"] = isInAPMode;
            sensor_t * s = esp_camera_sensor_get();
            if(s){
                statusDoc["resolution"] = framesizeToString(s->status.framesize);
                statusDoc["quality"] = s->status.quality;
                statusDoc["brightness"] = s->status.brightness;
                statusDoc["contrast"] = s->status.contrast;
                statusDoc["saturation"] = s->status.saturation;
            }
            statusDoc["led_brightness"] = g_current_led_brightness_percent;
            statusDoc["sleep_enabled"] = g_light_sleep_enabled;

            String statusMsg; serializeJson(statusDoc, statusMsg);
            webSocket.sendTXT(num, statusMsg);
            break;
        }
        case WStype_TEXT: {
            StaticJsonDocument<256> doc; 
            DeserializationError error = deserializeJson(doc, payload, length);
            if (error) {
                return;
            }

            const char* command = doc["command"];
            if (command) {
                if (strcmp(command, "control") == 0) {
                    const char* mode = doc["mode"] | "joystick";
                    if (strcmp(mode, "joystick") == 0) {
                        float x = doc["x"] | 0.0f; float y = doc["y"] | 0.0f;
                        processJoystickControlServos(x,y);
                    } else if (strcmp(mode, "sliders") == 0) {
                        float l = doc["left"] | 0.0f; float r = doc["right"] | 0.0f;
                        processSlidersControlServos(l,r);
                    }
                } else if (strcmp(command, "set_led") == 0) {
                    handleLedControl(num, doc);
                } else if (strcmp(command, "set_camera_settings") == 0) {
                    handleCameraSettings(num, doc);
                } else if (strcmp(command, "stream_control") == 0) { 
                    const char* action = doc["action"];
                    if (action) {
                        if (strcmp(action, "start") == 0) {
                            isStreaming = true; 
                            // The actual stream task is started by the HTTP GET /stream request
                            // This WS command just sets the state and informs clients.
                            broadcastStatusUpdate(); 
                        } else if (strcmp(action, "stop") == 0) {
                            isStreaming = false; 
                            broadcastStatusUpdate(); 
                        }
                    }
                } else if (strcmp(command, "get_calibration") == 0) {
                    handleGetCalibration(num);
                } else if (strcmp(command, "set_calibration") == 0) {
                    handleSetCalibration(num, doc);
                } else if (strcmp(command, "reset_calibration") == 0) {
                    handleResetCalibration(num);
                } else if (strcmp(command, "get_sleep_mode_pref") == 0) {
                    handleGetSleepModePref(num);
                } else if (strcmp(command, "set_sleep_mode") == 0) {
                    handleSetSleepModePref(num, doc);
                } else if (strcmp(command, "set_wifi_credentials") == 0) {
                    handleSetWiFiCredentials(num, doc);
                } else if (strcmp(command, "ping") == 0) {
                    webSocket.sendTXT(num, "{\"type\":\"pong\"}");
                }
            }
            break;
        }
        case WStype_BIN:
        case WStype_ERROR:			
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}

// --- Function to broadcast status updates via WebSocket ---
void broadcastStatusUpdate() {
    if (webSocket.connectedClients() > 0) {
        StaticJsonDocument<512> doc; 
        doc["type"] = "status_update";
        doc["lipo_v"] = readLipoVoltage();
        doc["rssi"] = WiFi.RSSI();
        doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
        doc["stream_active"] = isStreaming; 
        doc["is_sleeping"] = isLightSleeping;
        doc["is_ap_mode"] = isInAPMode;

        float current_lipo_v = doc["lipo_v"];
        if (current_lipo_v < BATTERY_CRITICAL_VOLTAGE) {
            doc["battery_status"] = "critical";
            isBatteryCritical = true;
        } else if (current_lipo_v < BATTERY_LOW_WARN_VOLTAGE) {
            doc["battery_status"] = "low";
            isBatteryCritical = false;
        } else {
            doc["battery_status"] = "ok";
            isBatteryCritical = false;
        }
        
        if (isStreaming && ra_filter.count > 0 && ra_filter.sum > 0) {
            uint32_t avg_frame_time_us = ra_filter.sum / ra_filter.count;
            if (avg_frame_time_us > 0) {
                doc["fps"] = round((1000000.0f / avg_frame_time_us) * 10) / 10.0f;
            } else {
                doc["fps"] = 0;
            }
        } else {
            doc["fps"] = 0;
        }

        String response;
        serializeJson(doc, response);
        webSocket.broadcastTXT(response);
    }
}


// --- Web Server Route Definitions ---
void setupWebServerRoutes() { 
    server.on("/", HTTP_GET, []() { 
        if (!checkAuthentication()) return; 
        lastClientActivityTimestamp = millis();
        server.sendHeader("Connection", "close"); 
        server.send_P(200, "text/html", htmlContentFlash); 
    });
    server.on("/ping", HTTP_GET, []() { 
        if (!checkAuthentication()) return;
        lastClientActivityTimestamp = millis();
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", "pong_http"); 
    });
    
    server.on("/stream", HTTP_GET, [](){ if (!checkAuthentication()) return; handleStartStreamForHttpRequest(); }); 
    // HTTP routes for /stopstream, /status, /lipo_voltage, /wifi_status are effectively replaced by WebSocket.
    // Kept /quality and /camera/settings as HTTP GET for potential direct testing/utility.
    
    server.on("/quality", HTTP_GET, []() { 
        if (!checkAuthentication()) return; 
        lastClientActivityTimestamp = millis();
        sensor_t * s = esp_camera_sensor_get(); 
        if (!s) { server.sendHeader("Connection", "close"); server.send(500, "text/plain", "Camera sensor error"); return; }
        
        bool settings_changed_applied = false;
        bool success = true;

        if (server.hasArg("resolution")) {
            framesize_t newSize = stringToFramesize(server.arg("resolution"));
            if (s->status.framesize != newSize) { 
                if(s->set_framesize(s, newSize) == ESP_OK) { nvs_cam_framesize = newSize; settings_changed_applied = true; }
                else { success = false; }
            }
        }
        if (server.hasArg("quality")) {
            int quality = constrain(server.arg("quality").toInt(), 10, 63);
            if (s->status.quality != quality) { 
                if (s->set_quality(s, quality) == ESP_OK) { nvs_cam_quality = quality; settings_changed_applied = true; }
                else { success = false; }
            }
        }
        if (server.hasArg("brightness")) {
            int val = constrain(server.arg("brightness").toInt(), -2, 2);
            if (s->set_brightness(s, val) == ESP_OK) { nvs_cam_brightness = val; settings_changed_applied = true;}
            else { success = false; }
        }
        if (server.hasArg("contrast")) {
            int val = constrain(server.arg("contrast").toInt(), -2, 2);
            if (s->set_contrast(s, val) == ESP_OK) { nvs_cam_contrast = val; settings_changed_applied = true;}
            else { success = false; }
        }
        if (server.hasArg("saturation")) {
            int val = constrain(server.arg("saturation").toInt(), -2, 2);
            if (s->set_saturation(s, val) == ESP_OK) { nvs_cam_saturation = val; settings_changed_applied = true;}
            else { success = false; }
        }

        if (settings_changed_applied && success) { saveCameraSettingsToNVS(); }
        server.sendHeader("Connection", "close");
        if (!success) { server.send(500, "text/plain", "Failed to apply one or more camera settings."); }
        else { server.send(200, "text/plain", settings_changed_applied ? "OK_Quality_Saved" : "NoChange_Quality"); }
    });

    server.on("/camera/settings", HTTP_GET, []() { 
        if (!checkAuthentication()) return; 
        lastClientActivityTimestamp = millis();
        sensor_t * s = esp_camera_sensor_get(); 
        if (!s) { server.sendHeader("Connection", "close"); server.send(500, "text/plain", "Camera sensor error"); return; }
        StaticJsonDocument<256> doc; 
        doc["resolution"] = framesizeToString(s->status.framesize); 
        doc["quality"] = s->status.quality;
        doc["brightness"] = s->status.brightness;
        doc["contrast"] = s->status.contrast;
        doc["saturation"] = s->status.saturation;
        String response; 
        serializeJson(doc, response); 
        server.sendHeader("Connection", "close");
        server.send(200, "application/json", response);
    });
    
    server.onNotFound([](){ 
        if (!checkAuthentication()) return; 
        lastClientActivityTimestamp = millis();
        server.sendHeader("Connection", "close");
        server.send(404, "text/plain", "Not Found"); 
    }); 
    server.begin(); 
}

// --- Helper functions for Camera Settings (Conversion between enum and string) ---
const char* framesizeToString(framesize_t fs) { 
    switch(fs) { 
        case FRAMESIZE_QQVGA: return "QQVGA"; case FRAMESIZE_QCIF: return "QCIF"; 
        case FRAMESIZE_HQVGA: return "HQVGA"; case FRAMESIZE_240X240: return "240X240"; 
        case FRAMESIZE_QVGA: return "QVGA"; case FRAMESIZE_CIF: return "CIF"; 
        case FRAMESIZE_HVGA: return "HVGA"; case FRAMESIZE_VGA: return "VGA"; 
        case FRAMESIZE_SVGA: return "SVGA"; case FRAMESIZE_XGA: return "XGA"; 
        case FRAMESIZE_HD: return "HD"; case FRAMESIZE_SXGA: return "SXGA"; 
        case FRAMESIZE_UXGA: return "UXGA"; default: return "UNKNOWN"; 
    }
}
framesize_t stringToFramesize(const String& fsStr) { 
    if (fsStr == "QQVGA") return FRAMESIZE_QQVGA; if (fsStr == "QCIF") return FRAMESIZE_QCIF; 
    if (fsStr == "HQVGA") return FRAMESIZE_HQVGA; if (fsStr == "240X240") return FRAMESIZE_240X240; 
    if (fsStr == "QVGA") return FRAMESIZE_QVGA; if (fsStr == "CIF") return FRAMESIZE_CIF; 
    if (fsStr == "HVGA") return FRAMESIZE_HVGA; if (fsStr == "VGA") return FRAMESIZE_VGA; 
    if (fsStr == "SVGA") return FRAMESIZE_SVGA; if (fsStr == "XGA") return FRAMESIZE_XGA; 
    if (fsStr == "HD") return FRAMESIZE_HD; if (fsStr == "SXGA") return FRAMESIZE_SXGA; 
    if (fsStr == "UXGA") return FRAMESIZE_UXGA;
    return FRAMESIZE_QVGA; 
}

// --- HTTP Server & WebSocket Task (Runs on Core 0) ---
void TaskHttpServer(void *pvParameters) { 
    (void)pvParameters; 
    unsigned long lastStatusPush = 0;
    for (;;) { 
        if (!otaIsActive && !isLightSleeping) { 
            server.handleClient(); 
            webSocket.loop(); 

            if (millis() - lastStatusPush > statusBroadcastInterval) {
                if(webSocket.connectedClients() > 0 && !isBatteryCritical) { 
                     broadcastStatusUpdate();
                }
                lastStatusPush = millis();
            }
        } else if (isLightSleeping) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else { 
             vTaskDelay(pdMS_TO_TICKS(10)); 
        }
        vTaskDelay(pdMS_TO_TICKS(2)); 
    } 
}
