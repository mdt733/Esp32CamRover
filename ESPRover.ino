// =================================================================================================
// ESP32-CAM Servo Robot with Video Stream, OTA, Basic Authentication, and NVS Calibration
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

// Key Version History (Recent Major Changes):
//   V3.7.3: Stream/AP Mode Fixes & Battery Critical Handling.
//   V3.7.0: WebSockets, Advanced Sleep, AP Mode, UI Enhancements.
//
// Version 4.0.1 - Enhanced Connection Indicator (2025-05-24)
// - Updated HTML/JS/CSS for improved connection status dot:
//   - Blue: Video stream active and rendering.
//   - Green: WebSocket connected and healthy (recent messages).
//   - Orange: WebSocket connected but lagging (no messages >5s).
//   - Red: WebSocket failed/closed, or active but no messages >20s.
//
// Version 4.0.0 - Definitive Edition (2025-05-22)
// - Final review and polish of all code and comments for clarity and long-term maintainability.
// - Comprehensive documentation embedded within the code.
// - Ensured all features up to V3.7.3 are stable and well-commented.
// - Removed all non-essential Serial.print statements.
// - This version is intended as a feature-complete, stable, and shareable baseline.
//
// Version 4.1.2 - No Serial & WS Status Refactor (2025-05-25)
// - Removed all Serial.print statements for embedded deployment.
// - Refactored WebSocket status sending:
//   - Created sendFullStatusUpdate(targetClient, isInitial) helper.
//   - WStype_CONNECTED now sends "initial_settings" via helper.
//   - broadcastStatusUpdate() now sends "status_update" via helper.
//
// Version 4.1.1 - WiFi Connection Logic Fix (2025-05-25)
//==========================================================================================

// =================================================================================================
// 1. INCLUDES
// =================================================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_camera.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/ledc.h"
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <math.h>

// =================================================================================================
// 2. CONFIGURATION & GLOBAL DEFINITIONS
// =================================================================================================

// --- Network & Server Configuration ---
const int HTTP_PORT = 80;
const int WEBSOCKET_PORT = 81;
char nvs_wifi_ssid[64] = "SSID"; // Default SSID if NVS is empty
char nvs_wifi_password[64] = "WIFIPASSWORD"; // Default PSK if NVS is empty
const char* AP_SSID = "ESP32-CAM-Robot-Setup";
const char* AP_PASSWORD = "password123";
bool isInAPMode = false;

// --- Basic Authentication Credentials ---
const char* HTTP_AUTH_USERNAME = "admin";
const char* HTTP_AUTH_PASSWORD = "authpassword"; // <<< CRITICAL: CHANGE THIS!
const char* AUTH_REALM = "ESP32-CAM Robot Access";

// --- OTA Configuration ---
const char* OTA_HOSTNAME = "esp32-cam-robot";
const char* OTA_PASSWORD = "otapassword"; // <<< CRITICAL: CHANGE THIS!
bool otaIsActive = false;

// --- Camera Pin Definitions (AI-Thinker ESP32-CAM) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
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
const int CAM_XCLK_FREQ = 20000000;
const pixformat_t CAM_PIXEL_FORMAT = PIXFORMAT_JPEG;
const int CAM_FB_COUNT = 2;
framesize_t nvs_cam_framesize = FRAMESIZE_QVGA; // Default, NVS stored
int nvs_cam_quality = 12;                     // Default (10-63, lower=better), NVS stored

// --- LED Configuration ---
const int LED_PIN = 4;
const ledc_channel_t LED_LEDC_CHANNEL_NUM = LEDC_CHANNEL_2;
const ledc_timer_t LED_LEDC_TIMER_NUM = LEDC_TIMER_2;
const int LED_RESOLUTION_BITS = 8;
const int LED_FREQUENCY = 5000;
const float LED_GAMMA = 2.2;
int g_current_led_brightness_percent = 0;

// --- Servo Configuration ---
const int SERVO_LEFT_PIN = 12;
const int SERVO_RIGHT_PIN = 13;
Servo servoLeft;
Servo servoRight;
const int DEFAULT_SERVO_STOP_US = 1500;
const int SERVO_PULSE_SPAN_US = 500;
const float SERVO_INPUT_DEADZONE_THRESHOLD = 0.08f; // Input threshold before servo moves
const int SERVO_PHYSICAL_DEADZONE_US = 30; // Assumed physical deadzone of servo around stop_us
int servo_left_stop_us = DEFAULT_SERVO_STOP_US;   // NVS stored
int servo_right_stop_us = DEFAULT_SERVO_STOP_US;  // NVS stored

// --- LiPo ADC Configuration ---
const int LIPO_ADC_PIN = 33;
const float DEFAULT_VOLTAGE_DIVIDER_RATIO = 3.2;
const float DEFAULT_ADC_REF_VOLTAGE = 3.3;
const int ADC_RESOLUTION_BITS = 12;
float lipo_calib_voltage_divider_ratio = DEFAULT_VOLTAGE_DIVIDER_RATIO; // NVS stored
const float BATTERY_LOW_WARN_VOLTAGE = 3.5f;
const float BATTERY_CRITICAL_VOLTAGE = 3.2f;
bool isBatteryCritical = false;
float g_filtered_lipo_voltage = 0.0f;
const float LIPO_FILTER_ALPHA = 0.1f;

// --- NVS (Non-Volatile Storage) ---
Preferences preferences;

// --- WebServer & WebSockets ---
WebServer server(HTTP_PORT);
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
#define BROADCAST_ALL_CLIENTS 0xFF // Special value for broadcasting

// --- Stream Task Configuration & Globals ---
const size_t HDR_BUF_LEN = 64;
const size_t MAX_FRAME_SIZE = 128 * 1024;
const int STREAM_TASK_STACK_SIZE = 8192;
const UBaseType_t STREAM_TASK_PRIORITY = 2;
const BaseType_t STREAM_TASK_CORE = 1;
const uint32_t STREAM_DELAY_MS = 20;
const char* PART_BOUNDARY = "123456789000000000000987654321";
const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
const char* STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
bool isStreaming = false;
WiFiClient streamClient;
TaskHandle_t streamTaskHandle = NULL;

// --- FPS Calculation (EMA) ---
static float avg_frame_time_us_ema = 0.0f;
const float EMA_ALPHA_FPS = 0.05f; // Smoothing factor for FPS EMA

// --- Light Sleep Management ---
unsigned long lastClientActivityTimestamp = 0;
const unsigned long lightSleepTimeoutMs = 5 * 60 * 1000; // 5 minutes
const uint64_t lightSleepPeriodicWakeupUs = 60 * 1000 * 1000ULL; // 60 seconds
TaskHandle_t sleepManagementTaskHandle = NULL;
bool isLightSleeping = false;
bool g_light_sleep_enabled = true; // NVS stored
unsigned long lastStatusBroadcastTime = 0;
const unsigned long statusBroadcastInterval = 750;


// =================================================================================================
// 3. HTML CONTENT (Stored in PROGMEM) - Simplified
// =================================================================================================
static const char htmlContentFlash[] PROGMEM = R"RAW_HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>
    <title>ESP32-CAM Robot V4.1.2</title>
    <style>
        :root {
            --primary-color: #007bff; --secondary-color: #28a745; --info-color: #17a2b8; 
            --danger-color: #dc3545; --warning-color: #ffc107; --background-color: #212529;
            --card-background-color: #343a40; --text-color: #f8f9fa; --text-muted-color: #adb5bd;
            --control-bg: rgba(52, 58, 64, 0.85); --input-bg: #495057; --input-border-color: #5a6268;
            --battery-bar-bg: #555; --battery-good: #28a745; --battery-medium: #ffc107; --battery-low: #dc3545;
            --rssi-bar-color: var(--primary-color); --conn-status-good: var(--secondary-color);
            --conn-status-bad: var(--danger-color); --conn-status-streaming: var(--primary-color);
            --conn-status-lagging: var(--warning-color); --toast-info-bg: rgba(23, 162, 184, 0.9);
            --toast-warn-bg: rgba(255, 193, 7, 0.9); --toast-error-bg: rgba(220, 53, 69, 0.9);
            --toast-text-color: #f1f1f1;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            background-color: var(--background-color); min-height: 100vh; overflow: hidden; 
            color: var(--text-color); touch-action: none; -ms-touch-action: none;
            overscroll-behavior: none !important; position: fixed; width: 100%; height: 100%;
            display: flex; flex-direction: column;
        }
        .container { position: relative; width: 100%; height: 100%; display: flex; align-items: center; justify-content: center; flex-grow: 1; }
        .video-container { position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 1; background: #000; display: flex; align-items: center; justify-content: center; overflow: hidden; }
        .stream-feedback { color: var(--text-muted-color); font-size: 1.2em; text-align: center; padding: 20px; position: absolute; z-index: 2; display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%; height: 100%; background-color: rgba(0,0,0,0.5); }
        .stream-feedback.hidden { display: none; }
        .spinner { border: 4px solid rgba(255, 255, 255, 0.2); border-left-color: var(--primary-color); border-radius: 50%; width: 30px; height: 30px; animation: spin 1s linear infinite; margin-bottom: 10px; }
        @keyframes spin { to { transform: rotate(360deg); } }
        #stream { width: 100%; height: 100%; object-fit: contain; display: none; transform-origin: center center; transform: rotate(-90deg); }
        .controls-overlay { position: absolute; z-index: 2; width: 100%; height: 100%; pointer-events: none; }
        .controls-top-left { position: absolute; top: 15px; left: 15px; display: flex; align-items: center; gap: 10px; pointer-events: all; background: var(--control-bg); padding: 10px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); backdrop-filter: blur(5px); }
        .controls-top-right-status { position: absolute; top: 15px; right: 15px; background: var(--control-bg); padding: 8px 12px; border-radius: 8px; font-size: 0.85em; z-index: 5; color: var(--text-muted-color); box-shadow: 0 2px 5px rgba(0,0,0,0.3); backdrop-filter: blur(5px); display: flex; flex-direction: column; align-items: flex-end; gap: 5px; pointer-events: all; }
        .status-item { display: flex; align-items: center; gap: 8px; }
        #lipo-voltage-display, #wifi-rssi-display { font-weight: bold; }
        #lipo-voltage-display { color: var(--warning-color); } #wifi-rssi-display { color: var(--info-color); }
        .connection-status-dot { width: 10px; height: 10px; border-radius: 50%; margin-left: 5px; background-color: var(--conn-status-bad); transition: background-color 0.3s ease-out; }
        .connection-status-dot.streaming { background-color: var(--conn-status-streaming); } .connection-status-dot.good { background-color: var(--conn-status-good); }
        .connection-status-dot.lagging { background-color: var(--conn-status-lagging); } .connection-status-dot.bad { background-color: var(--conn-status-bad); }
        .bar-container { width: 50px; height: 10px; background-color: var(--battery-bar-bg); border-radius: 3px; overflow: hidden; border: 1px solid #444; }
        .bar-level { height: 100%; width: 0%; transition: width 0.3s ease-out, background-color 0.3s ease-out; }
        #battery-bar-level { background-color: var(--battery-good); } #wifi-bar-level { background-color: var(--rssi-bar-color); }
        .stream-status-fps { background: transparent; color: white; padding: 0; font-size: 0.8em; text-align: right; display: none; } /* Initially hidden */
        #sleep-status-indicator { font-size: 0.8em; color: var(--text-muted-color); text-align: right; }
        .controls-joystick-panel { position: absolute; left: 15px; top: 50%; transform: translateY(-50%); pointer-events: all; display: flex; flex-direction: column; gap: 15px; background: var(--control-bg); padding: 15px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); backdrop-filter: blur(5px); }
        .joystick-container { width: 160px; height: 160px; background: rgba(255,255,255,0.05); border-radius: 8px; padding: 15px; display: block; } /* Always block for joystick only */
        #joystick { width: 100%; height: 100%; background: rgba(255,255,255,0.1); border-radius: 50%; position: relative; touch-action: none; }
        #stick { width: 40%; height: 40%; background: var(--primary-color); border-radius: 50%; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); cursor: pointer; box-shadow: 0 0 10px rgba(0,123,255,0.5); }
        .button { padding: 10px 15px; border: none; border-radius: 6px; cursor: pointer; font-size: 1em; background: var(--input-bg); color: var(--text-color); transition: background-color 0.2s, box-shadow 0.2s; white-space: nowrap; text-align: center; touch-action: manipulation; user-select: none; -webkit-user-select: none; display: flex; align-items: center; justify-content: center; line-height: 1; box-shadow: 0 2px 4px rgba(0,0,0,0.2); }
        .button:hover { background-color: #495057; box-shadow: 0 3px 6px rgba(0,0,0,0.3); }
        .button:active, .button.active { background-color: var(--primary-color); color: white; box-shadow: 0 1px 3px rgba(0,0,0,0.2) inset; }
        .icon-button .material-icons { font-size: 20px; }
        .settings-panel { position: fixed; top: 0; left: -320px; width: 320px; height: 100%; background: var(--card-background-color); z-index: 1000; transition: left 0.3s ease; padding: 20px; overflow-y: auto; box-shadow: 5px 0 15px rgba(0,0,0,0.5); touch-action: pan-y !important; }
        .settings-panel.active { left: 0; pointer-events: auto; }
        .settings-close { position: absolute; top: 15px; right: 15px; font-size: 24px; cursor: pointer; color: var(--text-muted-color); opacity: 0.7; z-index: 1001; }
        .settings-close:hover { opacity: 1; color: var(--text-color); }
        .settings-title { font-size: 1.4em; margin-bottom: 20px; font-weight: 500; color: var(--text-color); border-bottom: 1px solid #495057; padding-bottom: 10px; }
        .settings-section { margin-bottom: 25px; }
        .settings-section h4 { font-size: 1.1em; color: var(--text-color); margin-bottom: 15px; border-bottom: 1px solid #495057; padding-bottom: 8px;}
        .settings-subsection h5 { font-size: 0.9em; color: var(--text-muted-color); margin-top: 10px; margin-bottom: 5px; font-weight: normal; }
        .select-wrapper { position: relative; width: 100%; margin-bottom: 10px; }
        .settings-select, .settings-input { width: 100%; padding: 10px; appearance: none; -webkit-appearance: none; -moz-appearance: none; background-color: var(--input-bg); color: var(--text-color); border: 1px solid var(--input-border-color); border-radius: 5px; cursor: pointer; outline: none; font-size: 1em; margin-bottom: 8px; }
        .settings-input[type="number"] { -moz-appearance: textfield; } .settings-input::-webkit-outer-spin-button, .settings-input::-webkit-inner-spin-button { -webkit-appearance: none; margin: 0; }
        .select-wrapper::after { content: '\\25BC'; position: absolute; right: 12px; top: 50%; transform: translateY(-50%); pointer-events: none; color: var(--text-muted-color); }
        .settings-slider-container { width: 100%; margin: 10px 0; }
        .settings-slider { width: 100%; -webkit-appearance: none; height: 8px; border-radius: 4px; background: #5a6268; outline: none; cursor: pointer; }
        .settings-slider::-webkit-slider-thumb { -webkit-appearance: none; width: 20px; height: 20px; border-radius: 50%; background: var(--primary-color); cursor: pointer; box-shadow: 0 0 5px rgba(0,123,255,0.4); }
        .settings-slider::-moz-range-thumb { width: 20px; height: 20px; border-radius: 50%; background: var(--primary-color); cursor: pointer; border: none; box-shadow: 0 0 5px rgba(0,123,255,0.4); }
        .settings-slider-value { text-align: right; margin-top: 5px; font-size: 0.9em; color: var(--text-muted-color); }
        .overlay { position: fixed; top: 0; left: 0; right: 0; bottom: 0; background: rgba(0,0,0,0.6); z-index: 999; display: none; }
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
        #toast-container { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%); z-index: 2000; display: flex; flex-direction: column; align-items: center; gap: 10px; }
        .toast { background-color: var(--toast-info-bg); color: var(--toast-text-color); padding: 12px 20px; border-radius: 6px; box-shadow: 0 4px 12px rgba(0,0,0,0.2); opacity: 0; transition: opacity 0.5s, transform 0.5s; transform: translateY(20px); font-size: 0.9em; max-width: 90vw; text-align: center; }
        .toast.show { opacity: 1; transform: translateY(0); } .toast.error { background-color: var(--toast-error-bg); } .toast.warning { background-color: var(--toast-warn-bg); }
        #ap-mode-config { display: none; padding: 20px; background-color: var(--card-background-color); border-radius: 8px; margin: 20px; text-align: center; z-index: 1001; position: relative;}
        #ap-mode-config h3 { margin-bottom: 15px; } #ap-mode-config input { margin-bottom: 10px; }
        @media (max-width: 768px) { 
            .controls-joystick-panel { left: 50%; top: auto; bottom: 45px; transform: translateX(-50%); background: rgba(33, 37, 41, 0.7); padding: 10px; max-width: 180px; }
            .joystick-container { width: 130px; height: 130px; background: rgba(255,255,255,0.03); }
            .controls-top-right-status { font-size: 0.8em; padding: 6px 10px; }
            .stream-status-fps { font-size: 0.75em; padding: 0; } .bar-container { width: 40px; } 
        }
        @font-face { font-family: 'Material Icons'; font-style: normal; font-weight: 400; src: url(https://fonts.gstatic.com/s/materialicons/v141/flUhRq6tzZclQEJ-Vdg-IuiaDsNc.woff2) format('woff2'); }
        .material-icons { font-family: 'Material Icons'; font-weight: normal; font-style: normal; font-size: 24px; line-height: 1; letter-spacing: normal; text-transform: none; display: inline-block; white-space: nowrap; word-wrap: normal; direction: ltr; -webkit-font-smoothing: antialiased; text-rendering: optimizeLegibility; -moz-osx-font-smoothing: grayscale; font-feature-settings: 'liga'; }
    </style>
</head>
<body>
    <div id="toast-container"></div>
    <div class="container">
        <div class="video-container">
            <div class="stream-feedback" id="stream-feedback-placeholder"><span>Video Stream Offline</span></div>
            <div class="stream-feedback hidden" id="stream-feedback-loading"><div class="spinner"></div><span>Connecting to stream...</span></div>
            <img id="stream">
        </div>
        <div id="ap-mode-config">
            <h3>Robot in Setup Mode</h3> <p>Connect to Wi-Fi network:</p>
            <input type="text" id="ap-mode-ssid" class="settings-input" placeholder="Network Name (SSID)">
            <input type="password" id="ap-mode-password" class="settings-input" placeholder="Password">
            <button class="button" id="ap-mode-connect-button">Connect & Restart</button>
        </div>
        <div class="controls-overlay" id="main-controls-overlay">
            <div class="controls-top-left">
                <button class="button icon-button" id="stream-toggle" title="Start/Stop Stream"><span class="material-icons">play_arrow</span></button>
                <button class="button icon-button" id="settings-toggle" title="Settings"><span class="material-icons">settings</span></button>
            </div>
            <div class="controls-top-right-status">
                <div class="status-item"><span>Conn:</span> <span class="connection-status-dot" id="connection-status-dot"></span></div>
                <div class="status-item"><span>LiPo: <span id="lipo-voltage-display">N/A</span> V</span><div class="bar-container"><div class="bar-level" id="battery-bar-level"></div></div></div>
                <div class="status-item"><span>WiFi: <span id="wifi-rssi-display">N/A</span></span><div class="bar-container"><div class="bar-level" id="wifi-bar-level"></div></div></div>
                <div class="stream-status-fps" id="fps-status">FPS: ...</div>
                <div id="sleep-status-indicator">Awake</div>
            </div>
            <div class="controls-joystick-panel" id="joystick-panel">
                <div class="joystick-container" id="joystick-control"><div id="joystick"><div id="stick"></div></div></div>
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
                        <option value="QVGA" selected>QVGA (320x240)</option>
                        <option value="SVGA">SVGA (800x600)</option>
                        <option value="XGA">XGA (1024x768)</option>
                    </select>
                </div>
                <div class="settings-slider-container">
                    <label for="quality-slider" style="font-size:0.9em; color: var(--text-muted-color);">Quality (10-63, lower is better)</label>
                    <input type="range" min="10" max="63" value="12" class="settings-slider" id="quality-slider">
                    <div class="settings-slider-value" id="quality-value">12</div>
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
                <h4>Power Saving</h4>
                <div class="settings-toggle-switch">
                    <label for="light-sleep-toggle">Enable Light Sleep Mode</label>
                    <label class="switch"><input type="checkbox" id="light-sleep-toggle"><span class="slider"></span></label>
                </div>
            </div>
            <div class="settings-section" id="wifi-config-section-settings" style="display:none;">
                <h4>Wi-Fi Configuration (AP Mode)</h4>
                <input type="text" id="ap-mode-ssid-settings" class="settings-input" placeholder="New Network Name (SSID)">
                <input type="password" id="ap-mode-password-settings" class="settings-input" placeholder="New Password">
                <button class="button" id="ap-mode-connect-button-settings">Connect & Restart Robot</button>
            </div>
            <div class="settings-section">
                <h4>Calibration</h4>
                <div class="settings-subsection">
                    <h5>Left Servo Stop (µs)</h5>
                    <div class="calibration-input-group"><input type="number" class="settings-input" id="cal-servo-l-stop" min="1000" max="2000" step="10"></div>
                </div>
                <div class="settings-subsection">
                    <h5>Right Servo Stop (µs)</h5>
                    <div class="calibration-input-group"><input type="number" class="settings-input" id="cal-servo-r-stop" min="1000" max="2000" step="10"></div>
                </div>
                <div class="settings-subsection">
                    <h5>LiPo ADC Ratio</h5>
                    <div class="calibration-input-group"><input type="number" class="settings-input" id="cal-lipo-ratio" step="0.01" min="1.0" max="10.0"></div>
                </div>
                <div class="calibration-buttons">
                    <button class="button btn-info" id="btn-load-calib">Load Current</button>
                    <button class="button" id="btn-save-calib" style="background-color: var(--secondary-color);">Save to Robot</button>
                    <button class="button" id="btn-reset-calib" style="background-color: var(--danger-color);">Reset Defaults</button>
                </div>
            </div>
        </div>
    </div>
    <script>
        const el = id => document.getElementById(id);
        const streamImg = el('stream'), streamFeedbackPlaceholder = el('stream-feedback-placeholder'), streamFeedbackLoading = el('stream-feedback-loading');
        const streamToggle = el('stream-toggle'), joystickPanel = el('joystick-panel'), joystickControl = el('joystick-control');
        const joystick = el('joystick'), stick = el('stick'), lipoVoltageDisplay = el('lipo-voltage-display');
        const batteryBarLevel = el('battery-bar-level'), wifiRssiDisplay = el('wifi-rssi-display'), wifiBarLevel = el('wifi-bar-level');
        const connectionStatusDot = el('connection-status-dot'), fpsStatusDisplay = el('fps-status'), sleepStatusIndicator = el('sleep-status-indicator');
        const settingsToggle = el('settings-toggle'), settingsPanel = el('settings-panel'), settingsClose = el('settings-close');
        const settingsOverlay = el('settings-overlay'), resolutionSelect = el('resolution-select'), qualitySlider = el('quality-slider');
        const qualityValueDisplay = el('quality-value'), ledSlider = el('led-slider'), ledValueDisplay = el('led-value');
        const lightSleepToggle = el('light-sleep-toggle'), calServoLstop = el('cal-servo-l-stop'), calServoRstop = el('cal-servo-r-stop');
        const calLipoRatio = el('cal-lipo-ratio'), btnLoadCalib = el('btn-load-calib'), btnSaveCalib = el('btn-save-calib');
        const btnResetCalib = el('btn-reset-calib'), apModeConfigSection = el('ap-mode-config');
        const apModeSsidInput = el('ap-mode-ssid'), apModePasswordInput = el('ap-mode-password'), apModeConnectButton = el('ap-mode-connect-button');
        const apModeConfigSectionForSettings = el('wifi-config-section-settings'), apModeSsidInputForSettings = el('ap-mode-ssid-settings');
        const apModePasswordInputForSettings = el('ap-mode-password-settings'), apModeConnectButtonForSettings = el('ap-mode-connect-button-settings');
        const mainControlsOverlay = el('main-controls-overlay');

        let isStreamActive = false, joystickIsDragging = false, joystickCurrentX = 0, joystickCurrentY = 0;
        let lastControlSendTime = 0, clientKeepAliveIntervalId, webSocket, isRobotInAPMode = false, apModeToastShown = false;
        let attemptingStreamStart = false, streamStartAttemptTime = 0, isServerConnected = false, lastWebSocketMessageTime = Date.now();
        let connectionHealthStatus = 'good', isStreamActuallyRendering = false, healthCheckIntervalId;
        const CONTROL_THROTTLE_MS = 100, STREAM_START_GRACE_PERIOD = 3000;
        const LAGGING_THRESHOLD_MS = 5000, FAILED_THRESHOLD_MS = 20000;
        const streamUrlBase = '';

        function showToast(message, type = 'info', duration = 3000) {
            const tc = el('toast-container'), t = document.createElement('div');
            t.className = `toast ${type}`; t.textContent = message; tc.appendChild(t);
            setTimeout(() => t.classList.add('show'), 100);
            setTimeout(() => { t.classList.remove('show'); setTimeout(() => { if (t.parentNode === tc) tc.removeChild(t); }, 500); }, duration);
        }

        function connectWebSocket() {
            const wsUrl = `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.hostname}:81`;
            webSocket = new WebSocket(wsUrl);
            webSocket.onopen = () => {
                isServerConnected = true; lastWebSocketMessageTime = Date.now(); connectionHealthStatus = 'good'; 
                updateConnectionStatusIndicator(); showToast('Connected!', 'info'); 
                if (webSocket.readyState === WebSocket.OPEN) webSocket.send(JSON.stringify({ command: "get_full_status" }));
                if (isRobotInAPMode) handleAPModeUI(true, true);
            };
            webSocket.onmessage = event => {
                try {
                    lastWebSocketMessageTime = Date.now();
                    if (!isServerConnected || connectionHealthStatus !== 'good') { isServerConnected = true; connectionHealthStatus = 'good'; }
                    checkConnectionHealth();
                    const data = JSON.parse(event.data); lastClientActivityTimestamp = Date.now();
                    if (data.type === 'status_update' || data.type === 'initial_settings') { 
                        if (data.lipo_v !== undefined) updateLipoVoltageUI(data.lipo_v);
                        if (data.rssi !== undefined && data.wifi_connected !== undefined) updateWifiStatusUI(data.rssi, data.wifi_connected);
                        if (data.fps !== undefined && isStreamActive) updateFpsDisplayUI(data.fps); 
                        if (data.is_sleeping !== undefined) updateSleepStatusIndicatorUI(data.is_sleeping);
                        if (data.is_ap_mode !== undefined) {
                            const wasInAPMode = isRobotInAPMode; isRobotInAPMode = data.is_ap_mode; handleAPModeUI(isRobotInAPMode, webSocket.readyState === WebSocket.OPEN);
                            if (isRobotInAPMode && !wasInAPMode && !apModeToastShown) { showToast('Robot in AP Mode. Configure Wi-Fi via Settings.', 'warning', 10000); apModeToastShown = true; }
                            else if (!isRobotInAPMode) apModeToastShown = false;
                        }
                        if (data.battery_status === 'low') showToast('Battery Low!', 'warning', 5000);
                        if (data.battery_status === 'critical') showToast('BATTERY CRITICAL! Ops limited.', 'error', 10000);
                        if (data.stream_active !== undefined) {
                            const oldIsStreamActive = isStreamActive; isStreamActive = data.stream_active;
                            if (isStreamActive && !oldIsStreamActive) {
                                startStreamUI();
                                if (!streamImg.src.includes('/stream')) {
                                    streamImg.src = streamUrlBase + '/stream?' + new Date().getTime();
                                    streamFeedbackPlaceholder.classList.add('hidden'); streamFeedbackLoading.classList.remove('hidden'); streamImg.style.display = 'none';
                                }
                            } else if (!isStreamActive && oldIsStreamActive) stopStreamUI();
                            else if (oldIsStreamActive !== isStreamActive) { if (!isStreamActive) isStreamActuallyRendering = false; updateConnectionStatusIndicator(); }
                        }
                        if (isStreamActive) attemptingStreamStart = false;
                        if (data.type === 'initial_settings') { 
                            if (data.resolution) resolutionSelect.value = data.resolution;
                            if (data.quality) { qualitySlider.value = data.quality; qualityValueDisplay.textContent = data.quality; }
                            if (data.led_brightness !== undefined) { ledSlider.value = data.led_brightness; ledValueDisplay.textContent = (data.led_brightness == 0 ? "Off" : data.led_brightness + "%");}
                            if (data.sleep_enabled !== undefined) lightSleepToggle.checked = data.sleep_enabled;
                            showToast('Initial settings loaded.', 'info');
                        }
                    } else if (data.type === 'calibration_data') {
                        calServoLstop.value = data.sL_stop || ''; calServoRstop.value = data.sR_stop || '';
                        calLipoRatio.value = data.lipo_ratio || ''; showToast("Calibration values loaded.", 'info');
                    } else if (data.type === 'command_ack') {
                        showToast(data.status || `Cmd '${data.command}' OK.`, 'info');
                        if (data.command === 'reset_calibration') { loadCalibrationValuesToUI(); sendWebSocketCommand({ command: "get_full_status" }); }
                    } else if (data.type === 'error') showToast(`Error: ${data.message}`, 'error');
                } catch (e) { console.error('Error parsing WS msg:', e, event.data); }
            };
            webSocket.onclose = () => {
                showToast('Disconnected.', 'error'); isServerConnected = false;
                connectionHealthStatus = 'failed'; updateConnectionStatusIndicator();
                if (!isRobotInAPMode) setTimeout(connectWebSocket, 3000);
            };
            webSocket.onerror = () => {
                showToast('WS Connection Error.', 'error'); isServerConnected = false;
                connectionHealthStatus = 'failed'; updateConnectionStatusIndicator();
            };
        }

        function sendWebSocketCommand(cmd) {
            if (webSocket && webSocket.readyState === WebSocket.OPEN) { webSocket.send(JSON.stringify(cmd)); lastClientActivityTimestamp = Date.now(); }
            else { showToast('Not connected. Cmd not sent.', 'error'); }
        }

        function checkConnectionHealth() {
            const now = Date.now(); let prevHealth = connectionHealthStatus, prevConnected = isServerConnected;
            if (!webSocket || webSocket.readyState !== WebSocket.OPEN) isServerConnected = false;
            if (!isServerConnected) connectionHealthStatus = 'failed';
            else {
                const timeSince = now - lastWebSocketMessageTime;
                if (timeSince > FAILED_THRESHOLD_MS) connectionHealthStatus = 'failed';
                else if (timeSince > LAGGING_THRESHOLD_MS) connectionHealthStatus = 'lagging';
                else connectionHealthStatus = 'good';
            }
            if (connectionHealthStatus !== prevHealth || isServerConnected !== prevConnected) updateConnectionStatusIndicator();
        }

        function updateConnectionStatusIndicator() {
            if (!connectionStatusDot) return;
            connectionStatusDot.classList.remove('streaming', 'good', 'lagging', 'bad');
            if (!isServerConnected) { connectionStatusDot.classList.add('bad'); return; }
            if (isStreamActive && isStreamActuallyRendering) connectionStatusDot.classList.add('streaming');
            else {
                if (connectionHealthStatus === 'good') connectionStatusDot.classList.add('good');
                else if (connectionHealthStatus === 'lagging') connectionStatusDot.classList.add('lagging');
                else connectionStatusDot.classList.add('bad');
            }
        }
        
        function updateLipoVoltageUI(v) { lipoVoltageDisplay.textContent = v.toFixed(2); const minV = 3.2, maxV = 4.2; let p = ((v - minV) / (maxV - minV)) * 100; p = Math.max(0, Math.min(100, p)); batteryBarLevel.style.width = p + '%'; batteryBarLevel.style.backgroundColor = v > 3.8 ? 'var(--battery-good)' : v > 3.5 ? 'var(--battery-medium)' : 'var(--battery-low)'; }
        function updateWifiStatusUI(rssi, c) { if (c) { wifiRssiDisplay.textContent = rssi + ' dBm'; const minR = -90, maxR = -30; let p = ((rssi - minR) / (maxR - minR)) * 100; p = Math.max(0, Math.min(100, p)); wifiBarLevel.style.width = p + '%'; } else { wifiRssiDisplay.textContent = 'N/A'; wifiBarLevel.style.width = '0%'; }}
        function updateFpsDisplayUI(fps) { fpsStatusDisplay.textContent = `FPS: ${fps.toFixed(1)}`; fpsStatusDisplay.style.display = isStreamActive ? 'block' : 'none'; }
        function updateSleepStatusIndicatorUI(isSl) { if (sleepStatusIndicator) sleepStatusIndicator.textContent = isSl ? 'Sleeping...' : 'Awake'; }
        
        function handleAPModeUI(inAP, wsConn = false) {
            isRobotInAPMode = inAP;
            mainControlsOverlay.style.display = 'block';
            apModeConfigSection.style.display = (inAP && !wsConn) ? 'block' : 'none';
            if (inAP && !wsConn) mainControlsOverlay.style.display = 'none';
            if (apModeConfigSectionForSettings) apModeConfigSectionForSettings.style.display = inAP ? 'block' : 'none';
        }

        function sendAPModeCredentials() {
            const ssid = settingsPanel.classList.contains('active') ? apModeSsidInputForSettings.value : apModeSsidInput.value;
            const pass = settingsPanel.classList.contains('active') ? apModePasswordInputForSettings.value : apModePasswordInput.value;
            if (!ssid) { showToast('Please enter Wi-Fi SSID.', 'error'); return; }
            sendWebSocketCommand({ command: 'set_wifi_credentials', ssid: ssid, password: pass });
            showToast('Credentials sent. Robot will restart. Reconnect & refresh.', 'info', 10000);
        }
        if (apModeConnectButton) apModeConnectButton.addEventListener('click', sendAPModeCredentials);
        if (apModeConnectButtonForSettings) apModeConnectButtonForSettings.addEventListener('click', sendAPModeCredentials);

        function toggleStream() {
            if (isStreamActive) sendWebSocketCommand({ command: 'stream_control', action: 'stop' });
            else {
                attemptingStreamStart = true; streamStartAttemptTime = Date.now();
                sendWebSocketCommand({ command: 'stream_control', action: 'start' });
                streamFeedbackPlaceholder.classList.add('hidden'); streamFeedbackLoading.classList.remove('hidden'); streamImg.style.display = 'none';
                const stTimeout = setTimeout(() => { if (attemptingStreamStart || (isStreamActive && streamImg.style.display === 'none')) { showToast('Stream timed out.', 'error'); stopStreamUI(); } attemptingStreamStart = false; }, 10000);
                streamImg.onload = () => { clearTimeout(stTimeout); attemptingStreamStart = false; isStreamActuallyRendering = true; streamImg.style.display = 'block'; streamFeedbackLoading.classList.add('hidden'); updateConnectionStatusIndicator(); };
                streamImg.onerror = () => { clearTimeout(stTimeout); isStreamActuallyRendering = false; if (attemptingStreamStart || isStreamActive) { if (Date.now() - streamStartAttemptTime > STREAM_START_GRACE_PERIOD) showToast('Stream error.', 'error'); } stopStreamUI(); attemptingStreamStart = false; };
                streamImg.src = streamUrlBase + '/stream?' + new Date().getTime();
            }
        }

        function startStreamUI() { isStreamActive = true; attemptingStreamStart = false; streamFeedbackPlaceholder.classList.add('hidden'); streamFeedbackLoading.classList.add('hidden'); streamImg.style.display = 'block'; streamToggle.innerHTML = '<span class="material-icons">stop</span>'; streamToggle.classList.add('active'); fpsStatusDisplay.style.display = 'block'; }
        function stopStreamUI() { isStreamActive = false; isStreamActuallyRendering = false; attemptingStreamStart = false; streamImg.style.display = 'none'; streamImg.src = ''; streamFeedbackLoading.classList.add('hidden'); streamFeedbackPlaceholder.classList.remove('hidden'); streamToggle.innerHTML = '<span class="material-icons">play_arrow</span>'; streamToggle.classList.remove('active'); fpsStatusDisplay.style.display = 'none'; updateConnectionStatusIndicator(); }
        
        function toggleSettingsPanel() {
            const isActive = settingsPanel.classList.toggle('active'); settingsOverlay.classList.toggle('active');
            if (isActive) { sendWebSocketCommand({ command: 'get_calibration' }); sendWebSocketCommand({ command: 'get_sleep_mode_pref' }); if (apModeConfigSectionForSettings) apModeConfigSectionForSettings.style.display = isRobotInAPMode ? 'block' : 'none'; }
        }
        settingsToggle.addEventListener('click', toggleSettingsPanel); settingsClose.addEventListener('click', toggleSettingsPanel); settingsOverlay.addEventListener('click', toggleSettingsPanel);

        function sendCameraSettings() { sendWebSocketCommand({ command: 'set_camera_settings', resolution: resolutionSelect.value, quality: parseInt(qualitySlider.value) }); }
        resolutionSelect.addEventListener('change', sendCameraSettings);
        qualitySlider.addEventListener('input', () => qualityValueDisplay.textContent = qualitySlider.value);
        qualitySlider.addEventListener('change', sendCameraSettings);

        ledSlider.addEventListener('input', () => ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : ledSlider.value + "%"));
        ledSlider.addEventListener('change', () => sendWebSocketCommand({ command: 'set_led', brightness: parseInt(ledSlider.value) }));
        lightSleepToggle.addEventListener('change', () => sendWebSocketCommand({ command: 'set_sleep_mode', enabled: lightSleepToggle.checked }));
        
        function sendJoystickData(x, y, force = false) { const now = Date.now(); if (force || now - lastControlSendTime > CONTROL_THROTTLE_MS) { sendWebSocketCommand({ command: 'control', x: x, y: y }); lastControlSendTime = now; }}
        function handleJoystickMove(event) {
            if (!joystickIsDragging) return; const rect = joystick.getBoundingClientRect();
            const clientX = event.clientX || (event.touches && event.touches[0].clientX); const clientY = event.clientY || (event.touches && event.touches[0].clientY);
            let x = clientX - rect.left - (rect.width / 2); let y = clientY - rect.top - (rect.height / 2);
            const maxOffset = rect.width / 2 - stick.offsetWidth / 2;
            x = Math.max(-maxOffset, Math.min(maxOffset, x)); y = Math.max(-maxOffset, Math.min(maxOffset, y));
            stick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
            sendJoystickData(x / maxOffset, -y / maxOffset);
        }
        function onJoystickEnd() { if (!joystickIsDragging) return; joystickIsDragging = false; stick.style.transform = 'translate(-50%, -50%)'; sendJoystickData(0, 0, true); }
        stick.addEventListener('mousedown', e => { joystickIsDragging = true; e.preventDefault(); });
        document.addEventListener('mousemove', e => { if (joystickIsDragging) handleJoystickMove(e); });
        document.addEventListener('mouseup', onJoystickEnd);
        stick.addEventListener('touchstart', e => { joystickIsDragging = true; e.preventDefault(); handleJoystickMove(e.touches[0]); }, { passive: false });
        document.addEventListener('touchmove', e => { if (joystickIsDragging && e.touches.length > 0) { handleJoystickMove(e.touches[0]); e.preventDefault(); }}, { passive: false });
        document.addEventListener('touchend', onJoystickEnd); document.addEventListener('touchcancel', onJoystickEnd);

        function loadCalibrationValuesToUI() { sendWebSocketCommand({ command: 'get_calibration' }); sendWebSocketCommand({ command: 'get_sleep_mode_pref' }); }
        function saveCalibrationValuesFromUI() { sendWebSocketCommand({ command: 'set_calibration', data: { sL_stop: parseInt(calServoLstop.value) || 0, sR_stop: parseInt(calServoRstop.value) || 0, lipo_ratio: parseFloat(calLipoRatio.value) || 0 } }); }
        function resetCalibrationOnRobot() { if (confirm("Reset ALL calibration to defaults?")) sendWebSocketCommand({ command: 'reset_calibration' }); }
        btnLoadCalib.addEventListener('click', loadCalibrationValuesToUI); 
        btnSaveCalib.addEventListener('click', saveCalibrationValuesFromUI); 
        btnResetCalib.addEventListener('click', resetCalibrationOnRobot);

        function loadInitialSettings() {
            ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : ledSlider.value + "%"); updateConnectionStatusIndicator();
            if (clientKeepAliveIntervalId) clearInterval(clientKeepAliveIntervalId);
            clientKeepAliveIntervalId = setInterval(() => {
                if (webSocket && webSocket.readyState === WebSocket.OPEN) webSocket.send(JSON.stringify({ command: 'ping' }));
                else if (webSocket && webSocket.readyState !== WebSocket.CONNECTING && !isRobotInAPMode) { connectWebSocket(); }
            }, 5000);
            if (healthCheckIntervalId) clearInterval(healthCheckIntervalId);
            healthCheckIntervalId = setInterval(checkConnectionHealth, 1000);
        }
        document.body.addEventListener('touchstart', e => { if (e.target === document.body || e.target === el('container') || e.target === el('video-container')) e.preventDefault(); }, { passive: false });
        document.body.addEventListener('touchmove', e => { if (e.target === document.body || e.target === el('container') || e.target === el('video-container')) e.preventDefault(); }, { passive: false });
        
        streamToggle.addEventListener('click', toggleStream); 
        document.addEventListener('DOMContentLoaded', () => { connectWebSocket(); loadInitialSettings(); });
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
void loadNvsSettings(); 
void saveServoCalibrationToNVS(); 
void saveCameraSettingsToNVS(); 
void saveSleepModePrefToNVS(); 
void saveWiFiCredentialsToNVS(const char* ssid, const char* password);
void handleGetCalibration(uint8_t clientId); 
void handleSetCalibration(uint8_t clientId, JsonDocument& doc); 
void handleResetCalibration(uint8_t clientId); 
void handleGetSleepModePref(uint8_t clientId); 
void handleSetSleepModePref(uint8_t clientId, JsonDocument& doc); 
void handleSetWiFiCredentials(uint8_t clientId, JsonDocument& doc);
int getCalibratedServoPulse(float controlValue, int stop_us); 
void processJoystickControlServos(float x, float y); 
float readLipoVoltage(); 
void handleStartStreamForHttpRequest(); 
void streamTask(void* parameter); 
bool sendMJPEGFrame(const uint8_t* buf, size_t len); 
const char* framesizeToString(framesize_t fs); 
framesize_t stringToFramesize(const String& fsStr); 
void handleCameraSettings(uint8_t clientId, JsonDocument& doc);
void sendCommandAck(uint8_t clientId, const char* command, const char* status);
void sendFullStatusUpdate(uint8_t targetClientNum, bool isInitialSetup);


// =================================================================================================
// 5. SETUP FUNCTION
// =================================================================================================
void setup() {
    // Serial.begin(115200); // Serial removed
    // Serial.println("\n\nESP32-CAM Servo Robot V4.1.2 (No Serial) Initializing..."); 

    lastClientActivityTimestamp = millis(); 
    loadNvsSettings(); 

    // if(!psramFound()){ Serial.println("PSRAM not found! Camera performance limited."); } 
    // else { Serial.println("PSRAM found."); }

    initCamera(); 
    initWiFi(); 
    setupLed(); 
    initServos();
    setupWebServerRoutes(); 
    webSocket.begin(); 
    webSocket.onEvent(webSocketEvent); 
    setupOTA(); 

    xTaskCreatePinnedToCore(TaskHttpServer, "HttpServerTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskOtaHandler, "OTATask", 4096, NULL, 1, NULL, 0);  
    xTaskCreatePinnedToCore(sleepManagementTask, "SleepMgmtTask", 4096, NULL, 0, &sleepManagementTaskHandle, 0); 

    // Serial.println("Setup complete. Main tasks started.");
    // if (!isInAPMode) { Serial.printf("HTTP: %d, WS: %d, Stream: /stream\n", HTTP_PORT, WEBSOCKET_PORT); }
    // Serial.printf("HTTP Auth User: %s (Pass: ***) | OTA Host: %s (Pass: ***) <<< CHANGE THESE!\n", HTTP_AUTH_USERNAME, OTA_HOSTNAME);       
}

// =================================================================================================
// 6. LOOP FUNCTION (Not used, tasks handle logic)
// =================================================================================================
void loop() { vTaskDelete(NULL); }

// =================================================================================================
// 7. FUNCTION IMPLEMENTATIONS
// =================================================================================================

// --- Helper Function for WebSocket Acknowledgements ---
void sendCommandAck(uint8_t clientId, const char* command, const char* status) {
    StaticJsonDocument<128> ackDoc;
    ackDoc["type"] = "command_ack";
    ackDoc["command"] = command;
    ackDoc["status"] = status;
    String response; serializeJson(ackDoc, response);
    webSocket.sendTXT(clientId, response);
}

// --- Centralized WebSocket Status Update Function ---
void sendFullStatusUpdate(uint8_t targetClientNum, bool isInitialSetup) {
    if (targetClientNum != BROADCAST_ALL_CLIENTS && !webSocket.isConnected(targetClientNum)) {
        return; // Don't try to send to a specific disconnected client
    }
    if (targetClientNum == BROADCAST_ALL_CLIENTS && webSocket.connectedClients() == 0) {
        return; // No clients to broadcast to
    }

    StaticJsonDocument<384> statusDoc;
    statusDoc["type"] = isInitialSetup ? "initial_settings" : "status_update";
    
    statusDoc["lipo_v"] = readLipoVoltage();
    statusDoc["rssi"] = WiFi.RSSI();
    statusDoc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    statusDoc["stream_active"] = isStreaming;
    statusDoc["is_sleeping"] = isLightSleeping;
    statusDoc["is_ap_mode"] = isInAPMode;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        statusDoc["resolution"] = framesizeToString(s->status.framesize);
        statusDoc["quality"] = s->status.quality;
    }
    
    statusDoc["led_brightness"] = g_current_led_brightness_percent;
    statusDoc["sleep_enabled"] = g_light_sleep_enabled;

    float cur_lipo = statusDoc["lipo_v"];
    if (cur_lipo < BATTERY_CRITICAL_VOLTAGE && cur_lipo > 1.0) { 
        statusDoc["battery_status"]="critical"; 
        if(!isBatteryCritical){ isBatteryCritical=true; if(isStreaming)isStreaming=false; applyLedBrightness(0); }
    } else if (cur_lipo < BATTERY_LOW_WARN_VOLTAGE && cur_lipo > 1.0) { 
        statusDoc["battery_status"]="low"; 
        if(isBatteryCritical) isBatteryCritical=false;
    } else { 
        statusDoc["battery_status"]="ok"; 
        if(isBatteryCritical) isBatteryCritical=false;
    }

    if (isStreaming && avg_frame_time_us_ema > 0.0f) {
        statusDoc["fps"] = round((1000000.0f / avg_frame_time_us_ema) * 10) / 10.0f;
    } else {
        statusDoc["fps"] = 0;
    }

    String response;
    serializeJson(statusDoc, response);

    if (targetClientNum == BROADCAST_ALL_CLIENTS) {
        webSocket.broadcastTXT(response);
    } else {
        webSocket.sendTXT(targetClientNum, response);
    }
}


// --- Light Sleep Management Task ---
void sleepManagementTask(void *pvParameters) {
    (void)pvParameters;
    const TickType_t checkInterval = pdMS_TO_TICKS(15000);
    for (;;) {
        vTaskDelay(checkInterval);
        if (g_light_sleep_enabled && !isLightSleeping && !isStreaming && !otaIsActive && !isInAPMode &&
            (millis() - lastClientActivityTimestamp > lightSleepTimeoutMs)) {
            processJoystickControlServos(0,0); applyLedBrightness(0);
            esp_wifi_set_ps(WIFI_PS_MIN_MODEM); esp_sleep_enable_wifi_wakeup();
            esp_sleep_enable_timer_wakeup(lightSleepPeriodicWakeupUs);
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
            isLightSleeping = true; broadcastStatusUpdate(); // Inform before sleeping
            esp_light_sleep_start();
            // --- WAKE UP ---
            isLightSleeping = false; esp_wifi_set_ps(WIFI_PS_NONE);
            if (WiFi.status() != WL_CONNECTED) { WiFi.reconnect(); vTaskDelay(pdMS_TO_TICKS(1000)); }
            if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && (millis() - lastClientActivityTimestamp >= 2000)) {
                lastClientActivityTimestamp = millis() - lightSleepTimeoutMs + (30 * 1000);
            } else { lastClientActivityTimestamp = millis(); }
            broadcastStatusUpdate(); // Inform after waking
        } else if (isLightSleeping && (!g_light_sleep_enabled || isStreaming || otaIsActive || isInAPMode)) {
            isLightSleeping = false; esp_wifi_set_ps(WIFI_PS_NONE);
            broadcastStatusUpdate();
        }
    }
}

// --- Authentication Helper ---
bool checkAuthentication() {
    if (!server.authenticate(HTTP_AUTH_USERNAME, HTTP_AUTH_PASSWORD)) {
        server.requestAuthentication(BASIC_AUTH, AUTH_REALM, "Auth required.");
        return false; 
    }
    return true; 
}

// --- NVS Settings Functions ---
void loadNvsSettings() {
    preferences.begin("robotCfg", true);
    servo_left_stop_us = preferences.getInt("sL_stop", DEFAULT_SERVO_STOP_US);
    servo_right_stop_us = preferences.getInt("sR_stop", DEFAULT_SERVO_STOP_US);
    lipo_calib_voltage_divider_ratio = preferences.getFloat("lipo_ratio", DEFAULT_VOLTAGE_DIVIDER_RATIO);
    nvs_cam_framesize = static_cast<framesize_t>(preferences.getInt("cam_res", static_cast<int>(FRAMESIZE_QVGA))); 
    nvs_cam_quality = preferences.getInt("cam_qual", 12); 
    g_light_sleep_enabled = preferences.getBool("sleep_en", true); 
    preferences.getString("sta_ssid", nvs_wifi_ssid, sizeof(nvs_wifi_ssid));
    preferences.getString("sta_psk", nvs_wifi_password, sizeof(nvs_wifi_password));
    preferences.end();
}

void saveServoCalibrationToNVS() { 
    preferences.begin("robotCfg", false); 
    preferences.putInt("sL_stop", servo_left_stop_us); 
    preferences.putInt("sR_stop", servo_right_stop_us); 
    preferences.putFloat("lipo_ratio", lipo_calib_voltage_divider_ratio);
    preferences.end(); 
}

void saveCameraSettingsToNVS() { 
    preferences.begin("robotCfg", false); 
    preferences.putInt("cam_res", static_cast<int>(nvs_cam_framesize)); 
    preferences.putInt("cam_qual", nvs_cam_quality);
    preferences.end(); 
}

void saveSleepModePrefToNVS(){
    preferences.begin("robotCfg", false);
    preferences.putBool("sleep_en", g_light_sleep_enabled);
    preferences.end();
}

void saveWiFiCredentialsToNVS(const char* ssid, const char* password) {
    preferences.begin("robotCfg", false);
    preferences.putString("sta_ssid", ssid);
    preferences.putString("sta_psk", password);
    preferences.end();
    strlcpy(nvs_wifi_ssid, ssid, sizeof(nvs_wifi_ssid));
    strlcpy(nvs_wifi_password, password, sizeof(nvs_wifi_password));
}

// --- WebSocket Calibration Handlers ---
void handleGetCalibration(uint8_t clientId) { 
    lastClientActivityTimestamp = millis();
    StaticJsonDocument<128> doc;
    doc["type"] = "calibration_data";
    doc["sL_stop"] = servo_left_stop_us; 
    doc["sR_stop"] = servo_right_stop_us; 
    doc["lipo_ratio"] = lipo_calib_voltage_divider_ratio;
    String response; serializeJson(doc, response); 
    webSocket.sendTXT(clientId, response);
}

void handleSetCalibration(uint8_t clientId, JsonDocument& doc) { 
    lastClientActivityTimestamp = millis();
    JsonObject data = doc["data"];
    servo_left_stop_us = data["sL_stop"] | DEFAULT_SERVO_STOP_US; 
    servo_right_stop_us = data["sR_stop"] | DEFAULT_SERVO_STOP_US; 
    lipo_calib_voltage_divider_ratio = data["lipo_ratio"] | DEFAULT_VOLTAGE_DIVIDER_RATIO;
    saveServoCalibrationToNVS(); 
    sendCommandAck(clientId, "set_calibration", "Calibration Saved");
}

void handleResetCalibration(uint8_t clientId) { 
    lastClientActivityTimestamp = millis();
    servo_left_stop_us = DEFAULT_SERVO_STOP_US; 
    servo_right_stop_us = DEFAULT_SERVO_STOP_US; 
    lipo_calib_voltage_divider_ratio = DEFAULT_VOLTAGE_DIVIDER_RATIO; 
    saveServoCalibrationToNVS(); 
    
    nvs_cam_framesize = FRAMESIZE_QVGA; nvs_cam_quality = 12; 
    saveCameraSettingsToNVS();          
    
    g_light_sleep_enabled = true; 
    saveSleepModePrefToNVS();

    sensor_t * s = esp_camera_sensor_get(); 
    if (s) { s->set_framesize(s, nvs_cam_framesize); s->set_quality(s, nvs_cam_quality); }
    
    sendCommandAck(clientId, "reset_calibration", "All Calibration Reset");
}

// --- OTA Setup and Task ---
void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME); ArduinoOTA.setPassword(OTA_PASSWORD); 
    ArduinoOTA.onStart([]() { otaIsActive = true; if (isStreaming) { isStreaming = false; if (streamTaskHandle != NULL) { if (streamClient.connected()) streamClient.stop(); vTaskDelay(pdMS_TO_TICKS(200)); if (eTaskGetState(streamTaskHandle) != eDeleted) vTaskDelete(streamTaskHandle); streamTaskHandle = NULL;}}})
        .onEnd([]() { otaIsActive = false; ESP.restart(); }) // Simplified: just restart after OTA
        .onProgress([](unsigned int p, unsigned int t) { /* Progress can be silent */ }) 
        .onError([](ota_error_t e) { otaIsActive = false; /* Error can be silent */});
    ArduinoOTA.begin(); 
}
void TaskOtaHandler(void *pvParameters) { (void)pvParameters; for (;;) { ArduinoOTA.handle(); vTaskDelay(pdMS_TO_TICKS(10)); } }

// --- Camera Initialization ---
void initCamera() { 
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM; 
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; 
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM; 
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM; config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = CAM_XCLK_FREQ; config.pixel_format = CAM_PIXEL_FORMAT; config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; 
    config.fb_location = CAMERA_FB_IN_PSRAM; config.frame_size = nvs_cam_framesize; config.jpeg_quality = nvs_cam_quality;        
    config.fb_count = psramFound() ? CAM_FB_COUNT : 1;
    if (!psramFound() && config.frame_size > FRAMESIZE_QVGA) {
        config.frame_size = FRAMESIZE_QVGA; nvs_cam_framesize = FRAMESIZE_QVGA; saveCameraSettingsToNVS();
    }
    if (esp_camera_init(&config) != ESP_OK) { ESP.restart(); }
    sensor_t * s = esp_camera_sensor_get(); if (s && s->id.PID == OV3660_PID) { s->set_vflip(s, 1); }
}

// --- WiFi Initialization ---
void initWiFi(bool forceAP) {
    WiFi.mode(WIFI_STA);
    if (forceAP) { setupAPMode(); return; }
    if (strlen(nvs_wifi_ssid) > 0) { 
        WiFi.setHostname(OTA_HOSTNAME); WiFi.begin(nvs_wifi_ssid, nvs_wifi_password);
        for (int retries = 0; WiFi.status() != WL_CONNECTED && retries < 20; retries++) { delay(500); }
        if (WiFi.status() == WL_CONNECTED) { isInAPMode = false; return; }
    }
    setupAPMode();
}
void setupAPMode() {
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    isInAPMode = true; lastClientActivityTimestamp = millis(); 
}

// --- LED Control Functions ---
void setupLed() { 
    ledc_timer_config_t lt = {LEDC_LOW_SPEED_MODE, (ledc_timer_bit_t)LED_RESOLUTION_BITS, LED_LEDC_TIMER_NUM, LED_FREQUENCY, LEDC_AUTO_CLK}; 
    ledc_timer_config(&lt);
    ledc_channel_config_t lc = {LED_PIN, LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM, LEDC_INTR_DISABLE, LED_LEDC_TIMER_NUM, 0, 0}; 
    ledc_channel_config(&lc); applyLedBrightness(0); 
}
float gammaCorrection(float b, float g) { return pow(constrain(b / 100.0f, 0.0f, 1.0f), g); }
void applyLedBrightness(int p) {
    if (isBatteryCritical && p > 0) p = 0;
    g_current_led_brightness_percent = constrain(p, 0, 100);
    uint32_t duty = static_cast<uint32_t>(gammaCorrection(g_current_led_brightness_percent, LED_GAMMA) * ((1 << LED_RESOLUTION_BITS) - 1));
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM, duty) == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM);
    }
}
void handleLedControl(uint8_t clientId, JsonDocument& doc) { 
    lastClientActivityTimestamp = millis();
    if (doc.containsKey("brightness")) { int b = doc["brightness"]; if (!isBatteryCritical || b == 0) applyLedBrightness(b); } 
}

// --- Servo Control Functions ---
void initServos() { 
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); 
    servoLeft.attach(SERVO_LEFT_PIN); servoRight.attach(SERVO_RIGHT_PIN);
    servoLeft.writeMicroseconds(servo_left_stop_us); servoRight.writeMicroseconds(servo_right_stop_us);  
}
int getCalibratedServoPulse(float controlValue, int stop_us) { 
    controlValue = constrain(controlValue, -1.0f, 1.0f);
    if (abs(controlValue) < SERVO_INPUT_DEADZONE_THRESHOLD) return stop_us; 
    int travel = static_cast<int>(controlValue * (SERVO_PULSE_SPAN_US - SERVO_PHYSICAL_DEADZONE_US));
    return stop_us + (controlValue > 0 ? SERVO_PHYSICAL_DEADZONE_US : -SERVO_PHYSICAL_DEADZONE_US) + travel;
}
void processJoystickControlServos(float x, float y) { 
    if (isBatteryCritical) { servoLeft.writeMicroseconds(servo_left_stop_us); servoRight.writeMicroseconds(servo_right_stop_us); return; }
    float leftSpeed = y, rightSpeed = y; 
    if (x > 0.05f) { rightSpeed = y * (1.0f - (x * 1.5f)); if (fabs(y) < 0.15f) { leftSpeed = x; rightSpeed = -x; } } 
    else if (x < -0.05f) { leftSpeed = y * (1.0f + (x * 1.5f)); if (fabs(y) < 0.15f) { leftSpeed = x; rightSpeed = -x; } } 
    servoLeft.writeMicroseconds(getCalibratedServoPulse(constrain(leftSpeed, -1.0f, 1.0f), servo_left_stop_us)); 
    servoRight.writeMicroseconds(getCalibratedServoPulse(constrain(-rightSpeed, -1.0f, 1.0f), servo_right_stop_us)); 
}

// --- LiPo ADC Reading ---
float readLipoVoltage() { 
    uint32_t adc_r = 0; for (int i = 0; i < 16; i++) { adc_r += analogRead(LIPO_ADC_PIN); delayMicroseconds(50); } adc_r /= 16;
    float voltageAtPin = adc_r * (DEFAULT_ADC_REF_VOLTAGE / 4095.0f );
    float raw_cal_v = voltageAtPin * lipo_calib_voltage_divider_ratio; 
    g_filtered_lipo_voltage = (g_filtered_lipo_voltage == 0.0f) ? raw_cal_v : (LIPO_FILTER_ALPHA * raw_cal_v) + ((1.0f - LIPO_FILTER_ALPHA) * g_filtered_lipo_voltage);
    return g_filtered_lipo_voltage; 
}

// --- Stream Handling Functions ---
void handleStartStreamForHttpRequest() {
    lastClientActivityTimestamp = millis();
    if (isBatteryCritical) { server.send(503, "text/plain", "Battery critical"); return; }
    if (!isStreaming) { /* Stream should be enabled via WebSocket first */ }
    if (streamTaskHandle != NULL && streamClient.connected() && streamClient != server.client()) { 
        bool oldIsStreaming = isStreaming; isStreaming = false; streamClient.stop(); 
        vTaskDelay(pdMS_TO_TICKS(250)); 
        if (streamTaskHandle != NULL && eTaskGetState(streamTaskHandle) != eDeleted) { vTaskDelete(streamTaskHandle); streamTaskHandle = NULL; }
        isStreaming = oldIsStreaming; 
    }
    streamClient = server.client(); 
    if (!streamClient || !streamClient.connected()) { isStreaming = false; sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false); return; }
    String r = "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: " + String(STREAM_CONTENT_TYPE) + "\r\nConnection: keep-alive\r\nCache-Control: no-cache\r\n\r\n"; 
    if (streamClient.print(r) != r.length()) { streamClient.stop(); isStreaming = false; sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false); return; }
    if (isStreaming && streamTaskHandle == NULL) { 
        xTaskCreatePinnedToCore(streamTask, "StreamTask", STREAM_TASK_STACK_SIZE, NULL, STREAM_TASK_PRIORITY, &streamTaskHandle, STREAM_TASK_CORE);
    }
}
bool sendMJPEGFrame(const uint8_t* buf, size_t len) { 
    if (!streamClient || !streamClient.connected()) { isStreaming = false; return false; } 
    if (streamClient.print(STREAM_BOUNDARY) != strlen(STREAM_BOUNDARY)) { isStreaming = false; return false; } 
    char hdr[HDR_BUF_LEN]; snprintf(hdr, HDR_BUF_LEN, STREAM_PART, len); 
    if (streamClient.print(hdr) != strlen(hdr)) { isStreaming = false; return false; } 
    if (streamClient.write(buf, len) != len) { isStreaming = false; return false; } 
    return streamClient.print("\r\n") == 2; 
}
void streamTask(void* p) { 
    (void)p; camera_fb_t *fb = NULL; int64_t last_frame_us = esp_timer_get_time();
    avg_frame_time_us_ema = 0.0f; 
    while (isStreaming) { 
        if (isBatteryCritical) { isStreaming = false; break; }
        lastClientActivityTimestamp = millis(); 
        if (!streamClient || !streamClient.connected()) { isStreaming = false; break; }
        fb = esp_camera_fb_get(); 
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; } 
        bool success = (fb->format == PIXFORMAT_JPEG) ? sendMJPEGFrame(fb->buf, fb->len) : false;
        esp_camera_fb_return(fb); fb = NULL; 
        if (!success) { isStreaming = false; break; }
        int64_t frame_time_us = esp_timer_get_time() - last_frame_us; last_frame_us += frame_time_us;
        avg_frame_time_us_ema = (avg_frame_time_us_ema == 0.0f) ? frame_time_us : (EMA_ALPHA_FPS * frame_time_us) + ((1.0f - EMA_ALPHA_FPS) * avg_frame_time_us_ema);
        vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS)); 
    }
    if (streamClient.connected()) streamClient.stop();
    if (xTaskGetCurrentTaskHandle() == streamTaskHandle) streamTaskHandle = NULL;
    isStreaming = false; sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false); vTaskDelete(NULL); 
}

// --- WebSocket Sleep/WiFi/Camera Settings Handlers ---
void handleGetSleepModePref(uint8_t clientId) { lastClientActivityTimestamp = millis(); StaticJsonDocument<64> doc; doc["type"] = "sleep_mode_pref"; doc["enabled"] = g_light_sleep_enabled; String r; serializeJson(doc, r); webSocket.sendTXT(clientId, r); }
void handleSetSleepModePref(uint8_t clientId, JsonDocument& doc) { lastClientActivityTimestamp = millis(); if (doc.containsKey("enabled")) { g_light_sleep_enabled = doc["enabled"]; saveSleepModePrefToNVS(); sendCommandAck(clientId, "set_sleep_mode", g_light_sleep_enabled ? "Sleep Enabled" : "Sleep Disabled"); }}
void handleSetWiFiCredentials(uint8_t clientId, JsonDocument& doc) { 
    lastClientActivityTimestamp = millis(); 
    if (doc.containsKey("ssid")) { 
        String s = doc["ssid"]; String p = doc["password"]; 
        saveWiFiCredentialsToNVS(s.c_str(), p.c_str()); 
        sendCommandAck(clientId, "set_wifi_credentials", "WiFi Saved. Restarting..."); 
        delay(1000); ESP.restart(); 
    }
}
void handleCameraSettings(uint8_t clientId, JsonDocument& doc) {
    lastClientActivityTimestamp = millis(); sensor_t * s = esp_camera_sensor_get();
    if (!s) { sendCommandAck(clientId, "set_camera_settings", "Sensor Error"); return; }
    bool changed = false; bool ok = true;
    if (doc.containsKey("resolution")) { framesize_t nfs = stringToFramesize(doc["resolution"].as<String>()); if (s->status.framesize != nfs) { if(s->set_framesize(s, nfs)==ESP_OK) {nvs_cam_framesize=nfs; changed=true;} else ok=false;}}
    if (doc.containsKey("quality")) { int q = constrain(doc["quality"].as<int>(),10,63); if (s->status.quality != q) { if(s->set_quality(s,q)==ESP_OK) {nvs_cam_quality=q; changed=true;} else ok=false;}}
    if (changed && ok) saveCameraSettingsToNVS();
    sendCommandAck(clientId, "set_camera_settings", !ok ? "Apply Error" : (changed ? "Cam Settings Saved" : "NoChange_Cam"));
}

// --- WebSocket Event Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    lastClientActivityTimestamp = millis(); 
    switch(type) {
        case WStype_DISCONNECTED: break; // Silent
        case WStype_CONNECTED: {
            sendFullStatusUpdate(num, true); // Send initial comprehensive status
            break;
        }
        case WStype_TEXT: {
            StaticJsonDocument<192> doc; 
            if (deserializeJson(doc, payload, length)) return;
            const char* cmd = doc["command"];
            if (!cmd) return;
            if (strcmp(cmd, "control") == 0) { processJoystickControlServos(doc["x"]|0.0f, doc["y"]|0.0f); }
            else if (strcmp(cmd, "set_led") == 0) { handleLedControl(num, doc); }
            else if (strcmp(cmd, "set_camera_settings") == 0) { handleCameraSettings(num, doc); }
            else if (strcmp(cmd, "stream_control") == 0) { 
                const char* act = doc["action"];
                if (act) {
                    if (strcmp(act, "start")==0 && !isBatteryCritical) { if(!isStreaming){isStreaming=true; broadcastStatusUpdate();}}
                    else if (strcmp(act, "stop")==0) { if(isStreaming){isStreaming=false; broadcastStatusUpdate();}}
                    else if (strcmp(act, "start")==0 && isBatteryCritical) { sendCommandAck(num, "stream_control", "Battery Critical!");}
                }
            }
            else if (strcmp(cmd, "get_calibration") == 0) { handleGetCalibration(num); }
            else if (strcmp(cmd, "set_calibration") == 0) { handleSetCalibration(num, doc); }
            else if (strcmp(cmd, "reset_calibration") == 0) { handleResetCalibration(num); }
            else if (strcmp(cmd, "get_sleep_mode_pref") == 0) { handleGetSleepModePref(num); }
            else if (strcmp(cmd, "set_sleep_mode") == 0) { handleSetSleepModePref(num, doc); }
            else if (strcmp(cmd, "set_wifi_credentials") == 0) { handleSetWiFiCredentials(num, doc); }
            else if (strcmp(cmd, "ping") == 0) { webSocket.sendTXT(num, "{\"type\":\"pong\"}"); }
            else if (strcmp(cmd, "get_full_status") == 0) { sendFullStatusUpdate(num, true); } // Send full status to this specific client
            break;
        }
        default: break;
    }
}

// --- Broadcast Status Update (wrapper) ---
void broadcastStatusUpdate() {
    sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false);
}

// --- Web Server Route Definitions ---
void setupWebServerRoutes() { 
    server.on("/", HTTP_GET, []() { if (!checkAuthentication()) return; lastClientActivityTimestamp = millis(); server.sendHeader("Connection", "close"); server.send_P(200, "text/html", htmlContentFlash); });
    server.on("/stream", HTTP_GET, [](){ if (!checkAuthentication()) return; handleStartStreamForHttpRequest(); }); 
    server.on("/camera/settings", HTTP_GET, []() { if (!checkAuthentication()) return; lastClientActivityTimestamp = millis(); sensor_t *s=esp_camera_sensor_get(); if(!s){server.send(500,"text/plain","Cam Err");return;} StaticJsonDocument<128>d; d["resolution"]=framesizeToString(s->status.framesize); d["quality"]=s->status.quality; String r; serializeJson(d,r); server.send(200,"application/json",r);});
    server.onNotFound([](){ if (!checkAuthentication()) return; server.send(404, "text/plain", "Not Found"); }); 
    server.begin();
}

// --- Camera Frame Size String Conversion ---
const char* framesizeToString(framesize_t fs) { 
    if (fs == FRAMESIZE_QVGA) return "QVGA"; if (fs == FRAMESIZE_SVGA) return "SVGA"; 
    if (fs == FRAMESIZE_XGA) return "XGA";   return "UNKNOWN"; 
}
framesize_t stringToFramesize(const String& fsStr) { 
    if (fsStr == "QVGA") return FRAMESIZE_QVGA; if (fsStr == "SVGA") return FRAMESIZE_SVGA; 
    if (fsStr == "XGA") return FRAMESIZE_XGA;   return FRAMESIZE_QVGA; // Default
}

// --- HTTP Server & WebSocket Task (Core 0) ---
void TaskHttpServer(void *pvParameters) { 
    (void)pvParameters; unsigned long lastStatusPush = 0;
    for (;;) { 
        if (!otaIsActive && !isLightSleeping) { 
            server.handleClient(); webSocket.loop(); 
            if (millis() - lastStatusPush > statusBroadcastInterval) {
                if(webSocket.connectedClients() > 0 && !isBatteryCritical) broadcastStatusUpdate();
                lastStatusPush = millis();
            }
        } else if (isLightSleeping) { vTaskDelay(pdMS_TO_TICKS(100)); } 
        else { vTaskDelay(pdMS_TO_TICKS(10)); } 
        vTaskDelay(pdMS_TO_TICKS(2)); 
    } 
}
