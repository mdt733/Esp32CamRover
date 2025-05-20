// =================================================================================================
// ESP32-CAM Servo Robot with Video Stream, OTA, Basic Authentication, and NVS Calibration
// Version 3.2.0 - Polished Release
// - Comprehensive review and polish of comments and code structure for clarity.
// - Cleaned up verbose Serial print statements for a cleaner console output.
// - Ensured all recent features (stream client handling, NVS camera settings, UI updates)
//   are integrated and functioning.
// - This version is intended as a stable, shareable baseline.
//
// Project Overview:
// This firmware transforms an ESP32-CAM module into a versatile Wi-Fi controlled robot.
// It's designed for hobbyists and enthusiasts looking to build a feature-rich remote-controlled
// device with live video feedback.
//
// Key Features:
// - Live MJPEG Video Streaming: View real-time video from the ESP32-CAM.
//   (UI rotates the stream -90 degrees to compensate for typical camera module orientation).
// - Dual Servo Control: Precise control for two 360-degree continuous rotation servos,
//   enabling differential drive movement (forward, backward, turns, pivots).
// - Web-Based User Interface:
//   - Responsive design accessible from desktop and mobile browsers.
//   - Joystick Control: Intuitive virtual joystick for movement (default: left side;
//     mobile: bottom-center, semi-transparent).
//   - Slider Control: Alternative dual vertical sliders for independent left/right motor speed.
// - Onboard LED Control: Adjustable brightness for the ESP32-CAM's flash LED, with gamma
//   correction for more linear perceived brightness.
// - LiPo Battery Monitoring:
//   - Displays the connected LiPo battery's voltage in the UI.
//   - Visual battery bar (3.2V - 4.2V range) with color-coding for quick status.
//   - Updates frequently to observe voltage sag under load.
// - Over-The-Air (OTA) Firmware Updates: Update the ESP32's firmware wirelessly via the
//   Arduino IDE or a web browser, eliminating the need for physical serial connections.
// - Basic HTTP Authentication: Protects access to the web interface and video stream
//   with a username and password.
// - Non-Volatile Storage (NVS) for Calibration:
//   - Servo Calibration: Fine-tune servo stop pulse and deadzone for each motor via the UI.
//   - LiPo ADC Calibration: Calibrate the voltage divider ratio for accurate battery readings.
//   - Camera Settings: Persistently stores the last used video resolution and quality.
//   - All calibration data is saved to the ESP32's NVS and loaded on boot.
// - Dual-Core Processing (ESP32):
//   - Core 0: Dedicated to handling Wi-Fi, web server requests, API calls, and OTA updates.
//   - Core 1: Dedicated to capturing and streaming video frames for smoother performance.
// - Stream Client Management: Serves the video stream to only the most recently connected client.
//   If a new client requests the stream, the previous client is disconnected.
//
// Origins:
// Based on concepts from the "esp32-caretaker" project by positron48 and significantly
// extended and refactored.
//
// -------------------------------------------------------------------------------------------------
// GPIO Pin Assignments (AI-Thinker ESP32-CAM Model assumed):
// -------------------------------------------------------------------------------------------------
// Internal Camera Connections (Do Not Change - Standard for ESP32-CAM AI-Thinker):
// - GPIO 0:  XCLK_GPIO_NUM (Camera Clock Input)
// - GPIO 5:  Y2_GPIO_NUM   (Camera Data Line 0)
// - GPIO 18: Y3_GPIO_NUM   (Camera Data Line 1)
// - GPIO 19: Y4_GPIO_NUM   (Camera Data Line 2)
// - GPIO 21: Y5_GPIO_NUM   (Camera Data Line 3)
// - GPIO 36: Y6_GPIO_NUM   (Camera Data Line 4) (Note: Often labeled VP on schematics)
// - GPIO 39: Y7_GPIO_NUM   (Camera Data Line 5) (Note: Often labeled VN on schematics)
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
// - GPIO 1 (U0TXD), GPIO 3 (U0RXD): Used for Serial Programming/Debugging.
//
// -------------------------------------------------------------------------------------------------
// User Configuration Notes:
// -------------------------------------------------------------------------------------------------
// 1. WiFi Credentials: Update `WIFI_SSID` and `WIFI_PASSWORD` constants below with your network details.
// 2. HTTP Authentication: CRITICAL - CHANGE `HTTP_AUTH_USERNAME` and `HTTP_AUTH_PASSWORD`
//    for security before exposing the device, even on a local network.
// 3. OTA Password: CRITICAL - CHANGE `OTA_PASSWORD` for security.
// 4. Servo Calibration:
//    - Access the web UI (Settings -> Calibration).
//    - Fine-tune "Stop Pulse" (center point, typically around 1500µs) and "Pulse Deadzone"
//      (range around stop where servo doesn't move, e.g., 20-50µs) for each servo.
//    - The movement range (Min/Max pulse) is determined by `SERVO_PULSE_SPAN_US` relative to the stop pulse.
//    - Save settings to NVS. Initial defaults are provided but WILL LIKELY NEED ADJUSTMENT.
// 5. LiPo Voltage Divider Calibration:
//    - Connect your LiPo battery via a voltage divider to `LIPO_ADC_PIN` (GPIO33).
//    - In the UI (Settings -> Calibration), set the "Voltage Divider Ratio" based on your
//      resistor values (Ratio = (R1+R2)/R2, where R2 is connected to GND and ADC input is between R1 and R2).
//    - Example: If R1=22k, R2=10k, Ratio = (22+10)/10 = 3.2.
//    - Save to NVS.
// 6. Camera Settings:
//    - Resolution and JPEG quality can be adjusted in the UI (Settings -> Video).
//    - These settings are automatically saved to NVS when changed and loaded on boot.
// =================================================================================================

// =================================================================================================
// 1. INCLUDES
// =================================================================================================

// ESP32 Core Libraries (Author: Espressif Systems, Version: Tied to ESP32 Arduino Core v2.0.14, based on ESP-IDF v4.4.x)
#include <WiFi.h>             // For Wi-Fi connectivity (connect, manage network)
#include <WiFiUdp.h>          // For UDP communication, used by ArduinoOTA
#include <ESPmDNS.h>          // For mDNS (Multicast DNS) service discovery, used by ArduinoOTA
#include <ArduinoOTA.h>       // For Over-The-Air firmware updates
#include <WebServer.h>        // For creating the HTTP web server
#include <esp_camera.h>       // For ESP32 camera functions and configurations
#include <Preferences.h>      // For NVS (Non-Volatile Storage) to save settings persistently

// ESP-IDF Specific Headers (Author: Espressif Systems, Version: Tied to ESP-IDF used by ESP32 Arduino Core v2.0.14)
#include "soc/soc.h"           // For direct System-on-Chip register access (e.g., brownout disable)
#include "soc/rtc_cntl_reg.h"  // For Real-Time Clock control registers (used with brownout)
#include "driver/ledc.h"       // For ESP-IDF LEDC (PWM) peripheral control, used for flash LED brightness

// Commonly used Libraries (check Arduino IDE Library Manager for specific installed versions)
#include <ESP32Servo.h>       // For controlling servo motors on ESP32.
                              // (Often bundled with ESP32 core or installable. e.g., by Kevin Harrington/John K. Bennett)
#include <ArduinoJson.h>      // For efficient JSON parsing and generation.
                              // (Author: Benoit Blanchon, Version: e.g., v6.x or v7.x - check Library Manager)

// Standard C/C++ Libraries (Part of the compiler toolchain, e.g., GCC newlib)
#include <math.h>             // For mathematical functions like pow() (used in gamma correction)

// =================================================================================================
// 2. CONFIGURATION & GLOBAL DEFINITIONS
// =================================================================================================

// --- Rolling average filter type definition ---
// Used for calculating an average FPS.
typedef struct {
    size_t size;    // Number of samples in the filter
    size_t index;   // Current index in the values array
    size_t count;   // Number of values currently in the filter (up to 'size')
    int sum;        // Sum of current values in the filter
    int* values;    // Array to store the sample values
} ra_filter_t;

// --- Network & Server Configuration ---
const int HTTP_PORT = 80;       // Port for the web server
const char* WIFI_SSID = "SSID"; // <<< YOUR WIFI SSID
const char* WIFI_PASSWORD = "password"; // <<< YOUR WIFI PASSWORD

// --- Basic Authentication Credentials ---
const char* HTTP_AUTH_USERNAME = "admin"; 
const char* HTTP_AUTH_PASSWORD = "password"; // <<< CHANGE THIS FOR SECURITY!
const char* AUTH_REALM = "ESP32-CAM Robot Access"; // Realm message for auth prompt

// --- OTA Configuration ---
const char* OTA_HOSTNAME = "esp32-cam-robot"; // Hostname for OTA updates
const char* OTA_PASSWORD = "password"; // <<< CHANGE THIS FOR SECURITY!

// --- Camera Pin Definitions (AI-Thinker ESP32-CAM specific) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1 // -1 if not used
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26 // SCCB D
#define SIOC_GPIO_NUM     27 // SCCB C
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
const int CAM_XCLK_FREQ = 20000000; // Camera clock frequency (20MHz)
const pixformat_t CAM_PIXEL_FORMAT = PIXFORMAT_JPEG; // Output format for streaming
const int CAM_FB_COUNT = 2;         // Number of frame buffers (2 recommended for PSRAM)
// NVS: Global variables for camera settings, with firmware defaults
framesize_t nvs_cam_framesize = FRAMESIZE_QVGA; // Default resolution, loaded from/saved to NVS
int nvs_cam_quality = 12;                     // Default JPEG quality (10-63, lower is better), NVS stored


// --- LED Configuration ---
const int LED_PIN = 4; // GPIO for the onboard flash LED
const ledc_channel_t LED_LEDC_CHANNEL_NUM = LEDC_CHANNEL_2; // LEDC channel for PWM
const ledc_timer_t LED_LEDC_TIMER_NUM = LEDC_TIMER_2;       // LEDC timer for PWM
const int LED_RESOLUTION_BITS = 8;     // PWM resolution (8-bit = 0-255 duty cycle)
const int LED_FREQUENCY = 5000;      // PWM frequency in Hz
const float LED_GAMMA = 2.2;         // Gamma correction factor for perceived brightness

// --- Servo Configuration (Default Constants) ---
const int SERVO_LEFT_PIN = 12;  // GPIO for the left servo
const int SERVO_RIGHT_PIN = 13; // GPIO for the right servo
Servo servoLeft;
Servo servoRight;
const int DEFAULT_SERVO_STOP_US = 1500;   // Default stop pulse width (microseconds)
const int SERVO_PULSE_SPAN_US = 500;      // Servo movement range: STOP +/- SPAN_US
const int DEFAULT_SERVO_PULSE_DEADZONE_US = 50; // Default deadzone around stop pulse

// NVS: Global variables to hold current servo calibration values
int servo_left_stop_us    = DEFAULT_SERVO_STOP_US;
int servo_left_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;
int servo_right_stop_us   = DEFAULT_SERVO_STOP_US;
int servo_right_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;

// --- LiPo ADC Configuration (Default Constants) ---
const int LIPO_ADC_PIN = 33; // ADC pin for LiPo voltage measurement
const float DEFAULT_VOLTAGE_DIVIDER_RATIO = 3.2; // Default ratio for voltage divider
const float DEFAULT_ADC_REF_VOLTAGE = 3.3;     // ESP32 ADC reference voltage (can vary)
const int ADC_RESOLUTION_BITS = 12;            // ESP32 ADC resolution (0-4095)
// NVS: Global variable for LiPo calibration
float lipo_calib_voltage_divider_ratio = DEFAULT_VOLTAGE_DIVIDER_RATIO;

// --- NVS ---
Preferences preferences; // Object for Non-Volatile Storage access

// --- WebServer ---
WebServer server(HTTP_PORT); // Web server object

// --- Stream Task Configuration & Globals ---
const size_t HDR_BUF_LEN = 64;       // Buffer for MJPEG stream part headers
const size_t MAX_FRAME_SIZE = 64 * 1024; // Max expected JPEG frame size
const int STREAM_TASK_STACK_SIZE = 8192; // Stack size for the video streaming task
const UBaseType_t STREAM_TASK_PRIORITY = 2; // Priority for the streaming task
const BaseType_t STREAM_TASK_CORE = 1;      // Core to pin the streaming task to (Core 1 for video)
const uint32_t STREAM_DELAY_MS = 20;        // Small delay in stream loop to yield CPU

// MJPEG stream constants
const char* PART_BOUNDARY = "123456789000000000000987654321";
const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
const char* STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

bool isStreaming = false;        // Flag to indicate if video stream is active
WiFiClient streamClient;         // WiFi client object for the active stream
uint8_t* streamBuffer = nullptr; // Buffer for non-JPEG to JPEG conversion (if ever needed)
size_t streamBufferSize = 0;

ra_filter_t ra_filter; // Global instance of the rolling average filter for FPS

// Robot control mode
enum ControlMode { JOYSTICK, SLIDERS };
ControlMode currentControlMode = JOYSTICK; 

// =================================================================================================
// 3. HTML CONTENT (using Raw String Literal)
// =================================================================================================
String htmlContent = R"RAW_HTML(
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
    <title>ESP32-CAM ServoBot V3.2.0</title>
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
            --toast-bg: rgba(20, 20, 20, 0.9);
            --toast-text-color: #f1f1f1;
            --toast-error-bg: rgba(220, 53, 69, 0.9);
            --battery-bar-bg: #555;
            --battery-good: #28a745;
            --battery-medium: #ffc107;
            --battery-low: #dc3545;
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
        .stream-status { /* Default for desktop */
            position: absolute; bottom: 10px; right: 10px; background: rgba(0,0,0,0.7);
            color: white; padding: 5px 10px; border-radius: 5px; font-size: 0.8em; z-index: 10;
            box-shadow: 0 2px 5px rgba(0,0,0,0.3);
        }
        .controls-overlay { position: absolute; z-index: 2; width: 100%; height: 100%; pointer-events: none; }
        .controls-top-left { 
            position: absolute; top: 15px; left: 15px; display: flex; align-items: center;
            gap: 10px; pointer-events: all; background: var(--control-bg);
            padding: 10px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3);
            backdrop-filter: blur(5px);
        }
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
        .status-bar { /* Default for desktop */
            position: absolute; bottom: 10px; left: 10px;
            background: var(--control-bg); padding: 8px 12px; border-radius: 5px;
            font-size: 0.85em; z-index: 5; color: var(--text-muted-color);
            box-shadow: 0 2px 5px rgba(0,0,0,0.3);
            display: flex; align-items: center; gap: 8px;
        }
        #lipo-voltage-display { font-weight: bold; color: var(--warning-color); }
        .battery-bar-container {
            width: 50px; height: 10px; background-color: var(--battery-bar-bg);
            border-radius: 3px; overflow: hidden; border: 1px solid #444;
        }
        .battery-bar-level {
            height: 100%; width: 0%; background-color: var(--battery-good);
            transition: width 0.3s ease-out, background-color 0.3s ease-out;
        }

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

        @media (max-width: 768px) { 
            .controls-joystick-panel {
                left: 50%; top: auto; bottom: 20px;
                transform: translateX(-50%); 
                background: rgba(33, 37, 41, 0.7); padding: 10px;
            }
            .joystick-container { width: 130px; height: 130px; background: rgba(255,255,255,0.03); }
            .slider-vertical-area { width: 60px; height: 150px; padding: 8px; }
            .slider-left-area { left: 10px; }
            .slider-right-area { right: 10px; }
            .slider-thumb { width: 36px; height: 36px; margin-top: -18px; }
            
            .status-bar { 
                font-size: 0.8em; padding: 6px 10px; 
                bottom: 180px; 
                left: 10px; 
            }
            .stream-status {
                bottom: 180px; 
                right: 10px;
                font-size: 0.75em; 
                padding: 4px 8px;
            }
            .battery-bar-container { width: 40px; }
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
        
        <div class="controls-overlay">
            <div class="controls-top-left">
                <button class="button icon-button" id="stream-toggle" title="Start/Stop Stream">
                    <span class="material-icons">play_arrow</span>
                </button>
                <button class="button icon-button" id="settings-toggle" title="Settings">
                    <span class="material-icons">settings</span>
                </button>
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
            
            <div class="status-bar">
                <span>LiPo: <span id="lipo-voltage-display">N/A</span> V</span>
                <div class="battery-bar-container">
                    <div class="battery-bar-level" id="battery-bar-level"></div>
                </div>
            </div>
            <div class="stream-status" id="fps-status" style="display: none;">FPS: ...</div>
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
        const fpsStatusDisplay = document.getElementById('fps-status');

        const settingsToggle = document.getElementById('settings-toggle');
        const settingsPanel = document.getElementById('settings-panel');
        const settingsClose = document.getElementById('settings-close');
        const settingsOverlay = document.getElementById('settings-overlay');
        
        const resolutionSelect = document.getElementById('resolution-select');
        const qualitySlider = document.getElementById('quality-slider');
        const qualityValueDisplay = document.getElementById('quality-value');
        const ledSlider = document.getElementById('led-slider');
        const ledValueDisplay = document.getElementById('led-value');
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

        let isStreamActive = false;
        let currentUiControlMode = 'joystick'; 
        let joystickIsDragging = false;
        let joystickCurrentX = 0, joystickCurrentY = 0;
        let lastControlSendTime = 0; 
        const CONTROL_THROTTLE_MS = 100; 

        let showFps = false;
        let statusUpdateIntervalId = null;
        let lipoUpdateIntervalId = null;
        const LIPO_UPDATE_INTERVAL_MS = 750; 

        const streamUrlBase = ''; 

        function showAppStatus(message, isError = false) {
            console.log((isError ? "ERROR: " : "STATUS: ") + message);
        }

        function toggleStream() { if (isStreamActive) stopStream(); else startStream(); }

        function startStream() {
            isStreamActive = true;
            streamFeedbackPlaceholder.classList.add('hidden');
            streamFeedbackLoading.classList.remove('hidden');
            streamImg.style.display = 'none'; 

            const streamTimeout = setTimeout(() => {
                if (isStreamActive && streamImg.style.display === 'none') { 
                    showAppStatus('Stream timed out.', true); stopStream(); 
                }
            }, 10000); 

            streamImg.onload = () => {
                clearTimeout(streamTimeout);
                streamImg.style.display = 'block';
                streamFeedbackLoading.classList.add('hidden');
            };
            streamImg.src = streamUrlBase + '/stream?' + new Date().getTime(); 
            
            streamToggle.innerHTML = '<span class="material-icons">stop</span>';
            streamToggle.classList.add('active');
            showAppStatus('Attempting to start stream...');
            if (showFps) fpsStatusDisplay.style.display = 'block';
            startStatusUpdates(); 
        }

        function stopStream() {
            isStreamActive = false;
            fetch(streamUrlBase + '/stopstream').catch(err => console.error("Stop stream err:", err));
            streamImg.style.display = 'none'; streamImg.src = ''; 
            streamFeedbackLoading.classList.add('hidden');
            streamFeedbackPlaceholder.classList.remove('hidden');
            streamToggle.innerHTML = '<span class="material-icons">play_arrow</span>';
            streamToggle.classList.remove('active');
            showAppStatus('Stream stopped.');
            fpsStatusDisplay.style.display = 'none';
            stopStatusUpdates();
        }
        
        streamImg.onerror = () => { if(isStreamActive) { showAppStatus('Stream error or disconnected.', true); stopStream(); } };
        function toggleSettingsPanel() {
            const isActive = settingsPanel.classList.toggle('active');
            settingsOverlay.classList.toggle('active');
            if (isActive) { loadCalibrationValuesToUI(); }
        }
        settingsToggle.addEventListener('click', toggleSettingsPanel);
        settingsClose.addEventListener('click', toggleSettingsPanel);
        settingsOverlay.addEventListener('click', toggleSettingsPanel);

        function sendCameraSettings() {
            const resolution = resolutionSelect.value; const quality = qualitySlider.value;
            fetch(`/quality?resolution=${resolution}&quality=${quality}`)
                .then(res => {
                    if(res.ok) showAppStatus(`Camera settings sent: ${resolution}, Q${quality}. Will be saved.`);
                    else if(res.status === 401) showAppStatus('Unauthorized to set camera settings.', true);
                    else showAppStatus('Failed to set camera quality.', true);
                })
                .catch(err => showAppStatus('Error setting camera quality: ' + err, true));
        }
        resolutionSelect.addEventListener('change', sendCameraSettings);
        qualitySlider.addEventListener('input', () => qualityValueDisplay.textContent = qualitySlider.value);
        qualitySlider.addEventListener('change', sendCameraSettings);

        function sendLedBrightness() {
            const brightness = ledSlider.value;
            fetch(`/led?brightness=${brightness}`)
                .then(res => {
                    if(res.ok) showAppStatus(`LED brightness: ${brightness}%`);
                    else if(res.status === 401) showAppStatus('Unauthorized to set LED.', true);
                    else showAppStatus('Failed to set LED. Status: ' + res.status, true);
                })
                .catch(err => showAppStatus('Error setting LED: ' + err, true));
        }
        ledSlider.addEventListener('input', () => ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : ledSlider.value + "%"));
        ledSlider.addEventListener('change', sendLedBrightness);
        
        function setUiControlMode(mode) {
            currentUiControlMode = mode;
            joystickTab.classList.toggle('active', mode === 'joystick'); slidersTab.classList.toggle('active', mode === 'sliders');
            joystickPanel.style.display = (mode === 'joystick') ? 'flex' : 'none'; joystickControl.classList.toggle('active', mode === 'joystick'); 
            slidersPanel.style.display = (mode === 'sliders') ? 'block' : 'none'; slidersPanel.classList.toggle('active', mode === 'sliders');
            fetch('/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ mode: currentUiControlMode, x:0, y:0, left:0, right:0 }) })
                .catch(err => showAppStatus('Error switching control mode: ' + err, true));
            resetJoystickVisuals(); resetSlidersVisuals();
        }
        joystickTab.addEventListener('click', () => setUiControlMode('joystick')); slidersTab.addEventListener('click', () => setUiControlMode('sliders'));

        function sendJoystickData(x, y, force = false) {
            const now = Date.now();
            if (force || now - lastControlSendTime > CONTROL_THROTTLE_MS) {
                fetch('/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ mode: 'joystick', x: x, y: y }) })
                    .catch(err => console.error('Joystick send error:', err));
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
                 fetch('/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ mode: 'sliders', left: left, right: right }) })
                    .catch(err => console.error('Sliders send error:', err));
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

        function updateLipoVoltage() {
            fetch('/lipo_voltage').then(response => response.text()).then(voltageStr => {
                const voltage = parseFloat(voltageStr); lipoVoltageDisplay.textContent = voltage.toFixed(2);
                const minV = 3.2, maxV = 4.2; let perc = ((voltage - minV) / (maxV - minV)) * 100;
                perc = Math.max(0, Math.min(100, perc)); batteryBarLevel.style.width = perc + '%';
                if (voltage > 3.8) batteryBarLevel.style.backgroundColor = 'var(--battery-good)';
                else if (voltage > 3.5) batteryBarLevel.style.backgroundColor = 'var(--battery-medium)';
                else batteryBarLevel.style.backgroundColor = 'var(--battery-low)';
            }).catch(error => console.error('Lipo voltage error:', error));
        }
        function updateStreamStatus() { 
            if (!isStreamActive || !showFps) { fpsStatusDisplay.style.display = 'none'; return; }
            fetch(streamUrlBase + '/status').then(response => response.json()).then(data => {
                if (data.streaming && showFps) { fpsStatusDisplay.textContent = `FPS: ${data.fps.toFixed(1)}`; fpsStatusDisplay.style.display = 'block'; }
                else { fpsStatusDisplay.style.display = 'none'; }
            }).catch(error => console.error('Stream status error:', error));
        }
        function startStatusUpdates() {
            stopStatusUpdates(); updateLipoVoltage(); if (showFps) updateStreamStatus();
            lipoUpdateIntervalId = setInterval(updateLipoVoltage, LIPO_UPDATE_INTERVAL_MS); 
            if (showFps) statusUpdateIntervalId = setInterval(updateStreamStatus, 2000);
        }
        function stopStatusUpdates() {
            if (lipoUpdateIntervalId) clearInterval(lipoUpdateIntervalId); if (statusUpdateIntervalId) clearInterval(statusUpdateIntervalId);
            lipoUpdateIntervalId = null; statusUpdateIntervalId = null;
        }
        showFpsToggle.addEventListener('click', () => {
            showFps = !showFps; fpsToggleIcon.textContent = showFps ? 'visibility' : 'visibility_off';
            if (showFps && isStreamActive) { fpsStatusDisplay.style.display = 'block'; startStatusUpdates(); }
            else { fpsStatusDisplay.style.display = 'none'; if (!showFps && statusUpdateIntervalId) { clearInterval(statusUpdateIntervalId); statusUpdateIntervalId = null; } }
        });

        function loadCalibrationValuesToUI() { 
            fetch('/get_calibration').then(response => {
                if (response.status === 401) { showAppStatus("Unauthorized to load calibration.", true); return null; }
                if (!response.ok) { throw new Error('Network response was not ok for get_calibration'); } return response.json();
            }).then(data => {
                if (!data) return; calServoLstop.value = data.sL_stop || ''; calServoLdz.value = data.sL_dz || '';
                calServoRstop.value = data.sR_stop || ''; calServoRdz.value = data.sR_dz || ''; calLipoRatio.value = data.lipo_ratio || '';
                showAppStatus("Servo/LiPo calibration values loaded into UI.");
            }).catch(error => { console.error('Error loading calibration values:', error); showAppStatus("Error loading calibration.", true); });
        }
        function saveCalibrationValuesFromUI() { 
            const calData = { sL_stop: parseInt(calServoLstop.value) || 0, sL_dz: parseInt(calServoLdz.value) || 0, sR_stop: parseInt(calServoRstop.value) || 0, sR_dz: parseInt(calServoRdz.value) || 0, lipo_ratio: parseFloat(calLipoRatio.value) || 0 };
            fetch('/set_calibration', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(calData) }).then(response => {
                if (response.ok) { showAppStatus("Servo/LiPo calibration saved to robot successfully."); return response.text(); }
                else if (response.status === 401) { showAppStatus("Unauthorized to save calibration.", true); }
                else { showAppStatus("Failed to save calibration. Status: " + response.status, true); }
            }).catch(error => { console.error('Error saving calibration:', error); showAppStatus("Error saving calibration.", true); });
        }
        function resetCalibrationOnRobot() { 
            if (!confirm("Are you sure you want to reset ALL calibration values (Servos, LiPo, Camera) to firmware defaults?")) return;
            fetch('/reset_calibration', { method: 'POST' }).then(response => {
                 if (response.ok) {
                    showAppStatus("All calibration reset to defaults on robot."); loadCalibrationValuesToUI(); loadInitialSettings(); return response.text();
                } else if (response.status === 401) { showAppStatus("Unauthorized to reset calibration.", true); }
                else { showAppStatus("Failed to reset calibration. Status: " + response.status, true); }
            }).catch(error => { console.error('Error resetting calibration:', error); showAppStatus("Error resetting calibration.", true); });
        }
        btnLoadCalib.addEventListener('click', loadCalibrationValuesToUI); btnSaveCalib.addEventListener('click', saveCalibrationValuesFromUI); btnResetCalib.addEventListener('click', resetCalibrationOnRobot);

        function loadInitialSettings() { 
            fetch('/camera/settings').then(response => {
                 if (response.status === 401) { showAppStatus("Unauthorized to load camera settings.", true); return null; }
                 if (!response.ok) { throw new Error('Network response was not ok for camera_settings'); } return response.json();
            }).then(data => {
                if (!data) return; if (data.resolution) resolutionSelect.value = data.resolution;
                if (data.quality) { qualitySlider.value = data.quality; qualityValueDisplay.textContent = data.quality; }
                showAppStatus("Initial camera settings loaded from robot.");
            }).catch(error => { console.error('Failed to load initial camera settings:', error); showAppStatus("Error loading initial camera settings.", true); });
            ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : ledSlider.value + "%"); setUiControlMode('joystick'); 
            updateLipoVoltage(); streamFeedbackPlaceholder.classList.remove('hidden'); streamFeedbackLoading.classList.add('hidden');
        }
        document.body.addEventListener('touchstart', function(e) { if (e.target === document.body || e.target === document.querySelector('.container') || e.target === document.querySelector('.video-container')) e.preventDefault(); }, { passive: false });
        document.body.addEventListener('touchmove', function(e) { if (e.target === document.body || e.target === document.querySelector('.container') || e.target === document.querySelector('.video-container')) e.preventDefault(); }, { passive: false });
        streamToggle.addEventListener('click', toggleStream); document.addEventListener('DOMContentLoaded', loadInitialSettings);
    </script>
</body>
</html>
)RAW_HTML";

// =================================================================================================
// 4. FUNCTION PROTOTYPES 
// =================================================================================================
bool checkAuthentication(); 
void initCamera(); 
void initWiFi(); 
void setupLed(); 
float gammaCorrection(float brightnessPercent, float gamma); 
void handleLedControl(); 
void initServos(); 
void TaskHttpServer(void *pvParameters); 
void TaskOtaHandler(void *pvParameters); 
void setupWebServerRoutes(); 
void setupOTA(); 
void loadCalibrationFromNVS(); 
void saveCalibrationToNVS(); // For Servo & LiPo
void saveCameraSettingsToNVS(); // Specifically for camera settings
void handleGetCalibration(); 
void handleSetCalibration(); 
void handleResetCalibration(); // Will now also reset camera NVS settings
int getCalibratedServoPulse(float controlValue, int stop_us, int pulse_deadzone_us, float input_deadzone_threshold = 0.05f); 
void processJoystickControlServos(float x, float y); 
void processSlidersControlServos(float leftSlider, float rightSlider); 
float readLipoVoltage(); 
void handleLipoVoltage(); 
ra_filter_t* ra_filter_init(ra_filter_t* filter, size_t sample_size); 
int ra_filter_run(ra_filter_t* filter, int value); 
void handleStartStream(); 
void handleStopStream(); 
void handleStreamStatus(); 
void streamTask(void* parameter); 
bool sendMJPEGFrame(const uint8_t* buf, size_t len); 
const char* framesizeToString(framesize_t fs); // Helper to convert framesize enum to string
framesize_t stringToFramesize(const String& fsStr); // Helper to convert string to framesize enum

// =================================================================================================
// 5. SETUP FUNCTION
// =================================================================================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector (use with stable power supply)

    Serial.begin(115200);
    Serial.println("\n\nESP32-CAM Servo Robot V3.2.0 (Polished Release) Initializing..."); 

    loadCalibrationFromNVS(); // Loads Servo, LiPo, AND Camera settings from NVS

    if(!psramFound()){
        Serial.println("PSRAM not found! Camera performance will be limited.");
    } else {
        Serial.println("PSRAM found.");
    }

    initCamera(); // Initializes camera using NVS loaded settings (or defaults if NVS is empty)
    initWiFi(); 

    setupLed(); 
    initServos();

    ra_filter_init(&ra_filter, 20); // Initialize rolling average filter for FPS (20 samples)

    setupWebServerRoutes(); 
    setupOTA(); 

    // Start HTTP server task on Core 0
    xTaskCreatePinnedToCore(TaskHttpServer, "HttpServerTask", 8192, NULL, 1, NULL, 0);
    // Start OTA handler task on Core 0
    xTaskCreatePinnedToCore(TaskOtaHandler, "OTATask", 4096, NULL, 1, NULL, 0);  

    Serial.println("Setup complete. Main tasks started.");
    Serial.printf("Web server on port %d. Stream on /stream.\n", HTTP_PORT);
    Serial.println("Calibrated settings (Servo, LiPo, Camera) loaded from NVS or defaults applied.");
    Serial.printf("HTTP Auth User: %s (Password: %s) <<< CHANGE THESE!\n", HTTP_AUTH_USERNAME, HTTP_AUTH_PASSWORD);
    Serial.printf("OTA Hostname: %s (Password: %s) <<< CHANGE THIS!\n", OTA_HOSTNAME, OTA_PASSWORD);
}

// =================================================================================================
// 6. LOOP FUNCTION (Main loop is minimal as tasks handle most operations)
// =================================================================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Keep main loop alive, but tasks do the work
}

// =================================================================================================
// 7. FUNCTION IMPLEMENTATIONS
// =================================================================================================

// --- Authentication Helper ---
// Checks if the client provided correct HTTP Basic Authentication credentials.
bool checkAuthentication() {
    if (!server.authenticate(HTTP_AUTH_USERNAME, HTTP_AUTH_PASSWORD)) {
        server.requestAuthentication(BASIC_AUTH, AUTH_REALM, "Authentication failed or required. Please login.");
        return false; 
    }
    return true; 
}

// --- NVS Calibration Functions ---
// Loads all calibration data (Servo, LiPo, Camera) from NVS.
// If NVS is empty or a key is missing, it uses the firmware defaults.
void loadCalibrationFromNVS() {
    preferences.begin("robotCalib", true); // Open NVS in read-only mode first

    // Servo Calibration
    servo_left_stop_us    = preferences.getInt("sL_stop", DEFAULT_SERVO_STOP_US);
    servo_left_pulse_deadzone_us = preferences.getInt("sL_dz", DEFAULT_SERVO_PULSE_DEADZONE_US);
    servo_right_stop_us   = preferences.getInt("sR_stop", DEFAULT_SERVO_STOP_US);
    servo_right_pulse_deadzone_us = preferences.getInt("sR_dz", DEFAULT_SERVO_PULSE_DEADZONE_US);
    
    // LiPo Calibration
    lipo_calib_voltage_divider_ratio = preferences.getFloat("lipo_ratio", DEFAULT_VOLTAGE_DIVIDER_RATIO);

    // Camera Settings Calibration
    nvs_cam_framesize = (framesize_t)preferences.getInt("cam_res_idx", (int)FRAMESIZE_QVGA); 
    nvs_cam_quality = preferences.getInt("cam_qual", 12); // Default quality if not found
    
    preferences.end();
    Serial.println("All calibration data (Servo, LiPo, Camera) loaded from NVS or defaults applied.");
}

// Saves ONLY Servo and LiPo calibration data to NVS.
void saveCalibrationToNVS() {
    preferences.begin("robotCalib", false); // Open NVS in read-write mode
    preferences.putInt("sL_stop", servo_left_stop_us);
    preferences.putInt("sL_dz", servo_left_pulse_deadzone_us);
    preferences.putInt("sR_stop", servo_right_stop_us);
    preferences.putInt("sR_dz", servo_right_pulse_deadzone_us);
    preferences.putFloat("lipo_ratio", lipo_calib_voltage_divider_ratio);
    preferences.end();
    Serial.println("Servo & LiPo calibration data saved to NVS.");
}

// Saves ONLY Camera settings (resolution index and quality) to NVS.
void saveCameraSettingsToNVS() {
    preferences.begin("robotCalib", false); // Open NVS in read-write mode
    preferences.putInt("cam_res_idx", (int)nvs_cam_framesize); // Store enum as int
    preferences.putInt("cam_qual", nvs_cam_quality);
    preferences.end();
    Serial.println("Camera settings saved to NVS.");
    Serial.printf(" Saved Camera: Resolution Index=%d (%s), Quality=%d\n", (int)nvs_cam_framesize, framesizeToString(nvs_cam_framesize), nvs_cam_quality);
}

// HTTP Handler: Gets current Servo & LiPo calibration values for the UI.
void handleGetCalibration() {
    if (!checkAuthentication()) return;
    StaticJsonDocument<256> doc; 
    doc["sL_stop"] = servo_left_stop_us;
    doc["sL_dz"] = servo_left_pulse_deadzone_us;
    doc["sR_stop"] = servo_right_stop_us;
    doc["sR_dz"] = servo_right_pulse_deadzone_us;
    doc["lipo_ratio"] = lipo_calib_voltage_divider_ratio;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

// HTTP Handler: Sets Servo & LiPo calibration values from UI and saves to NVS.
void handleSetCalibration() {
    if (!checkAuthentication()) return;
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Bad Request: No JSON payload"); return;
    }
    StaticJsonDocument<256> doc; 
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if (error) {
        server.send(400, "text/plain", "Bad Request: Invalid JSON"); return;
    }

    servo_left_stop_us    = doc["sL_stop"] | DEFAULT_SERVO_STOP_US;
    servo_left_pulse_deadzone_us = doc["sL_dz"] | DEFAULT_SERVO_PULSE_DEADZONE_US;
    servo_right_stop_us   = doc["sR_stop"] | DEFAULT_SERVO_STOP_US;
    servo_right_pulse_deadzone_us = doc["sR_dz"] | DEFAULT_SERVO_PULSE_DEADZONE_US;
    lipo_calib_voltage_divider_ratio = doc["lipo_ratio"] | DEFAULT_VOLTAGE_DIVIDER_RATIO;

    saveCalibrationToNVS(); // Saves only Servo & LiPo
    server.send(200, "text/plain", "Servo/LiPo Calibration Saved");
    Serial.println("Servo/LiPo calibration updated and saved from UI.");
}

// HTTP Handler: Resets ALL calibration (Servo, LiPo, Camera) to firmware defaults and saves to NVS.
void handleResetCalibration() {
    if (!checkAuthentication()) return;
    
    // Reset Servo values to firmware defaults
    servo_left_stop_us    = DEFAULT_SERVO_STOP_US;
    servo_left_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;
    servo_right_stop_us   = DEFAULT_SERVO_STOP_US;
    servo_right_pulse_deadzone_us = DEFAULT_SERVO_PULSE_DEADZONE_US;
    // Reset LiPo value to firmware default
    lipo_calib_voltage_divider_ratio = DEFAULT_VOLTAGE_DIVIDER_RATIO;
    saveCalibrationToNVS(); // Save reset Servo & LiPo values

    // Reset Camera NVS values to firmware defaults
    nvs_cam_framesize = FRAMESIZE_QVGA; 
    nvs_cam_quality = 12;               
    saveCameraSettingsToNVS();          // Save reset camera values

    // Re-initialize camera with these defaults immediately
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, nvs_cam_framesize);
        s->set_quality(s, nvs_cam_quality);
        Serial.println("Camera re-initialized with default settings after reset.");
    } else {
        Serial.println("Could not get camera sensor to apply reset defaults.");
    }
    
    server.send(200, "text/plain", "All Calibration (Servo, LiPo, Camera) Reset to Defaults");
    Serial.println("All calibration reset to firmware defaults and saved.");
}


// --- OTA Setup and Task ---
void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME); 
    ArduinoOTA.setPassword(OTA_PASSWORD); 
    ArduinoOTA
        .onStart([]() { 
            String type; if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch"; else type = "filesystem"; 
            Serial.println("OTA: Start updating " + type);
            if (isStreaming) { // Stop stream if active during OTA
                isStreaming = false; 
                if(streamClient && streamClient.connected()) streamClient.stop();
                vTaskDelay(pdMS_TO_TICKS(200)); // Give stream task time to stop
            }
        })
        .onEnd([]() { Serial.println("\nOTA: End. Rebooting..."); })
        .onProgress([](unsigned int progress, unsigned int total) { Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error) { 
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
// Task to handle OTA updates. Runs on Core 0.
void TaskOtaHandler(void *pvParameters) { 
    (void)pvParameters; 
    for (;;) { ArduinoOTA.handle(); vTaskDelay(pdMS_TO_TICKS(10)); } 
}

// --- Camera Initialization ---
// Initializes the camera using settings loaded from NVS (or firmware defaults).
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
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // Grab frames when buffer is empty
    config.fb_location = CAMERA_FB_IN_PSRAM;   // Store frame buffer in PSRAM

    // Use NVS loaded settings for camera initialization
    config.frame_size = nvs_cam_framesize; 
    config.jpeg_quality = nvs_cam_quality;        
    
    if (psramFound()) {
        config.fb_count = CAM_FB_COUNT;   
    } else { // If no PSRAM, override to smaller settings to ensure functionality
        Serial.println("Warning: PSRAM not found. Overriding camera settings to QQVGA.");
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 15; // Higher number is lower quality for QQVGA
        config.fb_count = 1;
        // Update NVS vars to reflect this override if PSRAM not found, so UI is consistent on next boot
        nvs_cam_framesize = FRAMESIZE_QQVGA;
        nvs_cam_quality = 15;
        saveCameraSettingsToNVS(); // Save the override
    }

    esp_err_t err = esp_camera_init(&config); 
    if (err != ESP_OK) { 
        Serial.printf("Camera init failed with error 0x%x: %s\n", err, esp_err_to_name(err)); 
        ESP.restart(); return; 
    }
    Serial.println("Camera initialized successfully.");
    Serial.printf(" Initial Active Camera: Resolution=%s, Quality=%d\n", framesizeToString(nvs_cam_framesize), nvs_cam_quality);

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) { // Apply some sensor-specific settings if OV3660
        s->set_vflip(s, 1);       // Vertical flip
        s->set_brightness(s, 1);  // Adjust brightness
        s->set_saturation(s, -2); // Adjust saturation
    }
}

// --- WiFi Initialization ---
void initWiFi() { 
    WiFi.mode(WIFI_STA); 
    WiFi.setHostname(OTA_HOSTNAME); 
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi "); Serial.print(WIFI_SSID);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) { // 15-second timeout
        Serial.print("."); delay(500); 
    }
    if (WiFi.status() == WL_CONNECTED) { 
        Serial.println("\nConnected to WiFi.");
        Serial.print("IP address: "); Serial.println(WiFi.localIP()); 
    } else { 
        Serial.println("\nFailed to connect to WiFi. Check credentials or network."); 
    }
}

// --- LED Control Functions ---
void setupLed() { 
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution  = (ledc_timer_bit_t)LED_RESOLUTION_BITS, .timer_num = LED_LEDC_TIMER_NUM, .freq_hz = LED_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK }; 
    if (ledc_timer_config(&ledc_timer) != ESP_OK) { Serial.println("LEDC timer config failed!"); return; }
    ledc_channel_config_t ledc_channel_conf = { .gpio_num   = LED_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LED_LEDC_CHANNEL_NUM, .intr_type  = LEDC_INTR_DISABLE, .timer_sel  = LED_LEDC_TIMER_NUM, .duty = 0, .hpoint = 0 }; 
    if (ledc_channel_config(&ledc_channel_conf) != ESP_OK) { Serial.println("LEDC channel config failed!"); return; }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM, 0); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM);
    Serial.println("LED control initialized.");
}
// Applies gamma correction to a brightness percentage.
float gammaCorrection(float brightnessPercent, float gamma) { 
    float normalizedBrightness = constrain(brightnessPercent / 100.0, 0.0, 1.0);
    return pow(normalizedBrightness, gamma);
}
// HTTP Handler: Controls LED brightness.
void handleLedControl() { 
    if (server.hasArg("brightness")) { 
        int brightnessPercent = constrain(server.arg("brightness").toInt(), 0, 100); 
        float correctedBrightnessNormalized = gammaCorrection(brightnessPercent, LED_GAMMA); 
        uint32_t max_duty = (1 << LED_RESOLUTION_BITS) - 1;
        uint32_t pwmValue = constrain((uint32_t)(correctedBrightnessNormalized * max_duty), (uint32_t)0, max_duty);
        if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM, pwmValue) == ESP_OK) {
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM);
        } else {
            Serial.println("Error setting LEDC duty.");
        }
        server.send(200, "text/plain", "OK_LED"); 
    } else { 
        server.send(400, "text/plain", "Missing brightness parameter"); 
    } 
}

// --- Servo Control Functions ---
void initServos() { 
    ESP32PWM::allocateTimer(1); // Allocate a timer for ESP32Servo library (can be 0, 1, 2, or 3)
    ESP32PWM::allocateTimer(3); // Allocate another timer if using multiple servos on different timers
    servoLeft.attach(SERVO_LEFT_PIN); 
    servoRight.attach(SERVO_RIGHT_PIN);
    servoLeft.writeMicroseconds(servo_left_stop_us);    // Set to calibrated stop position
    servoRight.writeMicroseconds(servo_right_stop_us);  // Set to calibrated stop position
    Serial.println("Servos initialized and set to stop positions.");
}
// Calculates servo pulse width based on control input (-1.0 to 1.0) and calibration.
int getCalibratedServoPulse(float controlValue, int stop_us, int pulse_deadzone_us, float input_deadzone_threshold) { 
    controlValue = constrain(controlValue, -1.0, 1.0);
    if (abs(controlValue) < input_deadzone_threshold) return stop_us; // Apply input deadzone
    
    int max_cw_us = stop_us + SERVO_PULSE_SPAN_US; 
    int max_ccw_us = stop_us - SERVO_PULSE_SPAN_US; 
    
    if (controlValue > 0) { // Clockwise / Forward
        return map(controlValue * 1000, (int)(input_deadzone_threshold * 1000), 1000, stop_us + pulse_deadzone_us, max_cw_us);
    } else { // Counter-Clockwise / Backward
        return map(controlValue * 1000, -1000, -(int)(input_deadzone_threshold * 1000), max_ccw_us, stop_us - pulse_deadzone_us);
    } 
}
// Processes joystick input (x, y) to control servo speeds for differential drive.
void processJoystickControlServos(float x, float y) { 
    float leftSpeed = y; float rightSpeed = y; // Base speed from Y-axis
    // Apply differential steering based on X-axis
    if (x > 0.05) { // Turning right
        rightSpeed = y * (1.0 - (x * 1.5)); 
        if (fabs(y) < 0.15) { leftSpeed = x; rightSpeed = -x; } // Pivot turn if y (forward/backward) is small
    } else if (x < -0.05) { // Turning left
        leftSpeed = y * (1.0 + (x * 1.5)); 
        if (fabs(y) < 0.15) { leftSpeed = x; rightSpeed = -x; } // Pivot turn
    } 
    leftSpeed = constrain(leftSpeed, -1.0, 1.0); 
    rightSpeed = constrain(rightSpeed, -1.0, 1.0);
    int leftPulse = getCalibratedServoPulse(leftSpeed, servo_left_stop_us, servo_left_pulse_deadzone_us);
    int rightPulse = getCalibratedServoPulse(-rightSpeed, servo_right_stop_us, servo_right_pulse_deadzone_us); // Right servo often needs inverted control
    servoLeft.writeMicroseconds(leftPulse); 
    servoRight.writeMicroseconds(rightPulse); 
}
// Processes slider inputs to control servo speeds independently.
void processSlidersControlServos(float leftSlider, float rightSlider) { 
    int leftPulse = getCalibratedServoPulse(constrain(leftSlider, -1.0, 1.0), servo_left_stop_us, servo_left_pulse_deadzone_us);
    int rightPulse = getCalibratedServoPulse(constrain(-rightSlider, -1.0, 1.0), servo_right_stop_us, servo_right_pulse_deadzone_us); // Invert right servo
    servoLeft.writeMicroseconds(leftPulse); 
    servoRight.writeMicroseconds(rightPulse);
}

// --- LiPo ADC Reading ---
float readLipoVoltage() { 
    uint32_t adc_reading = 0; 
    for (int i = 0; i < 64; i++) { adc_reading += analogRead(LIPO_ADC_PIN); } // Oversample for stability
    adc_reading /= 64;
    float voltageAtPin = adc_reading * (DEFAULT_ADC_REF_VOLTAGE / ((1 << ADC_RESOLUTION_BITS) -1.0) );
    return voltageAtPin * lipo_calib_voltage_divider_ratio; 
}
// HTTP Handler: Returns current LiPo voltage.
void handleLipoVoltage() { 
    float voltage = readLipoVoltage(); 
    server.send(200, "text/plain", String(voltage, 2)); 
}

// --- Stream Handling Functions ---
// Initializes the rolling average filter structure.
ra_filter_t* ra_filter_init(ra_filter_t* filter, size_t sample_size) { 
    memset(filter, 0, sizeof(ra_filter_t)); 
    filter->values = (int*)malloc(sample_size * sizeof(int)); 
    if (!filter->values) { Serial.println("Failed to allocate RA filter values!"); return nullptr; } 
    memset(filter->values, 0, sample_size * sizeof(int)); 
    filter->size = sample_size; 
    return filter; 
}
// Adds a new value to the filter and returns the current average.
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

// HTTP Handler: Starts the MJPEG video stream for a client.
// If a stream is already active, it disconnects the old client and serves the new one.
void handleStartStream() {
    WiFiClient newStreamClient = server.client(); // Get the client that made the request

    if (!newStreamClient || !newStreamClient.connected()) {
        if (newStreamClient) newStreamClient.stop(); // Clean up if object was created but not connected
        Serial.println("Stream request from disconnected client.");
        return;
    }

    if (isStreaming) {
        Serial.println("Switching video stream to new client...");
        if (streamClient && streamClient.connected()) {
            streamClient.stop(); // Stop the connection to the old client
        }
        isStreaming = false; // Signal the old streamTask to stop
        
        // Brief delay to allow the old task to detect client disconnection and terminate
        vTaskDelay(pdMS_TO_TICKS(100)); // Increased delay slightly for task cleanup
    }

    streamClient = newStreamClient; // Assign the new client to the global variable

    // Send MJPEG headers to the new client
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Access-Control-Allow-Origin: *\r\n"; // Allow cross-origin requests
    response += "Content-Type: " + String(STREAM_CONTENT_TYPE) + "\r\n";
    response += "Connection: keep-alive\r\n"; // Keep the connection open for streaming
    response += "Cache-Control: no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n";
    response += "Pragma: no-cache\r\n";
    response += "Expires: -1\r\n\r\n"; 
    
    if (streamClient.print(response) != response.length()) {
        Serial.println("Failed to send stream headers to new client.");
        streamClient.stop();
        isStreaming = false; 
        return;
    }

    isStreaming = true; // Set flag for the new stream
    
    // Create a new task for the new client. The old task (if any) should self-terminate.
    xTaskCreatePinnedToCore(
        streamTask, 
        "StreamTask", 
        STREAM_TASK_STACK_SIZE, 
        NULL, // No parameters passed to the task
        STREAM_TASK_PRIORITY, 
        NULL, // No task handle stored globally for this simple handover
        STREAM_TASK_CORE
    );
    Serial.println("Video stream task started/switched for new client.");
}

// HTTP Handler: Signals the stream task to stop.
void handleStopStream() { 
    if (isStreaming) {
        isStreaming = false; // Signal the current streamTask to stop
        Serial.println("Stream stop requested by client.");
    }
    server.send(200, "text/plain", "Stream stop requested.");
}

// HTTP Handler: Returns stream status (active, FPS).
void handleStreamStatus() { 
    float fps = 0; 
    if (ra_filter.count > 0 && ra_filter.sum > 0) { 
        uint32_t avg_frame_time_us = ra_filter.sum / ra_filter.count; 
        if (avg_frame_time_us > 0) fps = 1000000.0 / avg_frame_time_us; 
    }
    StaticJsonDocument<128> doc; 
    doc["streaming"] = isStreaming; 
    doc["fps"] = round(fps * 10) / 10.0; // Round FPS to one decimal place
    String response; 
    serializeJson(doc, response); 
    server.send(200, "application/json", response);
}

// Sends a single MJPEG frame to the connected streamClient.
bool sendMJPEGFrame(const uint8_t* buf, size_t len) { 
    if (!streamClient || !streamClient.connected()) { isStreaming = false; return false; } // Client disconnected
    if (streamClient.print(STREAM_BOUNDARY) != strlen(STREAM_BOUNDARY)) { isStreaming = false; return false; } // Failed to send boundary
    char hdrBuf[HDR_BUF_LEN]; 
    snprintf(hdrBuf, HDR_BUF_LEN, STREAM_PART, len); // Format part header with content length
    if (streamClient.print(hdrBuf) != strlen(hdrBuf)) { isStreaming = false; return false; } // Failed to send part header
    if (streamClient.write(buf, len) != len) { isStreaming = false; return false; } // Failed to send frame data
    if (streamClient.print("\r\n") != 2) { isStreaming = false; return false; } // Failed to send CRLF after frame
    return true; 
}

// Task responsible for capturing frames and sending them to the streamClient.
// Runs on Core 1.
void streamTask(void* parameter) { 
    (void)parameter; // Unused parameter
    camera_fb_t *fb = NULL; 
    int64_t last_frame_us = esp_timer_get_time();

    // Allocate stream buffer if needed (only if camera format isn't already JPEG)
    // Note: CAM_PIXEL_FORMAT is set to PIXFORMAT_JPEG, so this buffer is a fallback.
    if (!streamBuffer && CAM_PIXEL_FORMAT != PIXFORMAT_JPEG) { 
        streamBuffer = (uint8_t*)ps_malloc(MAX_FRAME_SIZE); 
        if (!streamBuffer) { 
            Serial.println("FATAL: Failed to allocate stream buffer! Stream task cannot run."); 
            isStreaming = false; // Ensure global flag is correct
            vTaskDelete(NULL); // Delete self if buffer allocation fails
            return;
        } 
    }
    
    Serial.println("Stream task instance running for a client.");

    while (isStreaming) { // Primary loop condition: global flag
        if (!streamClient || !streamClient.connected()) { // Check if this specific client is still connected
            Serial.println("Stream client disconnected in task. Ending this stream instance.");
            isStreaming = false; // This task is no longer the active stream
            break; 
        }
        // This check is redundant if CAM_PIXEL_FORMAT is JPEG, but good for robustness
        if (!streamBuffer && CAM_PIXEL_FORMAT != PIXFORMAT_JPEG) { 
             Serial.println("Stream buffer missing for non-JPEG frame, exiting task.");
             isStreaming = false;
             break;
        }

        fb = esp_camera_fb_get(); // Get a frame from the camera
        if (!fb) { 
            Serial.println("Camera frame capture failed in stream task.");
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit before retrying
            continue; 
        }
        
        bool success = false;
        if (fb->format == PIXFORMAT_JPEG) { // If frame is already JPEG, send directly
            success = sendMJPEGFrame(fb->buf, fb->len); 
        } else { 
            // This path should not be taken if CAM_PIXEL_FORMAT is correctly set to PIXFORMAT_JPEG
            Serial.println("Warning: Frame format is not JPEG as expected. Cannot stream.");
            success = false; // Treat as failure if format is unexpectedly not JPEG
        }
        
        esp_camera_fb_return(fb); // Return frame buffer to camera driver
        fb = NULL; 

        if (!success) { 
            Serial.println("Failed to send MJPEG frame or unsupported format. Stopping stream for this client.");
            isStreaming = false; // Signal to stop this task
            break; 
        }
        
        // Calculate and filter frame time for FPS
        int64_t fr_end_us = esp_timer_get_time(); 
        int64_t frame_time_us = fr_end_us - last_frame_us;
        last_frame_us = fr_end_us; 
        ra_filter_run(&ra_filter, frame_time_us); 
        
        vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS)); // Small delay to control frame rate and yield CPU
    }

    // Cleanup for this specific task instance
    if (streamClient && streamClient.connected()) {
        streamClient.stop(); // Ensure this client connection is closed
    }
    // The global isStreaming flag might have been set to false by another part of the code 
    // (e.g., a new client connecting or handleStopStream). This task just cleans up itself.
    Serial.println("Stream task instance ended.");
    vTaskDelete(NULL); // Task deletes itself
}

// --- Web Server Route Definitions ---
void setupWebServerRoutes() { 
    server.on("/", HTTP_GET, []() { if (!checkAuthentication()) return; server.sendHeader("Connection", "close"); server.send(200, "text/html", htmlContent); });
    server.on("/ping", HTTP_GET, []() { server.send(200, "text/plain", "pong from ESP32-CAM Robot v3.2.0"); });
    
    // Apply authentication directly to the /stream endpoint
    server.on("/stream", HTTP_GET, [](){ if (!checkAuthentication()) return; handleStartStream(); });
    
    server.on("/stopstream", HTTP_GET, [](){ if (!checkAuthentication()) return; handleStopStream(); });
    server.on("/status", HTTP_GET, [](){ if (!checkAuthentication()) return; handleStreamStatus(); }); 

    // Handles changes to camera resolution and quality. Saves to NVS.
    server.on("/quality", HTTP_GET, []() { 
        if (!checkAuthentication()) return; 
        sensor_t * s = esp_camera_sensor_get(); 
        if (!s) { server.send(500, "text/plain", "Camera sensor error"); return; }
        
        bool settings_changed_applied = false;
        if (server.hasArg("resolution")) {
            framesize_t newSize = stringToFramesize(server.arg("resolution"));
            if (s->status.framesize != newSize) { 
                if(s->set_framesize(s, newSize) == ESP_OK) { 
                    nvs_cam_framesize = newSize; // Update global for NVS
                    settings_changed_applied = true;
                    Serial.printf("Camera resolution changed to: %s\n", server.arg("resolution").c_str());
                } else {
                    Serial.printf("Failed to set camera resolution to: %s\n", server.arg("resolution").c_str());
                }
            }
        }
        if (server.hasArg("quality")) {
            int quality = constrain(server.arg("quality").toInt(), 10, 63);
            if (s->status.quality != quality) { 
                if (s->set_quality(s, quality) == ESP_OK) { 
                    nvs_cam_quality = quality; // Update global for NVS
                    settings_changed_applied = true;
                    Serial.printf("Camera quality changed to: %d\n", quality);
                } else {
                     Serial.printf("Failed to set camera quality to: %d\n", quality);
                }
            }
        }
        if (settings_changed_applied) {
            saveCameraSettingsToNVS(); // Save updated camera settings to NVS
        }
        server.send(200, "text/plain", settings_changed_applied ? "OK_Quality_Saved" : "NoChange_Quality");
    });

    // Returns current ACTIVE camera settings (resolution and quality).
    server.on("/camera/settings", HTTP_GET, []() { 
        if (!checkAuthentication()) return; 
        sensor_t * s = esp_camera_sensor_get(); 
        if (!s) { server.send(500, "text/plain", "Camera sensor error"); return; }
        StaticJsonDocument<128> doc; 
        doc["resolution"] = framesizeToString(s->status.framesize); 
        doc["quality"] = s->status.quality;
        String response; 
        serializeJson(doc, response); 
        server.send(200, "application/json", response);
    });

    server.on("/led", HTTP_GET, [](){ if (!checkAuthentication()) return; handleLedControl(); });
    server.on("/control", HTTP_POST, []() { 
        if (!checkAuthentication()) return; 
        StaticJsonDocument<200> doc; 
        DeserializationError error = deserializeJson(doc, server.arg("plain"));
        if (error) { server.send(400, "text/plain", "Invalid JSON"); return; } 
        const char* mode_str = doc["mode"] | "joystick"; 
        // Check if it's just a mode switch request (only "mode" key present)
        if (doc.containsKey("mode") && !doc.containsKey("x") && !doc.containsKey("y") && !doc.containsKey("left") && !doc.containsKey("right")) {
            if (strcmp(mode_str, "joystick") == 0) currentControlMode = JOYSTICK; 
            else if (strcmp(mode_str, "sliders") == 0) currentControlMode = SLIDERS;
            processJoystickControlServos(0,0); // Stop servos on mode switch
            Serial.printf("Control mode switched to: %s\n", mode_str);
        } else { // It's a movement command
            if (strcmp(mode_str, "joystick") == 0) { 
                currentControlMode = JOYSTICK; 
                float x = doc["x"] | 0.0f; float y = doc["y"] | 0.0f; 
                processJoystickControlServos(x, y); 
            } else if (strcmp(mode_str, "sliders") == 0) { 
                currentControlMode = SLIDERS; 
                float left = doc["left"] | 0.0f; float right = doc["right"] | 0.0f; 
                processSlidersControlServos(left, right); 
            }
        } 
        server.send(200, "text/plain", "OK_Control");
    });
    server.on("/lipo_voltage", HTTP_GET, [](){ if (!checkAuthentication()) return; handleLipoVoltage(); });
    
    // Calibration endpoints
    server.on("/get_calibration", HTTP_GET, handleGetCalibration); 
    server.on("/set_calibration", HTTP_POST, handleSetCalibration); 
    server.on("/reset_calibration", HTTP_POST, handleResetCalibration);

    server.onNotFound([](){ server.send(404, "text/plain", "Not Found"); }); 
    server.begin(); // Start the web server
    Serial.println("HTTP server started.");
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
    return FRAMESIZE_QVGA; // Default fallback if string is not recognized
}

// --- HTTP Server Task (Runs on Core 0) ---
void TaskHttpServer(void *pvParameters) { 
    (void)pvParameters; 
    Serial.println("HTTP Server Task running on core " + String(xPortGetCoreID()));
    for (;;) { 
        server.handleClient(); // Handle incoming client requests
        vTaskDelay(pdMS_TO_TICKS(2)); // Small delay to yield CPU
    } 
}
