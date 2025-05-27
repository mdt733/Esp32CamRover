// =================================================================================================
// ESP32-CAM Servo Robot - V4.1.15
//
// Functionality: Wi-Fi controlled robot with MJPEG video stream, joystick control via web UI,
// OTA updates, and basic HTTP authentication.
//
// External Pin Usage:
// - GPIO 4:  Onboard Flash LED
// - GPIO 12: Left Servo Signal
// - GPIO 13: Right Servo Signal
// - GPIO 33: LiPo Battery Voltage ADC Input
//
// Initial Setup (if Wi-Fi connection fails):
// 1. Connect to AP: "ESP32-CAM-Robot-Setup" (Password: "password123")
// 2. Open browser to: 192.168.4.1
// 3. Use Settings (gear icon) -> "Wi-Fi Configuration" to enter your network details.
//    Click "Save Wi-Fi & Restart".
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
char nvs_wifi_ssid[64] = "Optus_BA1148";
char nvs_wifi_password[64] = "lakes45553wx";
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
framesize_t nvs_cam_framesize = FRAMESIZE_SVGA; // Default to SVGA
const int CAM_QUALITY = 12;                   // Hardcoded camera quality

// --- LED Configuration ---
const int LED_PIN = 4;
const ledc_channel_t LED_LEDC_CHANNEL_NUM = LEDC_CHANNEL_2;
const ledc_timer_t LED_LEDC_TIMER_NUM = LEDC_TIMER_2;
const int LED_RESOLUTION_BITS = 8;
const int LED_FREQUENCY = 5000;
const float LED_GAMMA = 2.2;
int g_current_led_brightness_percent = 0;

// --- Servo Configuration (Hardcoded) ---
const int SERVO_LEFT_PIN = 12;
const int SERVO_RIGHT_PIN = 13;
Servo servoLeft;
Servo servoRight;
const int SERVO_LEFT_STOP_US = 1500;
const int SERVO_RIGHT_STOP_US = 1500;
const int SERVO_PULSE_SPAN_US = 500;
const float SERVO_INPUT_DEADZONE_THRESHOLD = 0.08f; 
const int SERVO_PHYSICAL_DEADZONE_US = 30; 

// --- LiPo ADC Configuration (Hardcoded Ratio) ---
const int LIPO_ADC_PIN = 33;
const float LIPO_VOLTAGE_DIVIDER_RATIO = 2.3f; 
const float DEFAULT_ADC_REF_VOLTAGE = 3.3;
const int ADC_RESOLUTION_BITS = 12;
const float BATTERY_LOW_WARN_VOLTAGE = 3.5f;
const float BATTERY_CRITICAL_VOLTAGE = 3.2f;
bool isBatteryCritical = false;
float g_filtered_lipo_voltage = 0.0f;
const float LIPO_FILTER_ALPHA = 0.1f;

// --- NVS (Non-Volatile Storage) ---
Preferences preferences; // Used only for Wi-Fi credentials

// --- WebServer & WebSockets ---
WebServer server(HTTP_PORT);
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
#define BROADCAST_ALL_CLIENTS 0xFF

// --- Stream Task Configuration & Globals ---
const size_t HDR_BUF_LEN = 64;
// MAX_FRAME_SIZE: QVGA: ~10-15KB, SVGA: ~30-70KB, XGA: ~80-150KB. 128KB is a safe upper bound.
const size_t MAX_FRAME_SIZE = 128 * 1024; 
const int STREAM_TASK_STACK_SIZE = 8192;
const UBaseType_t STREAM_TASK_PRIORITY = 2;
const BaseType_t STREAM_TASK_CORE = 1;
const uint32_t STREAM_DELAY_MS = 33; // Aim for ~30 FPS (1000ms / 30fps = ~33ms)
const char* PART_BOUNDARY = "123456789000000000000987654321";
const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
const char* STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
bool isStreaming = false;
WiFiClient streamClient;
TaskHandle_t streamTaskHandle = NULL;

// --- FPS Calculation (EMA) ---
static float avg_frame_time_us_ema = 0.0f;
const float EMA_ALPHA_FPS = 0.05f;

// --- Light Sleep Management (Permanently Enabled) ---
unsigned long lastClientActivityTimestamp = 0;
const unsigned long lightSleepTimeoutMs = 5 * 60 * 1000; // 5 minutes
const uint64_t lightSleepPeriodicWakeupUs = 60 * 1000 * 1000ULL; // 60 seconds
TaskHandle_t sleepManagementTaskHandle = NULL;
bool isLightSleeping = false;
unsigned long lastStatusBroadcastTime = 0;
const unsigned long statusBroadcastInterval = 750; // milliseconds


// =================================================================================================
// 3. HTML CONTENT (Stored in PROGMEM)
// =================================================================================================
static const char htmlContentFlash[] PROGMEM = R"RAW_HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>
    <title>ESP32-CAM Robot V4.1.15</title>
    <style>
        :root {
            --primary-color: #007bff; --secondary-color: #28a745; --info-color: #17a2b8; 
            --danger-color: #dc3545; --warning-color: #ffc107; --background-color: #212529;
            --card-background-color: #343a40; --text-color: #f8f9fa; --text-muted-color: #adb5bd;
            --control-bg: rgba(52, 58, 64, 0.85); --input-bg: #495057; --input-border-color: #5a6268;
            --battery-bar-bg: #555; --battery-good: #28a745; --battery-medium: #ffc107; --battery-low: #dc3545;
            --rssi-bar-color: var(--primary-color); --conn-status-good: var(--secondary-color);
            --conn-status-bad: var(--danger-color); --conn-status-streaming: var(--primary-color);
            --conn-status-lagging: var(--warning-color);
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
        .stream-status-fps { background: transparent; color: white; padding: 0; font-size: 0.8em; text-align: right; display: none; }
        #general-notification-area { font-size: 0.8em; color: var(--text-muted-color); text-align: right; min-height: 1em; }
        .controls-joystick-panel { position: absolute; left: 15px; top: 50%; transform: translateY(-50%); pointer-events: all; display: flex; flex-direction: column; gap: 15px; background: var(--control-bg); padding: 15px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); backdrop-filter: blur(5px); }
        .joystick-container { width: 160px; height: 160px; background: rgba(255,255,255,0.05); border-radius: 8px; padding: 15px; display: block; }
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
        #ap-mode-config { display: none; padding: 20px; background-color: var(--card-background-color); border-radius: 8px; margin: 20px; text-align: center; z-index: 1001; position: relative;}
        #ap-mode-config h3 { margin-bottom: 15px; } #ap-mode-config input { margin-bottom: 10px; }
        @media (max-width: 768px) { 
            .controls-joystick-panel { left: 50%; top: auto; bottom: 45px; transform: translateX(-50%); background: rgba(33, 37, 41, 0.7); padding: 10px; max-width: 180px; }
            .joystick-container { width: 130px; height: 130px; background: rgba(255,255,255,0.03); }
            .controls-top-right-status { font-size: 0.8em; padding: 6px 10px; }
            .stream-status-fps { font-size: 0.75em; padding: 0; } .bar-container { width: 40px; } 
            #general-notification-area { font-size: 0.75em; }
        }
        @font-face { font-family: 'Material Icons'; font-style: normal; font-weight: 400; src: url(https://fonts.gstatic.com/s/materialicons/v141/flUhRq6tzZclQEJ-Vdg-IuiaDsNc.woff2) format('woff2'); }
        .material-icons { font-family: 'Material Icons'; font-weight: normal; font-style: normal; font-size: 24px; line-height: 1; letter-spacing: normal; text-transform: none; display: inline-block; white-space: nowrap; word-wrap: normal; direction: ltr; -webkit-font-smoothing: antialiased; text-rendering: optimizeLegibility; -moz-osx-font-smoothing: grayscale; font-feature-settings: 'liga'; }
    </style>
</head>
<body>
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
            <button class="button" id="ap-mode-connect-button">Save Wi-Fi & Restart</button>
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
                <div id="general-notification-area">Initializing...</div>
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
                        <option value="QVGA">QVGA (320x240)</option>
                        <option value="SVGA" selected>SVGA (800x600)</option> <option value="XGA">XGA (1024x768)</option>
                    </select>
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
            <div class="settings-section" id="wifi-config-section-settings">
                <h4>Wi-Fi Configuration</h4>
                <input type="text" id="ap-mode-ssid-settings" class="settings-input" placeholder="Network Name (SSID)">
                <input type="password" id="ap-mode-password-settings" class="settings-input" placeholder="Password">
                <button class="button" id="ap-mode-connect-button-settings" style="background-color: var(--secondary-color);">Save Wi-Fi & Restart</button>
            </div>
            <div class="settings-section">
                <h4>System</h4>
                <button class="button" id="btn-restart-esp" style="background-color: var(--danger-color); width: 100%;">Restart ESP</button>
            </div>
        </div>
    </div>
    <script>
        const el = id => document.getElementById(id);
        const streamImg = el('stream'), streamFeedbackPlaceholder = el('stream-feedback-placeholder'), streamFeedbackLoading = el('stream-feedback-loading');
        const streamToggle = el('stream-toggle'), joystickPanel = el('joystick-panel');
        const joystick = el('joystick'), stick = el('stick'), lipoVoltageDisplay = el('lipo-voltage-display');
        const batteryBarLevel = el('battery-bar-level'), wifiRssiDisplay = el('wifi-rssi-display'), wifiBarLevel = el('wifi-bar-level');
        const connectionStatusDot = el('connection-status-dot'), fpsStatusDisplay = el('fps-status');
        const generalNotificationArea = el('general-notification-area');
        const settingsToggle = el('settings-toggle'), settingsPanel = el('settings-panel'), settingsClose = el('settings-close');
        const settingsOverlay = el('settings-overlay'), resolutionSelect = el('resolution-select');
        const ledSlider = el('led-slider'), ledValueDisplay = el('led-value');
        const btnRestartEsp = el('btn-restart-esp');
        const apModeConfigSection = el('ap-mode-config');
        const apModeSsidInput = el('ap-mode-ssid'), apModePasswordInput = el('ap-mode-password'), apModeConnectButton = el('ap-mode-connect-button');
        const apModeSsidInputForSettings = el('ap-mode-ssid-settings');
        const apModePasswordInputForSettings = el('ap-mode-password-settings');
        const apModeConnectButtonForSettings = el('ap-mode-connect-button-settings');
        const mainControlsOverlay = el('main-controls-overlay');

        let isStreamActive = false, joystickIsDragging = false; // JS state for stream
        let lastControlSendTime = 0, clientKeepAliveIntervalId, webSocket, isRobotInAPModeGlobal = false;
        let attemptingStreamStart = false, streamStartAttemptTime = 0, isServerConnected = false, lastWebSocketMessageTime = Date.now();
        let connectionHealthStatus = 'good', isStreamActuallyRendering = false, healthCheckIntervalId;
        
        const CONTROL_THROTTLE_MS = 100, STREAM_START_GRACE_PERIOD = 3000;
        const LAGGING_THRESHOLD_MS = 5000, FAILED_THRESHOLD_MS = 20000;
        const STREAM_ATTEMPT_TIMEOUT = 10000;
        const WEBSOCKET_PORT_JS = 81; 

        function updateGeneralNotification(message, type = 'info') {
            if (generalNotificationArea) {
                generalNotificationArea.textContent = message;
                generalNotificationArea.style.color = type === 'error' ? 'var(--danger-color)' : type === 'warning' ? 'var(--warning-color)' : 'var(--text-muted-color)';
            }
        }

        function connectWebSocket() {
            const wsUrl = `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.hostname}:${WEBSOCKET_PORT_JS}`;
            webSocket = new WebSocket(wsUrl);
            webSocket.onopen = () => {
                isServerConnected = true; lastWebSocketMessageTime = Date.now(); connectionHealthStatus = 'good'; 
                updateConnectionStatusIndicator(); 
                updateGeneralNotification('Connected!', 'info');
                if (webSocket.readyState === WebSocket.OPEN) webSocket.send(JSON.stringify({ command: "get_full_status" }));
                handleAPModeUI(isRobotInAPModeGlobal, true); 
            };
            webSocket.onmessage = event => {
                try {
                    lastWebSocketMessageTime = Date.now();
                    if (!isServerConnected || connectionHealthStatus !== 'good') { isServerConnected = true; connectionHealthStatus = 'good'; }
                    checkConnectionHealth();
                    const data = JSON.parse(event.data); 
                    
                    if (data.type === 'status_update' || data.type === 'initial_settings') { 
                        if (data.lipo_v !== undefined) updateLipoVoltageUI(data.lipo_v);
                        if (data.rssi !== undefined && data.wifi_connected !== undefined) updateWifiStatusUI(data.rssi, data.wifi_connected);
                        if (data.fps !== undefined && isStreamActive) updateFpsDisplayUI(data.fps); 
                        
                        if (data.is_sleeping !== undefined) {
                           updateGeneralNotification(data.is_sleeping ? 'Sleeping...' : (generalNotificationArea.textContent === 'Sleeping...' ? '' : generalNotificationArea.textContent));
                        }

                        if (data.is_ap_mode !== undefined) {
                            const wasInAPMode = isRobotInAPModeGlobal;
                            isRobotInAPModeGlobal = data.is_ap_mode;
                            handleAPModeUI(isRobotInAPModeGlobal, webSocket.readyState === WebSocket.OPEN);
                            if (isRobotInAPModeGlobal && !wasInAPMode) updateGeneralNotification('Robot in AP Mode. Configure Wi-Fi.', 'warning');
                        }

                        if (data.battery_status === 'low') updateGeneralNotification('Battery Low!', 'warning');
                        else if (data.battery_status === 'critical') updateGeneralNotification('BATTERY CRITICAL!', 'error');
                        
                        // Handle stream state changes from server
                        if (data.stream_active !== undefined) {
                            if (data.stream_active && !isStreamActive) { // Server wants to start, and we think it's stopped
                                isStreamActive = true; // Update JS state
                                startStreamUI();
                            } else if (!data.stream_active && isStreamActive) { // Server wants to stop, and we think it's started
                                isStreamActive = false; // Update JS state
                                stopStreamUI();
                            }
                        }
                        if (isStreamActive) attemptingStreamStart = false; // If server confirms stream is active, stop "attempting"

                        if (data.type === 'initial_settings') { 
                            if (data.resolution) resolutionSelect.value = data.resolution; // Set dropdown from server's current res
                            if (data.led_brightness !== undefined) { 
                                ledSlider.value = data.led_brightness; 
                                ledValueDisplay.textContent = (data.led_brightness == 0 ? "Off" : data.led_brightness + "%");
                            }
                        }
                    } else if (data.type === 'command_ack') {
                        updateGeneralNotification(data.status || `Cmd '${data.command}' OK.`, 'info');
                    } else if (data.type === 'error') {
                        updateGeneralNotification(`Error: ${data.message}`, 'error');
                    }
                } catch (e) { console.error('Error parsing WS msg:', e, event.data); updateGeneralNotification('WS Parse Error', 'error');}
            };
            webSocket.onclose = () => {
                updateGeneralNotification('Disconnected.', 'error'); 
                isServerConnected = false; connectionHealthStatus = 'failed'; 
                stopStreamUI(); 
                updateConnectionStatusIndicator();
                if (!isRobotInAPModeGlobal) setTimeout(connectWebSocket, 3000);
            };
            webSocket.onerror = () => {
                updateGeneralNotification('WS Connection Error.', 'error'); 
                isServerConnected = false; connectionHealthStatus = 'failed'; 
                stopStreamUI(); 
                updateConnectionStatusIndicator();
            };
        }

        function sendWebSocketCommand(cmd) {
            if (webSocket && webSocket.readyState === WebSocket.OPEN) webSocket.send(JSON.stringify(cmd));
            else updateGeneralNotification('Not connected. Cmd not sent.', 'error'); 
        }

        function checkConnectionHealth() {
            const now = Date.now(); 
            let prevHealth = connectionHealthStatus;
            isServerConnected = webSocket && webSocket.readyState === WebSocket.OPEN;
            
            if (!isServerConnected) connectionHealthStatus = 'failed';
            else {
                const timeSince = now - lastWebSocketMessageTime;
                if (timeSince > FAILED_THRESHOLD_MS) connectionHealthStatus = 'failed';
                else if (timeSince > LAGGING_THRESHOLD_MS) connectionHealthStatus = 'lagging';
                else connectionHealthStatus = 'good';
            }
            if (connectionHealthStatus !== prevHealth) updateConnectionStatusIndicator();
        }

        function updateConnectionStatusIndicator() {
            if (!connectionStatusDot) return;
            connectionStatusDot.className = 'connection-status-dot'; 
            if (!isServerConnected) connectionStatusDot.classList.add('bad');
            else if (isStreamActive && isStreamActuallyRendering) connectionStatusDot.classList.add('streaming');
            else connectionStatusDot.classList.add(connectionHealthStatus); 
        }
        
        function updateLipoVoltageUI(v) { lipoVoltageDisplay.textContent = v.toFixed(2); const minV = 3.2, maxV = 4.2; let p = Math.max(0, Math.min(100, ((v - minV) / (maxV - minV)) * 100)); batteryBarLevel.style.width = p + '%'; batteryBarLevel.style.backgroundColor = v > 3.8 ? 'var(--battery-good)' : v > 3.5 ? 'var(--battery-medium)' : 'var(--battery-low)'; }
        function updateWifiStatusUI(rssi, c) { if (c) { wifiRssiDisplay.textContent = `${rssi} dBm`; const minR = -90, maxR = -30; let p = Math.max(0, Math.min(100, ((rssi - minR) / (maxR - minR)) * 100)); wifiBarLevel.style.width = `${p}%`; } else { wifiRssiDisplay.textContent = 'N/A'; wifiBarLevel.style.width = '0%'; }}
        function updateFpsDisplayUI(fps) { fpsStatusDisplay.textContent = `FPS: ${fps.toFixed(1)}`; fpsStatusDisplay.style.display = isStreamActive ? 'block' : 'none'; }
        
        function handleAPModeUI(inAP, wsConnected = false) {
            isRobotInAPModeGlobal = inAP;
            apModeConfigSection.style.display = (inAP && !wsConnected) ? 'block' : 'none';
            mainControlsOverlay.style.display = (inAP && !wsConnected) ? 'none' : 'block';
        }

        function sendAPModeCredentials(isFromSettingsPanel) {
            const ssid = (isFromSettingsPanel ? apModeSsidInputForSettings : apModeSsidInput).value;
            const pass = (isFromSettingsPanel ? apModePasswordInputForSettings : apModePasswordInput).value;
            if (!ssid) { updateGeneralNotification('Please enter Wi-Fi SSID.', 'error'); return; }
            sendWebSocketCommand({ command: 'set_wifi_credentials', ssid: ssid, password: pass });
            updateGeneralNotification('Wi-Fi credentials sent. Robot will restart.', 'info');
        }
        if (apModeConnectButton) apModeConnectButton.addEventListener('click', () => sendAPModeCredentials(false));
        if (apModeConnectButtonForSettings) apModeConnectButtonForSettings.addEventListener('click', () => sendAPModeCredentials(true));

        let streamAttemptTimer = null;
        function toggleStream() {
            if (streamAttemptTimer) clearTimeout(streamAttemptTimer);
            
            if (isStreamActive) { // User wants to STOP
                sendWebSocketCommand({ command: 'stream_control', action: 'stop' });
                // stopStreamUI() will be called via WebSocket confirmation (data.stream_active: false)
            } else { // User wants to START
                attemptingStreamStart = true; 
                streamStartAttemptTime = Date.now();
                updateGeneralNotification('Starting stream...', 'info');
                streamFeedbackPlaceholder.classList.add('hidden'); 
                streamFeedbackLoading.classList.remove('hidden'); 
                streamImg.style.display = 'none'; 

                streamAttemptTimer = setTimeout(() => {
                    if (attemptingStreamStart) {
                        updateGeneralNotification('Stream start timed out.', 'error');
                        stopStreamUI(); 
                    }
                }, STREAM_ATTEMPT_TIMEOUT);
                sendWebSocketCommand({ command: 'stream_control', action: 'start' }); 
                // Actual stream image loading is now triggered by startStreamUI()
                // which is called when WS confirms stream_active: true
            }
        }

        function startStreamUI() { 
            if (streamAttemptTimer) clearTimeout(streamAttemptTimer);
            attemptingStreamStart = false; 

            streamFeedbackPlaceholder.classList.add('hidden'); 
            streamFeedbackLoading.classList.remove('hidden'); 

            streamToggle.innerHTML = '<span class="material-icons">stop</span>'; 
            streamToggle.classList.add('active'); 
            fpsStatusDisplay.style.display = 'block';

            streamImg.onload = () => { 
                if (streamAttemptTimer) clearTimeout(streamAttemptTimer); 
                isStreamActuallyRendering = true; 
                streamImg.style.display = 'block'; 
                streamFeedbackLoading.classList.add('hidden'); 
                updateConnectionStatusIndicator(); 
            };
            streamImg.onerror = () => { 
                if (streamAttemptTimer) clearTimeout(streamAttemptTimer);
                isStreamActuallyRendering = false; 
                updateGeneralNotification('Stream image load error.', 'error');
                if(isStreamActive) { 
                    stopStreamUI(); 
                }
            };
            streamImg.src = `/stream?${new Date().getTime()}`; // Request the stream
        }

        function stopStreamUI() { 
            if (streamAttemptTimer) clearTimeout(streamAttemptTimer);
            isStreamActive = false; 
            isStreamActuallyRendering = false; 
            attemptingStreamStart = false; 

            streamImg.style.display = 'none'; 
            streamImg.src = ''; 
            streamFeedbackLoading.classList.add('hidden'); 
            streamFeedbackPlaceholder.classList.remove('hidden'); 
            streamToggle.innerHTML = '<span class="material-icons">play_arrow</span>'; 
            streamToggle.classList.remove('active'); 
            fpsStatusDisplay.style.display = 'none'; 
            updateConnectionStatusIndicator(); 
        }
        
        function toggleSettingsPanel() {
            settingsPanel.classList.toggle('active'); settingsOverlay.classList.toggle('active');
        }
        settingsToggle.addEventListener('click', toggleSettingsPanel); 
        settingsClose.addEventListener('click', toggleSettingsPanel); 
        settingsOverlay.addEventListener('click', toggleSettingsPanel);

        resolutionSelect.addEventListener('change', () => sendWebSocketCommand({ command: 'set_camera_settings', resolution: resolutionSelect.value }));
        ledSlider.addEventListener('input', () => ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : `${ledSlider.value}%`));
        ledSlider.addEventListener('change', () => sendWebSocketCommand({ command: 'set_led', brightness: parseInt(ledSlider.value) }));
        
        function sendJoystickData(x, y, force = false) { 
            const now = Date.now(); 
            if (force || now - lastControlSendTime > CONTROL_THROTTLE_MS) { 
                sendWebSocketCommand({ command: 'control', x: x, y: y }); 
                lastControlSendTime = now; 
            }
        }
        function handleJoystickMove(event) {
            if (!joystickIsDragging) return; 
            const rect = joystick.getBoundingClientRect();
            const clientX = event.clientX || (event.touches && event.touches[0].clientX); 
            const clientY = event.clientY || (event.touches && event.touches[0].clientY);
            let x = (clientX - rect.left - rect.width / 2) / (rect.width / 2 - stick.offsetWidth / 2);
            let y = (clientY - rect.top - rect.height / 2) / (rect.height / 2 - stick.offsetHeight / 2);
            x = Math.max(-1, Math.min(1, x)); 
            y = Math.max(-1, Math.min(1, y)); 

            stick.style.transform = `translate(calc(-50% + ${x * (rect.width / 2 - stick.offsetWidth / 2)}px), calc(-50% + ${y * (rect.height / 2 - stick.offsetHeight / 2)}px))`;
            sendJoystickData(x, -y); 
        }
        function onJoystickEnd() { 
            if (!joystickIsDragging) return; 
            joystickIsDragging = false; 
            stick.style.transform = 'translate(-50%, -50%)'; 
            sendJoystickData(0, 0, true);
        }
        stick.addEventListener('mousedown', e => { joystickIsDragging = true; e.preventDefault(); handleJoystickMove(e); });
        document.addEventListener('mousemove', e => { if (joystickIsDragging) handleJoystickMove(e); });
        document.addEventListener('mouseup', onJoystickEnd);
        stick.addEventListener('touchstart', e => { joystickIsDragging = true; e.preventDefault(); handleJoystickMove(e.touches[0]); }, { passive: false });
        document.addEventListener('touchmove', e => { if (joystickIsDragging && e.touches.length > 0) { handleJoystickMove(e.touches[0]); e.preventDefault(); }}, { passive: false });
        document.addEventListener('touchend', onJoystickEnd); 
        document.addEventListener('touchcancel', onJoystickEnd);

        function restartESP() {
            if (window.confirm("Restart the Robot?")) sendWebSocketCommand({ command: 'restart_esp' });
        }
        btnRestartEsp.addEventListener('click', restartESP); 

        function loadInitialUISettings() {
            ledValueDisplay.textContent = (ledSlider.value == 0 ? "Off" : `${ledSlider.value}%`);
            updateConnectionStatusIndicator(); 
            updateGeneralNotification('Connecting...');

            if (clientKeepAliveIntervalId) clearInterval(clientKeepAliveIntervalId);
            clientKeepAliveIntervalId = setInterval(() => {
                if (webSocket && webSocket.readyState === WebSocket.OPEN) webSocket.send(JSON.stringify({ command: 'ping' }));
                else if (webSocket && webSocket.readyState === WebSocket.CONNECTING) { /* Wait */ }
                else if (!isRobotInAPModeGlobal) connectWebSocket(); 
            }, 5000);

            if (healthCheckIntervalId) clearInterval(healthCheckIntervalId);
            healthCheckIntervalId = setInterval(checkConnectionHealth, 1000);
        }
        
        document.body.addEventListener('touchstart', e => { if (e.target === document.body || e.target.classList.contains('container') || e.target.classList.contains('video-container')) e.preventDefault(); }, { passive: false });
        document.body.addEventListener('touchmove', e => { if (e.target === document.body || e.target.classList.contains('container') || e.target.classList.contains('video-container')) e.preventDefault(); }, { passive: false });
        
        streamToggle.addEventListener('click', toggleStream); 
        document.addEventListener('DOMContentLoaded', () => { 
            connectWebSocket(); 
            loadInitialUISettings(); 
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
void initWiFi(bool forceAP = false); // Default argument here
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
void saveWiFiCredentialsToNVS(const char* ssid, const char* password);
void handleSetWiFiCredentials(uint8_t clientId, JsonDocument& doc);
void handleRestartESP(uint8_t clientId);
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
    lastClientActivityTimestamp = millis(); 
    loadNvsSettings(); 

    initCamera(); // Will use default nvs_cam_framesize (now SVGA)
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
}

// =================================================================================================
// 6. LOOP FUNCTION (Not used)
// =================================================================================================
void loop() { vTaskDelete(NULL); }

// =================================================================================================
// 7. FUNCTION IMPLEMENTATIONS
// =================================================================================================

void sendCommandAck(uint8_t clientId, const char* command, const char* status) {
    StaticJsonDocument<128> ackDoc;
    ackDoc["type"] = "command_ack";
    ackDoc["command"] = command;
    ackDoc["status"] = status;
    String response; 
    serializeJson(ackDoc, response);
    webSocket.sendTXT(clientId, response);
}

void sendFullStatusUpdate(uint8_t targetClientNum, bool isInitialSetup) {
    if (targetClientNum != BROADCAST_ALL_CLIENTS && !webSocket.remoteIP(targetClientNum)) return; 
    if (targetClientNum == BROADCAST_ALL_CLIENTS && webSocket.connectedClients() == 0) return; 

    StaticJsonDocument<384> statusDoc;
    statusDoc["type"] = isInitialSetup ? "initial_settings" : "status_update";
    statusDoc["lipo_v"] = readLipoVoltage();
    statusDoc["rssi"] = WiFi.RSSI();
    statusDoc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    statusDoc["stream_active"] = isStreaming; 
    statusDoc["is_sleeping"] = isLightSleeping;
    statusDoc["is_ap_mode"] = isInAPMode;

    sensor_t *s = esp_camera_sensor_get();
    if (s) statusDoc["resolution"] = framesizeToString(s->status.framesize);
    
    statusDoc["led_brightness"] = g_current_led_brightness_percent;

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

    if (targetClientNum == BROADCAST_ALL_CLIENTS) webSocket.broadcastTXT(response);
    else webSocket.sendTXT(targetClientNum, response);
}

void sleepManagementTask(void *pvParameters) {
    (void)pvParameters;
    const TickType_t checkInterval = pdMS_TO_TICKS(15000);
    for (;;) {
        vTaskDelay(checkInterval);
        if (!isLightSleeping && !isStreaming && !otaIsActive && !isInAPMode &&
            (millis() - lastClientActivityTimestamp > lightSleepTimeoutMs)) {
            
            processJoystickControlServos(0,0); applyLedBrightness(0);
            esp_wifi_set_ps(WIFI_PS_MIN_MODEM); 
            esp_sleep_enable_wifi_wakeup();
            esp_sleep_enable_timer_wakeup(lightSleepPeriodicWakeupUs);
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
            
            isLightSleeping = true; 
            broadcastStatusUpdate(); 
            esp_light_sleep_start(); 
            
            isLightSleeping = false; 
            esp_wifi_set_ps(WIFI_PS_NONE); 
            if (WiFi.status() != WL_CONNECTED) { WiFi.reconnect(); vTaskDelay(pdMS_TO_TICKS(1000)); }
            
            if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && 
                (millis() - lastClientActivityTimestamp >= 2000)) {
                lastClientActivityTimestamp = millis() - lightSleepTimeoutMs + (30 * 1000);
            } else {
                lastClientActivityTimestamp = millis();
            }
            broadcastStatusUpdate();
        } else if (isLightSleeping && (isStreaming || otaIsActive || isInAPMode)) {
            isLightSleeping = false; 
            esp_wifi_set_ps(WIFI_PS_NONE);
            broadcastStatusUpdate();
        }
    }
}

bool checkAuthentication() {
    if (!server.authenticate(HTTP_AUTH_USERNAME, HTTP_AUTH_PASSWORD)) {
        server.requestAuthentication(BASIC_AUTH, AUTH_REALM, "Auth required.");
        return false; 
    }
    return true; 
}

void loadNvsSettings() {
    preferences.begin("robotCfg", true);
    preferences.getString("sta_ssid", nvs_wifi_ssid, sizeof(nvs_wifi_ssid));
    preferences.getString("sta_psk", nvs_wifi_password, sizeof(nvs_wifi_password));
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

void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME); 
    ArduinoOTA.setPassword(OTA_PASSWORD); 
    ArduinoOTA
        .onStart([]() { 
            otaIsActive = true; 
            if (isStreaming) { 
                isStreaming = false; 
                if (streamTaskHandle != NULL) { 
                    if (streamClient.connected()) streamClient.stop(); 
                    vTaskDelay(pdMS_TO_TICKS(200)); 
                    if (eTaskGetState(streamTaskHandle) != eDeleted) vTaskDelete(streamTaskHandle); 
                    streamTaskHandle = NULL;
                }
            }
        })
        .onEnd([]() { otaIsActive = false; ESP.restart(); }) 
        .onProgress([](unsigned int p, unsigned int t) {}) 
        .onError([](ota_error_t e) { otaIsActive = false; });
    ArduinoOTA.begin(); 
}

void TaskOtaHandler(void *pvParameters) { (void)pvParameters; for (;;) { ArduinoOTA.handle(); vTaskDelay(pdMS_TO_TICKS(10)); } }

void initCamera() { 
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM; 
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; 
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM; 
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM; config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = CAM_XCLK_FREQ; 
    config.pixel_format = CAM_PIXEL_FORMAT; 
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; 
    config.fb_location = CAMERA_FB_IN_PSRAM; 
    config.frame_size = nvs_cam_framesize; 
    config.jpeg_quality = CAM_QUALITY;
    config.fb_count = psramFound() ? CAM_FB_COUNT : 1;

    if (!psramFound() && config.frame_size > FRAMESIZE_QVGA) {
        config.frame_size = FRAMESIZE_QVGA; 
        nvs_cam_framesize = FRAMESIZE_QVGA; 
    }
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP.restart();
        return;
    }
    sensor_t * s = esp_camera_sensor_get(); if (s && s->id.PID == OV3660_PID) s->set_vflip(s, 1);
}

void initWiFi(bool forceAP) { // Definition matches prototype (no default arg here)
    WiFi.mode(WIFI_STA);
    if (forceAP) { setupAPMode(); return; }

    if (strlen(nvs_wifi_ssid) > 0 && strcmp(nvs_wifi_ssid, "YOUR_DEFAULT_SSID") != 0) {
        WiFi.setHostname(OTA_HOSTNAME); 
        WiFi.begin(nvs_wifi_ssid, nvs_wifi_password);
        for (int retries = 0; WiFi.status() != WL_CONNECTED && retries < 20; retries++) delay(500); 
        if (WiFi.status() == WL_CONNECTED) { 
            isInAPMode = false; 
            return; 
        }
    }
    setupAPMode();
}

void setupAPMode() {
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    isInAPMode = true; 
    lastClientActivityTimestamp = millis(); 
}

void setupLed() { 
    ledc_timer_config_t lt = {LEDC_LOW_SPEED_MODE, (ledc_timer_bit_t)LED_RESOLUTION_BITS, LED_LEDC_TIMER_NUM, LED_FREQUENCY, LEDC_AUTO_CLK}; 
    ledc_timer_config(&lt);
    ledc_channel_config_t lc = {LED_PIN, LEDC_LOW_SPEED_MODE, LED_LEDC_CHANNEL_NUM, LEDC_INTR_DISABLE, LED_LEDC_TIMER_NUM, 0, 0}; 
    ledc_channel_config(&lc); 
    applyLedBrightness(0); 
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
    if (doc.containsKey("brightness")) { 
        int b = doc["brightness"]; 
        if (!isBatteryCritical || b == 0) applyLedBrightness(b); 
    } 
}

void initServos() { 
    ESP32PWM::allocateTimer(0); 
    ESP32PWM::allocateTimer(1); 
    servoLeft.attach(SERVO_LEFT_PIN); 
    servoRight.attach(SERVO_RIGHT_PIN);
    servoLeft.writeMicroseconds(SERVO_LEFT_STOP_US); 
    servoRight.writeMicroseconds(SERVO_RIGHT_STOP_US);  
}

int getCalibratedServoPulse(float controlValue, int stop_us) {
    controlValue = constrain(controlValue, -1.0f, 1.0f);
    if (abs(controlValue) < SERVO_INPUT_DEADZONE_THRESHOLD) return stop_us; 
    int travel = static_cast<int>(controlValue * (SERVO_PULSE_SPAN_US - SERVO_PHYSICAL_DEADZONE_US));
    return stop_us + (controlValue > 0 ? SERVO_PHYSICAL_DEADZONE_US : -SERVO_PHYSICAL_DEADZONE_US) + travel;
}

void processJoystickControlServos(float x, float y) { 
    if (isBatteryCritical) { 
        servoLeft.writeMicroseconds(SERVO_LEFT_STOP_US); 
        servoRight.writeMicroseconds(SERVO_RIGHT_STOP_US); 
        return; 
    }
    float leftSpeed = y, rightSpeed = y; 
    if (x > 0.05f) { 
        rightSpeed = y * (1.0f - (x * 1.5f)); 
        if (fabs(y) < 0.15f) { leftSpeed = x; rightSpeed = -x; } 
    } else if (x < -0.05f) { 
        leftSpeed = y * (1.0f + (x * 1.5f)); 
        if (fabs(y) < 0.15f) { leftSpeed = x; rightSpeed = -x; } 
    } 
    servoLeft.writeMicroseconds(getCalibratedServoPulse(constrain(leftSpeed, -1.0f, 1.0f), SERVO_LEFT_STOP_US)); 
    servoRight.writeMicroseconds(getCalibratedServoPulse(constrain(-rightSpeed, -1.0f, 1.0f), SERVO_RIGHT_STOP_US));
}

float readLipoVoltage() { 
    uint32_t adc_r = 0; 
    for (int i = 0; i < 16; i++) { adc_r += analogRead(LIPO_ADC_PIN); delayMicroseconds(50); } 
    adc_r /= 16;
    float voltageAtPin = adc_r * (DEFAULT_ADC_REF_VOLTAGE / 4095.0f );
    float raw_cal_v = voltageAtPin * LIPO_VOLTAGE_DIVIDER_RATIO; 
    g_filtered_lipo_voltage = (g_filtered_lipo_voltage == 0.0f) ? raw_cal_v : (LIPO_FILTER_ALPHA * raw_cal_v) + ((1.0f - LIPO_FILTER_ALPHA) * g_filtered_lipo_voltage);
    return g_filtered_lipo_voltage; 
}

void handleStartStreamForHttpRequest() {
    lastClientActivityTimestamp = millis();
    if (isBatteryCritical) { server.send(503, "text/plain", "Battery critical - stream disabled"); return; }
    
    if (streamTaskHandle != NULL && streamClient.connected() && streamClient != server.client()) { 
        bool oldIsStreamingState = isStreaming; 
        isStreaming = false; 
        streamClient.stop(); 
        vTaskDelay(pdMS_TO_TICKS(250)); 
        if (streamTaskHandle != NULL && eTaskGetState(streamTaskHandle) != eDeleted) { 
            vTaskDelete(streamTaskHandle); streamTaskHandle = NULL; 
        }
        isStreaming = oldIsStreamingState; 
    }

    streamClient = server.client(); 
    if (!streamClient || !streamClient.connected()) { 
        isStreaming = false; sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false); return; 
    }

    String r = "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: " + String(STREAM_CONTENT_TYPE) + "\r\nConnection: keep-alive\r\nCache-Control: no-cache\r\n\r\n"; 
    if (streamClient.print(r) != r.length()) { 
        streamClient.stop(); isStreaming = false; sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false); return; 
    }

    if (isStreaming && streamTaskHandle == NULL) { 
        xTaskCreatePinnedToCore(streamTask, "StreamTask", STREAM_TASK_STACK_SIZE, NULL, STREAM_TASK_PRIORITY, &streamTaskHandle, STREAM_TASK_CORE);
    }
}

bool sendMJPEGFrame(const uint8_t* buf, size_t len) { 
    if (!streamClient || !streamClient.connected()) { isStreaming = false; return false; } 
    if (streamClient.print(STREAM_BOUNDARY) != strlen(STREAM_BOUNDARY)) { isStreaming = false; return false; } 
    char hdr[HDR_BUF_LEN]; 
    snprintf(hdr, HDR_BUF_LEN, STREAM_PART, len); 
    if (streamClient.print(hdr) != strlen(hdr)) { isStreaming = false; return false; } 
    if (streamClient.write(buf, len) != len) { isStreaming = false; return false; } 
    return streamClient.print("\r\n") == 2;
}

void streamTask(void* p) { 
    (void)p; 
    camera_fb_t *fb = NULL; 
    int64_t last_frame_us = esp_timer_get_time();
    avg_frame_time_us_ema = 0.0f;

    while (isStreaming) { 
        if (isBatteryCritical) { isStreaming = false; break; }
        lastClientActivityTimestamp = millis(); 
        if (!streamClient || !streamClient.connected()) { 
            isStreaming = false; 
            break; 
        }

        fb = esp_camera_fb_get(); 
        if (!fb) { 
            vTaskDelay(pdMS_TO_TICKS(10)); 
            continue; 
        } 

        bool success = (fb->format == PIXFORMAT_JPEG) ? sendMJPEGFrame(fb->buf, fb->len) : false;
        esp_camera_fb_return(fb); fb = NULL; 
        
        if (!success) { 
            isStreaming = false; 
            break; 
        }

        int64_t frame_time_us = esp_timer_get_time() - last_frame_us; 
        last_frame_us += frame_time_us;
        avg_frame_time_us_ema = (avg_frame_time_us_ema == 0.0f) ? frame_time_us : (EMA_ALPHA_FPS * frame_time_us) + ((1.0f - EMA_ALPHA_FPS) * avg_frame_time_us_ema);
        
        vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS)); 
    }

    if (streamClient.connected()) streamClient.stop();
    if (xTaskGetCurrentTaskHandle() == streamTaskHandle) streamTaskHandle = NULL;
    
    if (isStreaming) { 
        isStreaming = false; 
        sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false); 
    }
    vTaskDelete(NULL);
}

void handleSetWiFiCredentials(uint8_t clientId, JsonDocument& doc) { 
    lastClientActivityTimestamp = millis(); 
    if (doc.containsKey("ssid")) { 
        String s = doc["ssid"]; String p = doc["password"]; 
        saveWiFiCredentialsToNVS(s.c_str(), p.c_str()); 
        sendCommandAck(clientId, "set_wifi_credentials", "Wi-Fi Saved. Restarting..."); 
        delay(1000); ESP.restart(); 
    } else {
        sendCommandAck(clientId, "set_wifi_credentials", "Error: SSID missing.");
    }
}

void handleRestartESP(uint8_t clientId) {
    lastClientActivityTimestamp = millis();
    sendCommandAck(clientId, "restart_esp", "Restarting ESP...");
    delay(1000); ESP.restart();
}

void handleCameraSettings(uint8_t clientId, JsonDocument& doc) {
    lastClientActivityTimestamp = millis();
    sensor_t *s_before = esp_camera_sensor_get();
    if (!s_before) {
        sendCommandAck(clientId, "set_camera_settings", "Sensor Error (pre-change)");
        return;
    }
    framesize_t original_framesize = s_before->status.framesize;

    bool resolutionChangeAttempted = false;
    bool resolutionChangeSuccess = false;

    if (doc.containsKey("resolution")) {
        framesize_t new_framesize = stringToFramesize(doc["resolution"].as<String>());

        if (original_framesize != new_framesize) {
            resolutionChangeAttempted = true;
            bool wasStreaming = isStreaming;

            if (wasStreaming) {
                isStreaming = false; 
                broadcastStatusUpdate(); 
                vTaskDelay(pdMS_TO_TICKS(300)); 
                 if (streamTaskHandle != NULL) { 
                    // Task should self-delete based on isStreaming flag.
                    // If it doesn't, a manual vTaskDelete could be added here, but is riskier.
                 }
                 streamTaskHandle = NULL; 
            }

            nvs_cam_framesize = new_framesize; 
            initCamera();                      

            sensor_t *s_after = esp_camera_sensor_get();
            if (s_after && s_after->status.framesize == new_framesize) {
                resolutionChangeSuccess = true;
            } else {
                nvs_cam_framesize = original_framesize; // Revert global
                initCamera(); // Re-init with original if change failed
            }

            if (wasStreaming && resolutionChangeSuccess) {
                isStreaming = true; 
                broadcastStatusUpdate(); 
            }
            // If wasStreaming and !resolutionChangeSuccess, stream remains off.
            // broadcastStatusUpdate() with stream_active:false was already sent.
        }
    }

    if (resolutionChangeAttempted) {
        sendCommandAck(clientId, "set_camera_settings", resolutionChangeSuccess ? "Resolution Changed" : "Resolution Apply Error");
    } else {
        sendCommandAck(clientId, "set_camera_settings", "No Change Requested");
    }
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    lastClientActivityTimestamp = millis();
    switch(type) {
        case WStype_DISCONNECTED: break; 
        case WStype_CONNECTED: {
            sendFullStatusUpdate(num, true);
            break;
        }
        case WStype_TEXT: {
            StaticJsonDocument<192> doc;
            if (deserializeJson(doc, payload, length)) return;

            const char* cmd = doc["command"];
            if (!cmd) return;

            if (strcmp(cmd, "control") == 0) processJoystickControlServos(doc["x"]|0.0f, doc["y"]|0.0f);
            else if (strcmp(cmd, "set_led") == 0) handleLedControl(num, doc);
            else if (strcmp(cmd, "set_camera_settings") == 0) handleCameraSettings(num, doc);
            else if (strcmp(cmd, "stream_control") == 0) { 
                const char* act = doc["action"];
                if (act) {
                    if (strcmp(act, "start")==0 && !isBatteryCritical) { 
                        if(!isStreaming){ 
                            isStreaming=true; 
                            broadcastStatusUpdate(); 
                        }
                    }
                    else if (strcmp(act, "stop")==0) { 
                        if(isStreaming){ 
                            isStreaming=false; 
                            broadcastStatusUpdate();
                        }
                    }
                    else if (strcmp(act, "start")==0 && isBatteryCritical) sendCommandAck(num, "stream_control", "Battery Critical!");
                }
            }
            else if (strcmp(cmd, "set_wifi_credentials") == 0) handleSetWiFiCredentials(num, doc);
            else if (strcmp(cmd, "restart_esp") == 0) handleRestartESP(num);
            else if (strcmp(cmd, "ping") == 0) webSocket.sendTXT(num, "{\"type\":\"pong\"}");
            else if (strcmp(cmd, "get_full_status") == 0) sendFullStatusUpdate(num, true); 
            break;
        }
        default: break;
    }
}

void broadcastStatusUpdate() {
    sendFullStatusUpdate(BROADCAST_ALL_CLIENTS, false);
}

void setupWebServerRoutes() { 
    server.on("/", HTTP_GET, []() { if (!checkAuthentication()) return; lastClientActivityTimestamp = millis(); server.sendHeader("Connection", "close"); server.send_P(200, "text/html", htmlContentFlash); });
    server.on("/stream", HTTP_GET, [](){ if (!checkAuthentication()) return; handleStartStreamForHttpRequest(); }); 
    server.onNotFound([](){ if (!checkAuthentication()) return; server.send(404, "text/plain", "Not Found"); }); 
    server.begin();
}

const char* framesizeToString(framesize_t fs) { 
    if (fs == FRAMESIZE_QVGA) return "QVGA"; 
    if (fs == FRAMESIZE_SVGA) return "SVGA"; 
    if (fs == FRAMESIZE_XGA) return "XGA";   
    return "UNKNOWN"; 
}
framesize_t stringToFramesize(const String& fsStr) { 
    if (fsStr == "QVGA") return FRAMESIZE_QVGA; 
    if (fsStr == "SVGA") return FRAMESIZE_SVGA; 
    if (fsStr == "XGA") return FRAMESIZE_XGA;   
    return FRAMESIZE_QVGA; // Default fallback if string is unknown
}

void TaskHttpServer(void *pvParameters) { 
    (void)pvParameters; 
    unsigned long lastStatusPush = 0;
    for (;;) { 
        if (!otaIsActive && !isLightSleeping) { 
            server.handleClient(); webSocket.loop(); 
            if (millis() - lastStatusPush > statusBroadcastInterval) {
                if(webSocket.connectedClients() > 0 && !isBatteryCritical) broadcastStatusUpdate();
                lastStatusPush = millis();
            }
        } else if (isLightSleeping) vTaskDelay(pdMS_TO_TICKS(100));
        else vTaskDelay(pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(2)); 
    } 
}
