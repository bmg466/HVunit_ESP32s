#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// HVunit_ESP32s
// NodeMCU ESP-32S v1.1 / ESP-WROOM-32
// LT3482 SiPM/APD HV controller
//
// Hardware:
//   GPIO25 / DAC1 -> +INA -> AD8606 -> 10k/10k -> CTRL LT3482
//   GPIO27        -> SHDN LT3482
//   GPIO34 / ADC1 <- V_SENSE buffer, HV divider 1M / 33k
//   GPIO35 / ADC1 <- I_SENSE buffer, RMON = 6.2k, no ADC divider
//   GPIO2         -> onboard LED
//   HV module supply: regulated 3.3 V from ESP32 board
//
// NOTE: With RMON=6.2k, MON node reaches 3.3 V at about 2.66 mA
// APD/SiPM current. Do not exceed the analog front-end input limits.
// ============================================================

// ---------------- Wi-Fi ----------------
const char* AP_SSID     = "SiPM-HV";
const char* AP_PASSWORD = "sipm1234";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

WebServer server(80);

// ---------------- Pins ----------------
const uint8_t CTRL_DAC_PIN = 25;
const uint8_t SHDN_PIN     = 27;
const uint8_t VSENSE_PIN   = 34;
const uint8_t ISENSE_PIN   = 35;
const uint8_t HV_LED_PIN   = 2;

// Change to false if the onboard LED works inverted.
const bool LED_ACTIVE_HIGH = true;

// ---------------- HV range ----------------
const float HV_MIN = 0.0f;
const float HV_MAX = 80.0f;

// ---------------- Power ----------------
// Informational value only. The control algorithm does not use it.
const float HV_MODULE_SUPPLY_V = 3.30f;

// ---------------- DAC / CTRL ----------------
// ESP32 DAC is 8-bit. Full-scale voltage is only a nominal starting
// value and will be replaced/compensated after calibration.
const float DAC_FULL_SCALE_V = 3.30f;
const float CTRL_DIVIDER     = 0.5f;   // 10k / 10k

// LT3482 FB divider: 1M / 14k
const float FB_R_TOP    = 1000000.0f;
const float FB_R_BOTTOM =   14000.0f;

// Calibration placeholder.
const float DAC_CALIBRATION = 1.0000f;

// ---------------- V_SENSE ----------------
// HV -> 1M -> sense node -> 33k -> GND -> buffer -> GPIO34
const float VSENSE_R_TOP    = 1000000.0f;
const float VSENSE_R_BOTTOM =   33000.0f;
const float HV_CALIBRATION  = 1.0000f;

// ---------------- I_SENSE ----------------
// LT3482 IMON = 0.20 * I_APD
// MON -> 6.2k -> GND, buffer -> GPIO35
// No additional divider between I_SENSE buffer and GPIO35.
const float R_MON               = 6200.0f;
const float MON_CURRENT_RATIO   = 0.20f;
const float ISENSE_ADC_DIVIDER  = 1.0f;
const float CURRENT_CALIBRATION = 1.0000f;

// ---------------- ADC filtering ----------------
// Stage 1: average multiple conversions.
// Stage 2: IIR low-pass filter for displayed voltage/current.
const uint8_t ADC_SAMPLES = 32;
const uint16_t ADC_SAMPLE_DELAY_US = 150;
const unsigned long ADC_INTERVAL_MS = 100;
const float ADC_FILTER_ALPHA = 0.10f;

// ---------------- DAC ramp ----------------
const unsigned long DAC_RAMP_INTERVAL_MS = 10;

// ---------------- Runtime ----------------
float setVoltage      = 0.0f;
float measuredVoltage = 0.0f;
float measuredCurrent = 0.0f; // mA

uint16_t vSenseRaw        = 0;
uint16_t iSenseRaw        = 0;
uint16_t vSenseMilliVolts = 0;
uint16_t iSenseMilliVolts = 0;

uint8_t currentDacCode = 0;
uint8_t targetDacCode  = 0;

bool hvEnabled = false;
bool firstAdcMeasurement = true;

unsigned long lastAdcTime  = 0;
unsigned long lastRampTime = 0;

struct AdcSample {
  uint16_t raw;
  uint16_t millivolts;
};

// ============================================================
// Helpers
// ============================================================

void setHVLed(bool state) {
  if (LED_ACTIVE_HIGH) {
    digitalWrite(HV_LED_PIN, state ? HIGH : LOW);
  } else {
    digitalWrite(HV_LED_PIN, state ? LOW : HIGH);
  }
}

AdcSample readAdcAverage(uint8_t pin) {
  uint32_t rawSum = 0;
  uint32_t mvSum  = 0;

  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    rawSum += analogRead(pin);
    mvSum  += analogReadMilliVolts(pin);
    delayMicroseconds(ADC_SAMPLE_DELAY_US);
  }

  AdcSample result;
  result.raw = (uint16_t)roundf((float)rawSum / ADC_SAMPLES);
  result.millivolts = (uint16_t)roundf((float)mvSum / ADC_SAMPLES);
  return result;
}

uint8_t voltageToDac(float voltage) {
  voltage = constrain(voltage, HV_MIN, HV_MAX);

  const float feedbackGain = 1.0f + FB_R_TOP / FB_R_BOTTOM;
  float requiredCtrl = voltage / feedbackGain;
  float requiredDacVoltage = requiredCtrl / CTRL_DIVIDER;
  requiredDacVoltage *= DAC_CALIBRATION;

  float code = requiredDacVoltage / DAC_FULL_SCALE_V * 255.0f;
  code = constrain(code, 0.0f, 255.0f);
  return (uint8_t)roundf(code);
}

void updateDacRamp() {
  if (!hvEnabled) return;

  unsigned long now = millis();
  if (now - lastRampTime < DAC_RAMP_INTERVAL_MS) return;
  lastRampTime = now;

  if (currentDacCode < targetDacCode) {
    ++currentDacCode;
    dacWrite(CTRL_DAC_PIN, currentDacCode);
  } else if (currentDacCode > targetDacCode) {
    --currentDacCode;
    dacWrite(CTRL_DAC_PIN, currentDacCode);
  }
}

void turnHVOn() {
  // Safe ramp from zero every time HV is enabled.
  currentDacCode = 0;
  dacWrite(CTRL_DAC_PIN, 0);
  targetDacCode = voltageToDac(setVoltage);

  digitalWrite(SHDN_PIN, HIGH);
  hvEnabled = true;
  setHVLed(true);

  Serial.printf("HV ON | SET %.2f V | DAC target %u\n", setVoltage, targetDacCode);
}

void turnHVOff() {
  // Hardware shutdown first, then remove CTRL.
  digitalWrite(SHDN_PIN, LOW);
  dacWrite(CTRL_DAC_PIN, 0);

  currentDacCode = 0;
  targetDacCode  = 0;
  hvEnabled = false;
  setHVLed(false);

  Serial.println("HV OFF");
}

void applySetVoltage() {
  targetDacCode = voltageToDac(setVoltage);
  Serial.printf("SET %.2f V | DAC target %u\n", setVoltage, targetDacCode);
}

void updateMeasurements() {
  unsigned long now = millis();
  if (now - lastAdcTime < ADC_INTERVAL_MS) return;
  lastAdcTime = now;

  // -------- Voltage --------
  AdcSample voltageSample = readAdcAverage(VSENSE_PIN);
  vSenseRaw = voltageSample.raw;
  vSenseMilliVolts = voltageSample.millivolts;

  float vSense = (float)vSenseMilliVolts / 1000.0f;
  const float voltageSenseGain =
      (VSENSE_R_TOP + VSENSE_R_BOTTOM) / VSENSE_R_BOTTOM;

  float newMeasuredVoltage = vSense * voltageSenseGain * HV_CALIBRATION;
  newMeasuredVoltage = constrain(newMeasuredVoltage, 0.0f, 100.0f);

  // -------- Current --------
  AdcSample currentSample = readAdcAverage(ISENSE_PIN);
  iSenseRaw = currentSample.raw;
  iSenseMilliVolts = currentSample.millivolts;

  float iSense = (float)iSenseMilliVolts / 1000.0f;
  float monVoltage = iSense / ISENSE_ADC_DIVIDER;
  float iApdAmpere = monVoltage / R_MON / MON_CURRENT_RATIO;
  float newMeasuredCurrent =
      iApdAmpere * 1000.0f * CURRENT_CALIBRATION;

  if (newMeasuredCurrent < 0.0f) newMeasuredCurrent = 0.0f;

  // -------- IIR filtering --------
  if (firstAdcMeasurement) {
    measuredVoltage = newMeasuredVoltage;
    measuredCurrent = newMeasuredCurrent;
    firstAdcMeasurement = false;
  } else {
    measuredVoltage += ADC_FILTER_ALPHA *
                       (newMeasuredVoltage - measuredVoltage);
    measuredCurrent += ADC_FILTER_ALPHA *
                       (newMeasuredCurrent - measuredCurrent);
  }

  if (measuredVoltage < 0.05f) measuredVoltage = 0.0f;
  if (measuredCurrent < 0.002f) measuredCurrent = 0.0f;
}

// ============================================================
// Web page
// ============================================================

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<title>SiPM HV</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;width:100%;min-height:100%;background:#111315;color:white;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif}
body{display:flex;justify-content:center}
.container{width:100%;max-width:520px;min-height:100vh;padding:max(22px,env(safe-area-inset-top)) 24px max(18px,env(safe-area-inset-bottom)) 24px;display:flex;flex-direction:column;align-items:center}
.title{margin:6px 0 28px;font-size:18px;font-weight:500;letter-spacing:2px;color:#c8c8c8}
.circle{width:min(76vw,330px);aspect-ratio:1/1;border-radius:50%;border:8px solid #55585b;display:flex;flex-direction:column;align-items:center;justify-content:center;transition:border-color .3s ease,box-shadow .3s ease}
.circle.on{border-color:#28d365;box-shadow:0 0 24px rgba(40,211,101,.25),inset 0 0 22px rgba(40,211,101,.06)}
.measureLabel{margin-bottom:8px;color:#777;font-size:11px;letter-spacing:3px}
.voltageRow{display:flex;justify-content:center;align-items:baseline}
.voltage{font-size:clamp(53px,16vw,82px);font-weight:300;line-height:1}
.unit{margin-left:7px;font-size:24px;font-weight:300;color:#999}
.status{margin-top:18px;font-size:14px;letter-spacing:4px;color:#777}
.circle.on .status{color:#28d365}
.currentBlock{margin-top:24px;text-align:center}
.currentValue{font-size:25px;font-weight:300}
.currentUnit{color:#888;font-size:15px}
.currentLabel{margin-top:5px;color:#666;font-size:10px;letter-spacing:2px}
.controlBlock{width:100%;margin-top:30px}
.label{margin-bottom:8px;color:#777;font-size:12px;letter-spacing:1.5px}
.inputWrap{position:relative;width:100%}
input{width:100%;height:60px;padding:0 55px 0 20px;border:1px solid #43474b;border-radius:12px;outline:none;background:#1c1f22;color:white;text-align:center;font-size:28px;appearance:textfield}
input:focus{border-color:#777c80;background:#202428}
.inputUnit{position:absolute;right:20px;top:50%;transform:translateY(-50%);color:#777;font-size:17px;pointer-events:none}
.setInfo{height:23px;margin-top:10px;color:#626568;text-align:center;font-size:13px}
button{width:100%;height:66px;margin-top:18px;border:none;border-radius:13px;background:#34383c;color:white;font-size:18px;font-weight:600;letter-spacing:2px;cursor:pointer;transition:background .25s,color .25s,transform .08s}
button:active{transform:scale(.985)}
button.on{background:#28d365;color:#071d0e}
.diagnostics{width:100%;margin-top:auto;padding-top:30px;text-align:center;color:#4f5356;font-size:10px;line-height:1.7;letter-spacing:.5px}
.diagTitle{margin-bottom:3px;color:#45484a;font-size:9px;letter-spacing:1.2px}
</style>
</head>
<body>
<div class="container">
  <div class="title">SiPM HIGH VOLTAGE</div>

  <div id="circle" class="circle">
    <div class="measureLabel">MEASURED</div>
    <div class="voltageRow">
      <span id="voltageDisplay" class="voltage">0.0</span>
      <span class="unit">V</span>
    </div>
    <div id="status" class="status">OFF</div>
  </div>

  <div class="currentBlock">
    <span id="currentDisplay" class="currentValue">0.000</span>
    <span class="currentUnit">mA</span>
    <div class="currentLabel">HV CURRENT</div>
  </div>

  <div class="controlBlock">
    <div class="label">SET VOLTAGE</div>
    <div class="inputWrap">
      <input id="voltageInput" type="number" min="0" max="80" step="0.1" value="0.0" inputmode="decimal">
      <div class="inputUnit">V</div>
    </div>
    <div id="setInfo" class="setInfo">SET: 0.0 V</div>
  </div>

  <button id="powerButton" onclick="togglePower()">TURN ON</button>

  <div class="diagnostics">
    <div class="diagTitle">CALIBRATION DATA</div>
    <div>DAC: <span id="dacCurrent">0</span> &nbsp; TARGET: <span id="dacTarget">0</span></div>
    <div>ADC V: <span id="adcV">0</span> &nbsp;&nbsp; ADC I: <span id="adcI">0</span></div>
    <div>VADC: <span id="vadcMv">0</span> mV &nbsp;&nbsp; IADC: <span id="iadcMv">0</span> mV</div>
    <div>HV MODULE: 3.3 V &nbsp; ESP32 · LT3482</div>
  </div>
</div>

<script>
let hvOn=false;
let firstStateReceived=false;

function updateScreen(data){
  hvOn=data.enabled;
  document.getElementById("voltageDisplay").innerText=Number(data.measuredVoltage).toFixed(1);
  document.getElementById("currentDisplay").innerText=Number(data.measuredCurrent).toFixed(3);
  document.getElementById("setInfo").innerText="SET: "+Number(data.setVoltage).toFixed(1)+" V";

  const input=document.getElementById("voltageInput");
  if(!firstStateReceived || document.activeElement!==input){
    input.value=Number(data.setVoltage).toFixed(1);
  }
  firstStateReceived=true;

  document.getElementById("dacCurrent").innerText=data.dacCurrent;
  document.getElementById("dacTarget").innerText=data.dacTarget;
  document.getElementById("adcV").innerText=data.vSenseRaw;
  document.getElementById("adcI").innerText=data.iSenseRaw;
  document.getElementById("vadcMv").innerText=data.vSenseMV;
  document.getElementById("iadcMv").innerText=data.iSenseMV;

  const circle=document.getElementById("circle");
  const status=document.getElementById("status");
  const button=document.getElementById("powerButton");

  if(hvOn){
    circle.classList.add("on");
    button.classList.add("on");
    status.innerText=data.ramping ? "RAMP" : "ON";
    button.innerText="TURN OFF";
  }else{
    circle.classList.remove("on");
    button.classList.remove("on");
    status.innerText="OFF";
    button.innerText="TURN ON";
  }
}

async function refreshState(){
  try{
    const response=await fetch("/api/state",{cache:"no-store"});
    if(!response.ok)return;
    updateScreen(await response.json());
  }catch(error){console.log("State error:",error)}
}

async function sendSetVoltage(){
  const input=document.getElementById("voltageInput");
  let voltage=parseFloat(input.value);
  if(isNaN(voltage)){await refreshState();return;}
  voltage=Math.max(0,Math.min(80,voltage));
  input.value=voltage.toFixed(1);
  try{
    await fetch("/api/set?voltage="+encodeURIComponent(voltage),{cache:"no-store"});
    await refreshState();
  }catch(error){console.log("SET error:",error)}
}

async function togglePower(){
  try{
    await fetch("/api/power?state="+(hvOn?0:1),{cache:"no-store"});
    await refreshState();
  }catch(error){console.log("Power error:",error)}
}

const voltageInput=document.getElementById("voltageInput");
voltageInput.addEventListener("change",sendSetVoltage);
voltageInput.addEventListener("keydown",function(event){if(event.key==="Enter")voltageInput.blur();});

refreshState();
setInterval(refreshState,500);
</script>
</body>
</html>
)rawliteral";

// ============================================================
// HTTP handlers
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleState() {
  String json;
  json.reserve(320);

  json += "{";
  json += "\"setVoltage\":" + String(setVoltage, 2);
  json += ",\"measuredVoltage\":" + String(measuredVoltage, 3);
  json += ",\"measuredCurrent\":" + String(measuredCurrent, 5);
  json += ",\"vSenseRaw\":" + String(vSenseRaw);
  json += ",\"iSenseRaw\":" + String(iSenseRaw);
  json += ",\"vSenseMV\":" + String(vSenseMilliVolts);
  json += ",\"iSenseMV\":" + String(iSenseMilliVolts);
  json += ",\"dacCurrent\":" + String(currentDacCode);
  json += ",\"dacTarget\":" + String(targetDacCode);
  json += ",\"ramping\":";
  json += (hvEnabled && currentDacCode != targetDacCode) ? "true" : "false";
  json += ",\"enabled\":";
  json += hvEnabled ? "true" : "false";
  json += "}";

  server.send(200, "application/json", json);
}

void handleSetVoltage() {
  if (!server.hasArg("voltage")) {
    server.send(400, "text/plain", "Missing voltage");
    return;
  }

  setVoltage = constrain(server.arg("voltage").toFloat(), HV_MIN, HV_MAX);
  applySetVoltage();
  server.send(200, "text/plain", "OK");
}

void handlePower() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Missing state");
    return;
  }

  bool newState = server.arg("state") == "1";
  if (newState && !hvEnabled) turnHVOn();
  if (!newState && hvEnabled) turnHVOff();

  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("================================");
  Serial.println(" HVunit ESP32 / LT3482");
  Serial.println(" HV module supply: 3.3 V");
  Serial.println("================================");

  pinMode(SHDN_PIN, OUTPUT);
  pinMode(HV_LED_PIN, OUTPUT);
  pinMode(VSENSE_PIN, INPUT);
  pinMode(ISENSE_PIN, INPUT);

  // Safe startup: converter disabled and CTRL = 0.
  digitalWrite(SHDN_PIN, LOW);
  setHVLed(false);
  dacWrite(CTRL_DAC_PIN, 0);
  hvEnabled = false;
  setVoltage = 0.0f;
  currentDacCode = 0;
  targetDacCode = 0;

  analogReadResolution(12);

  // This spelling matches the Arduino-ESP32 core currently used on
  // the development PC. GPIO34/35 are ADC1 pins and remain usable
  // while Wi-Fi is active.
  analogSetPinAttenuation(VSENSE_PIN, ADC_11db);
  analogSetPinAttenuation(ISENSE_PIN, ADC_11db);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Wi-Fi AP started");
  } else {
    Serial.println("Wi-Fi AP ERROR");
  }

  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP:   ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/set", HTTP_GET, handleSetVoltage);
  server.on("/api/power", HTTP_GET, handlePower);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("HTTP server started");
  Serial.println("Open http://192.168.4.1");
}

void loop() {
  server.handleClient();
  updateMeasurements();
  updateDacRamp();
  delay(1);
}
