#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// HVunit_ESP32s
// NodeMCU ESP-32S v1.1 / ESP-WROOM-32 + LT3482 HV module
//
// GPIO25 / DAC1 -> +INA -> AD8606 -> 10k/10k -> CTRL
// GPIO27        -> SHDN
// GPIO34 / ADC1 <- V_SENSE, divider 1M / 33k + buffer
// GPIO35 / ADC1 <- I_SENSE, MON resistor 6.2k + buffer
// GPIO2         -> onboard LED
// HV module supply: ESP32 3.3 V rail
//
// Calibration data:
//   HV SET / HV monitor: bench calibration
//   Current: Keithley 6485 + 47.22 kOhm load, 2026-08-14
//
// v2 adds a slow closed-loop trim around the calibrated DAC LUT.
// The LUT gives the initial DAC code. After the ramp, V_SENSE slowly
// trims the DAC by +/-1 code until measured HV is close to SET.
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
const bool LED_ACTIVE_HIGH = true;

// ---------------- Operating range ----------------
const float HV_MIN = 0.0f;
const float HV_MAX = 80.0f;
const float HV_MODULE_SUPPLY_V = 3.30f; // informational

// ============================================================
// HV SET calibration: real HV -> DAC code
// No-load measurements used as feed-forward LUT.
// ============================================================

const uint8_t HV_DAC_CAL_POINTS = 9;
const float HV_DAC_CAL_VOLTAGE[HV_DAC_CAL_POINTS] = {
   0.10f, 11.48f, 21.30f, 31.25f, 40.81f,
  50.40f, 59.40f, 68.90f, 78.50f
};
const uint8_t HV_DAC_CAL_CODE[HV_DAC_CAL_POINTS] = {
    0, 21, 43, 64, 85, 107, 128, 149, 171
};

// Loaded test, 47.22 kOhm, after first HV calibration:
// SET  DAC  GPIO25[mV] CTRL[mV] real HV[V]
//  10   18      329       163       9.75
//  20   40      587       291      19.38
//  30   61      840       417      28.25
//  40   83     1115       552      38.78
//  50  106     1384       685      48.50
//  60  129     1638       813      58.00   // corrected GPIO25 point
//  70  152     1910       944      67.50
//  80  174     2152      1062      76.30
//
// The load dependence above is handled by closed-loop V_SENSE trim,
// rather than baking one particular load into the feed-forward LUT.

// ============================================================
// HV measurement calibration
// ESP32 analogReadMilliVolts(GPIO34) -> real HV.
// ============================================================

const float HV_ADC_GAIN_V_PER_MV = 0.03044587f;
const float HV_ADC_OFFSET_V      = 0.701281f;
const uint16_t VSENSE_ZERO_RAW_MAX = 10;

// V_SENSE divider current is included in LT3482 MON.
const float VSENSE_TOTAL_KOHM = 1033.0f;

// ============================================================
// Current calibration
// ADC raw -> physically measured I_SENSE pin voltage.
// ============================================================

const float R_MON             = 6200.0f;
const float MON_CURRENT_RATIO = 0.20f;
const uint16_t ISENSE_ZERO_RAW_MAX = 10;

const uint8_t I_ADC_CAL_POINTS = 9;
const uint16_t I_ADC_CAL_RAW[I_ADC_CAL_POINTS] = {
     0, 141, 458, 773, 1120, 1459, 1790, 2133, 2443
};
const float I_ADC_CAL_SENSE_MV[I_ADC_CAL_POINTS] = {
     0.0f, 266.0f, 531.0f, 787.0f, 1064.0f,
  1340.0f, 1604.0f, 1879.0f, 2124.0f
};

// ============================================================
// ADC filtering
// ============================================================

const uint8_t ADC_SAMPLES = 32;
const uint16_t ADC_SAMPLE_DELAY_US = 150;
const unsigned long ADC_INTERVAL_MS = 100;
const float ADC_FILTER_ALPHA = 0.10f;

// ============================================================
// DAC ramp + closed-loop regulation
// ============================================================

const unsigned long DAC_RAMP_INTERVAL_MS = 10;

const bool HV_CLOSED_LOOP_ENABLED = true;

// One DAC step is roughly 0.4...0.5 V HV, so a deadband smaller than
// one step prevents unnecessary hunting while still holding HV tightly.
const float HV_FEEDBACK_DEADBAND_V = 0.30f;

// Feedback is intentionally slow compared with the ADC/IIR filter.
const unsigned long HV_FEEDBACK_INTERVAL_MS = 750;
const unsigned long HV_FEEDBACK_SETTLE_MS   = 2000;

// Allow the loop to compensate load/supply droop, but prevent it from
// wandering too far from the calibrated feed-forward LUT.
const int HV_FEEDBACK_MAX_TRIM_CODES = 16;

// Hard software ceiling for DAC output. 80 V under the 47.22k load
// is expected to need around 182 codes, so 185 leaves modest headroom.
const uint8_t HV_DAC_HARD_MAX_CODE = 185;

// Below this voltage the attenuated ESP32 ADC is close to its low-end
// non-linear region. Working LT3482 range is normally >=10 V anyway.
const float HV_FEEDBACK_MIN_SET_V = 8.0f;

// ============================================================
// Runtime
// ============================================================

float setVoltage      = 0.0f;
float measuredVoltage = 0.0f;
float measuredCurrent = 0.0f; // external load / SiPM current, mA

float totalApdCurrent    = 0.0f; // includes V_SENSE divider, mA
float dividerCurrent     = 0.0f; // V_SENSE divider current, mA
float calibratedISenseMv = 0.0f;

uint16_t vSenseRaw        = 0;
uint16_t iSenseRaw        = 0;
uint16_t vSenseMilliVolts = 0;
uint16_t iSenseMilliVolts = 0;

uint8_t feedForwardDacCode = 0;
uint8_t currentDacCode     = 0;
uint8_t targetDacCode      = 0;
int feedbackTrimCodes      = 0;

bool hvEnabled = false;
bool firstAdcMeasurement = true;

unsigned long lastAdcTime      = 0;
unsigned long lastRampTime     = 0;
unsigned long lastFeedbackTime = 0;
unsigned long feedbackHoldUntil = 0;

struct AdcSample {
  uint16_t raw;
  uint16_t millivolts;
};

// ============================================================
// Helpers
// ============================================================

void setHVLed(bool state)
{
  if (LED_ACTIVE_HIGH) {
    digitalWrite(HV_LED_PIN, state ? HIGH : LOW);
  } else {
    digitalWrite(HV_LED_PIN, state ? LOW : HIGH);
  }
}

AdcSample readAdcAverage(uint8_t pin)
{
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

float currentRawToSenseMv(uint16_t raw)
{
  if (raw <= ISENSE_ZERO_RAW_MAX) return 0.0f;

  for (uint8_t i = 1; i < I_ADC_CAL_POINTS; i++) {
    if (raw <= I_ADC_CAL_RAW[i]) {
      float x0 = (float)I_ADC_CAL_RAW[i - 1];
      float x1 = (float)I_ADC_CAL_RAW[i];
      float y0 = I_ADC_CAL_SENSE_MV[i - 1];
      float y1 = I_ADC_CAL_SENSE_MV[i];
      float f = ((float)raw - x0) / (x1 - x0);
      return y0 + f * (y1 - y0);
    }
  }

  const uint8_t i1 = I_ADC_CAL_POINTS - 1;
  const uint8_t i0 = I_ADC_CAL_POINTS - 2;
  float x0 = (float)I_ADC_CAL_RAW[i0];
  float x1 = (float)I_ADC_CAL_RAW[i1];
  float y0 = I_ADC_CAL_SENSE_MV[i0];
  float y1 = I_ADC_CAL_SENSE_MV[i1];
  float mv = y1 + ((float)raw - x1) * (y1 - y0) / (x1 - x0);
  return constrain(mv, 0.0f, 3300.0f);
}

uint8_t voltageToDac(float voltage)
{
  voltage = constrain(voltage, HV_MIN, HV_MAX);
  if (voltage <= 0.0f) return 0;

  if (voltage <= HV_DAC_CAL_VOLTAGE[0]) {
    return HV_DAC_CAL_CODE[0];
  }

  for (uint8_t i = 1; i < HV_DAC_CAL_POINTS; i++) {
    if (voltage <= HV_DAC_CAL_VOLTAGE[i]) {
      float v0 = HV_DAC_CAL_VOLTAGE[i - 1];
      float v1 = HV_DAC_CAL_VOLTAGE[i];
      float c0 = (float)HV_DAC_CAL_CODE[i - 1];
      float c1 = (float)HV_DAC_CAL_CODE[i];
      float f = (voltage - v0) / (v1 - v0);
      return (uint8_t)constrain((int)roundf(c0 + f * (c1 - c0)), 0, 255);
    }
  }

  const uint8_t i1 = HV_DAC_CAL_POINTS - 1;
  const uint8_t i0 = HV_DAC_CAL_POINTS - 2;
  float v0 = HV_DAC_CAL_VOLTAGE[i0];
  float v1 = HV_DAC_CAL_VOLTAGE[i1];
  float c0 = (float)HV_DAC_CAL_CODE[i0];
  float c1 = (float)HV_DAC_CAL_CODE[i1];
  float code = c1 + (voltage - v1) * (c1 - c0) / (v1 - v0);
  return (uint8_t)constrain((int)roundf(code), 0, 255);
}

void resetFeedbackTarget()
{
  feedForwardDacCode = voltageToDac(setVoltage);
  targetDacCode = min(feedForwardDacCode, HV_DAC_HARD_MAX_CODE);
  feedbackTrimCodes = 0;
  feedbackHoldUntil = millis() + HV_FEEDBACK_SETTLE_MS;
  lastFeedbackTime = 0;
}

// ============================================================
// DAC ramp and closed-loop trim
// ============================================================

void updateDacRamp()
{
  if (!hvEnabled) return;

  unsigned long now = millis();
  if (now - lastRampTime < DAC_RAMP_INTERVAL_MS) return;
  lastRampTime = now;

  if (currentDacCode < targetDacCode) {
    currentDacCode++;
    dacWrite(CTRL_DAC_PIN, currentDacCode);
  }
  else if (currentDacCode > targetDacCode) {
    currentDacCode--;
    dacWrite(CTRL_DAC_PIN, currentDacCode);
  }
}

void updateHvFeedback()
{
  if (!HV_CLOSED_LOOP_ENABLED || !hvEnabled) return;
  if (setVoltage < HV_FEEDBACK_MIN_SET_V) return;

  // Do not adjust while the DAC is still moving to the present target.
  if (currentDacCode != targetDacCode) return;

  unsigned long now = millis();
  if ((long)(now - feedbackHoldUntil) < 0) return;
  if (now - lastFeedbackTime < HV_FEEDBACK_INTERVAL_MS) return;
  lastFeedbackTime = now;

  float error = setVoltage - measuredVoltage;

  int minCode = max(0, (int)feedForwardDacCode - HV_FEEDBACK_MAX_TRIM_CODES);
  int maxCode = min((int)HV_DAC_HARD_MAX_CODE,
                    (int)feedForwardDacCode + HV_FEEDBACK_MAX_TRIM_CODES);

  if (error > HV_FEEDBACK_DEADBAND_V && (int)targetDacCode < maxCode) {
    targetDacCode++;
  }
  else if (error < -HV_FEEDBACK_DEADBAND_V && (int)targetDacCode > minCode) {
    targetDacCode--;
  }

  feedbackTrimCodes = (int)targetDacCode - (int)feedForwardDacCode;
}

// ============================================================
// HV ON / OFF
// ============================================================

void turnHVOn()
{
  currentDacCode = 0;
  dacWrite(CTRL_DAC_PIN, 0);

  resetFeedbackTarget();

  digitalWrite(SHDN_PIN, HIGH);
  hvEnabled = true;
  setHVLed(true);

  Serial.printf("HV ON | SET %.2f V | FF DAC %u\n",
                setVoltage, feedForwardDacCode);
}

void turnHVOff()
{
  digitalWrite(SHDN_PIN, LOW);
  dacWrite(CTRL_DAC_PIN, 0);

  currentDacCode = 0;
  targetDacCode = 0;
  feedForwardDacCode = 0;
  feedbackTrimCodes = 0;

  hvEnabled = false;
  setHVLed(false);
  Serial.println("HV OFF");
}

void applySetVoltage()
{
  resetFeedbackTarget();
  Serial.printf("SET %.2f V | FF DAC %u\n",
                setVoltage, feedForwardDacCode);
}

// ============================================================
// Measurements
// ============================================================

void updateMeasurements()
{
  unsigned long now = millis();
  if (now - lastAdcTime < ADC_INTERVAL_MS) return;
  lastAdcTime = now;

  // V_SENSE
  AdcSample voltageSample = readAdcAverage(VSENSE_PIN);
  vSenseRaw = voltageSample.raw;
  vSenseMilliVolts = voltageSample.millivolts;

  float newMeasuredVoltage = 0.0f;
  if (vSenseRaw > VSENSE_ZERO_RAW_MAX) {
    newMeasuredVoltage =
        HV_ADC_GAIN_V_PER_MV * (float)vSenseMilliVolts + HV_ADC_OFFSET_V;
    newMeasuredVoltage = constrain(newMeasuredVoltage, 0.0f, 100.0f);
  }

  // I_SENSE
  AdcSample currentSample = readAdcAverage(ISENSE_PIN);
  iSenseRaw = currentSample.raw;
  iSenseMilliVolts = currentSample.millivolts;

  float newMeasuredCurrent = 0.0f;
  float newTotalApdCurrent = 0.0f;
  float newDividerCurrent  = 0.0f;

  calibratedISenseMv = currentRawToSenseMv(iSenseRaw);

  if (iSenseRaw > ISENSE_ZERO_RAW_MAX) {
    // mV / (ohm * ratio) -> mA
    newTotalApdCurrent = calibratedISenseMv / (R_MON * MON_CURRENT_RATIO);

    // V / kOhm -> mA
    newDividerCurrent = newMeasuredVoltage / VSENSE_TOTAL_KOHM;

    newMeasuredCurrent = newTotalApdCurrent - newDividerCurrent;
    if (newMeasuredCurrent < 0.0f) newMeasuredCurrent = 0.0f;
  }

  totalApdCurrent = newTotalApdCurrent;
  dividerCurrent  = newDividerCurrent;

  // Display/control low-pass filter.
  if (firstAdcMeasurement) {
    measuredVoltage = newMeasuredVoltage;
    measuredCurrent = newMeasuredCurrent;
    firstAdcMeasurement = false;
  }
  else {
    measuredVoltage += ADC_FILTER_ALPHA * (newMeasuredVoltage - measuredVoltage);
    measuredCurrent += ADC_FILTER_ALPHA * (newMeasuredCurrent - measuredCurrent);
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
.currentBlock{margin-top:24px;text-align:center}.currentValue{font-size:25px;font-weight:300}.currentUnit{color:#888;font-size:15px}.currentLabel{margin-top:5px;color:#666;font-size:10px;letter-spacing:2px}
.controlBlock{width:100%;margin-top:30px}.label{margin-bottom:8px;color:#777;font-size:12px;letter-spacing:1.5px}.inputWrap{position:relative;width:100%}
input{width:100%;height:60px;padding:0 55px 0 20px;border:1px solid #43474b;border-radius:12px;outline:none;background:#1c1f22;color:white;text-align:center;font-size:28px;appearance:textfield}
input:focus{border-color:#777c80;background:#202428}.inputUnit{position:absolute;right:20px;top:50%;transform:translateY(-50%);color:#777;font-size:17px;pointer-events:none}.setInfo{height:23px;margin-top:10px;color:#626568;text-align:center;font-size:13px}
button{width:100%;height:66px;margin-top:18px;border:none;border-radius:13px;background:#34383c;color:white;font-size:18px;font-weight:600;letter-spacing:2px;cursor:pointer;transition:background .25s,color .25s,transform .08s}
button:active{transform:scale(.985)}button.on{background:#28d365;color:#071d0e}
.diagnostics{width:100%;margin-top:auto;padding-top:30px;text-align:center;color:#4f5356;font-size:10px;line-height:1.7;letter-spacing:.5px}.diagTitle{margin-bottom:3px;color:#45484a;font-size:9px;letter-spacing:1.2px}
</style>
</head>
<body>
<div class="container">
  <div class="title">SiPM HIGH VOLTAGE</div>

  <div id="circle" class="circle">
    <div class="measureLabel">MEASURED</div>
    <div class="voltageRow"><span id="voltageDisplay" class="voltage">0.0</span><span class="unit">V</span></div>
    <div id="status" class="status">OFF</div>
  </div>

  <div class="currentBlock">
    <span id="currentDisplay" class="currentValue">0.000</span><span class="currentUnit"> mA</span>
    <div class="currentLabel">LOAD CURRENT</div>
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
    <div class="diagTitle">CALIBRATION DATA · HV/I V1 · CLOSED LOOP V2</div>
    <div>DAC: <span id="dacCurrent">0</span> &nbsp; FF: <span id="dacFF">0</span> &nbsp; TARGET: <span id="dacTarget">0</span> &nbsp; TRIM: <span id="dacTrim">0</span></div>
    <div>ADC V: <span id="adcV">0</span> &nbsp;&nbsp; ADC I: <span id="adcI">0</span></div>
    <div>VADC: <span id="vadcMv">0</span> mV &nbsp;&nbsp; IADC: <span id="iadcMv">0</span> mV</div>
    <div>I_SENSE CAL: <span id="isenseCalMv">0</span> mV</div>
    <div>I TOTAL: <span id="iTotal">0</span> mA &nbsp;&nbsp; I DIV: <span id="iDiv">0</span> mA</div>
    <div>HV MODULE: 3.3 V &nbsp; ESP32 · LT3482</div>
  </div>
</div>

<script>
let hvOn=false, firstStateReceived=false;

function updateScreen(data){
  hvOn=data.enabled;
  document.getElementById("voltageDisplay").innerText=Number(data.measuredVoltage).toFixed(1);
  document.getElementById("currentDisplay").innerText=Number(data.measuredCurrent).toFixed(3);
  document.getElementById("setInfo").innerText="SET: "+Number(data.setVoltage).toFixed(1)+" V";

  const input=document.getElementById("voltageInput");
  if(!firstStateReceived || document.activeElement!==input) input.value=Number(data.setVoltage).toFixed(1);
  firstStateReceived=true;

  document.getElementById("dacCurrent").innerText=data.dacCurrent;
  document.getElementById("dacFF").innerText=data.dacFeedForward;
  document.getElementById("dacTarget").innerText=data.dacTarget;
  document.getElementById("dacTrim").innerText=(data.dacTrim>=0?"+":"")+data.dacTrim;
  document.getElementById("adcV").innerText=data.vSenseRaw;
  document.getElementById("adcI").innerText=data.iSenseRaw;
  document.getElementById("vadcMv").innerText=data.vSenseMV;
  document.getElementById("iadcMv").innerText=data.iSenseMV;
  document.getElementById("isenseCalMv").innerText=Number(data.iSenseCalMV).toFixed(1);
  document.getElementById("iTotal").innerText=Number(data.totalCurrent).toFixed(3);
  document.getElementById("iDiv").innerText=Number(data.dividerCurrent).toFixed(3);

  const circle=document.getElementById("circle"), status=document.getElementById("status"), button=document.getElementById("powerButton");
  if(hvOn){
    circle.classList.add("on"); button.classList.add("on");
    status.innerText=data.ramping ? "RAMP" : (data.regulating ? "REG" : "ON");
    button.innerText="TURN OFF";
  }else{
    circle.classList.remove("on"); button.classList.remove("on"); status.innerText="OFF"; button.innerText="TURN ON";
  }
}

async function refreshState(){
  try{const r=await fetch("/api/state",{cache:"no-store"});if(r.ok)updateScreen(await r.json());}catch(e){console.log(e)}
}
async function sendSetVoltage(){
  const input=document.getElementById("voltageInput"); let v=parseFloat(input.value);
  if(isNaN(v)){await refreshState();return;} v=Math.max(0,Math.min(80,v)); input.value=v.toFixed(1);
  try{await fetch("/api/set?voltage="+encodeURIComponent(v),{cache:"no-store"});await refreshState();}catch(e){console.log(e)}
}
async function togglePower(){
  try{await fetch("/api/power?state="+(hvOn?0:1),{cache:"no-store"});await refreshState();}catch(e){console.log(e)}
}
const voltageInput=document.getElementById("voltageInput");
voltageInput.addEventListener("change",sendSetVoltage);
voltageInput.addEventListener("keydown",e=>{if(e.key==="Enter")voltageInput.blur();});
refreshState();setInterval(refreshState,500);
</script>
</body>
</html>
)rawliteral";

// ============================================================
// HTTP
// ============================================================

void handleRoot()
{
  server.send_P(200, "text/html", PAGE);
}

void handleState()
{
  String json;
  json.reserve(560);

  bool ramping = hvEnabled && currentDacCode != targetDacCode;
  bool regulating = hvEnabled && !ramping &&
                    HV_CLOSED_LOOP_ENABLED &&
                    setVoltage >= HV_FEEDBACK_MIN_SET_V &&
                    fabsf(setVoltage - measuredVoltage) > HV_FEEDBACK_DEADBAND_V;

  json += "{";
  json += "\"setVoltage\":" + String(setVoltage, 2);
  json += ",\"measuredVoltage\":" + String(measuredVoltage, 3);
  json += ",\"measuredCurrent\":" + String(measuredCurrent, 5);
  json += ",\"totalCurrent\":" + String(totalApdCurrent, 5);
  json += ",\"dividerCurrent\":" + String(dividerCurrent, 5);
  json += ",\"iSenseCalMV\":" + String(calibratedISenseMv, 2);
  json += ",\"vSenseRaw\":" + String(vSenseRaw);
  json += ",\"iSenseRaw\":" + String(iSenseRaw);
  json += ",\"vSenseMV\":" + String(vSenseMilliVolts);
  json += ",\"iSenseMV\":" + String(iSenseMilliVolts);
  json += ",\"dacCurrent\":" + String(currentDacCode);
  json += ",\"dacFeedForward\":" + String(feedForwardDacCode);
  json += ",\"dacTarget\":" + String(targetDacCode);
  json += ",\"dacTrim\":" + String(feedbackTrimCodes);
  json += ",\"ramping\":" + String(ramping ? "true" : "false");
  json += ",\"regulating\":" + String(regulating ? "true" : "false");
  json += ",\"enabled\":" + String(hvEnabled ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleSetVoltage()
{
  if (!server.hasArg("voltage")) {
    server.send(400, "text/plain", "Missing voltage");
    return;
  }

  setVoltage = constrain(server.arg("voltage").toFloat(), HV_MIN, HV_MAX);
  applySetVoltage();
  server.send(200, "text/plain", "OK");
}

void handlePower()
{
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Missing state");
    return;
  }

  bool newState = server.arg("state") == "1";
  if (newState && !hvEnabled) turnHVOn();
  if (!newState && hvEnabled) turnHVOff();
  server.send(200, "text/plain", "OK");
}

void handleNotFound()
{
  server.send(404, "text/plain", "Not found");
}

// ============================================================
// Setup / loop
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("================================");
  Serial.println(" HVunit ESP32 / LT3482");
  Serial.println(" HV/I calibration v1 + closed loop v2");
  Serial.println(" HV module supply: 3.3 V");
  Serial.println("================================");

  pinMode(SHDN_PIN, OUTPUT);
  pinMode(HV_LED_PIN, OUTPUT);
  pinMode(VSENSE_PIN, INPUT);
  pinMode(ISENSE_PIN, INPUT);

  digitalWrite(SHDN_PIN, LOW);
  setHVLed(false);
  dacWrite(CTRL_DAC_PIN, 0);

  analogReadResolution(12);
  analogSetPinAttenuation(VSENSE_PIN, ADC_11db);
  analogSetPinAttenuation(ISENSE_PIN, ADC_11db);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) Serial.println("Wi-Fi AP started");
  else Serial.println("Wi-Fi AP ERROR");

  Serial.print("SSID: "); Serial.println(AP_SSID);
  Serial.print("IP:   "); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/set", HTTP_GET, handleSetVoltage);
  server.on("/api/power", HTTP_GET, handlePower);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("HTTP server started");
  Serial.println("Open http://192.168.4.1");
}

void loop()
{
  server.handleClient();
  updateMeasurements();
  updateDacRamp();
  updateHvFeedback();
  delay(1);
}
