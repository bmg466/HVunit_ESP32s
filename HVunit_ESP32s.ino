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
// Calibration:
//   HV SET/measurement v1: no-load bench data
//   Current v1: Keithley 6485 + 47.22 kOhm load, 2026-08-14
//
// IMPORTANT:
//   I_SENSE measures TOTAL APD-node current, including the V_SENSE
//   divider current. The firmware subtracts the 1M+33k divider
//   current so the web page shows external load / SiPM current.
// ============================================================

// ============================================================
// Wi-Fi
// ============================================================

const char* AP_SSID     = "SiPM-HV";
const char* AP_PASSWORD = "sipm1234";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

WebServer server(80);

// ============================================================
// Pins
// ============================================================

const uint8_t CTRL_DAC_PIN = 25;   // DAC1
const uint8_t SHDN_PIN     = 27;
const uint8_t VSENSE_PIN   = 34;   // ADC1
const uint8_t ISENSE_PIN   = 35;   // ADC1
const uint8_t HV_LED_PIN   = 2;

const bool LED_ACTIVE_HIGH = true;

// ============================================================
// Operating range
// ============================================================

const float HV_MIN = 0.0f;
const float HV_MAX = 80.0f;

const float HV_MODULE_SUPPLY_V = 3.30f;  // informational

// ============================================================
// HV SET calibration: requested voltage -> DAC code
// ============================================================
// Original no-load measurements:
//
// DAC   real HV [V]
//   0      0.10
//  21     11.48
//  43     21.30
//  64     31.25
//  85     40.81
// 107     50.40
// 128     59.40
// 149     68.90
// 171     78.50
//
// voltageToDac() uses inverse piecewise-linear interpolation.
// Above 78.5 V the final measured segment is extrapolated.
// ============================================================

const uint8_t HV_DAC_CAL_POINTS = 9;

const float HV_DAC_CAL_VOLTAGE[HV_DAC_CAL_POINTS] = {
   0.10f,
  11.48f,
  21.30f,
  31.25f,
  40.81f,
  50.40f,
  59.40f,
  68.90f,
  78.50f
};

const uint8_t HV_DAC_CAL_CODE[HV_DAC_CAL_POINTS] = {
    0,
   21,
   43,
   64,
   85,
  107,
  128,
  149,
  171
};

// ============================================================
// HV measurement calibration
// ============================================================
// No-load calibration using ESP32 analogReadMilliVolts(GPIO34):
//
// real_HV[V] = gain * VADC[mV] + offset
//
// Loaded run confirms good agreement through ~70 V. Near the top
// end the ESP32 ADC becomes less linear, so the raw ADC code remains
// visible on the page for future refinement.
// ============================================================

const float HV_ADC_GAIN_V_PER_MV = 0.03044587f;
const float HV_ADC_OFFSET_V      = 0.701281f;

const uint16_t VSENSE_ZERO_RAW_MAX = 10;

// Physical V_SENSE divider: 1 MOhm + 33 kOhm.
// This current is included in LT3482 MON and must be subtracted
// when displaying external SiPM/load current.
const float VSENSE_TOTAL_KOHM = 1033.0f;

// ============================================================
// Current monitor calibration
// ============================================================
// LT3482:
//   IMON = 0.20 * I_APD_TOTAL
//
// Hardware:
//   MON -> 6.2k -> GND
//   MON -> AD8606 buffer -> GPIO35
//
// The analog section itself agrees extremely well with theory:
//   I_SENSE[mV] ~= I_APD_TOTAL[mA] * (6200 * 0.20)
//                = I_APD_TOTAL[mA] * 1240
//
// The dominant error is ESP32 ADC non-linearity. Therefore the
// calibration below maps averaged RAW ADC code directly to the
// physically measured I_SENSE pin voltage.
//
// Calibration run with Keithley 6485 and 47.22 kOhm load:
//
// ADC raw   measured I_SENSE [mV]
//      0             0
//    141           266
//    458           531
//    773           787
//   1120          1064
//   1459          1340
//   1790          1604
//   2133          1879
//   2443          2124
//
// Firmware then computes total APD current and subtracts current
// through the 1.033 MOhm V_SENSE divider.
// ============================================================

const float R_MON             = 6200.0f;
const float MON_CURRENT_RATIO = 0.20f;

const uint16_t ISENSE_ZERO_RAW_MAX = 10;

const uint8_t I_ADC_CAL_POINTS = 9;

const uint16_t I_ADC_CAL_RAW[I_ADC_CAL_POINTS] = {
     0,
   141,
   458,
   773,
  1120,
  1459,
  1790,
  2133,
  2443
};

const float I_ADC_CAL_SENSE_MV[I_ADC_CAL_POINTS] = {
     0.0f,
   266.0f,
   531.0f,
   787.0f,
  1064.0f,
  1340.0f,
  1604.0f,
  1879.0f,
  2124.0f
};

// ============================================================
// ADC filtering
// ============================================================

const uint8_t ADC_SAMPLES = 32;
const uint16_t ADC_SAMPLE_DELAY_US = 150;
const unsigned long ADC_INTERVAL_MS = 100;
const float ADC_FILTER_ALPHA = 0.10f;

// ============================================================
// DAC ramp
// ============================================================

const unsigned long DAC_RAMP_INTERVAL_MS = 10;

// ============================================================
// Runtime
// ============================================================

float setVoltage      = 0.0f;
float measuredVoltage = 0.0f;
float measuredCurrent = 0.0f;       // external load current, mA

float totalApdCurrent = 0.0f;       // diagnostic, mA
float dividerCurrent  = 0.0f;       // diagnostic, mA
float calibratedISenseMv = 0.0f;    // diagnostic

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
// LED
// ============================================================

void setHVLed(bool state)
{
  if (LED_ACTIVE_HIGH) {
    digitalWrite(HV_LED_PIN, state ? HIGH : LOW);
  } else {
    digitalWrite(HV_LED_PIN, state ? LOW : HIGH);
  }
}

// ============================================================
// ADC averaging
// ============================================================

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

// ============================================================
// Generic piecewise interpolation for current ADC calibration
// ============================================================

float currentRawToSenseMv(uint16_t raw)
{
  if (raw <= ISENSE_ZERO_RAW_MAX) {
    return 0.0f;
  }

  if (raw <= I_ADC_CAL_RAW[0]) {
    return I_ADC_CAL_SENSE_MV[0];
  }

  for (uint8_t i = 1; i < I_ADC_CAL_POINTS; i++) {
    if (raw <= I_ADC_CAL_RAW[i]) {
      float x0 = (float)I_ADC_CAL_RAW[i - 1];
      float x1 = (float)I_ADC_CAL_RAW[i];
      float y0 = I_ADC_CAL_SENSE_MV[i - 1];
      float y1 = I_ADC_CAL_SENSE_MV[i];

      float fraction = ((float)raw - x0) / (x1 - x0);
      return y0 + fraction * (y1 - y0);
    }
  }

  // Above the final calibration point, extrapolate the last segment.
  const uint8_t i1 = I_ADC_CAL_POINTS - 1;
  const uint8_t i0 = I_ADC_CAL_POINTS - 2;

  float x0 = (float)I_ADC_CAL_RAW[i0];
  float x1 = (float)I_ADC_CAL_RAW[i1];
  float y0 = I_ADC_CAL_SENSE_MV[i0];
  float y1 = I_ADC_CAL_SENSE_MV[i1];

  float mv = y1 + ((float)raw - x1) * (y1 - y0) / (x1 - x0);

  // Protect calculations if the ADC is driven outside the calibrated range.
  return constrain(mv, 0.0f, 3300.0f);
}

// ============================================================
// Requested HV -> calibrated DAC code
// ============================================================

uint8_t voltageToDac(float voltage)
{
  voltage = constrain(voltage, HV_MIN, HV_MAX);

  if (voltage <= 0.0f) {
    return 0;
  }

  if (voltage <= HV_DAC_CAL_VOLTAGE[0]) {
    return HV_DAC_CAL_CODE[0];
  }

  for (uint8_t i = 1; i < HV_DAC_CAL_POINTS; i++) {
    if (voltage <= HV_DAC_CAL_VOLTAGE[i]) {
      float v0 = HV_DAC_CAL_VOLTAGE[i - 1];
      float v1 = HV_DAC_CAL_VOLTAGE[i];
      float c0 = (float)HV_DAC_CAL_CODE[i - 1];
      float c1 = (float)HV_DAC_CAL_CODE[i];

      float fraction = (voltage - v0) / (v1 - v0);
      float code = c0 + fraction * (c1 - c0);

      return (uint8_t)constrain((int)roundf(code), 0, 255);
    }
  }

  const uint8_t i1 = HV_DAC_CAL_POINTS - 1;
  const uint8_t i0 = HV_DAC_CAL_POINTS - 2;

  float v0 = HV_DAC_CAL_VOLTAGE[i0];
  float v1 = HV_DAC_CAL_VOLTAGE[i1];
  float c0 = (float)HV_DAC_CAL_CODE[i0];
  float c1 = (float)HV_DAC_CAL_CODE[i1];

  float code = c1 +
      (voltage - v1) * (c1 - c0) / (v1 - v0);

  return (uint8_t)constrain((int)roundf(code), 0, 255);
}

// ============================================================
// DAC ramp
// ============================================================

void updateDacRamp()
{
  if (!hvEnabled) {
    return;
  }

  unsigned long now = millis();

  if (now - lastRampTime < DAC_RAMP_INTERVAL_MS) {
    return;
  }

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

// ============================================================
// HV ON / OFF
// ============================================================

void turnHVOn()
{
  currentDacCode = 0;
  dacWrite(CTRL_DAC_PIN, 0);

  targetDacCode = voltageToDac(setVoltage);

  digitalWrite(SHDN_PIN, HIGH);
  hvEnabled = true;
  setHVLed(true);

  Serial.printf(
      "HV ON | SET %.2f V | calibrated DAC target %u\n",
      setVoltage,
      targetDacCode
  );
}

void turnHVOff()
{
  digitalWrite(SHDN_PIN, LOW);
  dacWrite(CTRL_DAC_PIN, 0);

  currentDacCode = 0;
  targetDacCode  = 0;

  hvEnabled = false;
  setHVLed(false);

  Serial.println("HV OFF");
}

void applySetVoltage()
{
  targetDacCode = voltageToDac(setVoltage);

  Serial.printf(
      "SET %.2f V | calibrated DAC target %u\n",
      setVoltage,
      targetDacCode
  );
}

// ============================================================
// Measurements
// ============================================================

void updateMeasurements()
{
  unsigned long now = millis();

  if (now - lastAdcTime < ADC_INTERVAL_MS) {
    return;
  }

  lastAdcTime = now;

  // ----------------------------------------------------------
  // V_SENSE
  // ----------------------------------------------------------

  AdcSample voltageSample = readAdcAverage(VSENSE_PIN);

  vSenseRaw = voltageSample.raw;
  vSenseMilliVolts = voltageSample.millivolts;

  float newMeasuredVoltage;

  if (vSenseRaw <= VSENSE_ZERO_RAW_MAX) {
    newMeasuredVoltage = 0.0f;
  }
  else {
    newMeasuredVoltage =
        HV_ADC_GAIN_V_PER_MV * (float)vSenseMilliVolts +
        HV_ADC_OFFSET_V;

    newMeasuredVoltage = constrain(newMeasuredVoltage, 0.0f, 100.0f);
  }

  // ----------------------------------------------------------
  // I_SENSE
  // ----------------------------------------------------------

  AdcSample currentSample = readAdcAverage(ISENSE_PIN);

  iSenseRaw = currentSample.raw;
  iSenseMilliVolts = currentSample.millivolts;

  float newMeasuredCurrent = 0.0f;
  float newTotalApdCurrent = 0.0f;
  float newDividerCurrent  = 0.0f;

  calibratedISenseMv = currentRawToSenseMv(iSenseRaw);

  if (iSenseRaw > ISENSE_ZERO_RAW_MAX) {
    // mV / ohm -> mA
    newTotalApdCurrent =
        calibratedISenseMv /
        (R_MON * MON_CURRENT_RATIO);

    // V / kOhm -> mA
    newDividerCurrent =
        newMeasuredVoltage /
        VSENSE_TOTAL_KOHM;

    // What the user normally wants is the external SiPM/load current,
    // not the current consumed by the monitor divider itself.
    newMeasuredCurrent =
        newTotalApdCurrent - newDividerCurrent;

    if (newMeasuredCurrent < 0.0f) {
      newMeasuredCurrent = 0.0f;
    }
  }

  totalApdCurrent = newTotalApdCurrent;
  dividerCurrent  = newDividerCurrent;

  // ----------------------------------------------------------
  // IIR low-pass filtering
  // ----------------------------------------------------------

  if (firstAdcMeasurement) {
    measuredVoltage = newMeasuredVoltage;
    measuredCurrent = newMeasuredCurrent;
    firstAdcMeasurement = false;
  }
  else {
    measuredVoltage +=
        ADC_FILTER_ALPHA *
        (newMeasuredVoltage - measuredVoltage);

    measuredCurrent +=
        ADC_FILTER_ALPHA *
        (newMeasuredCurrent - measuredCurrent);
  }

  if (measuredVoltage < 0.05f) {
    measuredVoltage = 0.0f;
  }

  if (measuredCurrent < 0.002f) {
    measuredCurrent = 0.0f;
  }
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
    <div class="diagTitle">CALIBRATION DATA · HV V1 · I V1</div>
    <div>DAC: <span id="dacCurrent">0</span> &nbsp; TARGET: <span id="dacTarget">0</span></div>
    <div>ADC V: <span id="adcV">0</span> &nbsp;&nbsp; ADC I: <span id="adcI">0</span></div>
    <div>VADC: <span id="vadcMv">0</span> mV &nbsp;&nbsp; IADC: <span id="iadcMv">0</span> mV</div>
    <div>I_SENSE CAL: <span id="isenseCalMv">0</span> mV</div>
    <div>I TOTAL: <span id="iTotal">0</span> mA &nbsp;&nbsp; I DIV: <span id="iDiv">0</span> mA</div>
    <div>HV MODULE: 3.3 V &nbsp; ESP32 · LT3482</div>
  </div>

</div>

<script>
let hvOn=false;
let firstStateReceived=false;

function updateScreen(data){
  hvOn=data.enabled;

  document.getElementById("voltageDisplay").innerText=
    Number(data.measuredVoltage).toFixed(1);

  document.getElementById("currentDisplay").innerText=
    Number(data.measuredCurrent).toFixed(3);

  document.getElementById("setInfo").innerText=
    "SET: "+Number(data.setVoltage).toFixed(1)+" V";

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
  document.getElementById("isenseCalMv").innerText=Number(data.iSenseCalMV).toFixed(1);
  document.getElementById("iTotal").innerText=Number(data.totalCurrent).toFixed(3);
  document.getElementById("iDiv").innerText=Number(data.dividerCurrent).toFixed(3);

  const circle=document.getElementById("circle");
  const status=document.getElementById("status");
  const button=document.getElementById("powerButton");

  if(hvOn){
    circle.classList.add("on");
    button.classList.add("on");
    status.innerText=data.ramping ? "RAMP" : "ON";
    button.innerText="TURN OFF";
  }
  else{
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
  }
  catch(error){
    console.log("State error:",error);
  }
}

async function sendSetVoltage(){
  const input=document.getElementById("voltageInput");
  let voltage=parseFloat(input.value);

  if(isNaN(voltage)){
    await refreshState();
    return;
  }

  voltage=Math.max(0,Math.min(80,voltage));
  input.value=voltage.toFixed(1);

  try{
    await fetch(
      "/api/set?voltage="+encodeURIComponent(voltage),
      {cache:"no-store"}
    );
    await refreshState();
  }
  catch(error){
    console.log("SET error:",error);
  }
}

async function togglePower(){
  try{
    await fetch(
      "/api/power?state="+(hvOn?0:1),
      {cache:"no-store"}
    );
    await refreshState();
  }
  catch(error){
    console.log("Power error:",error);
  }
}

const voltageInput=document.getElementById("voltageInput");
voltageInput.addEventListener("change",sendSetVoltage);
voltageInput.addEventListener("keydown",function(event){
  if(event.key==="Enter"){
    voltageInput.blur();
  }
});

refreshState();
setInterval(refreshState,500);
</script>

</body>
</html>
)rawliteral";

// ============================================================
// HTTP handlers
// ============================================================

void handleRoot()
{
  server.send_P(200, "text/html", PAGE);
}

void handleState()
{
  String json;
  json.reserve(460);

  json += "{";

  json += "\"setVoltage\":";
  json += String(setVoltage, 2);

  json += ",\"measuredVoltage\":";
  json += String(measuredVoltage, 3);

  json += ",\"measuredCurrent\":";
  json += String(measuredCurrent, 5);

  json += ",\"totalCurrent\":";
  json += String(totalApdCurrent, 5);

  json += ",\"dividerCurrent\":";
  json += String(dividerCurrent, 5);

  json += ",\"iSenseCalMV\":";
  json += String(calibratedISenseMv, 2);

  json += ",\"vSenseRaw\":";
  json += String(vSenseRaw);

  json += ",\"iSenseRaw\":";
  json += String(iSenseRaw);

  json += ",\"vSenseMV\":";
  json += String(vSenseMilliVolts);

  json += ",\"iSenseMV\":";
  json += String(iSenseMilliVolts);

  json += ",\"dacCurrent\":";
  json += String(currentDacCode);

  json += ",\"dacTarget\":";
  json += String(targetDacCode);

  json += ",\"ramping\":";
  json +=
      (hvEnabled && currentDacCode != targetDacCode)
      ? "true"
      : "false";

  json += ",\"enabled\":";
  json += hvEnabled ? "true" : "false";

  json += "}";

  server.send(200, "application/json", json);
}

void handleSetVoltage()
{
  if (!server.hasArg("voltage")) {
    server.send(400, "text/plain", "Missing voltage");
    return;
  }

  setVoltage = constrain(
      server.arg("voltage").toFloat(),
      HV_MIN,
      HV_MAX
  );

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

  if (newState && !hvEnabled) {
    turnHVOn();
  }

  if (!newState && hvEnabled) {
    turnHVOff();
  }

  server.send(200, "text/plain", "OK");
}

void handleNotFound()
{
  server.send(404, "text/plain", "Not found");
}

// ============================================================
// Setup
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("================================");
  Serial.println(" HVunit ESP32 / LT3482");
  Serial.println(" HV calibration v1 / I calibration v1");
  Serial.println(" HV module supply: 3.3 V");
  Serial.println("================================");

  pinMode(SHDN_PIN, OUTPUT);
  pinMode(HV_LED_PIN, OUTPUT);
  pinMode(VSENSE_PIN, INPUT);
  pinMode(ISENSE_PIN, INPUT);

  digitalWrite(SHDN_PIN, LOW);
  setHVLed(false);
  dacWrite(CTRL_DAC_PIN, 0);

  hvEnabled = false;
  setVoltage = 0.0f;
  currentDacCode = 0;
  targetDacCode = 0;

  analogReadResolution(12);

  analogSetPinAttenuation(
      VSENSE_PIN,
      ADC_11db
  );

  analogSetPinAttenuation(
      ISENSE_PIN,
      ADC_11db
  );

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Wi-Fi AP started");
  }
  else {
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

// ============================================================
// Main loop
// ============================================================

void loop()
{
  server.handleClient();
  updateMeasurements();
  updateDacRamp();
  delay(1);
}
