const int acButtonPin = A0;
const int pressurePin = A1;
const int compressorPin = 7;
const int fanPin = 8;

// Pressure calibration
const float PRESSURE_MIN_V = 0.5;
const float PRESSURE_MAX_V = 4.5;
const float PRESSURE_MAX_PSI = 400.0;

// Pressure thresholds for cycle timing
const float HIGH_CYCLE_MIN_PSI = 250.0;
const float HIGH_CYCLE_MAX_PSI = 310.0;
const float LOW_CYCLE_MIN_PSI = 180.0;
const float LOW_CYCLE_MAX_PSI = 250.0;

// Static pressure validation
const float STATIC_PSI = 40.0;

enum ShutdownReason {
  MANUAL,
  SAFETY_LIMIT,
  RUNTIME_LIMIT
};

// System state
unsigned long compressorStartTime = 0;
unsigned long compressorStopTime = 0;
bool compressorEnabled = false;
ShutdownReason lastShutdownReason = MANUAL;
unsigned long storedRunTimeLimit = 0;  
unsigned long storedRestartDelay = 0;  

float adcToPsi(int adcValue) {
  float voltage = (adcValue / 1023.0) * 5.0;
  if (voltage < PRESSURE_MIN_V) return 0.0;
  if (voltage > PRESSURE_MAX_V) return PRESSURE_MAX_PSI;
  return ((voltage - PRESSURE_MIN_V) / (PRESSURE_MAX_V - PRESSURE_MIN_V)) * PRESSURE_MAX_PSI;
}

void controlCompressor(bool enable, ShutdownReason reason = MANUAL) {
  digitalWrite(compressorPin, enable ? LOW : HIGH);
  
  if (enable) {
    compressorStartTime = millis();
    lastShutdownReason = MANUAL;
  } else {
    compressorStopTime = millis();
    lastShutdownReason = reason;
  }
  compressorEnabled = enable;
}

void setup() {
  pinMode(compressorPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  controlCompressor(false);
  Serial.begin(9600);
}

void loop() {
  // Read sensors
  int buttonState = analogRead(acButtonPin);
  float pressurePSI = adcToPsi(analogRead(pressurePin));

  // System checks
  bool sensorFault = (pressurePSI <= 0) || (pressurePSI >= PRESSURE_MAX_PSI);
  bool staticPressureValid = (!compressorEnabled && (buttonState < 20)) ? (pressurePSI >= STATIC_PSI) : true;
  bool pressureSafe = (pressurePSI > 20) && (pressurePSI < HIGH_CYCLE_MAX_PSI);
  bool systemFault = sensorFault || !staticPressureValid;

  // Fan control (LOW activates relay)
  bool fanActive = (buttonState < 20) && !systemFault;
  digitalWrite(fanPin, fanActive ? LOW : HIGH);

  // Diagnostics
  Serial.print("BUTTON: ");
  Serial.print(buttonState);
  Serial.print(" (");
  Serial.print((buttonState / 1023.0) * 5.0, 2);
  Serial.print("V) | AC: ");
  Serial.print(compressorEnabled ? "ON" : "OFF");
  Serial.print(" | PRESSURE: ");
  Serial.print(pressurePSI, 1);
  
  if (sensorFault) {
    Serial.print(" PSI (SENSOR FAULT)");
  } else if (!staticPressureValid) {
    Serial.print(" PSI (CHARGE FAULT)");
  } else if (!pressureSafe && compressorEnabled) {
    Serial.print(" PSI (UNSAFE)");
  } else {
    Serial.print(" PSI");
  }

  bool acRequested = (buttonState < 20) && !systemFault;
  bool runtimeLimit = compressorEnabled && (millis() - compressorStartTime >= storedRunTimeLimit);

  // Automatic shutdown
  if (compressorEnabled) {
    if (runtimeLimit) {
      controlCompressor(false, RUNTIME_LIMIT);
    } 
    // Explicit high-pressure cutoff
    else if (pressurePSI >= HIGH_CYCLE_MAX_PSI) {
      controlCompressor(false, SAFETY_LIMIT);
    }
    // General safety checks
    else if (!pressureSafe || systemFault) {
      controlCompressor(false, SAFETY_LIMIT);
    }
  }

  // Compressor startup logic
  if (acRequested && !compressorEnabled && !systemFault) {
    bool pressureNowSafe = (pressurePSI > 20) && (pressurePSI < HIGH_CYCLE_MAX_PSI);
    bool safetyCooldownActive = (lastShutdownReason != MANUAL) && (millis() - compressorStopTime < storedRestartDelay);
    
    if (pressureNowSafe && !safetyCooldownActive) {
      // Set cycle timing based on current pressure
      if (pressurePSI >= HIGH_CYCLE_MIN_PSI && pressurePSI <= HIGH_CYCLE_MAX_PSI) {
        storedRunTimeLimit = 600000;  // 10 minutes
        storedRestartDelay = 60000;   // 60 seconds
      } else if (pressurePSI >= LOW_CYCLE_MIN_PSI && pressurePSI < LOW_CYCLE_MAX_PSI) {
        storedRunTimeLimit = 480000;  // 8 minutes
        storedRestartDelay = 90000;   // 90 seconds
      } else {
        storedRunTimeLimit = 300000;  // 5 minutes
        storedRestartDelay = 30000;   // 30 seconds
      }
      controlCompressor(true);
    }
  } else if (!acRequested && compressorEnabled) {
    controlCompressor(false, MANUAL);
  }

  // Status display
  Serial.print(" | FAN: ");
  Serial.print(fanActive ? "ON" : "OFF");
  
  if (compressorEnabled) {
    Serial.print(" | RUNNING: ");
    Serial.print((millis() - compressorStartTime) / 1000);
    Serial.print("s/");
    Serial.print(storedRunTimeLimit / 1000);
    Serial.println("s");
  } else if (systemFault) {
    Serial.println(" | SYSTEM FAULT");    
  } else if (lastShutdownReason != MANUAL) {
    long remainingCooldown = storedRestartDelay - (millis() - compressorStopTime);
    remainingCooldown = remainingCooldown < 0 ? 0 : remainingCooldown;
    Serial.print(" | COOLDOWN: ");
    Serial.print(remainingCooldown / 1000);
    Serial.print("s (");
    Serial.print((pressurePSI > 20 && pressurePSI < HIGH_CYCLE_MAX_PSI) ? "Safe" : "Unsafe");
    Serial.println(")");
  } else {
    Serial.println(" | SYSTEM READY");
  }
  
  delay(350);
}
