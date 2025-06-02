// HVAC/Compressor Control Code with Engine Start Delay, Pressure Safety,
// Compressor Cycling, Cooldown Timer, and Button & AC Sensor Voltage Display

// Pin definitions
const int acButtonPin    = A0;   // Analog input for AC start button (AC sensor)
const int pressurePin    = A1;   // Analog input for pressure sensor
const int oilPressurePin = A3;   // Analog input for oil pressure sensor
const int compressorPin  = 7;    // Digital output: controls compressor (active LOW)
const int fanPin         = 8;    // Digital output: controls fan (active LOW)

// Pressure sensor calibration parameters
const float PRESSURE_MIN_V   = 0.5;
const float PRESSURE_MAX_V   = 3.2;
const float PRESSURE_MAX_PSI = 400.0;
const float STATIC_PSI       = 25.0;   // Minimum safe pressure

// Oil pressure threshold: engine considered running if oil voltage exceeds this.
const float OIL_PRESS_ENGINE_ON = 1.0;

// Engine start delay: delay after engine comes online before allowing activation.
const unsigned long ENGINE_START_DELAY = 5000; // 5 seconds

// Compressor cycle thresholds (PSI ranges for setting runtime parameters)
const float HIGH_CYCLE_MIN_PSI = 180.0;
const float HIGH_CYCLE_MAX_PSI = 280.0;
const float LOW_CYCLE_MIN_PSI  = 120.0;
const float LOW_CYCLE_MAX_PSI  = 180.0;

// Global variables for engine and compressor timing
unsigned long engineStartTime     = 0;
unsigned long compressorStartTime = 0;
unsigned long compressorStopTime  = 0;

// Compressor cycle runtime parameters (in milliseconds)
unsigned long storedRunTimeLimit = 180000;     // Default runtime limit
unsigned long storedRestartDelay = 30000;      // Default restart delay

// Flag that tracks if the compressor is active (ON)
bool compressorOn = false;

// Flag set when pressure becomes unsafe. Forces outputs off until a fresh button cycle.
bool waitingForReset = false;

// Converts an ADC reading to PSI using a linear mapping.
float adcToPsi(int adcValue) {
  float voltage = (adcValue / 1023.0) * 5.0;
  if (voltage < PRESSURE_MIN_V) return 0.0;
  if (voltage > PRESSURE_MAX_V) return PRESSURE_MAX_PSI;
  return ((voltage - PRESSURE_MIN_V) / (PRESSURE_MAX_V - PRESSURE_MIN_V)) * PRESSURE_MAX_PSI;
}

// Set runtime parameters based on the measured pressure.
void setRuntimeParameters(float pressure) {
  if (pressure >= HIGH_CYCLE_MIN_PSI && pressure <= HIGH_CYCLE_MAX_PSI) {
    storedRunTimeLimit = 300000;    // 5 minutes run time
    storedRestartDelay = 10000;    // 10 seconds off time
  } 
  else if (pressure >= LOW_CYCLE_MIN_PSI && pressure < LOW_CYCLE_MAX_PSI) {
    storedRunTimeLimit = 180000;    // 3 minutes run time
    storedRestartDelay = 20000;     // 20 seconds off time
  } 
  else {
    storedRunTimeLimit = 120000;    // 2 minutes run time (default)
    storedRestartDelay = 30000;     // 30 seconds off time (default)
  }
}

void setup() {
  pinMode(acButtonPin, INPUT);
  pinMode(pressurePin, INPUT);
  pinMode(oilPressurePin, INPUT);
  pinMode(compressorPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  
  Serial.begin(9600);
  
  // Ensure outputs are off at startup (active LOW outputs)
  digitalWrite(compressorPin, HIGH);
  digitalWrite(fanPin, HIGH);
}

void loop() {
  // Read all sensor values at start of loop
  int buttonADC = analogRead(acButtonPin);
  int pressureADC = analogRead(pressurePin);
  int oilADC = analogRead(oilPressurePin);
  
  // Convert ADC readings to voltages and PSI
  float buttonVoltage = (buttonADC / 1023.0) * 5.0;
  float oilVoltage = (oilADC / 1023.0) * 5.0;
  float pressurePSI = adcToPsi(pressureADC);
  float pressureVoltage = (pressureADC / 1023.0) * 5.0;

  // Determine engine state based on oil pressure
  bool engineRunning = (oilVoltage >= OIL_PRESS_ENGINE_ON);
  
  // Detect engine start transition and record start time
  static bool prevEngineRunning = false;
  if (engineRunning && !prevEngineRunning) {
    engineStartTime = millis();
  }
  prevEngineRunning = engineRunning;
  
  // Check if engine start delay has passed
  bool engineStartDelayPassed = engineRunning && ((millis() - engineStartTime) >= ENGINE_START_DELAY);
  
  // Determine if AC (compressor) is requested (button pressed)
  bool compressorRequested = (buttonADC < 90);
  
  // Pressure safety check: safe if PSI is at least STATIC_PSI and below 300 PSI
  bool pressureSafe = (pressurePSI >= STATIC_PSI && pressurePSI < 280);
  
  // If pressure is unsafe, set waiting flag; system won't activate until new button cycle
  if (!pressureSafe) {
    waitingForReset = true;
  }

  // Reset cooldown timer when engine is off or AC button is released
  if (!engineRunning || !compressorRequested) {
    compressorStopTime = 0;  // Reset cooldown timer
  }

  // Compressor control logic
  if (compressorRequested && engineRunning && engineStartDelayPassed && pressureSafe && !waitingForReset) {
    // Update cycle parameters based on current pressure
    setRuntimeParameters(pressurePSI);
    
    if (compressorOn) {
      // Check if compressor has reached its runtime limit
      if ((millis() - compressorStartTime) >= storedRunTimeLimit) {
        compressorOn = false;
        compressorStopTime = millis();  // Record stop time for cooldown
      }
    } 
    else {
      // Check if cooldown period has passed or if it's first run (no stop time recorded)
      if (compressorStopTime == 0 || (millis() - compressorStopTime) >= storedRestartDelay) {
        compressorOn = true;
        compressorStartTime = millis();
      }
    }
  } 
  else {
    // Turn off compressor if conditions not met
    if (compressorOn) {
      compressorOn = false;
    }
  }
  
  // Force compressor off if pressure unsafe
  if (waitingForReset && compressorOn) {
    compressorOn = false;
  }
  
  // Calculate remaining cooldown time (if applicable)
  long cooldownRemaining = 0;
  if (compressorStopTime != 0) {
    unsigned long elapsedSinceStop = millis() - compressorStopTime;
    if (elapsedSinceStop < storedRestartDelay) {
      cooldownRemaining = (storedRestartDelay - elapsedSinceStop) / 1000; // in seconds
    }
  }
  
  // Fan control: matches compressor conditions
  bool fanActive = compressorRequested && engineRunning && engineStartDelayPassed && pressureSafe && !waitingForReset;
  
  // Reset waiting flag when button is released
  if (waitingForReset && (buttonADC >= 90)) {
    waitingForReset = false;
  }
  
  // Update outputs (active LOW means LOW turns the device ON)
  digitalWrite(compressorPin, compressorOn ? LOW : HIGH);
  digitalWrite(fanPin, fanActive ? LOW : HIGH);
  
  // Diagnostics output
  Serial.print("ENGINE: ");
  Serial.print(engineRunning ? "ON" : "OFF");
  if (engineRunning && !engineStartDelayPassed) {
    long delayRemaining = (ENGINE_START_DELAY - (millis() - engineStartTime)) / 1000;
    Serial.print(" (STARTING IN ");
    Serial.print(delayRemaining);
    Serial.print("s)");
  }
  
  Serial.print(" (");
  Serial.print(oilVoltage, 2);
  Serial.print("V)");
  
  Serial.print(" | BUTTON: ");
  Serial.print(buttonADC);
  Serial.print(" (");
  Serial.print(buttonVoltage, 2);
  Serial.print("V)");
  
  Serial.print(" | PRESSURE: ");
  Serial.print(pressurePSI, 1);
  Serial.print(" PSI (");
  Serial.print(pressureVoltage, 2);
  Serial.print("V)");

  if (!pressureSafe) {
    Serial.print(" (UNSAFE)");
  }
  
  Serial.print(" | COMPRESSOR: ");
  Serial.print(compressorOn ? "ON" : "OFF");
  
  // Show compressor run time if running; cooldown timer if cooling down
  if (compressorOn) {
    long runTime = (millis() - compressorStartTime) / 1000;
    Serial.print(" [RUN: ");
    Serial.print(runTime);
    Serial.print("s]");
  } else if (cooldownRemaining > 0) {
    Serial.print(" [COOLDOWN: ");
    Serial.print(cooldownRemaining);
    Serial.print("s]");
  }
  
  Serial.print(" | FAN: ");
  Serial.print(fanActive ? "ON" : "OFF");

  if (!pressureSafe) {
    Serial.print(" | UNSAFE - RESET BUTTON CYCLE REQUIRED");
  }  

  // Only show cycle parameters when engine start delay has passed
  if (engineStartDelayPassed && pressureSafe) {
    Serial.print(" | CYCLE: ");
    Serial.print(storedRunTimeLimit/1000);
    Serial.print("s ON/");
    Serial.print(storedRestartDelay/1000);
    Serial.print("s OFF");
  }
  
  Serial.println();  // Final newline to complete the output line
  
  delay(350);
}
