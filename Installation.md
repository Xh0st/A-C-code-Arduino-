### **Installation Guide (Code-Compatible)**  
**For 5V-Powered A/C Transducer & Protected HVAC Button Signal**  

---

### **1. Hardware Setup**  
#### **Components Required**:  
- Arduino Uno/Nano  
- 12V-to-5V Step-Down Converter (e.g., LM7805 or DC-DC Buck)  
- 2-Channel Relay Module (for compressor and fan control)  
- Automotive A/C Pressure Transducer (e.g., 151-3)  
- **Resistors**: 3kΩ, 4.7kΩ  
- **Zener Diodes**: 5.1V (x2)  
- Automotive 12V power source (car battery)  

---

### **2. Wiring Diagram**  
#### **A/C Transducer**:  
```
Transducer VREF (Pin 50) → Arduino 5V  
Transducer SIGRTN (Pin 23) → Arduino GND  
Transducer ACPT (Pin 32) → Arduino A1 (Analog Input)  
```

#### **HVAC A/C Button Signal Protection**:  
```
HVAC Button Signal (8V) ── 3kΩ Resistor ──┬── Arduino A0 (Analog Input)  
                                          │  
                                        4.7kΩ Resistor ── GND  
                                          │  
                                      5.1V Zener Diode (Cathode → Signal)  
```

#### **Step-Down Converter Protection**:  
```
12V Car Battery ── Step-Down Input  
Step-Down Output (+) ──┬── Arduino VIN  
                      │  
                  5.1V Zener Diode (Cathode → +, Anode → -)  
                      │  
Step-Down Output (-) ─┴── Arduino GND  
```

#### **Relay Connections**:  
```
Arduino D7 → Compressor Relay Control  
Arduino D8 → Fan Relay Control  
```

---

### **3. Critical Details**  
#### **A/C Transducer**:  
- Ensure the transducer is rated for **5V operation** (confirm via datasheet).  
- Directly powered by Arduino’s 5V pin.  

#### **HVAC Button Protection**:  
- **Voltage Divider**: Scales 8V → Safe 3.1V for Arduino:  
  \[
  V_{\text{out}} = 8V \times \frac{4.7kΩ}{3kΩ + 4.7kΩ} ≈ 3.1V
  \]  
- **Zener Diode**: Clamps voltage spikes >5.1V (cathode faces the HVAC signal).  

#### **Step-Down Protection**:  
- The Zener diode clamps the 5V output to **5.1V** if the step-down fails.  

---

### **4. Calibration**  
#### **Pressure Transducer**:  
1. Measure the ACPT voltage at **0 PSI** and **MAX PSI** using a refrigerant gauge.  
2. Update these constants in the code (if necessary):  
   ```cpp
   const float PRESSURE_MIN_V = 0.45;  // Voltage at 0 PSI (default: 0.5V)  
   const float PRESSURE_MAX_V = 4.5;    // Voltage at 400 PSI (default: 4.5V)  
   ```  

#### **HVAC Button**:  
- **Pressed**: Arduino A0 reads **< 20** (0.1V).  
- **Unpressed**: Arduino A0 reads **~3.1V** (624 analog value).  

---

### **5. Testing Procedure**  
1. **Transducer Test**:  
   - Disconnect the transducer hose. Pressure should read **0 PSI**.  
   - Reconnect and verify readings match a refrigerant gauge.  

2. **Button Test**:  
   - Press the HVAC button. Serial Monitor should show:  
     ```arduino
     BUTTON: 0 (0.00V) | ...  
     ```  
   - Release the button. Serial Monitor should show:  
     ```arduino
     BUTTON: 624 (3.10V) | ...  
     ```  

3. **Safety Checks**:  
   - Force a pressure spike (>310 PSI). The compressor should shut off immediately (`SAFETY_LIMIT`).  
   - Disconnect the transducer. The system should report `SENSOR FAULT`.  

---

### **6. Safety Precautions**  
| Component              | Expected Voltage       | Fault Action |  
|-------------------------|------------------------|---------------|  
| **A/C Transducer (A1)** | 0.45V–4.5V             | Replace transducer |  
| **HVAC Button (A0)**    | 0V (pressed), 3.1V (unpressed) | Check resistors/Zener |  
| **Step-Down Output**    | 5.0V ±0.2V             | Replace step-down/Zener |  

---

### **7. Final Wiring Summary**  
![Wiring Diagram](https://i.imgur.com/7j6XWz9.png) 

**Example Output**:  
```arduino
BUTTON: 0 (0.00V) | AC: OFF | PRESSURE: 45.0 PSI | FAN: ON | SYSTEM READY  
BUTTON: 624 (3.10V) | AC: ON | PRESSURE: 220.5 PSI | FAN: ON | RUNNING: 30s/480s  
```

