### **HVAC Compressor Controller Installation Guide**   

---

### **1. Components Required**  
- **Arduino Uno/Nano**  
- **5V SPDT Relays** (x2: compressor and fan)  
- **Resistors**:  
  - **3kΩ** (x1, for HVAC button voltage divider)  
  - **4.7kΩ** (x1, for HVAC button voltage divider)  
  - **10kΩ** (x1, for oil pressure sensor voltage divider)  
  - **3kΩ** (x1, for oil pressure sensor voltage divider)  
- **Zener Diodes**: **5.1V** (x2, for voltage spike protection)  
- **Flyback Diodes**: **1N4001** (x2, for relay coil protection)  
- **Automotive Fuses**: 5A (for Arduino power)  

---

### **2. Wiring Diagram**  
#### **A/C Button (Violet Wire)**:  
```  
HVAC Violet Wire (8V) ── 3kΩ ──┬── Arduino A0  
                                │  
                              4.7kΩ ── GND  
                                │  
                            5.1V Zener (Cathode → Violet Wire)  
```  
- **Function**: Converts 8V to **~3.1V** (safe for Arduino).  
- **Calibration**:  
  - **Pressed**: A0 reads **< 20** (0.1V).  
  - **Not Pressed**: A0 reads **~640** (3.1V).  

#### **Oil Pressure Sensor (PCM Pin 5)**:  
```  
PCM Pin 5 (Oil Pressure Signal) ── 10kΩ ──┬── Arduino A3  
                                          │  
                                        3kΩ ── GND  
                                          │  
                                      5.1V Zener (Cathode → PCM Pin 5)  
```  
- **Function**: Converts 12V to **~2.77V** (safe for Arduino).  
- **Calibration**:  
  - **Engine Off**: A3 reads **< 1.0V**.  
  - **Engine Running**: A3 reads **~2.77V**.
 

#### **A/C Refrigerant Pressure Transducer Wiring**:  
```      
Transducer VREF (Pin 50)  → Arduino 5V  
Transducer SIGRTN (Pin 23) → Arduino GND  
Transducer ACPT (Pin 32)  → Arduino A1 (Analog Input)
```

#### **Compressor Relay**:  
```  
Arduino D7 ──── Relay Coil (+)  
Relay Coil (-) ──── GND  
Relay COM ──── Car Chassis Ground  
Relay NO ──── Compressor Control Wire (ground trigger)  
Flyback Diode: Across relay coil (cathode to +5V, anode to GND).  
```  

#### **Fan Relay (PCM Pin 52)**:  
```  
Arduino D8 ──── Relay Coil (+)  
Relay Coil (-) ──── GND  
Relay COM ──── Car Chassis Ground  
Relay NO ──── PCM Pin 52 (ground trigger for factory fan relay)  
Flyback Diode: Across relay coil (cathode to +5V, anode to GND).  
```  

---

### **3. Installation Steps**  
1. **HVAC Button Circuit**:  
   - Connect the violet wire to the **3kΩ/4.7kΩ voltage divider** and Zener diode.  
   - Verify A0 voltage matches **3.1V (unpressed)** and **< 0.5V (pressed)**.  

2. **Oil Pressure Sensor Circuit**:  
   - Tap into PCM Pin 5 and connect to the **10kΩ/3kΩ voltage divider** with Zener diode.  
   - Confirm A3 reads **> 2.0V** when the engine is running.  

3. **Relay Wiring**:  
   - **Compressor**: Connect D7’s relay NO terminal to the compressor control wire.  
   - **Fan**: Connect D8’s relay NO terminal to PCM Pin 52.  
   - Ensure **COM** terminals are grounded to the chassis.  

4. **Power the Arduino**:  
   - Use a **switched 12V source** (ignition-on power) with a 5A fuse.  

---

### **4. System Behavior**  
- **Compressor Activation**:  
  - Requires **ALL** of:  
    1. Engine running (`oilVoltage ≥ 2.0V`).  
    2. A/C button pressed (`buttonState < 80`).  
    3. Pressure between **25–300 PSI**.  
  - Shuts down immediately if:  
    - Oil pressure drops (`oilVoltage < 2.0V`).  
    - Pressure exceeds **340 PSI**.  

- **Fan Activation**:  
  - Turns on when A/C button is pressed **AND** engine is running.  

---

### **5. Diagnostics & Testing**  
#### **Serial Monitor Output**:  
```arduino  
ENGINE: ON | OIL: 2.8V | BUTTON: 15 (0.07V) | AC: ON | PRESSURE: 210 PSI | FAN: ON | RUNNING: 30s/600s  
```  

#### **Functional Tests**:  
1. **Engine Off**:  
   - Compressor/fan should **not activate** when the button is pressed.  
2. **Engine Running**:  
   - Compressor starts after pressure stabilizes (5-second delay).  
   - Fan activates with compressor.  

---

### **6. Safety Checks**  
1. **Voltage Limits**:  
   - Confirm A0/A3 never exceed **3.3V** (use multimeter).  
2. **Grounding**:  
   - Ensure all components share the same chassis ground.  
3. **Relay Protection**:  
   - Flyback diodes **must** be installed to protect the Arduino.  

---

### **7. Troubleshooting**  
| Issue                          | Solution                                  |  
|--------------------------------|-------------------------------------------|  
| **Compressor/fan not activating** | Check relay wiring and ground connections. |  
| **Erratic pressure readings**  | Add a 0.1µF capacitor between A1 and GND. |  
| **Oil pressure misread**       | Recalibrate `OIL_PRESS_ENGINE_ON` in code. |  

---

### **8. Wiring Summary**  
![Installation Diagram](https://i.imgur.com/7j6XWz9.png)
