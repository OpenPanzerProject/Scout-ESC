// We can measure temperature with an onboard thermistor, current through the motor driver chips, and voltage (through a voltage divider resistor setup)

// -------------------------------------------------------------------------------------------------------------------------------------------------->
// THERMISTOR - TEMP SENSING
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// Checks thermistor temp. Saves value in *C to global variable BoardTemp. 
void CheckTemp()
{
    float reading;
    
    // Read thermistor voltage divider
    reading = analogRead(TempSense);
 
    // Convert value to resistance
    reading = (1023 / reading) - 1;
    reading = TEMP_SERIES_RESISTOR / reading;

    // Thanks to LadyAda for the handy thermistor tutorial: 
    // https://learn.adafruit.com/thermistor/using-a-thermistor
    // But we only need this if we care to know the temperature in celsius. For purposes of controlling the fan, 
    // this is not strictly necessary. 
    float steinhart;
    steinhart = (float)reading / THERMISTOR_NOM_RES;    // (R/Ro)
    steinhart = log(steinhart);                         // ln(R/Ro)
    steinhart /= THERMISTOR_BCONSTANT;                  // 1/B * ln(R/Ro)
    steinhart += 1.0 / (THERMISTOR_NOM_TEMP + 273.15);  // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invert
    steinhart -= 273.15;                                // convert to C

    BoardTemp = (uint16_t)steinhart;                    // Set our global variable, to one degree Celsius precision which is close enough for our purposes.

//    Serial.print("Temperature "); 
//    Serial.print(steinhart);
//    Serial.println(" *C");

    // The VNH2SP30/VNH5019 chips have their own overtemperature protection and will go into thermal shutdown at 175*C typical (but possibly as low as 150*C) 
    // Thermal reset is at 135*C (meaning, it has to get back down to 135 before the chips turn back on).
    // The throw a fault (En/Diag brought low) on overtemp which we check for as well, so basically we have two overcurrent detection methods, though the chips
    // are likely to register far sooner than the board thermistor we are measuring here. The thermistor is still useful however for knowing absolue temperature
    // which we use to control the cooling fan. 
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// CURRENT SENSE
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// Checks current. Saves value in mA to global variable TotalCurrent_mA. If over the set current limit, will return true, otherwise if in bounds returns false. 
boolean CheckCurrent()
{
uint16_t Current;
static boolean firstPass1 = true;
static boolean firstPass2 = true;
static uint8_t OverCurrentCount1 = 0;
static uint8_t OverCurrentCount2 = 0;

// How many successive overcurrent readings do we require before returning true
// Set this variable in combination with the frequency this function is called. 
// For example, if we call this ten times a second and we want the over-current 
// measurement to persist for 2 seconds before treating it as a fault condition, 
// we would set this to 20. 
// Presently frequency is 100mS, or 10 times per second. Combined with count of 10 means 1 second total:
const uint8_t MaxOverCurrentCount = 10;

// To reduce the effect of noise we run our readings through a low-pass filter.
// With an alpha of 0.9, it is like giving the current reading a weight of 10%, and past readings a weight of 90%
const float alpha = 0.9;
  
// Calculations: 
    // VNH2SP30 datasheet 
        // Iout/Isense = 11370 (typical)
    // VNH5019 datasheet
        // Iout/Isense = 9245 (this is not directly from the datasheet, calculated from your known Rsense and Iout & Vsense from the listed values at 3A, 8A, 15A, 25A
// Rearranged
    // Isense = Iout/(Typical Iout/Isense)
// ADC input
    // Vsense = Isense * Rsense (where Vsense is the input into ADC and Rsense is 1.5k)
// Combined
    // Vsense = (Iout / 11370) * 1.5k   Implies 132 millivolts reading per amp for VNH2SP30
    // Vsense = (Iout /  9245) * 1.5k   Implies 162 millivolts reading per amp for VNH5019
// Now that we know the volts reading per amp, we can calculate the amps per ADC step:
    // 5V / 1024 ADC steps / 0.132 V per A = 37 mA current per ADC count VNH2SP30
    // 5V / 1024 ADC steps / 0.162 V per A = 30 mA current per ADC count VNH5019
// The specific value is held in mA_per_ADC which is determined by the setting of the variable MotorChipVersion

    // Motor 1
    Current = analogRead(M1_CS) * mA_per_ADC;                                   // Instantaneous current in milli-amps
    if (firstPass1) { M1_Current_mA = Current; firstPass1 = false; }            // Filter
    M1_Current_mA = alpha * (float)M1_Current_mA + (1.0 - alpha) * Current;
    if (M1_Current_mA > (MaxCurrent_A * 1000)) OverCurrentCount1 += 1;          // Compare in milliamps, increment count if we are over
    else OverCurrentCount1 = 0;                                                 // If we are below the limit, reset the count

    // Motor 2
    Current = analogRead(M2_CS) * mA_per_ADC;                                   // Instantaneous current in milli-amps
    if (firstPass2) { M2_Current_mA = Current; firstPass2 = false; }            // Filter
    M2_Current_mA = alpha * (float)M2_Current_mA + (1.0 - alpha) * Current; 
    if (M2_Current_mA > (MaxCurrent_A * 1000)) OverCurrentCount2 += 1;          // Compare in milliamps, increment count if we are over
    else OverCurrentCount2 = 0;                                                 // If we are below the limit, reset the count

    // Save the combined current draw to our global variable
    TotalCurrent_mA = M1_Current_mA + M2_Current_mA; 
    
    // Are we over the limit? 
    if ((OverCurrentCount1 > MaxOverCurrentCount) || (OverCurrentCount2 > MaxOverCurrentCount))
    {
        OverCurrentCount1 = OverCurrentCount2 = 0;                              // Reset the counts. We rely on the calling routine to decide what action to take.
        return true;                                                            // Over-current condition
    }    
    else return false;                                                          // If we make it to here, current is within limits for both motors. 
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// VOLTAGE SENSE
// -------------------------------------------------------------------------------------------------------------------------------------------------->
float ReadVoltage(void)
{
int voltSense;                  // What we read on the analog pin
float unFilteredVoltage;        // Converted to voltage on pin
static float filteredVoltage;   // Voltage on pin after being run through a simple low-pass filter
float Voltage;                  // Pin voltage converted to battery voltage
static boolean firstPass = true;       

// We are using this voltage divider:
// 
// GND |-----/\/\/\-----------------/\/\/\-------> +V Batt
//            4.7k         |         10k
//                         |
//                      Measure

// The voltage we measure on the pin isn't the actual voltage of the battery, it is the battery voltage divided by some number:
//multiplier = (4.7 + 10) / 4.7 = 3.1277
const float multiplier = 3.1277;    // Multiply this by our measured voltage and we will have battery voltage

// In some cases we may also need to apply a fixed offset, for example in cases where an input polarity diode drops the input voltage by a set amount. 
// On the Scout ESC we actually don't use an input polarity diode, although we do have input polarity MOSFETs (that only really protect the motor driver chips)
const float vAdj = 0.0;             // Adjustment factor 

// To reduce the effect of noise, we run our voltage through a low-pass filter.
// With an alpha of 0.9, it is like giving the voltage reading a weight of 10%, and past readings a weight of 90%
const float alpha = 0.9;

    // Ok, here we go: first take an analog reading (will give us a number between 0-1023)
        voltSense = analogRead(VSense);
    //Convert the reading to actual voltage read (0 - 5 Vdc)
        unFilteredVoltage = (voltSense / 1024.0) * 5.0;
    // Now run it through low-pass filter
        if (firstPass) { filteredVoltage = unFilteredVoltage; firstPass = false; }
        filteredVoltage = alpha * filteredVoltage + (1.0 - alpha) * unFilteredVoltage;
    // Now convert the measured voltage to the external battery voltage by accounting for our voltage divider
        Voltage = filteredVoltage * multiplier;
    // Now also add our adjustment factor to account for the voltage drop on the input polarity diode
        Voltage += vAdj;
    
    //Serial.print("Voltage: ");
    //Serial.println(Voltage,2);

    return Voltage;
}


