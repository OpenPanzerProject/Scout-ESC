/* Scout ESC            Open Panzer dual brushed motor controller
 * Source:              openpanzer.org              
 * Authors:             Luke Middleton
 * Version:             9.01    (for use with Hardware Revision 9)
 * Last Updated:        10/11/2016
 *                      
 * Copyright 2016 Open Panzer
 *   
 * For more information, see the Open Panzer Wiki:
 * http://openpanzer.org/wiki/
 * 
 * Scout ESC GitHub Repository:
 * https://github.com/OpenPanzerProject/Scout-ESC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */ 


// DEFINES AND GLOBAL VARIABLES
// -------------------------------------------------------------------------------------------------------------------------------------------------->
    // Serial comms
        #define BAUD_RATE_DEFAULT           38400       // Baud rate fixed at boot - the user can change it via serial command, but this way they always know what rate the device is initialized to. 
        boolean SerialWatchdog =            false;      // If false (default), the motors will continue at their last commanded speed forever until a new command arrives. If no new command ever arrives, they will keep going. 
                                                        // If true, the absence of any serial communication beyond SerialTimeout will cause the motors to be halted. Disabled at boot by default. 
        uint16_t SerialWatchdogTimeout_mS = 1000;       // When SerialWatchdog = true, this is the time in milliseconds after the last serial reception when the motors will automatically be halted as a safety precaution. 
                                                        // Default to 1 second but will be set by the user to some custom time if they enable SerialWatchdog.
        #define SerialBlinkTimeout_mS       1000        // When the serial watchdog is disabled, we will use this length of time with no serial commands to blink an LED, but we won't stop the motors. 
        #define SENTENCE_BYTES              4           // How many bytes in a valid sentence
        struct DataSentence {                           // Serial commands should have four bytes (plus termination). 
            uint8_t    Address =            0;          // We use a struct for convenience
            uint8_t    Command =            0;
            uint8_t    Value =              0;
            uint8_t    Checksum =           0;
        };
        #define BAUD_CODE_2400              1           // Codes for changing baud rates
        #define BAUD_CODE_9600              2           // These are the same codes used by certain Dimension Engineering Sabertooth controllers
        #define BAUD_CODE_19200             3           //
        #define BAUD_CODE_38400             4           //
        #define BAUD_CODE_115200            5           //

    // RC defines
        #define NUM_RC_CHANNELS             2           // Number of RC channels we can read
        #define PULSE_WIDTH_ABS_MIN         800         // Absolute minimum pulse width considered valid
        #define PULSE_WIDTH_ABS_MAX         2200        // Absolute maximum pulse width considered valid
        #define PULSE_WIDTH_TYP_MIN         1000        // Typical minimum pulse width
        #define PULSE_WIDTH_TYP_MAX         2000        // Typical maximum pulse width        
        #define PULSE_WIDTH_CENTER          1500        // Stick centered pulse width (motor off)
        #define PULSE_WIDTH_DEADBAND        5           // How many microseconds around center do we ignore
        
        // RC state machine
        typedef char _RC_STATE;
        #define RC_SIGNAL_ACQUIRE           0
        #define RC_SIGNAL_SYNCHED           1
        #define RC_SIGNAL_LOST              2
        _RC_STATE RC_State =                0;
        
        // RC times
        #define RC_PULSECOUNT_TO_ACQUIRE    5           // How many pulses on each channel to read before considering ourselves in SIGNAL_SYNCHED
        #define RC_TIMEOUT_US               100000      // How many micro-seconds without a signal from either channel before we go to SIGNAL_LOST. Note a typical RC pulse would arrive once every 20,000 uS
        #define RC_TIMEOUT_MS               100         // In milliseconds, so RC_TIMEOUT_US / 1000
        uint32_t LastGoodRCPulse[NUM_RC_CHANNELS];      // What time did we last receive a good pulse

    // Device address
        #define AddressA                    131         // Device has two possible addresses, set by dipswitch
        #define AddressB                    132         // These are within the range of addresses also used by Dimension Engineering Sabertooth devices (128-135)
        uint8_t MyAddress;                              // Address of device. Set by dipswitch. 

    // Device mode
        typedef char _INPUT_MODE;                       // We accept two types of input - hobby RC PWM commands, or serial
        #define INPUT_UNKNOWN               0           // This is the input mode on startup
        #define INPUT_RC                    1           
        #define INPUT_SERIAL                2
        #define INPUT_ERROR                 3           
        _INPUT_MODE InputMode =             INPUT_UNKNOWN;  // Global variable for input mode

    // Error states
        typedef char _ERROR_STATE;
        #define ERROR_NONE                  0           // No errors 
        #define ERROR_OVERCURRENT           1           // Current exceeds limit
        #define ERROR_OVERVOLTAGE           2           // Voltage exceeds limit
        #define ERROR_UNDERVOLTAGE          3           // Voltage too low
        #define ERROR_OVERTEMP              4           // Temperature exceeds limit (or motor leads shorted)
        #define EMERGENCY_STOP              5           // Manual override - emergency stop
        _ERROR_STATE ErrorState =           ERROR_NONE;     // Global variable for error state
        
    // Error handling
        #define OVERCURRENT_DELAY_MS        5000        // How long to wait after an over-current condition before reseting, in milliseconds
        #define VOLTAGE_DELAY_MS            5000        // How long to wait after an over- or under-voltage condition before reseting, in milliseconds
        #define OVERTEMP_DELAY_MS           5000        // How long to wait after an over-temp condition before resetting, in milliseconds (applies also to short conditions)

    // Status LED states
        typedef char _LED_STATES;                       // Various states the status LED can be in
        #define TURN_OFF                    0           // Off 
        #define SOLID                       1           // On solid
        #define BLINK_HEARTBEAT             2           // Blink slowly to indicate waiting
        #define BLINK_LOST_SIGNAL           3           // Blink quickly to indicate lost signal (see below for rate)
        #define BLINK_EMERGENCY_STOP        4           // Blink quickly to indicate emergency stop (see below for rate)
        #define BLINK_STREAM_2              5           // Blink stream - twice
        #define BLINK_STREAM_3              6           // Blink stream - three times
        #define BLINK_STREAM_4              7           // Blink stream - four times
        #define BLINK_STREAM_5              8           // Blink stream - five times

        #define BLINK_RATE_HEARTBEAT        750         // How often to blink the light in heartbeat mode, in milliseconds
        #define BLINK_RATE_LOST_SIGNAL      40          // Blink rate when radio signal lost
        #define BLINK_RATE_EMERGENCYSTOP    40          // Blink rate when emergency stop switch has been held to ground

    // Generic blinker struct
        // We can create a string of sequential blinks any number of times, define here the length of time the LED is on, off, and how long it remains off
        // after the last blink (PauseTime), before repeating the blink sequence. 
        #define StreamBlinkOnTime           200
        #define StreamBlinkOffTime          200
        #define StreamBlinkPauseTime        1000

        typedef struct 
        {   byte            LEDPin =        0;          // What pin does this blinker apply to
            uint8_t         BlinkTimes =    0;          // How many times to blink
            uint8_t         BlinkPos =      0;          // What position in the blink stream are we presently at
            uint32_t        LastChangeTime =0;          // When did we last update
            uint16_t        WaitTime =      0;          // How long does this position in the stream last 
            boolean         Active =        false;      // Is this blinker active       
        } BlinkStream;
        
        // Create two global BlinkStreams, one for each LED
        BlinkStream         StatusLEDBlinker;
        BlinkStream         ErrorLEDBlinker;

    // Voltage sensing
        boolean EnableVoltageSensing =      true;       // If false, no voltage sensing will take place, and there will be no under or over-voltage protection aside from that provided by the chips (5.5v to 16v)
        float MinVoltage =                  6.0;        // Minimum battery voltage before motors are enabled. Can be set by user to any value from 6-16 via serial commands. 
        float MaxVoltage =                  16.0;       // Maximum allowed voltage before motors are disabled.
        #define VoltageReadFrequency_mS     500         // How often to measure voltage in milliseconds

    // Current sensing
        boolean EnableOverCurrent =         true;       // If false we will not enter an error condition even if the current goes over the limit. Use only for testing. In any case the device will still turn itself off if gets too hot, which it will at high currents
        uint16_t MaxCurrent_A =             12;         // Maximum current allowed in amps. Default to 12 amps. Current is per channel (not total device current). Can be changed by serial command (1-30A)
        #define CurrentReadFrequency_mS     100         // How often to measure current draw in milliseconds
        uint16_t TotalCurrent_mA =          0;          // How much total current is being drawn by both motors combined, in milliamps
        uint16_t M1_Current_mA =            0;          // Current for motor 1
        uint16_t M2_Current_mA =            0;          // Current for motor 2 

    // Thermistor/temp-sensing
        #define TEMP_MEASURE_FREQ_MS        200         // How often to measure temperature in milliseconds
        // Thermistor is Murata NXP18XH103F03RB
        #define THERMISTOR_NOM_RES          10000       // Resistance at nominal temperature in ohms
        #define THERMISTOR_NOM_TEMP         25          // Nominal temperature
        #define THERMISTOR_BCONSTANT        3455        // Beta coefficient of thermistor from datasheet (25-100*C range)
        #define TEMP_SERIES_RESISTOR        10000       // What is the value in ohms of the series resistor inline with the thermistor (R22 on rev 8 of the Scout board)
        #define TEMP_CUTOFF                 150         // Temperature cutoff. Same as the VNH2SP30 chips, which would probably register this temp much sooner than the thermistor. 
        uint16_t BoardTemp =                0;          // What is the temperature of the onboard thermistor in Celsius (no decimal places)

    // Fan
        boolean ManualFanControl =          false;      // Flag to specify whether fan is controlled by the user via serial commands, or allowed to blow automatically. Default is auto unless the user specifically requests otherwise. 
        uint8_t MinimumFanSpeed =           80;         // Very low PWM values won't even turn the fan on. Set the level between 0-255 that is the lowest level that will reliably turn it on. 
        uint8_t MaximumFanSpeed =           255;        // Depends on how you set up the PWM, but in our case is 255. Do not change this unless you have also for some reason changed the fan PWM setup! 
        
    
// PINS                       ARDUINO PIN      ARDUINO   DIRECTION                                             ATMEGA PORT PIN       OTHER
// -------------------------------------------------------------------------------------------------------------------------------------------------->
    // RESERVED                    0            // D0    Input     - This is the serial RX pin                      PD0
    // RESERVED                    1            // D1    Output    - This is the serial TX pin                      PD1
    const byte RC2 =               2;           // D2    Input     - RC input for motor 2                           PD2              INT0
    const byte RC1 =               3;           // D3    Input     - RC input for motor 1                           PD3              INT1 / Timer 2 / PWM OC2B
    const byte M2_DirA =           4;           // D4    Output    - Motor 2 direction A                            PD4
    const byte M1_DirA =           5;           // D5    Output    - Motor 1 direction A                            PD5              Timer 0 / PWM OC0B
    const byte AddressSwitch =     6;           // D6    Input     - Address selection switch                       PD6              Timer 0 / PWM OC0A
    const byte M1_DirB =           7;           // D7    Output    - Motor 1 direction B                            PD7
    const byte M1_DIAG =           8;           // D8    Input     - Motor 1 status                                 PB0
    const byte M2_PWM =            9;           // D9    Output    - Motor 2 PWM (Left/Turret Rotation)             PB1              Timer 1 / PWM OC1A
    const byte M1_PWM =           10;           // D10   Output    - Motor 1 PWM (Right/Barrel Elevation)           PB2              Timer 1 / PWM OC1B
    const byte Fan_PWM =          11;           // D11   Output    - Fan motor PWM                                  PB3              Timer 2 / PWM OC2A / MOSI
    const byte EmergencyStop =    12;           // D12   Input     - Emergency stop                                 PB4              MISO
    const byte RedLED =           13;           // D13   Output    - Red LED - used to indicate errors              PB5              SCK
    const byte M1_CS =            14;           // A0    Input     - Motor 1 current sense                          PC0              ADC0
    const byte TempSense =        15;           // A1    Input     - Thermistor                                     PC1              ADC1
    const byte M2_DirB =          16;           // A2    Output    - Motor 2 direction B                            PC2              ADC2
    const byte BlueLED =          17;           // A3    Output    - Blue LED - used to indicate status             PC3              ADC3
    const byte M2_CS =            18;           // A4    Input     - Motor 2 current sense                          PC4              ADC4
    const byte M2_DIAG =          19;           // A5    Input     - Motor 2 status                                 PC5              ADC5
    const byte VSense =           A6;           // A6    Input     - Battery voltage readin through divider         N/A              ADC6
    // ADC6 and ADC7 are not broken out in the DIP version of the 328 chip and are therefore not referenced by the original Arduino Duemilanove board. 
    // They are however broken out on the TQFP chip and if we set our board type to Nano we can reference the names A6 and A7 in Arduino code. 

// MOTOR 1 & 2 PWM 
// -------------------------------------------------------------------------------------------------------------------------------------------------->
    #define MOTOR_PWM_TOP       381             // We will set ICR1 equal to our custom TOP value
    #define M2_OCR              OCR1A           // The output compare register associated with PB1
    #define M1_OCR              OCR1B           // The output compare register associated with PB2

// FAN PWM
// -------------------------------------------------------------------------------------------------------------------------------------------------->
    #define FAN_OCR             OCR2A           // The output compare register associated with PB3


void setup()
{
    // PINS
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // MOTOR 1
        pinMode(M1_DirA, OUTPUT);               // Output   - Direction        
        pinMode(M1_DirB, OUTPUT);               // Output   - Direction
        pinMode(M1_PWM, OUTPUT);                // Ouput    - PWM
        pinMode(M1_CS, INPUT);                  // Input    - Current sense
        pinMode(M1_DIAG, INPUT_PULLUP);         // Input    - Motor 1 enable/diagnostic notification. Must be pulled up. 
        
        // MOTOR 2
        pinMode(M2_DirA, OUTPUT);               // Output   - Direction        
        pinMode(M2_DirB, OUTPUT);               // Output   - Direction
        pinMode(M2_PWM, OUTPUT);                // Ouput    - PWM
        pinMode(M2_CS, INPUT);                  // Input    - Current sense
        pinMode(M2_DIAG, INPUT_PULLUP);         // Input    - Motor 2 enable/diagnostic notification. Must be pulled up. 

        // EMERGENCY STOP
        pinMode(EmergencyStop, INPUT_PULLUP);   // Input    - If brought to ground will cause immediate motor stop. Connect to active-low button or switch in implementations that need emergency human override. 

        // FAN
        pinMode(Fan_PWM, OUTPUT);               // Output   - Fan PWM

        // TEMP SENSE
        pinMode(TempSense, INPUT);              // Input    - Thermistor

        // VOLTAGE SENSE
        // Because we are using ADC6 which is a somewhat unusual pin that has no digital I/O ability, we don't need to use pinMode
        
        // RC INPUTS
        pinMode(RC1, INPUT_PULLUP);             // Input    - RC PWM input
        pinMode(RC2, INPUT_PULLUP);             // Input    - RC PWM input

        // LEDs
        pinMode(BlueLED, OUTPUT);               // Output   - Blue LED
        digitalWrite(BlueLED, LOW);             //            Initialize to off
        pinMode(RedLED, OUTPUT);                // Output   - Red LED
        digitalWrite(RedLED, LOW);              //            Initialize to off
        StatusLEDBlinker.LEDPin = BlueLED;      // Assign also these pins to our two StatusLEDBlinker structs
        ErrorLEDBlinker.LEDPin = RedLED;

        // SWITCH
        pinMode(AddressSwitch, INPUT_PULLUP);   // Input    - Address selection switch


    // DEVICE ADDRESS
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // The address is set by the position of dipswitch 1
        digitalRead(AddressSwitch) ? MyAddress = AddressA : MyAddress = AddressB;

       
    // SERIAL
    // ---------------------------------------------------------------------------------------------------------------------------------------------->        
        Serial.begin(BAUD_RATE_DEFAULT);


    // INPUT MODE and ERROR STATE
    // ---------------------------------------------------------------------------------------------------------------------------------------------->        
        InputMode = INPUT_UNKNOWN;              // Start in unknwon state
        ErrorState = ERROR_NONE;                // Start with no errors
               

    // TIMER 1 - MOTOR PWM
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // We use Timer 1 to generate ~21KHz PWM on Arduino pins 9 & 10 (PB1 and PB2). Ken Shirriff has a good tutorial on the various types of PWM possible: http://www.righto.com/2009/07/secrets-of-arduino-pwm.html
    
        // We want: 
        // - Waveform Generation Mode bits (WGM1 3:0) - split across TCCR1A and TCCR1B. These set the PWM modes, we want Mode 10 - Phase Correct with a custom TOP value set by ICR1. See page 137
        // - Clock Select (CS1 2:0) - last three bits of TCCR1B which set the prescaler, we want prescaler of 1 (none). See page 138
        // - Compare Match Output A & B bits (COM1A, COM1B): Two bits each, in TCCR1A register, these enable/disable PWM on the associated pins. See page 136. 
        //   The pins we are using are Arduino pins 9 & 10 / Atmega pins 13 & 14 (on the TQFN package), or in other words, Atmega port B pin 1 and 2.
        
        // To control the duty cycle (set the motor speed), we write to OCR1A or OCR1B a value between 0-TOP
        //OCR1A = 0-TOP;    // To set duty cycle on Arduino pin 9
        //OCR1B = 0-TOP;    // To set duty cycle on Arduino pin 10
       
        // The frequency is determined by the PWM mode (phase correct in our case), the clock frequency, the prescaler, and our TOP value. 
        // Formula is Clock/(2*Prescaler*TOP) = 16,000,000 / (2 * 1 * 381) = 20,997 Hz
        // Low frequencies result in "motor whine" which is annoying and doesn't sound realistic on a model. The VNH2SP30 motor drivers used on this device have a maximum PWM frequency input of 20Khz, though this
        // is not absolute. Higher frequencies cause less efficient operation and create more heat. Human hearing typically extends to about 20Khz, so above that the PWM should be inaudbile. 
        // We choose a value of 381 because it gives us a frequency just above 20KHz, but also because it is equal to 127 * 3. As this device will accept a 7-bit speed number (meaning values from 0-127), we only 
        // need to multiply the speed number * 3 to get the correct OCR1A/B value for the duty cycle. 

        TCCR1A = 0xA2;              // COM1 A1:A0, COM1 B1:B0 = 10 (connect non-inverted PWM to pins), WGM1 3:0 = 1010 (Mode 10, Phase Correct PWM, TOP = ICR1)
        TCCR1B = 0x11;              // CS1 2:0 = 001 (Prescaler = 1 / no prescaler)
        TIFR1 =  0x27;              // Start off with all Timer 1 flags clear (by setting bits to 1 in TIFR1 register)
        TIMSK1 = 0x00;              // Disable Timer 1 interrupts (by clearing TIMSK1 bits). PWM is generated in hardware and we won't need interrupts enabled for it. 
        ICR1 =  MOTOR_PWM_TOP;      // Set ICR1 to our custom TOP value
        TCNT1 = 0;                  // Reset TCNT1
        OCR1A = 0;                  // Start off with duty cycle for both motors at 0 for safety.
        OCR1B = 0;                  // 


    // TIMER 2 - FAN MOTOR PWM
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // We use Timer 2 to generate PWM on Arduino pin 11 (PB3). This will be used to control the speed of a cooling fan, and for this purpose it is not critical we achieve ultrasonic frequencies. 
        // Timer 2 is an 8-bit timer which we will use in Fast PWM Mode with TOP set to 0xFF and a prescaler of 8, that will give us a PWM frequency of ~8KHz. That will be plenty quiet for a fan. 
    
        // We want: 
        // - Waveform Generation Mode bits (WGM2 2:0) - split across TCCR2A and TCCR2B. These set the PWM modes, we want Mode 3 - Fast PWM with TOP set to 0xFF (255). See page 161
        // - Clock Select (CS2 2:0) - last three bits of TCCR2B which set the prescaler, we want prescaler of 8. See page 163
        // - Compare Match Output A & B bits (COM2A, COM2B): Two bits each, in TCCR2A register, these enable/disable PWM on the associated pins. See page 159 
        //   The pin we are using is Arduino pin 11 / Atmega pin 15 (on the TQFN package), or in other words, Atmega port B pin 3. This is OC2A.
        //   We do *not* want PWM on OC2B (Atmega PD3) because we are using that as an input for one of the RC channels. 
        
        // To control the duty cycle (set the motor speed), we write to OCR2A a value between 0-255.
       
        // The frequency is determined by the PWM mode (Fast PWM in this case), the clock frequency, the prescaler, and our TOP value. 
        // Formula is Clock/Prescaler/TOP = 16,000,000 / 8 / 255 = 7,843 Hz
        // Low frequencies result in "motor whine" which can be annoying, whereas higher frequencies cause less efficient operation and create more heat. 
        // Cooling fans are pretty quiet to begin with so we don't worry about having the absolute highest frequency, which could cause other issues. 

        TCCR2A = 0x83;              // COM2 A1:A0 = 10, COM2 B1:B0 = 00 (connect non-inverted PWM to OC2A, disconnect PWM from OC2B ), WGM2 2:0 = 011 (Mode 3, Fast PWM, TOP = OxFF)
        TCCR2B = 0x02;              // CS2 2:0 = 010 (Prescaler = 8) and the first bit of WGM2 to 0
        TIFR2 =  0x07;              // Start off with all Timer 2 flags clear (by setting bits to 1 in TIFR2 register)
        TIMSK2 = 0x00;              // Disable Timer 2 interrupts (by clearing TIMSK2 bits). PWM is generated in hardware and we won't need interrupts enabled for it. 
        TCNT2 = 0;                  // Reset TCNT2
        OCR2A = 0;                  // Start off with duty cycle at 0 (motor stopped) for safety


    // EXTERNAL INTERRUPTS INT0 and INT1
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // We use external interrupts INT0 and INT1 to measure pulse-widths of the two RC signals on PD2 and PD3. The length of the PWM signal determines
        // the value of the radio command. Pulse-widths should be between 1000 and 2000uS long, with 1500uS indicating stick centered. We will enable 
        // an interrupt whenever the pin changes state (rise or falls). When the pin rises we will record the time, when it falls we will check how much time
        // has passed, and that will be our pulsewidth. 
        EICRA = 0x05;               // Set both INT0 and INT1 to interrupt on change, we will check the pin in the ISR to determine whether it was rising or falling
        EnableRCInterrupts();       // Enable INT0 and INT1 interrupts
}


void loop()
{
    uint32_t            uS;                                     // Temp variable to hold the current time in microseconds
    
    static _LED_STATES  StatusLEDState;                         // The current state of our Status LED
    static _LED_STATES  ErrorLEDState;                          // The current state of our Error LED
    static uint32_t     LastBlinkTime_Status = 0;               // When did we last toggle the Status LED while blinking
    static uint32_t     LastBlinkTime_Error = 0;                // When did we last toggle the Error  LED while blinking

    static DataSentence Sentence;                               // A struct to store incoming commands
    
    static uint32_t     TimeLastRCCheck = 0;                    // Time we last did a watchdog check on the RC signal
    static uint32_t     TimeLastSerial = 0;                     // Time of last received serial data
    
    static uint32_t     LastTempMeasure = 0;                    // When did we last measure the board temperature
    
    static uint32_t     LastCurrentMeasure = 0;                 // When did we last measure current draw
    static uint32_t     LastOverCurrent = 0;                    // When did we last detect an overcurrent condition
    
    static uint32_t     LastVoltageMeasure = 0;                 // When did we last measure voltage
    static uint32_t     LastOverVoltage = 0;                    // When did we last detect an overvoltage condition

    static uint32_t     LastOverTemp = 0;                       // When did we last detect an overtemp condition 
    

    // Read sensors and set any error conditions
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    if (ErrorState == ERROR_NONE)
    {
        // Check emergency stop switch every time through loop
        if (digitalRead(EmergencyStop) == LOW) 
        {   
            ErrorState = EMERGENCY_STOP;                            // The emergency stop pin has been held low.
        }

        // Check temperature every so often
        if (millis() - LastTempMeasure > TEMP_MEASURE_FREQ_MS)
        {
            CheckTemp();                                            // Check the temperature
            LastTempMeasure = millis();                             // Save the time of this reading
            if (BoardTemp > TEMP_CUTOFF)                            // Check overtemp
            {
                // We count this as an overtemperature condition, though if the thermistor is over 150 we can be pretty darn sure the VNH2SP30s already returned an over-temp fault anyway
                ErrorState = ERROR_OVERTEMP;
            }
        }            
        
        // Check VNH2DSP30 chip diagnostic pins for fault condition
        // These pins go low in the case of overtemperature or a short on the motor leads, but we won't know exactly which condition caused it. 
        if (digitalRead(M1_DIAG) == LOW || digitalRead(M2_DIAG) == LOW)
        {
            // We could try to infer the cause of the fault by reading the onboard thermistor. If it is cool then we can assume this is a short. But if it is hot, 
            // we still don't know exactly whether the cause was heat or a short. So we don't bother, and we treat both of these cases together as basically overtemp
            ErrorState = ERROR_OVERTEMP;
        }
        
        // Check current draw every so often
        if ((millis() - LastCurrentMeasure > CurrentReadFrequency_mS))
        {
            LastCurrentMeasure = millis();                          // Save the time of this reading
            if (CheckCurrent())                                     // CheckCurrent returns true if we are over-current. Regardless we still want to check so we update the global variable. 
            {
                if (EnableOverCurrent)                              // We only cause an error if EnableOverCurrent is true (unless you are testing, it should be)
                {
                    LastOverCurrent = LastCurrentMeasure;           // Save the time of this event
                    ErrorState = ERROR_OVERCURRENT;                 // Set the error state
                }
            }
        }
        
        // Check for over or under voltage
        if (EnableVoltageSensing && millis() - LastVoltageMeasure > VoltageReadFrequency_mS)
        {
            float voltage = ReadVoltage();                          // Read voltage
            LastVoltageMeasure = millis();                          // Save the time of this reading
            if (voltage < MinVoltage)
            {
                ErrorState = ERROR_UNDERVOLTAGE;                    // Battery is too low
            }
            else if (voltage > MaxVoltage)
            {
                ErrorState = ERROR_OVERVOLTAGE;                     // Voltage too high
            }
        }
    }
    else
    {
        // Process errors and clear them if appropriate
        // ---------------------------------------------------------------------------------------------------------------------------------------------->        
    
        // All errors require stopping both motors
        StopMotors();

        // Now process error 
        switch (ErrorState)
        {
            case ERROR_OVERTEMP:
                // This error occurs when one of the VNH2SP30 chips has reported a fault condition. The fault could actually be overtemperature or a short, but we won't know
                // which it is so we just treat it as an overtemp condition. We can also find ourselves here if the onboard thermistor registers an overtemp condition, but 
                // the motor chips would almost certainly have thrown an overtemp error long before the thermistor would have caught up. 
                
                // After a set amount of time, we try to clear the fault
                if (millis() - LastOverTemp > OVERTEMP_DELAY_MS)
                {
                    // Attempt to clear the fault conditions on the VNH2SP30 chips
                    ClearVNH2SP30Faults();
                    LastOverTemp = millis();    // Reset this, we will use it to keep toggling the chips after a set amount of time if the fault doesn't immediately clear
                }

                // Check also the temperature
                if (millis() - LastTempMeasure > TEMP_MEASURE_FREQ_MS)
                {
                    CheckTemp();                                            // Check the temperature
                    LastTempMeasure = millis();                             // Save the time of this reading
                }

                // If diag pins went back to high and board temp is below cutoff, we can exit the error state
                if (digitalRead(M1_DIAG) == HIGH && digitalRead(M2_DIAG) == HIGH && BoardTemp < TEMP_CUTOFF)
                {
                    ErrorState = ERROR_NONE;
                }
                else
                {   
                    // Still in wver-temp indication - red blink two times
                    StatusLEDState = TURN_OFF;
                    ErrorLEDState = BLINK_STREAM_2;
                }
                break;
                
            case ERROR_OVERCURRENT:
                // If enough time has passed, clear the fault and resume normal operation. We know current will be below the threshold because we have stopped the motors. 
                if (millis() - LastOverCurrent > OVERCURRENT_DELAY_MS)
                {
                    ErrorState = ERROR_NONE;
                }
                else
                {
                    // Over-current indication - red blink three times
                    StatusLEDState = TURN_OFF;
                    ErrorLEDState = BLINK_STREAM_3;
                }
                break;

            case ERROR_UNDERVOLTAGE: 
                // Battery was too low. Is it still? 
                if (ReadVoltage() > MinVoltage && (millis() - LastOverVoltage > VOLTAGE_DELAY_MS))
                {   
                    ErrorState = ERROR_NONE;
                }
                else
                {
                    // Low battery indication - red blink four times
                    StatusLEDState = TURN_OFF;
                    ErrorLEDState = BLINK_STREAM_4;     
                }
                break;
                
            case ERROR_OVERVOLTAGE:
                // Battery was too high. Is it still?                 
                if (ReadVoltage() < MaxVoltage && (millis() - LastOverVoltage > VOLTAGE_DELAY_MS))
                {   
                    ErrorState = ERROR_NONE; 
                }
                else
                {
                    // High battery indication - red blink five times in a row. 
                    // This one shouldn't happen very often so we use the unwieldy 5 blink notification for it. 
                    StatusLEDState = TURN_OFF;
                    ErrorLEDState = BLINK_STREAM_5;     
                }
                break;

            case EMERGENCY_STOP:
                // The emergency stop pin was held low. 
                // But check to see if it is still low, if not, clear the error condition so we can resume normal operation
                if (digitalRead(EmergencyStop)) 
                {
                    ErrorState = ERROR_NONE;
                }
                else
                {
                    ErrorLEDState = BLINK_EMERGENCY_STOP;       // Error LED blinks like crazy
                    StatusLEDState = TURN_OFF;                  // Status LED off
                }
                break;
        }
    }
    
    
    // Read and Process Input, if not in error state
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    if (ErrorState == ERROR_NONE)
    {
        // If we are not in an error state, turn the red LED off
        ErrorLEDState = TURN_OFF;

        switch (InputMode)
        {
            case INPUT_SERIAL:
                if (ReadData(&Sentence)) 
                {
                    ProcessCommand(&Sentence);                  // Do whatever we're told
                    TimeLastSerial = millis();                  // Save the time
                }

                // Status LED behavior depends on if we have the SerialWatchdog enabled or not
                if (SerialWatchdog)
                {   
                    if (millis() - TimeLastSerial > SerialWatchdogTimeout_mS)
                    {
                        StopMotors();                           // If SerialWatchdog is enabled, and we haven't received a signal in the set amount of time, stop the motors as a safety precaution.
                        StatusLEDState = BLINK_LOST_SIGNAL;     // We also blink the status LED with the lost signal pattern (very quickly)
                    }
                    else
                    {
                        StatusLEDState = SOLID;                 // We've gotten a signal within the watchdog time period, so keep the status LED solid
                    }
                }
                else
                {   // If SerialWatchdog is not enabled, we give a heartbeat signal after SerialBlinkTimeout_mS, but we never go to BLINK_LOST_SIGNAL nor will we stop the motors unless the user commands it
                    if (millis() - TimeLastSerial > SerialBlinkTimeout_mS) 
                    {
                        StatusLEDState = BLINK_HEARTBEAT;       // Blink the status LED slowly if we aren't receiving serial data
                    }
                    else
                    {
                        StatusLEDState = SOLID;                 // So long as we are receiving data, keep the LED on solid
                    }
                }
                break;
    
            
            case INPUT_RC:
                switch (RC_State)
                {
                    case RC_SIGNAL_ACQUIRE:
                        StatusLEDState = BLINK_HEARTBEAT;        // We do the regular heartbeat during the acquiring state, but this state won't last long before becoming either synched or lost
                        break;
    
                    case RC_SIGNAL_SYNCHED:
                        StatusLEDState = SOLID;                  // So long as we are receiving data, keep the LED on solid
                        
                        // The INT0/INT1 ISRs try to determine the status of each channel, but of course if a channel becomes disconnected the ISR won't even trigger. 
                        // So we also do a check here in the main loop, not every time through, just once every so often (RC_TIMEOUT_MS) 
                        if (millis() - TimeLastRCCheck > RC_TIMEOUT_MS)
                        {
                            TimeLastRCCheck = millis();
                            cli();                               // We need to disable interrupts for this check, otherwise value of (uS - LastGoodRCPulse) could return very big number if channel updates between the next two lines
                                uS = micros();                   // Current time
                                if ((uS - LastGoodRCPulse[0] > RC_TIMEOUT_US) && (uS - LastGoodRCPulse[1] > RC_TIMEOUT_US))    // Are BOTH channels overdue? 
                                {   
                                    StopMotors();                // Ok, we've lost radio - stop the motors and set the state to lost
                                    RC_State = RC_SIGNAL_LOST;
                                }
                            sei();                               // Resume interrupts
                        }
                        break;           
    
                    case RC_SIGNAL_LOST:
                        StatusLEDState = BLINK_LOST_SIGNAL;      // If we had a radio signal, but then lost it, blink the status LED faster
                        break;
                       
                }
                break;
    
            case INPUT_UNKNOWN:
                // In this case we have yet to identify a serial or RC command, so check both
                if (RC_State == RC_SIGNAL_SYNCHED)
                {
                    InputMode = INPUT_RC;                       // Set mode to RC
                    StatusLEDState = SOLID;
                }
                else if (ReadData(&Sentence)) 
                {
                    InputMode = INPUT_SERIAL;                   // Set mode to Serial
                    DisableRCInterrupts();                      // Don't check the RC inputs anymore
                    ProcessCommand(&Sentence);                  // Do whatever we're told
                    TimeLastSerial = millis();                  // Save the time
                    StatusLEDState = SOLID;
                }
                
                // While in unknown mode, blink the Red LED slowly - once we exit unknown mode, turn it off. 
                // We are only in unknown mode until the first signal is received - after that we will never return to unknown mode until the device is re-started. 
                InputMode == INPUT_UNKNOWN ? ErrorLEDState = BLINK_HEARTBEAT : ErrorLEDState = TURN_OFF;
                break;
        }
    }

    // Update Status LED
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    switch (StatusLEDState)
    {
        case SOLID:
            StatusLEDOn();
            break;

        case TURN_OFF:
            StatusLEDOff();
            break;
            
        case BLINK_HEARTBEAT:
            if (millis() - LastBlinkTime_Status > BLINK_RATE_HEARTBEAT) 
            {
                ToggleStatusLED();
                LastBlinkTime_Status = millis();
            }
            break;

        case BLINK_LOST_SIGNAL:
            if (millis() - LastBlinkTime_Status > BLINK_RATE_LOST_SIGNAL) 
            {
                ToggleStatusLED();
                LastBlinkTime_Status = millis();
            }
            break;

        // These are the generic blinker streams
        case BLINK_STREAM_2:
            if (!StatusLEDBlinker.Active || StatusLEDBlinker.BlinkTimes != 2)    InitializeBlinker(&StatusLEDBlinker, 2);
            UpdateBlinker(&StatusLEDBlinker);
            break;

        case BLINK_STREAM_3:
            if (!StatusLEDBlinker.Active || StatusLEDBlinker.BlinkTimes != 3)    InitializeBlinker(&StatusLEDBlinker, 3);
            UpdateBlinker(&StatusLEDBlinker);
            break; 

        case BLINK_STREAM_4:
            if (!StatusLEDBlinker.Active || StatusLEDBlinker.BlinkTimes != 4)    InitializeBlinker(&StatusLEDBlinker, 4);
            UpdateBlinker(&StatusLEDBlinker);
            break; 
        
        case BLINK_STREAM_5:
            if (!StatusLEDBlinker.Active || StatusLEDBlinker.BlinkTimes != 5)    InitializeBlinker(&StatusLEDBlinker, 5);
            UpdateBlinker(&StatusLEDBlinker);
            break; 
    }

    // Update Error LED
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    switch (ErrorLEDState)
    {
        case SOLID:
            ErrorLEDOn();
            break;

        case TURN_OFF:
            ErrorLEDOff();
            break;

        case BLINK_HEARTBEAT:
            if (millis() - LastBlinkTime_Error > BLINK_RATE_HEARTBEAT) 
            {
                ToggleErrorLED();
                LastBlinkTime_Error = millis();
            }
            break;
       
        case BLINK_LOST_SIGNAL:
            if (millis() - LastBlinkTime_Error > BLINK_RATE_LOST_SIGNAL) 
            {
                ToggleErrorLED();
                LastBlinkTime_Error = millis();
            }
            break;
        
        case BLINK_EMERGENCY_STOP:
            if (millis() - LastBlinkTime_Error > BLINK_RATE_EMERGENCYSTOP) 
            {
                ToggleErrorLED();
                LastBlinkTime_Error = millis();
            }
            break;            
        
        // These are the generic blinker streams
        case BLINK_STREAM_2:
            if (!ErrorLEDBlinker.Active || ErrorLEDBlinker.BlinkTimes != 2)    InitializeBlinker(&ErrorLEDBlinker, 2);
            UpdateBlinker(&ErrorLEDBlinker);
            break;

        case BLINK_STREAM_3:
            if (!ErrorLEDBlinker.Active || ErrorLEDBlinker.BlinkTimes != 3)    InitializeBlinker(&ErrorLEDBlinker, 3);
            UpdateBlinker(&ErrorLEDBlinker);
            break; 

        case BLINK_STREAM_4:
            if (!ErrorLEDBlinker.Active || ErrorLEDBlinker.BlinkTimes != 4)    InitializeBlinker(&ErrorLEDBlinker, 4);
            UpdateBlinker(&ErrorLEDBlinker);
            break; 

        case BLINK_STREAM_5:
            if (!ErrorLEDBlinker.Active || ErrorLEDBlinker.BlinkTimes != 5)    InitializeBlinker(&ErrorLEDBlinker, 5);
            UpdateBlinker(&ErrorLEDBlinker);
            break;           
    }

    // Update Fan Speed (it will automatically take care of manual control)
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        AutoFanControl();
}


void InitializeBlinker(BlinkStream *blinker, uint8_t blinktimes)
{
    blinker->BlinkTimes = blinktimes;           // How many times to blink
    blinker->BlinkPos = 0;                      // Start at the beginning
    blinker->LastChangeTime = 0;                // Assume we haven't changed in a long time
    blinker->WaitTime = StreamBlinkOnTime;      // Start with On time
    blinker->Active = true;                     // Start the blinker
}

void HaltBlinker(BlinkStream *blinker)
{
    blinker->Active = false;
}


void UpdateBlinker(BlinkStream *b)
{
uint32_t m = millis();
    
    if (m - b->LastChangeTime > b->WaitTime)            // If enough time has passed, change state
    {
        if (b->BlinkPos % 2 == 0)                        
        {
            digitalWrite(b->LEDPin, HIGH);              // Even numbers in the stream mean light on
            b->WaitTime = StreamBlinkOnTime;            // How long does this position last
        }
        else
        {
            digitalWrite(b->LEDPin, LOW);
            b->WaitTime = StreamBlinkOffTime;
        }

        b->LastChangeTime = m;                          // Save the time of this update

        b->BlinkPos += 1;                               // Increment to next position                
        
        // But, if this happens to be the very last position, the wait time is actually StreamBlinkPauseTime
        if (b->BlinkPos == 2 * b->BlinkTimes)
        {
            b->WaitTime = StreamBlinkPauseTime;
            b->BlinkPos = 0;                            // Roll back over to beginning
        }
    }
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// EXTERNAL INTERRUPTS
// -------------------------------------------------------------------------------------------------------------------------------------------------->
ISR(INT1_vect)
{
    // INT1 is used to read RC Channel 1, and it controls Motor 1
    ProcessRCPulse(0);
}

ISR(INT0_vect)
{
    // INT0 is used to read RC Channel 2, and it controls Motor 2
    ProcessRCPulse(1);
}

void ProcessRCPulse(uint8_t ch)
{
    static uint32_t     last[NUM_RC_CHANNELS];
    static _RC_STATE    Ch_State[NUM_RC_CHANNELS] = {RC_SIGNAL_ACQUIRE, RC_SIGNAL_ACQUIRE};   // Start off in the acquiring state for each channel
    static uint8_t      Ch_PulseCount[NUM_RC_CHANNELS] = {0, 0};                              // How many pulses have we received in the acquiring state
    uint32_t            pulseWidth;                                                           // Measured pulse width
    uint32_t            uS;                                                                   // Temp storage of current micros()
    boolean             pulseReceived = false;

    uS =  micros();    // Do this very first so we aren't introducing unnecessary latency

    // Channel 1 is attached to Port D pin 3 
    if (ch == 0)
    {
        if (PIND & (1 << PD3))
        {   // Rising edge - save the time and exit
            last[ch] = uS;
            return;
        }
        else
        {   // Falling edge - pulse received
            pulseWidth = uS - last[ch];                        
            pulseReceived = true;
        }
    }

    // Channel 2 is attached to Port D pin 2 
    if (ch == 1)
    {
        if (PIND & (1 << PD2))
        {   // Rising edge - save the time and exit
            last[ch] = uS;
            return;
        }
        else
        {   // Falling edge - pulse received
            pulseWidth = uS - last[ch];            
            pulseReceived = true;
        }
    }

    // Process received pulse
    if (pulseReceived)
    {
        // Measure the pulsewidth.
        if (pulseWidth >= PULSE_WIDTH_ABS_MIN && pulseWidth <= PULSE_WIDTH_ABS_MAX)
        {
            // This is a valid pulse. Save the time.
            LastGoodRCPulse[ch] = uS;
            
            switch (Ch_State[ch])
            {
                case RC_SIGNAL_LOST:
                    Ch_State[ch] = RC_SIGNAL_ACQUIRE;
                    Ch_PulseCount[ch] = 1;
                    break;
                
                case RC_SIGNAL_ACQUIRE:
                    Ch_PulseCount[ch] += 1;
                    if (Ch_PulseCount[ch] >= RC_PULSECOUNT_TO_ACQUIRE)
                    {
                        Ch_State[ch] = RC_SIGNAL_SYNCHED;
                    }
                    break;

                case RC_SIGNAL_SYNCHED:
                    if ((pulseWidth >= (PULSE_WIDTH_CENTER - PULSE_WIDTH_DEADBAND)) && (pulseWidth <= (PULSE_WIDTH_CENTER + PULSE_WIDTH_DEADBAND)))
                    {   // The stick is centered, or within the deadband of center. Make sure motor is stopped.
                        StopMotor(ch + 1);                                      // Add 1 to channel because it is zero-based and StopMotor takes 1 or 2
                    }
                    else
                    {   // We have a command for the motor
                        setSpeed(ch + 1, getSpeedCommand_fromRC(pulseWidth));   // Add 1 to channel because it is zero-based and setSpeed takes 1 or 2
                    }
                    break;
            }
        }
        else
        {
            // Invalid pulse. If we haven't had a good pulse for a while, stop the motor and set the state to SIGNAL_LOST. 
            if (uS - LastGoodRCPulse[ch] > RC_TIMEOUT_US)
            {
                Ch_State[ch] = RC_SIGNAL_LOST;
                Ch_PulseCount[ch] = 0;
                StopMotor(ch + 1);                                              // Add 1 to channel because it is zero-based and StopMotor takes 1 or 2
            }
        }

        // We know what the individual channel's state is, but let's combine both channel's states into a single 'RC state' 
        // If both channels share the same state, then that is also the state of the overall RC system
        if (Ch_State[0] == Ch_State[1]) 
        {
            RC_State = Ch_State[0];
        }
        else
        {   // In this case, the channels have different states. 
            // If even one channel is synched, then we consider the system synched:
            if (Ch_State[0] == RC_SIGNAL_SYNCHED || Ch_State[1] == RC_SIGNAL_SYNCHED) RC_State = RC_SIGNAL_SYNCHED;
            // The only remaining options are that neither channel is synched, and at least one of the channels is lost (meaning, it was synched but then signal was removed)
            // We could this as system lost:
            else RC_State = RC_SIGNAL_LOST;
        }
    }
}

int16_t getSpeedCommand_fromRC(uint16_t pulseWidth)
{
    // Constrain pulse width to typical range.
    pulseWidth = constrain(pulseWidth, PULSE_WIDTH_TYP_MIN, PULSE_WIDTH_TYP_MAX);

    // Now scale pulse width from the 1000 to 2000 range, to -/+ our PWM duty cycle range
    return map(pulseWidth, PULSE_WIDTH_TYP_MIN, PULSE_WIDTH_TYP_MAX, -MOTOR_PWM_TOP, MOTOR_PWM_TOP);
}

void EnableRCInterrupts(void)
{   // Enable INTO and INT1 external interrupts
    EIMSK = 0x03;               // Enable INT0 and INT1 interrupts
    EIFR = 0x03;                // Clear interrupts to start (by setting flag bits to 1)
}

void DisableRCInterrupts(void)
{
    // Disable INT0 and INT1 external interupts
    EIMSK = 0;                  // Disable INT0 and INT1 interrupts
}


// -------------------------------------------------------------------------------------------------------------------------------------------------->
// SERIAL COMMANDS
// -------------------------------------------------------------------------------------------------------------------------------------------------->
boolean ReadData(DataSentence * sentence)
{
    byte ByteIn;
    static boolean addressReceived = false;     // Have we received a byte that matches our address
    static char input_line[SENTENCE_BYTES];
    static uint8_t numBytes = 0;                // Start off with no data received
    
    boolean SentenceReceived = false;           // Start off false, will get set to true if a valid sentence was received


    // Read all the bytes that are available, starting with the first byte that matches our address
    while(Serial.available())               
    {
        ByteIn = Serial.read();
        if (ByteIn == MyAddress)
        {
            addressReceived = true;         // Matching address
            input_line[0] = ByteIn;         // Save it in our array
            numBytes = 1;                   // Subsequent bytes will be added to the array until we have enough to compare against INIT_STRING
        }
        else if (addressReceived)
        {
            input_line[numBytes++] = ByteIn;
            if (numBytes >= SENTENCE_BYTES) break;  // We have enough bytes for a full sentence, so evaluate it
        }
    }

    // If we have enough bytes for a full sentence, save it
    if (numBytes >= SENTENCE_BYTES)
    {   // We have enough bytes for a full sentence
        sentence->Address = input_line[0];
        sentence->Command = input_line[1];
        sentence->Value = input_line[2];
        sentence->Checksum = input_line[3];

        // Now verify the checksum
        if (ChecksumValid(sentence))
        {
            SentenceReceived = true;    // Yes, a valid sentence has been received!                         
        }

        // Start everything over
        input_line[0] = '\0';
        addressReceived = false;
        numBytes = 0;
    }

    return SentenceReceived;
}

boolean ChecksumValid(DataSentence * sentence)
{
    uint8_t check = (sentence->Address + sentence->Command + sentence->Value) & B01111111;

    if (check = sentence->Checksum) return true;
    else                            return false;
}

void ProcessCommand(DataSentence * sentence)
{
    // Address bytes have values greater than 127 
    switch (sentence->Command)
    {
        case 0:         
            // Motor 1 Forward
            setSpeed(1, getSpeedCommand_fromSerial(sentence->Value));
            break;

        case 1:         
            // Motor 1 Reverse
            setSpeed(1, -getSpeedCommand_fromSerial(sentence->Value));
            break;

        case 2: 
            // Set minimum voltage
            // Used to set a custom minimum voltage for the battery supplying power (essentially LVC level). If battery voltage drops below this value, 
            // motors will be turned off. This value is cleared at startup, so must be set each run. The value is sent in .2 volt 
            // increments with a command of zero corresponding to 6v, which is the minimum. Valid data is from 0 to 50 (6-16 volts). 
            // The function for converting volts to command data is Value = (desired volts-6) x 5
            
            // If valid value sent, update MinVoltage
            if (sentence->Value <= 50)
            {
                MinVoltage = 6.0 + ((float)sentence->Value * 0.2);
            }
            break;

        case 3:
            // Set maximum voltage - not implemented. Actual max voltage is approximately 16 volts.
            break;
            
        case 4: 
            // Motor 2 Forward
            setSpeed(2, getSpeedCommand_fromSerial(sentence->Value));
            break;

        case 5: 
            // Motor 2 Reverse
            setSpeed(2, -getSpeedCommand_fromSerial(sentence->Value));
            break;

        // cases  6-13 reserved for future compatibility with the equivalent Sabertooth commands 
    
        case 14: 
            // Serial Watchdog (disabled by default on each boot)
            // Values greater than 0 will enabled the watchdog. The value specifies what length of time the controller will wait for a new serial command, after which if it does not receive one it will 
            // stop the motors as a safety precaution. This can help guard against for example the communication cable becoming disconnected. 
            // The the value passed is 0 it will disable the watchdog, however the feature is disabled by default on each restart so you don't need to do anything if you don't want it. 
            // Note also the serial watchdog has no effect when the Scout is running in RC mode. 

            if (sentence->Value == 0)
            {
                SerialWatchdog = false;
            }
            else
            {
                // The length of time for the watchdog to wait is set in 100mS increments, so for example a value of 10 would equate to a watchdog timeout of 1000mS (1 second). 
                // Valid data is a number from 0 to 255 which corresponds to watchdog timeouts of 100ms (1/10 second) to 25500mS (25.5 seconds)
                // The function for converting watchdog time to command data is Value = desired time in mS / 100 
                SerialWatchdogTimeout_mS = sentence->Value * 100;
                SerialWatchdog = true;
            }
            break;

        case 15: 
            // Change baud rate. If valid value passed, re-start the hardware serial port at the selected baud rate
            switch (sentence->Value)
            {
                case BAUD_CODE_2400:    Serial.begin(2400);     break;
                case BAUD_CODE_9600:    Serial.begin(9600);     break;
                case BAUD_CODE_19200:   Serial.begin(19200);    break;
                case BAUD_CODE_38400:   Serial.begin(38400);    break;
                case BAUD_CODE_115200:  Serial.begin(115200);   break;
            }
            break;

        // cases 16-17 reserved for future compatibility with Sabertooth commands (ramping and deadband)
        // case  19 presently un-assigned

        case 20:
            // Direct fan control
            // Set fan speed to value 0-255
            setFanSpeed(sentence->Value);
            // If users sets a fan speed, we switch to ManualFanControl. They can revert to automatic control by issuing command 21
            ManualFanControl = true;
            break;

        case 21: 
            // Set fan control to "Automatic"
            // Fan control is automatic by default, so this command doesn't need to be issued unless the user initiated manual fan control (command 20) and now wants to revert to automatic.
            // Value is ignored. 
            ManualFanControl = false;
            break; 

        case 22: 
            // Set maximum current
            // Used to set a maximum current PER MOTOR (not total device current). If current on either motor exceeds the maximum level, BOTH motors will be stopped. 
            // Value is in amps and can be any value from 1 to 30. Default is 12 amps, if the user chooses to set a higher level it is highly recommended to use a cooling fan. 
            // One may wonder why we don't permit milliamp-level adjustment the way we do with voltage. The reason is that current sensing on this device is crude and 
            // setting a precision current limit isn't possible anyway. 
            
            // If valid value sent, update MaxCurrent_A
            if (sentence->Value > 0 && sentence->Value <= 30)
            {
                MaxCurrent_A = sentence->Value;
            }
            break;
            
        default:
            break;
    }
}
 
int16_t getSpeedCommand_fromSerial(uint8_t val)
{
    // Serial speed commands should be 0 to 127.
    val = constrain(val, 0, 127);

    // Now multiply by 3 to scale our speed value (from 0 to 127) to our PWM duty cycle range (0 to 381) 
    return val * 3;        
}


// -------------------------------------------------------------------------------------------------------------------------------------------------->
// MOTOR CONTROL
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// This function expects the speed value to be a number from -MOTOR_PWM_TOP to +MOTOR_PWM_TOP
void setSpeed(uint8_t motor, int16_t speed)
{
    if (speed > 0)
    {   //Forward
        if (motor == 1)
        {
            digitalWrite(M1_DirA, HIGH);
            digitalWrite(M1_DirB, LOW);
        }
        else if (motor == 2)
        {
            digitalWrite(M2_DirA, HIGH);
            digitalWrite(M2_DirB, LOW);            
        }
    }
    else 
    {   //Reverse
        if (motor == 1)
        {
            digitalWrite(M1_DirA, LOW);
            digitalWrite(M1_DirB, HIGH);
        }
        else if (motor == 2)
        {
            digitalWrite(M2_DirA, LOW);
            digitalWrite(M2_DirB, HIGH);            
        }
    }

    // Now set the PWM, always a positive number
    if (motor == 1)
    {
        M1_OCR = abs(speed); 
    }
    else if (motor == 2)
    {
        M2_OCR = abs(speed);
    }
}

void StopMotor1(void)
{
    M1_OCR = 0;
}

void StopMotor2(void)
{
    M2_OCR = 0;
}

void StopMotor(uint8_t m)
{
    if (m == 1) StopMotor1();
    if (m == 2) StopMotor2();
}

void StopMotors(void)
{
    StopMotor1();
    StopMotor2();
}

boolean MotorsRunning(void)
{
    // Return true if either motor is running
    if (M1_OCR > 0 || M2_OCR > 0) return true;
    else return false;
}

void ClearVNH2SP30Faults(void)
{
    // This should already have been done, but just to be safe
    StopMotors();

    // To clear a fault condition we must toggle the inputs to the opposite of what they are presently
    ToggleAllDirectionPins();

    // Now we wait a little bit for the fun of it, though this probably isn't necessary
    delay(200);

    // Then put them back
    ToggleAllDirectionPins();

    // Wait just a little bit more
    delay(50);
}

void ToggleAllDirectionPins()
{   
    // Set every motor direction pin to the opposite of what it is now 
    digitalWrite(M1_DirA, !digitalRead(M1_DirA));
    digitalWrite(M1_DirB, !digitalRead(M1_DirB));
    digitalWrite(M2_DirA, !digitalRead(M2_DirA));
    digitalWrite(M2_DirB, !digitalRead(M2_DirB));
}


// -------------------------------------------------------------------------------------------------------------------------------------------------->
// FAN CONTROL
// -------------------------------------------------------------------------------------------------------------------------------------------------->
void setFanSpeed(uint8_t s)
{
    FAN_OCR = s;
}

void StopFan(void)
{
    FAN_OCR = 0;
}

void setFanFullSpeed(void)
{
    setFanSpeed(255);
}

void AutoFanControl()
{
uint8_t tempComponent; 
uint8_t currentComponent; 
uint16_t maxMotorCurrent;

    // Here we set the fan automatically as a function of the measured temperature and current draw of the device. 

    // This function should be called routinely by the main loop, but AFTER the main loop has also updated the temparature and current measurements
    // by calling CheckTemp() and CheckCurrent()
    
    // Empirical testing needs to be done to characterize the fan's ability to cool the chips. One could also implement a sort of PID loop to attempt to 
    // keep the chips at a set temperature. 
    
    // Until that testing has been performed, we take the conservative approach of blasting the fan whenever we have even modest current or temperature.

    // The first thing we check is for certain error states that will always result in maximum fan speed, and yes, will even override any manual speed control
    if (ErrorState == ERROR_OVERTEMP || ErrorState == ERROR_OVERCURRENT)
    {
        setFanFullSpeed();                          // Full speed
        return;                                     // Exit
    }

    // If we are not in an error state, we permit manual control of the fan speed.
    if (ManualFanControl) return;                   // In manual mode exit without doing anything. 

    // Ok, we need to set the fan to some level automatically. We calculate two speeds based on temperature and current, 
    // and apply whichever one ends up higher. 

    // Temperature - turn the fan on starting at 40*C (~100*F) and go to full speed by 70*C (~160*F)
    if (BoardTemp >= 40)
    {
        if (BoardTemp > 70) 
        {   // Anything over 70, go to full speed
            tempComponent = MaximumFanSpeed; 
        }
        else
        {   // Between 40 and 70, map to fan speed range
            tempComponent = map(BoardTemp, 40, 70, MinimumFanSpeed, MaximumFanSpeed);
        }
    }
    else
    {   // Anything under 40, the fan can be off
        tempComponent = 0;
    }

    // Current - turn the fan on starting at 6 amps and go to full speed by 12 amps (remember motor current variables are in milliamps)
    maxMotorCurrent = max(M1_Current_mA, M2_Current_mA);    // Base it off whichever motor is currently drawing the most
    if (maxMotorCurrent >= 6000)
    {
        if (maxMotorCurrent > 12000)
        {   // Anything over 12 amps, go to full speed
            currentComponent = MaximumFanSpeed;
        }
        else
        {   // Between 6 and 12, map to fan speed range
            currentComponent = map(maxMotorCurrent, 6000, 12000, MinimumFanSpeed, MaximumFanSpeed);
        }
    }
    else
    {   // Anything under 6 amps the fan can be off
        currentComponent = 0;
    }

    // Now we set the fan speed to whichever component is higher
    setFanSpeed(max(tempComponent, currentComponent)); 
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// STATUS LED
// -------------------------------------------------------------------------------------------------------------------------------------------------->
void ToggleStatusLED(void)
{
    // Toggle the status LED - if it is on, turn off; if it is off, turn on. 
    digitalWrite(BlueLED, !digitalRead(BlueLED));
}
void StatusLEDOn(void)
{
    digitalWrite(BlueLED, HIGH);
}
void StatusLEDOff(void)
{
    digitalWrite(BlueLED, LOW);
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// ERROR LED
// -------------------------------------------------------------------------------------------------------------------------------------------------->
void ToggleErrorLED(void)
{
    // Toggle the error LED - if it is on, turn off; if it is off, turn on. 
    digitalWrite(RedLED, !digitalRead(RedLED));
}
void ErrorLEDOn(void)
{
    digitalWrite(RedLED, HIGH);
}
void ErrorLEDOff(void)
{
    digitalWrite(RedLED, LOW);
}



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

    // VNH2SP30 goes into thermal shutdown at 175*C typical (but could also shut down as low as 150*C) 
    // Thermal reset is at 135*C (meaning, it has to get back down to 135 before it turns back on)

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
// For examplel, if we call this ten times a second and we want the over-current 
// measurement to persist for 2 seconds before treating it as a fault condition, 
// we would set this to 20. 
const uint8_t MaxOverCurrentCount = 10;

// To reduce the effect of noise we run our readings through a low-pass filter.
// With an alpha of 0.9, it is like giving the current reading a weight of 10%, and past readings a weight of 90%
const float alpha = 0.9;
  
// Calculations: 
// VNH2SP30 datasheet 
    // Iout/Isense = 11370 (typical)
// Rearranged
    // Isense = Iout/11370
// ADC input
    // Vsense = Isense * Rsense (where Vsense is the input into ADC and Rsense is 1.5k)
// Combined
    // Vsense = (Iout / 11370) * 1.5k   (implies 132 millivolts reading per amp)
// Now that we know the volts reading per amp, we can calculate the amps per ADC step:
    // 5V / 1024 ADC steps / 0.132 V per A = 37 mA current per ADC count

    // Motor 1
    Current = analogRead(M1_CS) * 37;                                           // Instantaneous current in milli-amps
    if (firstPass1) { M1_Current_mA = Current; firstPass1 = false; }            // Filter
    M1_Current_mA = alpha * (float)M1_Current_mA + (1.0 - alpha) * Current;
    if (M1_Current_mA > (MaxCurrent_A * 1000)) OverCurrentCount1 += 1;          // Compare in milliamps, increment count if we are over
    else OverCurrentCount1 = 0;                                                 // If we are below the limit, reset the count

    // Motor 2
    Current = analogRead(M2_CS) * 37;                                           // Instantaneous current in milli-amps
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
// On the Scout ESC we actually don't use an input polarity diode, although we do have input polarity MOSFETs (that only really protect the VNH2SP30 chips)
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




