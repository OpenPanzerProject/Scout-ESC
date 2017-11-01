/* Scout ESC            Open Panzer dual brushed motor controller
 * Source:              openpanzer.org              
 * Authors:             Luke Middleton
 * Version:             00.90.03    (for use with board revisions 8,9,10,11)
 * Last Updated:        03/11/2017
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
        #define BAUD_CODE_57600             6           // The preceding codes are numbered identically to the codes used for Sabertooth controllers, which do not include 57600. That is why 57600 is number 6 and not number 5.

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

    // Motor handling
        int16_t M1_Speed =                  0;          // Global speed variable (signed!)
        int16_t M2_Speed =                  0;          // Global speed variable (signed!)
        boolean BrakeAtStop =           false;          // Should the motors be braked when stopped, or allowed to freewheel. Can be set through serial.
        boolean DragInnerTrack =        false;          // If both motors are running, should we attempt to prevent the slower motor from free-wheeling. Can be set through serial. 
        #define DRAG_FREQ_MS               12           // How often to drag the slower motor by braking it, in milliseconds
        #define DRAG_TIME_MS                3           // How long to apply the brake in milliseconds. The brake will be pulsed concurrently with the actual drive speed. 
                                                        // (1000/ DRAG_FREQ_MS + DRAG_TIME_MS) * DRAG_TIME_MS = the amount of time each second the motor is being braked rather than run. 
                                                        // (1000/ 12 + 3) * 3 = 200mS per second is being dragged, in other words, 1/5 of the time it is being braked, 4/5 of time it is being driven. 
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
    const byte VSense =           A6;           // A6    Input     - Battery voltage reading through divider        N/A              ADC6
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

    static uint32_t     LastDrag = 0;                           // When did we last drag the slow motor (if that feature is enabled)
    

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
        
        // Check VNH2SP30 chip diagnostic pins for fault condition
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

    // Drag the slower motor if that feature is enabled
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    if (DragInnerTrack && (millis() - LastDrag > DRAG_FREQ_MS))
    {
        DragMotor();
        LastDrag = millis();
    }
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






