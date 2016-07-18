// DEFINES AND GLOBAL VARIABLES
// -------------------------------------------------------------------------------------------------------------------------------------------------->
    // Serial comms
        #define SerialTimeout               1000       // Time to wait in milliseconds (mS) after the last serial reception, before blinking the status light
        #define SENTENCE_BYTES              4          // How many bytes in a valid sentence
        struct DataSentence {                          // Serial commands should have four bytes (plus termination). 
            uint8_t    Address =            0;         // We use a struct for convenience
            uint8_t    Command =            0;
            uint8_t    Value =              0;
            uint8_t    Checksum =           0;
        };

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
        _RC_STATE RC_State = 0;
        
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
        _INPUT_MODE InputMode;                          // Global variable for input mode

    // Status LED states
        typedef char _LED_STATES;                       // Various states the status LED can be in
        #define TURN_OFF                    0           // Off 
        #define SOLID                       1           // On solid
        #define BLINK_HEARTBEAT             2           // Blink slowly to indicate waiting
        #define BLINK_NORADIO               3           // Blink quickly to indicate no radio signal

        #define BLINK_RATE_HEARTBEAT        750         // How often to blink the light in heartbeat mode, in milliseconds
        #define BLINK_RATE_NORADIO          40          // Blink rate when radio signal lost

    // Thermistor/temp-sensing
        #define TEMP_MEASURE_FREQ_MS        200         // How often to measure temperature in milliseconds
        // Thermistor is Murata NXP18XH103F03RB
        #define THERMISTOR_NOM_RES          10000       // Resistance at nominal temperature in ohms
        #define THERMISTOR_NOM_TEMP         25          // Nominal temperature
        #define THERMISTOR_BCONSTANT        3455        // Beta coefficient of thermistor from datasheet (25-100*C range)
        #define TEMP_SERIES_RESISTOR        10000       // What is the value in ohms of the series resistor inline with the thermistor (R22 on rev 8 of the Scout board)
        
        
    
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
    const byte M1_EN =             8;           // D8    Input     - Motor 1 status (EN)                            PB0
    const byte M2_PWM =            9;           // D9    Output    - Motor 2 PWM (Left/Turret Rotation)             PB1              Timer 1 / PWM OC1A
    const byte M1_PWM =           10;           // D10   Output    - Motor 1 PWM (Right/Barrel Elevation)           PB2              Timer 1 / PWM OC1B
    const byte Fan_PWM =          11;           // D11   Output    - Fan motor PWM                                  PB3              Timer 2 / PWM OC2A / MOSI
    //                            12;           // D12             - MISO, used for ISP                             PB4             
    const byte RedLED =           13;           // D13   Output    - Red LED - used to indicate errors              PB5              SCK
    const byte M1_CS =            14;           // A0    Input     - Motor 1 current sense                          PC0              ADC0
    const byte TempSense =        15;           // A1    Input     - Thermistor                                     PC1              ADC1
    const byte M2_DirB =          16;           // A2    Output    - Motor 2 direction B                            PC2              ADC2
    const byte BlueLED =          17;           // A3    Output    - Blue LED - used to indicate status             PC3              ADC3
    const byte M2_CS =            18;           // A4    Input     - Motor 2 current sense                          PC4              ADC4
    const byte M2_EN =            19;           // A5    Input     - Motor 2 status (EN)                            PC5              ADC5
    
    // ADC6 and ADC7 are not broken out in the DIP version of the 328 chip and are therefore not referenced by the original Arduino Duemilanove board. 
    // They are however broken out on the TQFP chip and if we set our board type to Nano we can reference the names A6 and A7 in Arduino code. 
    // But we managed to not need them... they are the only two pins left on the chip that are not used. 

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
        pinMode(M1_EN, INPUT_PULLUP);           // Input    - Motor 1 enable notification
        
        // MOTOR 2
        pinMode(M2_DirA, OUTPUT);               // Output   - Direction        
        pinMode(M2_DirB, OUTPUT);               // Output   - Direction
        pinMode(M2_PWM, OUTPUT);                // Ouput    - PWM
        pinMode(M2_CS, INPUT);                  // Input    - Current sense
        pinMode(M2_EN, INPUT_PULLUP);           // Input    - Motor 1 enable notification

        // FAN
        pinMode(Fan_PWM, OUTPUT);               // Output   - Fan PWM

        // TEMP SENSE
        pinMode(TempSense, INPUT);              // Input    - Thermistor

        // RC INPUTS
        pinMode(RC1, INPUT_PULLUP);             // Input    - RC PWM input
        pinMode(RC2, INPUT_PULLUP);             // Input    - RC PWM input

        // LEDs
        pinMode(BlueLED, OUTPUT);               // Output   - Blue LED
        digitalWrite(BlueLED, LOW);             //            Initialize to off
        pinMode(RedLED, OUTPUT);                // Output   - Red LED
        digitalWrite(RedLED, LOW);              //            Initialize to off

        // SWITCH
        pinMode(AddressSwitch, INPUT_PULLUP);   // Input    - Address selection switch



    // DEVICE ADDRESS
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // The address is set by the position of dipswitch 1
        digitalRead(AddressSwitch) ? MyAddress = AddressA : MyAddress = AddressB;

       
    // SERIAL
    // ---------------------------------------------------------------------------------------------------------------------------------------------->        
        Serial.begin(38400);


    // INPUT MODE
    // ---------------------------------------------------------------------------------------------------------------------------------------------->        
        InputMode = INPUT_UNKNOWN;      // Start in unknwon state
             

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
        // Timer 2 is an 8-bit timer which we will use in Fast PWM Mode with TOP set to 0xFF and a prescaler of 8, that will give us a PWM frequency of ~8KHz. That will be plenty quiet on a fan. 
    
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
    static _LED_STATES  StatusLedState;                    // The current state of our Status LED
    static _LED_STATES  ErrorLedState;                     // The current state of our Error LED
    static DataSentence Sentence;
    static uint32_t     TimeLastRCCheck = 0;               // Time we last did a watchdog check on the RC signal
    static uint32_t     TimeLastSerial = 0;                // Time of last received serial data
    static uint32_t     LastBlinkTime_Status = 0;          // When did we last toggle the Status LED while blinking
    static uint32_t     LastBlinkTime_Error = 0;           // When did we last toggle the Error  LED while blinking
    static uint32_t     LastTempMeasure = 0;               // When did we last measure the board temperature
    uint32_t            uS;
    
    // Read and Process Input
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    switch (InputMode)
    {
        case INPUT_SERIAL:
            if (ReadData(&Sentence)) 
            {
                ProcessCommand(&Sentence);                  // Do whatever we're told
                TimeLastSerial = millis();                  // Save the time
            }
            
            if (millis() - TimeLastSerial > SerialTimeout) 
            {
                StatusLedState = BLINK_HEARTBEAT;           // If we haven't received data in SerialTimeout time, set the LED to blinking
            }
            else
            {
                StatusLedState = SOLID;                     // So long as we are receiving data, keep the LED on solid
            }
            break;

        
        case INPUT_RC:
            switch (RC_State)
            {
                case RC_SIGNAL_ACQUIRE:
                    StatusLedState = BLINK_HEARTBEAT;        // Acquiring gives us the regular heartbeat, but this state won't last long before becoming either synched or lost
                    break;

                case RC_SIGNAL_SYNCHED:
                    StatusLedState = SOLID;                  // So long as we are receiving data, keep the LED on solid
                    
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
                    StatusLedState = BLINK_NORADIO;          // If we had a radio signal, but then lost it, blink the status LED faster
                    break;
                   
            }
            break;

        case INPUT_UNKNOWN:
            // In this case we have yet to identify a serial or RC command, so check both
            if (RC_State == RC_SIGNAL_SYNCHED)
            {
                InputMode = INPUT_RC;                       // Set mode to RC
                StatusLedState = SOLID;
            }
            else if (ReadData(&Sentence)) 
            {
                InputMode = INPUT_SERIAL;                   // Set mode to Serial
                DisableRCInterrupts();                      // Don't check the RC inputs anymore
                ProcessCommand(&Sentence);                  // Do whatever we're told
                TimeLastSerial = millis();                  // Save the time
                StatusLedState = SOLID;
            }
            // While in unknown mode, blink the Red LED slowly - once we exit unknown mode, turn it offz
            InputMode == INPUT_UNKNOWN ? ErrorLedState = BLINK_HEARTBEAT : ErrorLedState = TURN_OFF;
            break;
    }


    // Update Status LED
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    switch (StatusLedState)
    {
        case SOLID:
            digitalWrite(BlueLED, HIGH);
            break;

        case TURN_OFF:
            digitalWrite(BlueLED, LOW);
            break;
            
        case BLINK_HEARTBEAT:
            if (millis() - LastBlinkTime_Status > BLINK_RATE_HEARTBEAT) 
            {
                ToggleStatusLED();
                LastBlinkTime_Status = millis();
            }
            break;

        case BLINK_NORADIO:
            if (millis() - LastBlinkTime_Status > BLINK_RATE_NORADIO) 
            {
                ToggleStatusLED();
                LastBlinkTime_Status = millis();
            }
            break;
    }

    // Update Error LED
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    switch (ErrorLedState)
    {
        case SOLID:
            digitalWrite(RedLED, HIGH);
            break;

        case TURN_OFF:
            digitalWrite(RedLED, LOW);
            break;

        case BLINK_HEARTBEAT:
            if (millis() - LastBlinkTime_Error > BLINK_RATE_HEARTBEAT) 
            {
                ToggleErrorLED();
                LastBlinkTime_Error = millis();
            }
            break;

        case BLINK_NORADIO:
            if (millis() - LastBlinkTime_Error > BLINK_RATE_NORADIO) 
            {
                ToggleErrorLED();
                LastBlinkTime_Error = millis();
            }
            break;
    }


    // Check the temperature
    // ---------------------------------------------------------------------------------------------------------------------------------------------->
    if (millis() - LastTempMeasure > TEMP_MEASURE_FREQ_MS)
    {
        CheckTemp();
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

        case 4: 
            // Motor 2 Forward
            setSpeed(2, getSpeedCommand_fromSerial(sentence->Value));
            break;

        case 5: 
            // Motor 2 Reverse
            setSpeed(2, -getSpeedCommand_fromSerial(sentence->Value));
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

void setSpeed_fromRC(uint8_t motor, uint16_t s)
{
    // In this case we are being given an RC pulsewidth, and we scale it to 
    s = constrain(s, -127, 127);

    // Now multiply by 3 to scale our speed number (from 0 to 127) to our PWM duty cycle range (0 to 381) 
    int16_t speed = s * 3;         
    
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
    M1_OCR = 0;
    M2_OCR = 0;
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// STATUS LED
// -------------------------------------------------------------------------------------------------------------------------------------------------->

void ToggleStatusLED(void)
{
    // Toggle the status LED - if it is on, turn off; if it is off, turn on. 
    digitalWrite(BlueLED, !digitalRead(BlueLED));
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// ERROR LED
// -------------------------------------------------------------------------------------------------------------------------------------------------->

void ToggleErrorLED(void)
{
    // Toggle the error LED - if it is on, turn off; if it is off, turn on. 
    digitalWrite(RedLED, !digitalRead(RedLED));
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// THERMISTOR - TEMP SENSING
// -------------------------------------------------------------------------------------------------------------------------------------------------->
void CheckTemp()
{
    float reading;
    
    // Read thermistor voltage divider
    reading = analogRead(TempSense);
 
    // Convert value to resistance
    reading = (1023 / reading) - 1;
    reading = TEMP_SERIES_RESISTOR / reading;
    
    //  Serial.print("Thermistor resistance "); 
    //  Serial.println(reading);

    // Thanks to LadyAda for the handy thermistor tutorial: 
    // https://learn.adafruit.com/thermistor/using-a-thermistor
    float steinhart;
    steinhart = (float)reading / THERMISTOR_NOM_RES;    // (R/Ro)
    steinhart = log(steinhart);                         // ln(R/Ro)
    steinhart /= THERMISTOR_BCONSTANT;                  // 1/B * ln(R/Ro)
    steinhart += 1.0 / (THERMISTOR_NOM_TEMP + 273.15);  // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invert
    steinhart -= 273.15;                                // convert to C
    
//    Serial.print("Temperature "); 
//    Serial.print(steinhart);
//    Serial.println(" *C");

    // VNH2SP30 goes into thermal shutdown at 175*C typical (150-200 min/max)
    // Thermal reset is at 135*C (meaning, it has to get back down to 135 before it turns back on)

}
