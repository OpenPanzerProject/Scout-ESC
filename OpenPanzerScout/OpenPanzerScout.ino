// DEFINES AND GLOBAL VARIABLES
// -------------------------------------------------------------------------------------------------------------------------------------------------->
    // Serial comms
        #define NEWLINE             '\n'        // Newline char
        #define SerialTimeout        1000       // Time to wait in milliseconds (mS) after the last serial reception, before blinking the status light
        #define SENTENCE_BYTES       4          // How many bytes in a valid sentence
        struct DataSentence {                   // Serial commands should have four bytes (plus termination). 
            uint8_t    Address = 0;             // We use a struct for convenience
            uint8_t    Command = 0;
            uint8_t    Value = 0;
            uint8_t    Checksum = 0;
        };

    // Device address
        #define AddressA            191         // Device has two possible addresses, set by dipswitch
        #define AddressB            192         // 
        uint8_t MyAddress;                      // Address of device. Set by dipswitch. 

    // Device mode
        typedef char _INPUT_MODE;               // We accept two types of input - hobby RC PWM commands, or serial
        #define INPUT_RC            0           // The actiuve mode is set by dipswitch
        #define INPUT_SERIAL        1
        _INPUT_MODE InputMode;                  // Global variable for input mode

    // Status LED states
        typedef char _LED_STATES;               // Various states the status LED can be in
        #define TURN_OFF            0           // Off 
        #define SOLID               1           // On solid
        #define BLINK_HEARTBEAT     2           // Blink slowly to indicate waiting
        #define BLINK_NORADIO       3           // Blink quickly to indicate no radio signal

        #define BLINK_RATE_HEARTBEAT    750     // How often to blink the light in heartbeat mode, in milliseconds
        #define BLINK_RATE_NORADIO      40      // Blink rate when radio signal lost

    
// PINS
// -------------------------------------------------------------------------------------------------------------------------------------------------->
    // RESERVED                    0            // D0    Input     - This is the serial RX pin
    // RESERVED                    1            // D1    Output    - This is the serial TX pin
    const byte RC_M1 =             2;           // D2    Input     - RC input for motor 1 (INT0)
    const byte RC_M2 =             3;           // D3    Input     - RC input for motor 2 (INT1)
//    const byte  =                4;           // D4    Output    - 
    const byte M2_DirA =           5;           // D5    Output    - Motor 2 INA  (Timer 0)
    const byte M2_DirB =           6;           // D6    Output    - Motor 2 INB  (Timer 0)
    const byte M1_DirA =           7;           // D7    Output    - Motor 1 INA
    const byte M1_DirB =           8;           // D8    Output    - Motor 1 INB
    const byte M1_PWM =            9;           // D9    Output    - Motor 1 PWM (Left/Turret Rotation)    (Timer 1)
    const byte M2_PWM =           10;           // D10   Output    - Motor 2 PWM (Right/Barrel Elevation)  (Timer 1)
    const byte Dip1 =             11;           // D11   Input     - Dipswitch 1
//    const byte RedLED =           12;       // D12   Output    - 
    const byte StatusLED =        13;           // D13   Output    - 
    const byte M1_CS =            14;           // A0    Input     - Motor 1 current sense (D14 = A0)
    const byte M2_CS =            15;           // A1    Input     - Motor 2 current sense (D15 = A1)
    const byte M1_EN =            16;           // A2    Input     - Motor 1 status (D16 = A2)
    const byte M2_EN =            17;           // A3    Input     - Motor 2 status (D17 = A3)        
    const byte Dip2 =             18;           // A4    Input     - Dipswitch 2    (D18 = A4)
    const byte Dip3 =             19;           // A5    Input     - Dipswitch 3    (D19 = A5)


void setup()
{
    // PINS
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // MOTOR 1
        pinMode(M1_DirA, OUTPUT);               // Output   - Direction        
        pinMode(M1_DirB, OUTPUT);               // Output   - Direction
        pinMode(M1_PWM, OUTPUT);                // Ouput    - PWM
        pinMode(M1_CS, INPUT_PULLUP);           // Input    - Current sense
        pinMode(M1_EN, INPUT_PULLUP);           // Input    - Motor 1 enable notification
        
        // MOTOR 2
        pinMode(M2_DirA, OUTPUT);               // Output   - Direction        
        pinMode(M2_DirB, OUTPUT);               // Output   - Direction
        pinMode(M2_PWM, OUTPUT);                // Ouput    - PWM
        pinMode(M2_CS, INPUT_PULLUP);           // Input    - Current sense
        pinMode(M2_EN, INPUT_PULLUP);           // Input    - Motor 1 enable notification

        // RC INPUTS
        pinMode(RC_M1, INPUT_PULLUP);           // Input    - RC PWM input
        pinMode(RC_M2, INPUT_PULLUP);           // Input    - RC PWM input

        // LED
        pinMode(StatusLED, OUTPUT);             // Output   - LED
        digitalWrite(StatusLED, LOW);           //            Initialize to off

        // DIPSWITCH
        pinMode(Dip1, INPUT_PULLUP);            // Input    - Dipswtich 1
        pinMode(Dip2, INPUT_PULLUP);            // Input    - Dipswtich 2
        pinMode(Dip3, INPUT_PULLUP);            // Input    - Dipswtich 3


    // DEVICE ADDRESS
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // The address is set by the position of dipswitch 1
//        if (DipswitchOn(1)) MyAddress = AddressA;
//        else                MyAddress = AddressB;
        MyAddress = AddressB;

       
    // SERIAL
    // ---------------------------------------------------------------------------------------------------------------------------------------------->        
//        if (DipswitchOn(2) Serial.begin(38400);
//        else               Serial.begin(115000);
        Serial.begin(38400);


    // INPUT MODE
    // ---------------------------------------------------------------------------------------------------------------------------------------------->        
//        if (DipswtichOn(3)) InputMode = INPUT_SERIAL;
//        else                InputMode = INPUT_RC;
        InputMode = INPUT_SERIAL;
              

    // TIMER 1
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
        // We use Timer 1 to generate ~21KHz PWM on Arduino pins 9 & 10. Ken Shirriff has a good tutorial on the various types of PWM possible: http://www.righto.com/2009/07/secrets-of-arduino-pwm.html
    
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
        
        #define MOTOR_PWM_TOP 381   // We will set ICR1 equal to our custom TOP value
    
        #define M1_OCR   OCR1A      // The output compare register associated with Arduino pin 9
        #define M2_OCR   OCR1B      // The output compare register associated with Arduino pin 10
        
        TCCR1A = 0xA2;              // COM1 A1:A0, COM1 B1:B0 = 10 (connect non-inverted PWM to pins), WGM1 3:0 = 1010 (Mode 10, Phase Correct PWM, TOP = ICR1)
        TCCR1B = 0x11;              // CS1 2:0 = 001 (Prescaler = 1 / no prescaler)
        TIFR1 =  0x27;              // Start off with all Timer 1 flags clear (by setting bits to 1 in TIFR1 register)
        TIMSK1 = 0x00;              // Disable Timer 1 interrupts (by clearing TIMSK1 bits). PWM is generated in hardware and we won't need interrupts enabled for it. 
        ICR1 =  MOTOR_PWM_TOP;      // Set ICR1 to our custom TOP value
        TCNT1 = 0;                  // Reset TCNT1
        OCR1A = 0;                  // Start off with duty cycle for both motors at 0 for safety.
        OCR1B = 0;                  // 

}


void loop()
{
    static _LED_STATES LedState;                            // The current state of our status LED
    static DataSentence Sentence;
    static unsigned long TimeLastSerial = 0;                // Time of last received serial data
    static unsigned long LastBlinkTime = 0;                 // When did we last toggle the status LED while blinking
    
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
                LedState = BLINK_HEARTBEAT;                 // If we haven't received data in SerialTimeout time, set the LED to blinking
            }
            else
            {
                LedState = SOLID;                           // So long as we are receiving data, keep the LED on solid
            }
            break;

        
        case INPUT_RC:
        
            break;
    }


    // Update LED
    // ---------------------------------------------------------------------------------------------------------------------------------------------->    
    switch (LedState)
    {
        case SOLID:
            digitalWrite(StatusLED, HIGH);
            break;

        case BLINK_HEARTBEAT:
            if (millis() - LastBlinkTime > BLINK_RATE_HEARTBEAT) 
            {
                ToggleStatusLED();
                LastBlinkTime = millis();
            }
            break;

        case BLINK_NORADIO:
            if (millis() - LastBlinkTime > BLINK_RATE_NORADIO) 
            {
                ToggleStatusLED();
                LastBlinkTime = millis();
            }
            break;
            
    }

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
            setSpeed(1, sentence->Value);
            break;

        case 1:         
            // Motor 1 Reverse
            setSpeed(1, -(sentence->Value));
            break;

        case 4: 
            // Motor 2 Forward
            setSpeed(2, sentence->Value);
            break;

        case 5: 
            // Motor 2 Reverse
            setSpeed(2, -(sentence->Value));
            break;

        default:
            break;
    }
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// DIPSWITCH UTILITIES
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// The dipswitches are held to ground when "ON"
// Physically these are left floating when "OFF", but we have input pullups turned on, so they are actually high. 
// In short, low is ON and high is OFF.

boolean DipSwitchOn(uint8_t switchNum)
{   
    return !DipSwitchOff(switchNum);    // If it's off, it's not on
}

boolean DipSwitchOff(uint8_t switchNum)
{   
    switch (switchNum)
    {
        case 1:     return digitalRead(Dip1);   break;
        case 2:     return digitalRead(Dip2);   break;
        case 3:     return digitalRead(Dip3);   break;
        default:    return false;
    }        
}



// -------------------------------------------------------------------------------------------------------------------------------------------------->
// MOTOR CONTROL
// -------------------------------------------------------------------------------------------------------------------------------------------------->
void setSpeed(uint8_t motor, int16_t s)
{
    // Ensure speed is valid
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

void stopMotor(uint8_t motor)
{
    if (motor == 1)
    {
        M1_OCR = 0; 
    }
    else if (motor == 2)
    {
        M2_OCR = 0;
    }
}

void stopMotors(void)
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
    digitalWrite(StatusLED, !digitalRead(StatusLED));
}

