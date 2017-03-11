
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// SERIAL COMMANDS - Check the serial port, do whatever we're told
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

    if (check == sentence->Checksum) return true;
    else                             return false;
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
                case BAUD_CODE_57600:   Serial.begin(57600);    break;
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

        case 23: 
            // Brake motors at stop
            // The default method of stopping the motors is simply to quit powering them. However this does nothing to keep them from free-wheeling in response to an external
            // force. If we actually want them to remain stopped, we need to "brake" them. This is done by shorting both motor leads to ground, this creates resistance inside 
            // the motor that can help keep it stationary. The motor of course can still be turned, the brake is not very strong. But it can make a difference in some applications. 
            // For tracked vehicles we will typically want this enabled since turning at slow speed is accomplished by keeping one track stationary. 
            // Pass value of true to enable, false to disable. 
            if (sentence->Value)
            {
                BrakeAtStop = true;
                // If a motor is already stopped go ahead and apply the brakes now
                if (M1_Speed == 0) BrakeMotor1();
                if (M2_Speed == 0) BrakeMotor2();
            }
            else
            {
                BrakeAtStop = false;
            }
            break;
        
        case 24: 
            // Drag inner track
            if (sentence->Value) DragInnerTrack = true;
            else                 DragInnerTrack = false;
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


