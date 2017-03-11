
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// EXTERNAL INTERRUPTS - used to read PWM pulses on pins INT0 and INT1
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

