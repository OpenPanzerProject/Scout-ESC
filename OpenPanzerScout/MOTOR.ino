
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
    else if (speed < 0)
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
    else
    {   // In this case speed = 0
        // We may want to "brake" the motors so they can't freewheel (as easily). 
        // The brake motor commands set both DirA and DirB to the same state (short the motor terminals). 
        if (BrakeAtStop) BrakeMotor(motor);
    }

    // Now set the PWM, always a positive number. Also save the speed to our global variable
    if (motor == 1)
    {
        M1_Speed = speed;
        M1_OCR = abs(M1_Speed); 
    }
    else if (motor == 2)
    {
        M2_Speed = speed;
        M2_OCR = abs(M2_Speed);
    }
}

void StopMotor1(void)
{
    M1_Speed = 0;
    M1_OCR = 0;
    if (BrakeAtStop) BrakeMotor1();
}

void StopMotor2(void)
{
    M2_Speed = 0;
    M2_OCR = 0;
    if (BrakeAtStop) BrakeMotor2();
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


// While clockwise and counterclockwise rotation are accomplished by setting the DirA/DirB pins 
// to opposite states of each other, brake is accomplished by setting both to the same value. 
// If both DirA/DirB are set to 1, then the motor is "braked to VCC" (ie, both motor terminals held 
// to VCC). Alternately if they are both set to 0 then the motor is "braked to GND."
// I'm not sure if there is any reason to use one over the other and the datasheet doesn't go into
// further detail. But in testing the effect seemed to work best when braked to VCC.
void BrakeMotor1(void)
{
    // We brake to VCC since it seemed to work better with the Taigen gearboxes we tested with
    digitalWrite(M1_DirA, HIGH);
    digitalWrite(M1_DirB, HIGH);    
}

void BrakeMotor2(void)
{
    // We brake to VCC since it seemed to work better with the Taigen gearboxes we tested with
    digitalWrite(M2_DirA, HIGH);
    digitalWrite(M2_DirB, HIGH);    
}

void BrakeMotor(uint8_t m)
{
    if (m == 1) BrakeMotor1();
    if (m == 2) BrakeMotor2();
}

void BrakeMotors()
{
    BrakeMotor1();
    BrakeMotor2();
}

void ClearMotorDriverChipFaults(void)
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


// If "DragInnerTrack" is set to true, the main loop will call the DragMotor() function at routine intervals as set by the defines DRAG_FREQ_MS and 
// DRAG_TIME_MS specified at the top of the sketch. 
// 
// When this function is called, it determines if one motor is supposed to be moving slower than the other. If so, it brakes the slower motor by 
// shorting the motor leads for DRAG_TIME_MS, after which it reverts the motor polarity to what it was previously based on its current direction 
// (accomplished through calling setSpeed again). 
//
// The effect is that the slower motor is still driven at whatever speed we tell it. But interspersed with voltage to spin the motor we pulse the brake
// as well. We can't pulse the brakes too often or else the motor won't spin at all, but we need to pulse it often enough to try to prevent the motor
// from spinning faster than we want it when being dragged by an external force. Of course this whole process is inefficient and results in a higher
// current draw. If you don't have a problem steering or if you're not even controlling a differentially-steered vehicle in the first place, you don't
// need to enable this feature to begin with.         
void DragMotor()
{
    #define speedOffset 5   // Only start dragging if there is this minimum amount of difference between the track speeds

    if (M1_Speed == 0) BrakeMotor1();
    else if (abs(M1_Speed) < (abs(M2_Speed) - speedOffset))
    {
        BrakeMotor1();
        delay(DRAG_TIME_MS);        // We use straight up delays for now! 
        setSpeed(1, M1_Speed);
    }

    if (M2_Speed == 0) BrakeMotor2();
    else if (abs(M2_Speed) < (abs(M1_Speed) - speedOffset))
    {
        BrakeMotor2();
        delay(DRAG_TIME_MS);        // We use straight up delays for now! 
        setSpeed(2, M2_Speed);
    }
}




