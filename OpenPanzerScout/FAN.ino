
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

