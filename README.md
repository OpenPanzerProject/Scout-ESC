![Open Panzer Scout ESC](http://www.openpanzer.org/images/github/scout_git_r9.jpg)

# Open Panzer Scout ESC

The Scout ESC is an open-source, dual brushed-motor speed controller that accepts both standard RC inputs or logic-level serial commands. The onboard processor is an ATmega 328 and can be programmed with the Arduino IDE using a standard FTDI cable or adapter. It operates at ultrasonic frequencies (no motor whine), at voltages up to 16 volts, and is rated at 10 amps continuous per channel, but the addition of a fan can increase the current capacity. The Scout has its own onboard fan controller that can drive any standard 12 volt, 2-pin PC case fan (the 40mm size works well). An onboard thermistor also allows the processor to monitor the board temperature. 

The Scout was designed with the Open Panzer Tank Control Board in mind and requires no special setup in that application other than to plug and play. It is the perfect size for controlling even the heaviest 1/16th scale RC tanks. For more details on connecting the Scout and TCB together see the [Serial Motor Controller](http://openpanzer.org/wiki:tcb:tcbinstall:motors:serialmotor) page of the TCB Wiki. 

The Scout can also be used in many other applications including robotics. You can think of it as an open-source variant of the [Dimension Engineering Sabertooth 2x12](http://www.dimensionengineering.com/products/sabertooth2x12) or the [Pololu Qik 2s12v10](https://www.pololu.com/product/1112). The remainder of this page covers technical documentation for those wanting to use the Scout in custom projects. 

## Files Included in this Repo
There are two folders in this repository:<br />
  * **OpenPanzerScout** - This folder contains the firmware (Arduino sketch) that actually runs _on_ the Scout device.
  * **libraries\** - This folder contains a C++ library that you can use in your projects in order to _control_ the Scout via serial from another Arduino or microcontroller. 

## Additional Resources
  * For Scout board files, schematics, and  bill of materials, see the [Open Panzer Downloads page](http://openpanzer.org/downloads).
  * For more information on the Open Panzer project, see the [OpenPanzer Wiki](http://www.openpanzer.org/wiki).
  * For interfacing the Scout with the Open Panzer Tank Control Board (TCB), see the [Serial Motor Controller page](http://openpanzer.org/wiki/doku.php?id=wiki:tcb:tcbinstall:motors:serialmotor) in the TCB wiki.
  * To discuss the project, feel free to join the [Open Panzer Community](http://openpanzer.org/forum/index.php?action=forum).
<br />
<br />
<br />
![Scout ESC board layout](http://www.openpanzer.org/images/github/scout_layout.jpg)
<br />
<br />

# Functional Overview 

Battery power should be connected to the screw terminals between the motor outputs. There is a secondary power _output_ useful for providing battery voltage to a secondary device, such as the TCB. Do not connect the battery to the power _output_! It is not polarity-protected. 

An optional physical switch can be connected to GND and MISO pins of the ICSP connector, see the diagram above. When this switch is closed both motors will be immediately stopped regardless of the inputs. This emergency stop switch should be implemented when the Scout is used in any device that could endanger humans. 

The Scout accepts either standard RC inputs (from any hobby receiver) or standard TTL serial commands at 38400 baud (5v max). 

On boot the Scout blinks the red LED slowly until a signal is detected. Whichever type of signal is detected first is the communication protocol the Scout will use until the next reboot, and any signal on the alternate input will be ignored. 

During normal operation the blue LED indicates the status of the incoming signal. If a fault is detected the motors will be immediately stopped and the red LED will flash a numeric sequence indicating the reason. 

<br />
![Scout LED Key](http://www.openpanzer.org/images/github/scout_ledpatterns.jpg)
<br />
<br />


# RC Operation

RC operation requires no special setup, just connect RC input 1 and 2 directly to a standard hobby receiver on any output that will operate a servo. RC input 1 controls motor 1, and RC input 2 controls motor 2. 

The accepted pulse-width range is 1000-2000 microseconds. A pulse-width of 1500 uS (plus/minus a small deadband amount) equates to motor stopped. 1000 uS equals full speed reverse and 2000 uS full speed forward. Pulse-widths beyond those bounds will not result in any faster movement. 

It is recommended to begin with your radio channels set to 100% travel with no subtrim, and then use trim if needed if the motor doesn't stop when the sticks are centered. 

If an RC channel becomes disconnected the motor it controls will be stopped. If both channels become disconnected or no signal is received on both channels, the Scout will stop both motors and flash the blue LED rapidly to indicate radio signal lost. 

# Serial Operation

The Scout uses the same packetized serial protocol used by the [Dimension Engineering](https://www.dimensionengineering.com/) Sabertooth line of motor controllers. Not all commands from the Sabertooth are supported, but the ones that are match exactly. Serial is TTL or "logic level", meaning signals into the Scout should remain within 0-5 volts. If you wish to use a true RS-232 device such as the serial port on a computer, you must add an RS-232 level shifter on the serial input line first. Level shifters based on the MAX232 chip are inexpensive and widely available from SparkFun, eBay, and elsewhere.  

The baud rate on the Scout is fixed at 38400 but can be changed by modifying the sketch, it is the first definition at the top of the file. Serial is in the 8N1 protocol: each data byte consists of 8 bits, no parity bit, and 1 stop bit. This is the most common configuration for serial communication today, and is probably already the default on whatever master device you are using. 

The serial "packet" is always 4 bytes long: it starts with an address byte, then a command byte, a data byte, and a 7 bit checksum. 

The Scout recognizes two addresses, set by the Address Select switch on the device:<br />
Address A = 131<br />
Address B = 132<br />

In other words, up to 2 Scouts can be used on the same serial line without interference, if each is set to a different address. Note these addresses are within the larger range recognized also by Sabertooth devices (128-135). But as long as you set your Sabertooth to a different address, you can control a Scout and Sabertooth on the same serial line. 

## Command Reference
Here are the command bytes recognized by the Scout. Commands below 14 match the equivalent Sabertooth commands (though not all Sabertooth commands are implemented). Commands 14 and above are specific to the Scout.  

**0 &nbsp;&nbsp;&nbsp;&nbsp; Motor 1 Forward (0x00, b00000000)**<br />
This command will spin Motor 1 forward by the speed passed in the data byte. Valid speed data ranges from 0 to 127 with 0 being off (stopped) and 127 being full speed forward.

**1 &nbsp;&nbsp;&nbsp;&nbsp; Motor 1 Reverse (0x01, b00000001)**<br />
This command will spin Motor 1 reverse by the speed passed in the data byte. Valid speed data ranges from 0 to 127 with 0 being off (stopped) and 127 being full speed forward.

**2 &nbsp;&nbsp;&nbsp;&nbsp; Set Minimum Voltage (0x02, b00000010)**<br />
By default the Scout will stop the motors if the input voltage dips below 6 volts, but you can set a custom low-voltage cutoff to any level you want between 6 and 16 volts by 0.2 volt increments. Values are not saved on reboot, so must be set with each power cycle. The data byte that follows this command must specify the desired voltage level. The function for converting desired volts to data byte is:<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Data Byte = (Desired Volts - 6) x 5<br />
The valid range for the data byte is therefore 0 through 50. A data byte of 0 will equal a cutoff of 6 volts, which is the minimum. A data byte of 50 will equal a cutoff of 16 volts, which is the maximum. 

**3 &nbsp;&nbsp;&nbsp;&nbsp; Not Implemented (0x03, b00000011)**<br />
On the Sabertooth command 3 is used to set the maximum voltage. On the Scout the maximum voltage is hard-coded to 16 volts, which is the limit of the onboard driver ICs. 

**4 &nbsp;&nbsp;&nbsp;&nbsp; Motor 2 Forward (0x04, b00000100)**<br />
This command will spin Motor 2 forward by the speed passed in the data byte. Valid speed data ranges from 0 to 127 with 0 being off (stopped) and 127 being full speed forward.

**5 &nbsp;&nbsp;&nbsp;&nbsp; Motor 2 Reverse (0x05, b00000101)**<br />
This command will spin Motor 2 reverse by the speed passed in the data byte. Valid speed data ranges from 0 to 127 with 0 being off (stopped) and 127 being full speed forward.

**6-13 &nbsp;&nbsp;&nbsp;&nbsp; Not Implemented**<br />
Not implemented and reserved for future compatible functionality with the equivalent Sabertooth commands. 

**14-19 &nbsp;&nbsp;&nbsp;&nbsp; Not Implemented**<br />
Not implemented and reserved for unknown.

**20 &nbsp;&nbsp;&nbsp;&nbsp; Direct Fan Control (0x14, b00001110)**<br />
By default the fan output is controlled automatically by the Scout in response to temperature changes. However it is also possible to control it directly from your host application by using this command followed by a speed byte with a value from 0 to 255, with 0 being off and 255 being fully on. As soon as this command is received, internal Scout control is disabled and the fan output will maintain whatever speed you specify until you specify a new speed or revert control back to the Scout (see below). You would not have to plug a fan into this output, you could use it as a third, uni-directional (no reverse) speed control for a low current motor. Maximum current on this output should be kept to no more than 1 amp. Note: voltage on the fan output is equal to the input battery voltage. 

**21 &nbsp;&nbsp;&nbsp;&nbsp; Set Fan to Automatic Control (0x15, b00001111)**<br />
This command is typically not necessary because by default the Scout powers up with automatic fan control already enabled. But if you issue direct fan control commands (see above) and then in the same session want to return automatic control to the Scout, you can do so by sending this command. The data byte should still be included but its value will be ignored. 

**22 &nbsp;&nbsp;&nbsp;&nbsp; Set Maximum Current (0x16, b00010000)**<br />
By default the Scout will turn off the motors when either one of them exceeds a current draw of 12 amps, but you can increase or decrease that threshold by issuing this command followed by a data byte with a value between 1 and 30 to represent a current limit of 1 to 30 amps //per motor// (not total device current). Although the limit is per motor, if either motor exceeds the limit, both motors are disabled for safety. The Scout will re-enable them after some length of time has passed, typically about 5 seconds. Note: if you decide to set a current threshold greater than the default it is highly recommended you implement a cooling fan and heatsinks on the driver chips, otherwise you will run into shutdowns due to over-temperature conditions.  

**23 &nbsp;&nbsp;&nbsp;&nbsp; Enable Serial Watchdog (0x17, b00010001)**<br />
The Serial Watchdog is disabled by default. When enabled, it functions as a safety feature that disables the motors if a serial command is not received within a user-defined length of time. This would protect for example against the case where the communication cable between the Scout and the host device becomes disconnected. Without the Serial Watchdog the Scout would just keep turning the motors at the same speed and direction as the last command. But with the Serial Watchdog enabled, the time allotment would expire and the watchdog would shut everything down. This is a useful feature but it also requires code on the host device to continuously send commands even when those commands don't change, this is why it is disabled by default. 

To enable the watchdog send this command followed by a data byte that specifies the length of the timeout in 10 mS increments. A data value of 0 corresponds to 50mS (the minimum timeout), while a data value of 255 corresponds to a timeout of 2600mS (2.60 seconds, the maximum). The function for converting watchdog time to command data is<br />
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Data Byte = (Desired Watchdog Time in mS - 50) / 10 

**24 &nbsp;&nbsp;&nbsp;&nbsp; Disable Serial Watchdog (0x18, b00011000)**<br />
This command disables the serial watchdog. Since it is disabled by default on power-up, the only time this command would be needed is if you first enabled the watchdog by issuing Command 23 and then later during the same session wanted to disable it.



# Specifications

<html><table>
<tr>
    <td width="40%">Input voltage:</td>
    <td width="60%">6 - 16 volts</td>
</tr>
<tr>
    <td>Operating current:</td>
    <td>10 amps per channel continuous without fan<br />20 amps peak</td>
</tr>
<tr>
    <td>Motor PWM:</td>
    <td>21 kHz</td>
</tr>
<tr>
    <td>RC Inputs:</td>
    <td>Standard 1000-2000 uS pulsewidth<br />(1500 uS = motor stopped)</td>
</tr>
<tr>
    <td>Serial Input:</td>
    <td>38400 baud; 8 data bits, no parity, one stop bit; TTL level (5v max)</td>
</tr>
<tr>
    <td>Dimensions (L x W):</td>
    <td>2.6" x 1.9" / 65mm x 47mm</td>
</tr>
<tr>
    <td>Mounting holes:</td>
    <td>1.57" / 40mm (use 4-40 or 3mm screws)</td>
</tr>
</table></html>



# License
Firmware for the Scout ESC is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License version 3 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

For more specific details see [http://www.gnu.org/licenses](http://www.gnu.org/licenses), the [Quick Guide to GPLv3.](http://www.gnu.org/licenses/quick-guide-gplv3.html) and the [copying.txt](https://github.com/OpenPanzerProject/Scout-ESC/blob/master/COPYING.txt) file in the codebase.

The GNU operating system which is under the same license has an informative [FAQ here](http://www.gnu.org/licenses/gpl-faq.html).

