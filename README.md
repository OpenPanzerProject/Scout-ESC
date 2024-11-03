![Open Panzer Scout ESC](https://www.openpanzer.org/images/github/scout_rev10_git.jpg)

# Open Panzer Scout ESC

The Scout ESC is an open-source, dual brushed-motor speed controller that accepts both standard RC inputs or logic-level serial commands. The onboard processor is an ATmega 328 and can be programmed with the Arduino IDE using a standard FTDI cable or adapter (precompiled firmwares can also be downloaded and flashed to the Scout using the [OP Config program](http://openpanzer.org/downloads)). It operates at ultrasonic frequencies (no motor whine), at voltages up to 16 volts, and is rated at 12 amps continuous per motor without additional heatsinking, but the addition of a fan can increase the current capacity (30 amps max, requires serial commands to increase overcurrent limit beyond 12 amps). The Scout has its own onboard fan controller that can drive any standard 12 volt, 2-pin PC case fan (the 40mm size works well). The board has over-temperature protection, over and under-voltage protection, over-current protection, and is reverse-polarity protected. Current and voltage limits can be adjusted via serial commands. 

The Scout was designed with the Open Panzer Tank Control Board in mind and requires no special setup in that application other than to plug and play. It is the perfect size for controlling even the heaviest 1/16th scale RC tanks. For more details on connecting the Scout and TCB together see the [Serial Motor Controller](https://openpanzer.org/wiki/doku.php?id=wiki:tcb:tcbinstall:motors:serialmotor) page of the TCB Wiki. 

However the Scout can also be used in many other applications including robotics. You can think of it as an open-source variant of the [Dimension Engineering Sabertooth 2x12](http://www.dimensionengineering.com/products/sabertooth2x12) or the [Pololu Qik 2s12v10](https://www.pololu.com/product/1112). The remainder of this page covers technical documentation for those wanting to use the Scout in custom projects. 

## Files Included in this Repo
There are two folders in this repository:<br />
  * **OpenPanzerScout** - This folder contains the firmware (Arduino sketch) that actually runs _on_ the Scout device.
  * **libraries** - This folder contains a C++ library that you can use in your projects in order to _control_ the Scout via serial from another Arduino or microcontroller. Or, just control it directly via RC signals from any RC receiver.

## Additional Resources
  * [Eagle Board and Schematic](https://www.openpanzer.org/secure_downloads/scout/eagle/ScoutESC_v1_r11.zip) (zip)
  * [Printable Schematic](https://www.openpanzer.org/secure_downloads/scout/eagle/ScoutESC_v1_r11_Schematic.pdf) (8.5x11 PDF)
  * Drawing: [Board Top](https://www.openpanzer.org/images/downloadpage/scout_top.png) - [Board Bottom](http://www.openpanzer.org/images/downloadpage/scout_bottom.png)
  * Bill of Materials - [PDF](https://www.openpanzer.org/secure_downloads/scout/bom/Scout_ESC_BOM.pdf) - [Excel](https://www.openpanzer.org/secure_downloads/scout/bom/Scout_ESC_BOM.xls)
  * [Bare boards at OSH Park](https://oshpark.com/shared_projects/Ntf3RzTZ) - [Solder paste stencils at OSH Stencils](https://www.oshstencils.com/#projects/658838f2dd36906dd623376fcd23a949230f5a80)
  * [Gerber files](https://www.openpanzer.org/secure_downloads/scout/eagle/ScoutESC_v1_r11_Gerber_SEEED.zip) formatted for [SEEED Studio PCB service](https://www.seeedstudio.com/fusion_pcb.html)			   
  * For more information on the Open Panzer project, see the [OpenPanzer Wiki](https://openpanzer.org/wiki/).
  * For interfacing the Scout with the Open Panzer Tank Control Board (TCB), see the [Serial Motor Controller page](https://openpanzer.org/wiki/doku.php?id=wiki:tcb:tcbinstall:motors:serialmotor) in the TCB wiki.
  * To discuss the project, feel free to join the [Open Panzer Community](https://openpanzer.org/forum/index.php?action=forum).
  
## For Developers
The sketch located in the OpenPanzerScout folder can be compiled in the Arduino IDE. First select "Arduino/Genuino Uno" as the board selection from the Tools menu. The sketch can then be flashed to the Scout using an FTDI cable or something like [Adafruit's FTDI Friend](https://www.adafruit.com/product/284) adapter. 

If you are building your own board from scratch you will first need to load the bootloader before flashing firmware. This must be done using the ISP connector and a device such as the AVRISP mkII or the USBASP. The bootloader used on the Scout is the Arduino/Genuino Uno bootloader (in the Arduino IDE, under the Tools menu, go to Board, and select "Arduino/Genuino Uno" from the Arduino AVR Boards category. Next, provide power to the Scout, make your ISP connection, then again under the Tools menu of the IDE, select Burn Bootloader).  
<br />
<br />
<br />
![Scout ESC board layout](https://www.openpanzer.org/images/github/scout_layout_r11.jpg)
<br />
<br />

# Functional Overview 

Battery power should be connected to the screw terminals between the motor outputs. There is a secondary power _output_ useful for providing battery voltage to a secondary device, such as the TCB. Do not connect the battery to the power _output_! It is not polarity-protected. 

An optional physical switch can be connected to GND and MISO pins of the ICSP connector, see the diagram above. When this switch is closed both motors will be immediately stopped regardless of the inputs. This emergency stop switch should be implemented when the Scout is used in any device that could endanger humans. 

The Scout accepts either standard RC inputs (from any hobby receiver) or standard TTL serial commands at 38400 baud (5v max). 

On boot the Scout blinks the red LED slowly until a signal is detected. Whichever type of signal is detected first is the communication protocol the Scout will use until the next reboot, and any signal on the alternate input will be ignored. 

During normal operation the blue LED indicates the status of the incoming signal. If a fault is detected the motors will be immediately stopped and the red LED will flash a numeric sequence indicating the reason. 
<br />
<br />
![Scout LED Key](https://www.openpanzer.org/images/github/scout_ledpatterns.jpg)
<br />
<br />


# RC Operation

RC operation requires no special setup, just connect RC input 1 and 2 directly to a standard hobby receiver on any output that will operate a servo. RC input 1 controls motor 1, and RC input 2 controls motor 2. 

The accepted pulse-width range is 1000-2000 microseconds. A pulse-width of 1500 uS (plus/minus a small deadband amount) equates to motor stopped. 1000 uS equals full speed reverse and 2000 uS full speed forward. Pulse-widths beyond those bounds will not result in any faster movement. 

It is recommended to begin with your radio channels set to 100% travel with no subtrim, and then use trim if needed if the motor doesn't stop when the sticks are centered. 

If an RC channel becomes disconnected the motor it controls will be stopped. If both channels become disconnected or no signal is received on both channels, the Scout will stop both motors and flash the blue LED rapidly to indicate radio signal lost. 

# Serial Operation

The Scout uses the same packetized serial protocol used by the [Dimension Engineering](https://www.dimensionengineering.com/) Sabertooth line of motor controllers. Not all commands from the Sabertooth are supported, but the ones that are match exactly. Serial is TTL or "logic level", meaning signals into the Scout should remain within 0-5 volts. If you wish to use a true RS-232 device such as the serial port on a computer, you must add an RS-232 level shifter on the serial input line first. Level shifters based on the MAX232 chip are inexpensive and widely available from SparkFun, eBay, and elsewhere.  

The baud rate on the Scout defaults to 38400 but can be changed by modifying the sketch or via serial commands. Serial is in the 8N1 protocol: each data byte consists of 8 bits, no parity bit, and 1 stop bit. This is the most common configuration for serial communication today, and is probably already the default on whatever master device you are using. 

The serial "packet" is always 4 bytes long: it starts with an address byte, then a command byte, a data (value) byte, and a 7 bit checksum. 

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

**3 &nbsp;&nbsp;&nbsp;&nbsp; Set Maximum Voltage (0x03, b00000011)**<br />
Used to change the maximum voltage above which the Scout will stop the motors. Defaults to 16 volts and in fact 16 volts is the absolute maximum on all board revisions up to 11 (later designs may increase this, the VNH5019 driver chips can handle up to 24 volts but other components on the current board design are capped at 16). Values are not saved on reboot, so must be set with each power cycle. The data byte that follows this command must specify the desired voltage level in 0.2 volt increments. The function for converting desired volts to data byte is:<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Data Byte = (Desired Volts) x 5<br />
The serial processor can accept data values from 30 to 140 (equating to voltages from 6 to 28) but they will be constrained to an absolute maximum which again for the current board design is capped at 16 volts (in other words, any data byte over 80 will be ignored). 

**4 &nbsp;&nbsp;&nbsp;&nbsp; Motor 2 Forward (0x04, b00000100)**<br />
This command will spin Motor 2 forward by the speed passed in the data byte. Valid speed data ranges from 0 to 127 with 0 being off (stopped) and 127 being full speed forward.

**5 &nbsp;&nbsp;&nbsp;&nbsp; Motor 2 Reverse (0x05, b00000101)**<br />
This command will spin Motor 2 reverse by the speed passed in the data byte. Valid speed data ranges from 0 to 127 with 0 being off (stopped) and 127 being full speed forward.

**6-13 &nbsp;&nbsp;&nbsp;&nbsp; Not Implemented**<br />
Not implemented and reserved for future compatibility with the equivalent Sabertooth commands. 

**14 &nbsp;&nbsp;&nbsp;&nbsp; Serial Watchdog (0x0E, b00001110)**<br />
The Serial Watchdog is disabled by default. When enabled, it functions as a safety feature that disables the motors if a serial command is not received within a user-defined length of time. This would protect for example against the case where the communication cable between the Scout and the host device becomes disconnected. Without the Serial Watchdog the Scout would just keep turning the motors at the same speed and direction as the last command. But with the Serial Watchdog enabled, the time allotment would expire and the watchdog would shut the motors down. This is a useful feature but it also requires code on the host device to continuously send commands even when those commands don't change, this is why it is disabled by default. 

To enable the watchdog send this command followed by a data byte that specifies the length of the timeout in 100 mS increments. A data value of 1 corresponds to 100mS (the minimum timeout), while a data value of 255 corresponds to a timeout of 25500mS (25.5 seconds, the maximum). The function for converting watchdog time to command data is<br />
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Data Byte = ( Desired Watchdog Time in mS / 10 )

To disable the watchdog send this command with a data byte of 0. Since this feature is disabled by default on power-up, the only time you would need to issue a specific disable command is if you first enabled the watchdog and then later during the same session wanted to disable it.

**15 &nbsp;&nbsp;&nbsp;&nbsp; Change Baud Rate (0x0F, b00001111)**<br />
When power is applied the Scout will always initialize its serial port to a baud rate of 38400. This takes all the guesswork out of knowing what rate to use to communicate with it. However you can tell it to switch to a new baud rate by issuing this command followed by one of the following data bytes: <br />
1: 2400<br />
2: 9600<br />
3: 19200<br />
4: 38400<br />
5: 115200<br />

Upon receiving one of these commands the Scout will switch its serial port to the new baud rate and from then on will only accept commands at the new rate. The setting is _not_ remembered on shutdown, and upon next boot the Scout will re-initialize to 38400 once again. 

**16-19 &nbsp;&nbsp;&nbsp;&nbsp; Not Implemented**<br />
Not implemented and reserved for future expansion or compatibility with equivalent Sabertooth commands. 

**20 &nbsp;&nbsp;&nbsp;&nbsp; Direct Fan Control (0x14, b00010100)**<br />
By default the fan output is controlled automatically by the Scout in response to temperature changes. However it is also possible to control it directly from your host application by using this command followed by a speed byte with a value from 0 to 255, with 0 being off and 255 being fully on. As soon as this command is received, internal Scout control is disabled and the fan output will maintain whatever speed you specify until you specify a new speed or revert control back to the Scout (see below). You would not have to plug a fan into this output, you could use it as a third, uni-directional (no reverse) speed control for a low current motor. Maximum current on this output should be kept to no more than 1 amp. Note: voltage on the fan output is equal to the input battery voltage. 

**21 &nbsp;&nbsp;&nbsp;&nbsp; Set Fan to Automatic Control (0x15, b00010101)**<br />
This command is typically not necessary because by default the Scout powers up with automatic fan control already enabled. But if you issue direct fan control commands (see above) and then in the same session want to return automatic control to the Scout, you can do so by sending this command. The data byte should still be included but its value will be ignored. 

**22 &nbsp;&nbsp;&nbsp;&nbsp; Set Maximum Current (0x16, b00010110)**<br />
By default the Scout will turn off the motors when either one of them exceeds a current draw of 12 amps, but you can increase or decrease that threshold by issuing this command followed by a data byte with a value between 1 and 30 to represent a current limit of 1 to 30 amps _per motor_ (not total device current). Although the limit is per motor, if either motor exceeds the limit, both motors are disabled for safety. The Scout will re-enable them after some length of time has passed, typically about 5 seconds. Note: if you decide to set a current threshold greater than the default it is highly recommended you implement a cooling fan and heatsinks on the driver chips, otherwise you will run into shutdowns due to over-temperature conditions.  

**23 &nbsp;&nbsp;&nbsp;&nbsp; Brake at Stop (0x17, b00010111)**<br />
The default method of stopping the motors is simply to quit powering them. However this does nothing to keep them from free-wheeling in response to an external force. If we actually want them to remain stopped, we need to "brake" them. This is done by shorting both motor leads together, this creates resistance inside the motor that can help keep it stationary. The motor of course can still be turned as the brake is not very strong, but it can make a difference in some applications. Typically you will want to enable this for tracked vehicles or robots that use differential motor speed to steer. For cars or other applications you may prefer the motors to be able to freewheel. 

The brake at stop setting defaults to false at bootup but can be set by the user via this serial command. Pass a value of 1 (true) in the data byte to enable, or 0 (false) to disable.  
            
**24 &nbsp;&nbsp;&nbsp;&nbsp; Drag Inner Track (0x18, b00011000)**<br />
This function was created to compensate for certain gearboxes such as the Taigen V2 Steel 3:1 and 4:1 gearboxes that have inadequate internal friction to prevent freewheeling with any sort of external force. When used in a tank or similar vehicle that requires differential motor speed in order to turn, the model may be difficult or impossible to steer.

The effect is most pronounced on heavy, wide-tracked models with metal upgrades, such as the King Tiger, Jagdtiger, Panther, Jagdpanther, etc.... In these cases reducing voltage to the inner track to steer does nothing, the outer (faster) track has enough traction with the ground to keep driving the model straight forward and the inner track just freewheels to keep up, rather than "dragging" the model into a turn as it should. 

The DragInnerTrack setting defaults to false at bootup but can be set by the user via this serial command. Pass a value of 1 (true) in the data byte to enable, or 0 (false) to disable. When enabled, the Scout will determine which motor is supposed to be turning slower than the other, and attempt to prevent it from freewheeling beyond the desired speed by "dragging" it with brief, pulsed brake commands interspersed with the actual speed command.

The option should be left disabled unless you specifically have a need to use it. 


# To-Do List
If you want to contribute to the project here are a few firmware improvements that still need work: 
  * Add serial commands 6-13 and 16-17 compatible with Sabertooth protocol (low resolution commands, mixed-mode driving, ramping, and adjustable deadband).
  * Characterize empirically the board heat profile at various currents, and improve the fan control algorithm. 
  * Test various fans and decide on a specific model. 


# Specifications

<html><table>
<tr>
    <td width="40%">Input voltage:</td>
    <td width="60%" valign="top">6 - 16 volts</td>
</tr>
<tr>
    <td>Operating current:</td>
    <td valign="top">12 amps per channel continuous without fan<br />30 amps absolute max<br />
    Current limit set to 12 amps by default, can be increased via serial commands</td>
</tr>
<tr>
    <td>Motor PWM:</td>
    <td>21 kHz</td>
</tr>
<tr>
    <td>RC Inputs:</td>
    <td valign="top">Standard 1000-2000 uS pulsewidth<br />(1500 uS = motor stopped)</td>
</tr>
<tr>
    <td>Serial Input:</td>
    <td valign="top">38400 baud; 8 data bits, no parity, one stop bit<br />TTL level (5v max)</td>
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

