![Open Panzer Scout ESC](http://www.openpanzer.org/images/github/scout_git_r9.jpg)
# Scout ESC

The Scout is a dual brushed motor controller that accepts both TTL serial and RC inputs. The onboard processor is an ATmega 328 and can be programmed with the Arduino IDE using a standard FTDI cable or adapter. 

The motor drivers are rated for a max current of 30 amps, however the chips will turn off at lower currents due to overheat protection. In dead air at room temperature without additional heatsinking, the board should be able to sustain a continuous current of 10 amps per motor. The addition of heatsinks and a fan can increase the current capacity.

The board includes a driver circuit for an external fan to be controlled by the processor. An onboard thermistor also allows the processor to monitor the board temperature. 

## Resources
  * For the Scout hardware files and bill of materials, see the [Open Panzer Downloads page](http://openpanzer.org/downloads).
  * For more information on the Open Panzer project, see the [OpenPanzer Wiki](http://www.openpanzer.org/wiki).
  * To discuss the project, feel free to join the [Open Panzer Community](http://openpanzer.org/forum/index.php?action=forum).

## License
Firmware for the Scout ESC is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License version 3 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

For more specific details see [http://www.gnu.org/licenses](http://www.gnu.org/licenses), the [Quick Guide to GPLv3.](http://www.gnu.org/licenses/quick-guide-gplv3.html) and the [copying.txt](https://github.com/OpenPanzerProject/TCB/blob/master/COPYING.txt) file in the codebase.

The GNU operating system which is under the same license has an informative [FAQ here](http://www.gnu.org/licenses/gpl-faq.html).

