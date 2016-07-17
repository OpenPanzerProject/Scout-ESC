![Open Panzer Scout ESC](http://www.openpanzer.org/images/github/scout_git.jpg)
# Scout ESC

The Scout is a dual brushed motor controller that accepts both TTL serial and RC inputs. The onboard processor is an ATmega 328 and can be programmed with the Arduino IDE using a standard FTDI cable or adapter. 

The motor drivers are rated for a max current of 30 amps, however the chips will turn off at lower currents due to overheat protection. In dead air at room temperature without additional heatsinking, the board should be able to sustain a continuous current of 10 amps per motor. The addition of heatsinks and a fan can increase the current capacity.

The board includes a driver circuit for an external fan to be controlled by the processor. An onboard thermistor also allows the processor to monitor the board temperature. 

