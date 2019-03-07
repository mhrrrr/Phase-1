# README #


### What is this repository for? ###

* This repositary consist of Companion computer code for General Aeronautics Flying Vehicles based on Mavlink 
* 0.0

### How do I get set up? ###

* Install Pymavlink
* Enable serial0 port on RPi
* Connect serial0 to Telem1/2 of PX4
* Set baudrate of SERIAL1/2 to 921600 on APM code stack
* Other modules will be required based on the specific vehicle. For example, GA3Agri might use pwm output and will require RPIO to be installed

* Add pymavlink_main.py to run at boot using update-rc.d

* run program by giving following command

		python pymavlink_main.py --vehicle <VehicleName>

* Currently supported vehicles are 'GA3', 'GA3T', 'GA3A', 'GA3M'

### Testing ###
* For SITL testing run pymavlink

		python pymavlink_main.py --vehicle <VehicleName> --sitludp
		python pymavlink_main.py --vehicle <VehicleName> --sitltcp

### Who do I talk to? ###

* Sachchit Vekaria