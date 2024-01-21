# Haptic-Walking-Aid-Device
The objective of this project is to develop a haptic walking aid using embedded systems concepts to detect simple and compound objects with ultrasonic sensors. It is important to note that this device is not a medical device and serves as a proof of concept to demonstrate basic engineering principles. The PVC pipe used in this project is a mock-up and not designed to bear weight. A practical haptic walking aid solution would require a more robust support structure and extensive testing.
The project comprises the implementation of various functionalities, including a reboot command, optimization commands, event definitions, and storage, compound event creation, event erasing, event display, haptic feedback control, and pattern management. These functionalities will be stored in the EEPROM and recalled upon power-up. The system aims to prioritize events in case multiple events occur simultaneously and detect common obstacles such as walls, stairs, or items grouped by height.
The device utilizes a TM4C123GXL board, ultrasonic sensors, a motor with weight, and various other components to achieve its goals. 

Hardware requirments:
Front Sensor Assembly: Holds 3 HC-SR04 ultrasonic sensors.
Back Sensor Assembly: Supports front sensor structure.
Lid: Covers electronics.
Power Switch Slide Bar: Access to power switch.
PVC Pipe (35”): Main support.
PVC Pipe (5” - Handle): User handle.
Grip: Comfortable handling.
Foot: Stability when stationary.
Sheet Metal Screws (4): Attach sensor assemblies.
3xAA Battery Pack: Power source.
AA Batteries (3): Power supply.
Sheet Metal Screws (4): Secures cover to battery pack and assembly.
Dupont Connector Pair: Connects battery to power board.
Pin Headers: Component connections.
Small PC Board: Power distribution.
Motor with Weight: Haptic feedback.
SRF04 Sensors (3): Distance measurement.
Foam Support: Sensor padding.
Dupont Wire Jumpers: Component connections.
MOSFET: Motor control.
1kohm Resistor: Connects MOSFET to PWM pin.
10kohm Resistor: MOSFET gate to ground.
TM4C123GXL Board: Microcontroller.
