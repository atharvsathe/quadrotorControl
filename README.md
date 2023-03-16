# CMU 18776 Course Project

## Installation
- Copy the folders 'lqr_att_controller' and 'observer' in 'Firmware/src/modules'.
- Add line 'modules/lqr_att_controller' and 'module/observer' in 'Firmware/boards/px4/sitl/default.cmake'.
- In some versions of px4, we may need to comment out lines for AUTO_DISARM_PREFLIGHT in 'Firmware/src/modules/commander/ commander.cpp.' This prevents auto disarming of the vehicle when we manually arm the vehicle. In version 1.9 comment out lines 1715 to 1718.
- Compile with make px4_sitl jmavsim.

## Usage
lqr_att_controller
- Arm the vehicle using command: 'commander arm'.
- Land the vehicle using command: 'commander land'.
- Start the controller using command: 'lqr_att_controller start'.
- Stop the controller using command: 'lqr_att_controller stop'.
- Set height for the controller using command: 'lqr_att_controller set_height |value|'.  Note: the value should be negative as UP direction is negative in the simulator.
- Set x position for the controller using command: 'lqr_att_controller set_x |value|'.
- Set y position for the controller using command: 'lqr_att_controller set_y |value|'.
- Set yaw angle for the controller using command: 'lqr_att_controller set_yaw |value|'.
- Display current height, x, y, yaw using command: 'lqr_att_controller show_data'.
Usage instructions will be visible by typing ‘lqr_att_controller’ in the command prompt.

## Noise Injection
Choose noise levels by modifying lines 89 - 92 in lqr_att_controller.hpp

## Maneuvers
Default is manual control

- Maneuver 1: Travels from origin to (5 ,5 ,5) 
For Maneuver 1: Uncomment lines 167 - 172 in lqr_att_controller.cpp

- Maneuver 2: Travels in a 3d circle 
For Maneuver 2: Uncomment lines 176 - 188 in lqr_att_controller.cpp

## Non Linear Predictor
To enable non linear predictor uncomment line 157 lqr_att_controller.cpp

## Complementary filter
To enable complementry filter uncomment line 203 lqr_att_controller.cpp
