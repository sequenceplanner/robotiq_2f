## A ROS2 driver for the Robotiq 2f gripper
Work in progress...

Uses modbus protocol with RTU frame format for serial interfacing

### Tested with:
- Model: AGC-GRP-085-2016
- OS: Linux
- ROS version: Galactic (r2r v.0.6.2.)
- Serial standard: RS-485

### Preliminaries
- sudo usermod -a -G dialout $USER
- sudo reboot
- Get the msg package from: todo...

### Low level input mapping

Byte 0: Gripper Status (gOBJ, gSTA, gGTO, gACT)\
Byte 1: Reserved\
Byte 2: Fault Status (kFLT, gFLT)\
Byte 3: Position Request Echo (gPR)\
Byte 4: Position (gPO)\
Byte 5: Current (gCU)

#### Byte 0: Gripper Status
- 0: gACT - Activation status, echo of the rACT bit (activation bit).
    - 0x0 - Gripper reset.
    - 0x1 - Gripper activation.
- 1: Reserved
- 2: Reserved
- 3: gGTO - Action status, echo of the rGTO bit (go to bit).
    - 0x0 - Stopped (or performing activation / automatic release).
    - 0x1 - Go to Position Request.
- 4, 5: gSTA - Gripper status, returns the current status & motion of the Gripper fingers.
    - 0x00 - Gripper is in reset ( or automatic release ) state. See Fault Status if Gripper is activated.
    - 0x01 - Activation in progress.
    - 0x02 - Not used.
    - 0x03 - Activation is completed.
- 6, 7: gOBJ - Object detection status, is a built-in feature that provides information on possible object pick-up. Ignore if gGTO == 0.
    - 0x00 - Fingers are in motion towards requested position. No object detected.
    - 0x01 - Fingers have stopped due to a contact while opening before requested position. Object detected opening.
    - 0x02 - Fingers have stopped due to a contact while closing before requested position. Object detected closing.
    - 0x03 - Fingers are at requested position. No object detected or object has been loss / dropped.

#### Byte 1: Reserved

#### Byte 2: Fault Status
- 0-3: gFLT: Fault status returns general error messages that are useful for troubleshooting. Fault LED (red) is present on the Gripper chassis, LED can be blue, red or both and be solid or blinking.
    - 0x00 - No fault (LED is blue)
    - Priority faults (LED is blue)
        - 0x05 - Action delayed, activation (reactivation) must be completed prior to perfmoring the action.
        - 0x07 - The activation bit must be set prior to action.
    - Minor faults (LED continuous red)
        - 0x08 - Maximum operating temperature exceeded, wait for cool-down.
        - 0x09 - No communication during at least 1 second.
    - Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed).
        - 0x0A - Under minimum operating voltage.
        - 0x0B - Automatic release in progress.
        - 0x0C - Internal fault; contact support@robotiq.com.
        - 0x0D - Activation fault, verify that no interference or other error occurred.
        - 0x0E - Overcurrent triggered.
        - 0x0F - Automatic release completed.
- 3-7: kFLT - See your optional Controller Manual (input registers & status).

#### Byte 3: Position request echo
- 0-7: gPR - Echo of the requested position for the Gripper, value between 0x00 and 0xFF
    - 0x00 - Full opening.
    - 0xFF - Full closing.

#### Byte 4: Position
- 0-7: gPO: Actual position of the Gripper obtained via the encoders, value between 0x00 and 0xFF.
    - 0x00 - Fully opened.
    - 0xFF - Fully closed.

#### Byte 5: Current
- 0-7: gCU: The current is read instantaneously from the motor drive, value between 0x00 and 0xFF, approximate current equivalent is 10 * value read in mA.

### High level input mapping
todo
### Low level output mapping
todo
### High level output mapping
todo