# Description of Code for Arduinos
## Working Radio Code:
- [Node_00001](\NODE_00001\NODE_00001.ino): This is the code for the first node in the network, which is the node attached to the computer and receives serial commands from the MATLAB dynamics simulation.
- [Node_00002](\NODE_00002\NODE_00002.ino): The second node in the network, receives signal from the first node and sends it to it's children.
- [Node_00003](\NODE_00003\NODE_00003.ino):
- [Node_00004](\NODE_00004\NODE_00004.ino):
- [Node_00005](\NODE_00005\NODE_00005.ino):
### Parent-Child Relationship Graphic:
<img src="graphics\Node_communication_graphic.png" alt="An image depicting the communication network I developed to communicate with 5 out of the 8 nodes shown in the picture." width="800">

## Working Motor Code:
- [encoder_test](\encoder_test\encoder_test.ino): This code is used to read values from the encoder on the [motors](https://www.servocity.com/60-rpm-hd-premium-planetary-gear-motor-w-encoder/). I am also including code from the TimerInterruptTest code in an attempt to get the encoder to work with simultaneous timer interrupts.
## Code Under Development:
### Motor Code:
- [MorphTestMotorControl_EDITED](\MorphTestMotorControl_EDITED\MorphTestMotorControl_EDITED.ino): A copied and edited version of the [MorphTestMotorControl](https://byu.app.box.com/folder/226341030881) code written by Dr. Usevitch's co-worker at Standford. This code is intended to be used to control the motors on the soft robot. The code currently compiles and runs on the Arduino, but gets stuck in the while loop and doesn't reach the interrupt functions. 

