# Self-Balancing-Inverted-Pendelum
Modeling and simulation of an inverted pendulum that is self balanced with 3 reaction wheels, 2 for the x and y axis and 1 for the z axis.

Control will be implemented soon as its still been built in fablab Sevilla :)

#### CAD model of the system:

The system was created in FreeCAD and all the files can be found in the link below:

https://grabcad.com/library/self-balancing-stick-1

And an image preview of it:

![Alt text](https://github.com/richaeell/Self-Balancing-Inverted-Pendelum/blob/master/docs/Images/CADmodel.gif)

#### Measuring data

In this system we have two sensors per motor, a current sensor and an encoder.

For the encoder I have used the AS5060 magnetic encoder. I purchased the chips on their own and designed the boards to fit in the small gap behind the motors.

![Alt text](https://github.com/richaeell/Self-Balancing-Inverted-Pendelum/blob/master/docs/Images/Image_PCB.png)

The board were designed in EasyEDA, and the Greber files are available to download.

![](https://github.com/richaeell/Self-Balancing-Inverted-Pendelum/blob/master/docs/Images/PCB_Encoder-board.png =300x300)


#### Demo of dynamics of a similar system:

The parameters of the system were assumed and are not from the model, this animation demostrates the speeding up of the wheel from the ground position and followed by a sudden deacceleration that results in a torque applyed on the pivot point that ultimatly lifts the pendulum up.

![Alt text](https://github.com/richaeell/Self-Balancing-Inverted-Pendelum/blob/master/docs/Images/Animation.gif)
