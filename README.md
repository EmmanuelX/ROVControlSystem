Programs to run our ROV. 

ROVControlBox.ino
This file runs the controller above the water surface. The controller consists of a Joystick and a controller box with switches and potentiometers that pick up data and packs it into an array. Then, it sends the data to the ROV system underwater via Arduino serial connection.

ROVControlSystem.ino
This file runs the ROV underwater. The ROV system receives data packed in an array. The ROV consists of 4 thrusters(2 horizontal, 2 verticals). After receiving data, it packs and sends data back to the controller. In the case of a misconnection, the ROV sets the thrusters off.
