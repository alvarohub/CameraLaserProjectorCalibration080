# CameraLaserProjectorCalibration080
***calibrating camera and custom laser projector using mbed-based laser projector***

This is an extension of the camera/projector calibration work explained here: 

https://www.youtube.com/watch?v=pCq7u2TvlxU

In this case, a laser projector is being calibrated instead of a normal projector.
In the second part of the video, the camera is used to track a rectangular shape, and the projector maps 
a text at 200 "frames" per second making it look very stable, as if printed on the paper. 
This is a first step towards a "laser printer at-a-distance"...

NOTE: check the new OpenFrameworks add-on by kikko (Cyril Diagne), who did an amazing work clarifying and encapsulating the 
code so it is compatible with ofxCV by Kyle Mc. Donald: 

https://github.com/kikko/ofxCvCameraProjectorCalibration

The new code for this demo is here: 
https://github.com/alvarohub/CameraLaserProjectorCalibration080

It still uses OF 080 and the OfxCv2. But the changes should be minor if using Cyril Diagne code instead. What is needed is to send
points coordinates through serial port to the laser projector, and do some simple image processing to avoid saturating the captured
image. 

Indeed, if you have succeeded calibrating a normal projector with a camera, using ofxCvCameraProjectorCalibration, 
you will notice that the code for the laser projector is slightly different, and uses the serial port to send data 
to the laser projector. The code used by the laser projector is here:

http://developer.mbed.org/users/mbedalvaro/code/laserUI/

Unfortunately I cannot explain the details of the hardware design right now. I will try to make an instructables or 
something like that when time permits. 
