#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"

//NOTE: Annoying problem: it is not possible to get rid of the taskbar in OF_WINDOW mode, and it is not possible to get "dual" monitor (in fact, extended
// screen) in OF_FULLSCREEN or GAME_MODE. And even if we could get rid of the taskbar in fullscreen, then there is the window bar: ALL THIS CHANGES THE ACTUAL
// POSITION OF THE DISPLAYING WINDOWS, and INTERFERES WITH THE CALIBRATION - albeit only a little, but results are not right!

#define TASKBAR_HEIGHT 0 //22

// RETINA DISPLAY? (this is a horrible hack!):
#define RETINA_DISPLAY

//USING KINECT CAMERA or normal camera:
//#define USING_KINECT_CAMERA

// NORMAL PROJECTOR OR LASER PROJECTOR?
//#define USING_LASER_PROJECTOR
#define LASER_PROJ_WIDTH  4096
#define LASER_PROJ_HEIGHT 4096
// Important: when calibrating the laser projector (mbed program laserSensingDisplay), we need to set the rendering interval to:
// #define RENDER_INTERVAL 0.000054 //0.0001 is good for CALIBRATION grid, 0.000054 good for normal display // in seconds (Ticker)
// in the "laserSensingDisplay.h" file!!

// ==================================================================
// WE NEED TO DEFINE HERE the size of the computer screen and the projector screen. This cannot be done using ofGetScreenWidth() and the like
// because in order to properly calibrate camera and projector, we need OF in "extended desktop mode".
// Of course, it would be possible to use a single screen (the projector) to perform the whole calibration (but we need a way to check if the 
// printed pattern is visible - SOUND?).

// NOTE: Retina Display 15' :2880x1800, other: 1280x1024, 1440x900, etc
#define COMPUTER_DISP_WIDTH 2880 //1280
#define COMPUTER_DISP_HEIGHT 1800 //1024

// RESOLUTION OF CAMERA AND PROJECTOR: 
// - ideally, this should also be in a file (in particular if we want to calibrate several cameras/projectors). 
// Resolution of the projector: DELL M110 native resolution is: WXGA (1280 x 800)
#define PROJ_WIDTH  1280 
#define PROJ_HEIGHT 800

// Resolution of the camera OR KINECT (or at least resolution at which we want to calibrate it):
#define CAM_WIDTH 640
#define CAM_HEIGHT 480

// ==================================================================


enum CalibState {CAMERA_ONLY, CAMERA_AND_PROJECTOR_PHASE1, CAMERA_AND_PROJECTOR_PHASE2, AR_DEMO};

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
    void initialization(CalibState initialmode);
   
    
#ifdef USING_KINECT_CAMERA
    ofxKinect kinect;
#else
	ofVideoGrabber cam;
#endif
    
	ofImage undistorted; // perhaps not used
    
 	ofPixels previous; 
	ofPixels diff;
	float diffMean;
	
	float lastTime;
	bool active;
    bool displayAR;
    bool newBoardAquired;
    bool enableDynamicProjection;
    bool dynamicProjectionInside;
    bool semiFixedMode;
	
    // For tests:
    ofVideoPlayer 		eyeMovie;
    
    ofTrueTypeFont	verdana14;
    
    // VARIABLES and METHODS THAT SHOULD BELONG TO A STEREO-CALIBRATION OBJECT (probably using multiple cameras and projectors)
	ofRectangle viewportComputer, viewportProjector;
    ofxCv::Calibration calibrationCamera, calibrationProjector;
    CalibState stateCalibration;
    
    //Extrinsics (should belong to the Stereo calibration object)
    cv::Mat rotCamToProj, transCamToProj; // in fact, there should be one pair of these for all the possible pairs camera-projector, camera-camera, projector-projector. 
    string extrinsics;
    void saveExtrinsics(string filename, bool absolute = false) const;
    void loadExtrinsics(string filename, bool absolute = false);
    
#ifdef USING_LASER_PROJECTOR
    ofSerial myPort;
    void sendIntegerNumber_Laser(unsigned short num);
    void sendFloatNumber_Laser(float num);
    void sendText(string _text);
    void displayLaserGrid(int posX, int posY, int sizeX, int sizeY, int nx, int ny);
    int currentSensingMode; // 0 for no sensing, 1 for whole scene sensing, and 2 for per-object sensing
    void changeSensingMode(int sensingMode);
    bool drawCandidateImagePoints_Laser(ofxCv::Calibration &calibrationProjector, int px, int py, int sx, int sy, unsigned char color);
    void sendPoseMatrix_Laser();
    void sendExtrinsicsMatrix_Laser();
    void sendIntrinsicsMatrix_Laser();
    // Serial port for the Arduino (check on the Arduino IDE):
    string arduinoSerialPort;//="/dev/tty.usbserial-A600etoh";
    bool canSendData;
    float lastTimeSent;
#endif

};
