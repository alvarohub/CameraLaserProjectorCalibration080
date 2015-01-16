#include "testApp.h"

using namespace ofxCv;
using namespace cv;

const float diffThreshold = 6.0; // maximum amount of movement
const float timeThreshold = 1.0; // minimum time between snapshots (seconds)

const float maxErrorCamera=0.3;
const int preCalibrateCameraTimes = 15; // this is for calibrating the camera BEFORE starting projector calibration.
const int startCleaningCamera = 4; // start cleaning outliers after this many samples (10 is ok...). Should be < than preCalibrateCameraTimes

#ifndef USING_LASER_PROJECTOR
const float maxErrorProjector=0.4; // ATTN! it depends on the size of the projector and the resolution of the camera!
#else
const float maxErrorProjector=5.0; // reprojection error depends in fact on the resolution of the projector and the camera sensing (the laser projector has a big resolution, but the camera does not change)
#endif
const int startCleaningProjector = 5;
const int startDynamicProjectorPattern=5; // after this number of projector/camera calibration, the projection will start following the
// printed pattern to facilitate larger exploration of the field of view. If this number is larger than minNumGoodBoards, then this will never
// happen automatically.
const int minNumGoodBoards=15; // after this number of simultaneoulsy acquired "good" boards, IF the projector total reprojection error is smaller than a certain threshold, we end calibration (and move to AR mode automatically)


// ****** INITIAL MODE ******
CalibState InitialMode=CAMERA_AND_PROJECTOR_PHASE1;//CAMERA_ONLY;//CAMERA_AND_PROJECTOR_PHASE1;//CAMERA_ONLY;//AR_DEMO;//CAMERA_AND_PROJECTOR_PHASE1; //CAMERA_ONLY;// AR_DEMO;//

void testApp::setup() {
    
	ofSetVerticalSync(true);
    
    // Font:
	ofTrueTypeFont::setGlobalDpi(72);//old OF default is 96 - but this results in fonts looking larger than in other programs.
	verdana14.loadFont("verdana.ttf", 14, true, true);
	verdana14.setLineHeight(18.0f);
	verdana14.setLetterSpacing(1.037);
    
#ifndef USING_KINECT_CAMERA
    // CAMERA SETTING:
    cout << "Listing Camera devices: " << endl;
    cam.listDevices();
    cam.setDeviceID(0);
	cam.initGrabber(CAM_WIDTH, CAM_HEIGHT);
    
    // cam.videoSettings(); // not supported in Lion...
    // cam.setDesiredFrameRate(5);
    
    imitate(undistorted, cam);
	imitate(previous, cam);
	imitate(diff, cam);
    
#else
    
    // enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
        
	}
    
    imitate(undistorted, kinect);
	imitate(previous, kinect);
	imitate(diff, kinect);
#endif
    

    
    //eyeMovie.loadMovie("movies/ojo.mov");
	//eyeMovie.play();
    
    
    // (1) Load the pattern data to recognize, for camera and for projector:
    calibrationCamera.loadCalibrationShape("settingsPatternCamera.yml");
#ifdef USING_LASER_PROJECTOR
    calibrationProjector.loadCalibrationShape("settingsLaserProjectionPatternPixels.yml");
#else
    calibrationProjector.loadCalibrationShape("settingsProjectionPatternPixels.yml");
#endif
    
    // (2) Setting the imager size:
    // Note: in case of projector, this is NOT the size of the acquired camera image: this needs to be set manually (of from a file)
    //       To make things more clear, we will do this also for objects of type camera.
    calibrationCamera.setImagerResolution(cv::Size(CAM_WIDTH, CAM_HEIGHT));
#ifdef USING_LASER_PROJECTOR
    calibrationProjector.setImagerResolution(cv::Size(LASER_PROJ_WIDTH, LASER_PROJ_HEIGHT));
#else
    calibrationProjector.setImagerResolution(cv::Size(PROJ_WIDTH, PROJ_HEIGHT));
#endif
    
	
    // (3) Define viewports for each display:
    // ATTENTION: I cannot use ofGetScreenWidth() and the like, because we need to put OF in "extended desktop" mode!
    viewportComputer.set(0,0,COMPUTER_DISP_WIDTH, COMPUTER_DISP_HEIGHT);
    
#ifdef RETINA_DISPLAY
    viewportProjector.set(1440,0,PROJ_WIDTH, PROJ_HEIGHT);
#else
    viewportProjector.set(COMPUTER_DISP_WIDTH,0,PROJ_WIDTH, PROJ_HEIGHT);
#endif
    
    // Other graphics settings:
    ofBackground(0,0,0);
    ofEnableAlphaBlending();
    ofEnableSmoothing();
    ofSetCircleResolution(30);
    
#ifdef USING_LASER_PROJECTOR
    arduinoSerialPort="/dev/tty.usbmodem1412";
    cout << "Listing Serial ports: " << endl;
    myPort.listDevices();
	myPort.setup(arduinoSerialPort,  230400); //115200); //
    
    ofSleepMillis(1000); // seems needed the first time (at least some waiting)
    
    // Initialize projector in "pause" mode:
    myPort.writeByte('}');
    
    // handshake:
    canSendData=true;
    // or periodic sending:
    lastTimeSent=ofGetElapsedTimef();
    
    // sensing mode:
    currentSensingMode=0;
#endif
    
    
    // Initialization of the state machine (note: THIS PROGRAM will mostly be a METHOD of a STEREO CALIBRATION CLASS)
    // NOTE: should be called AFTER setting serial port in case of laser projector
    initialization(InitialMode);
    
    // Change this to OF_LOG_SILENT or OF_LOG_VERBOSE or something else if you prefer:
    ofLog(OF_LOG_NOTICE);
}

void testApp::initialization(CalibState initialmode) {
    // Initialization of the state machine (note: THIS PROGRAM will mostly be a METHOD of a STEREO CALIBRATION CLASS)
    string mytext;
    
    lastTime = ofGetElapsedTimef();
	active = false; // changeable with keyboard SPACE
    newBoardAquired=false;
    enableDynamicProjection=true;
    dynamicProjectionInside=false;
    displayAR=false;
    semiFixedMode=false; // this means that the "fixed" pattern (i.e., the pattern loaded from the pattern file) will move around the image plane.
    
    switch (initialmode) {
        case CAMERA_ONLY: // (1) calibrate camera before anything else
            stateCalibration=CAMERA_ONLY;
            break;
            
        case CAMERA_AND_PROJECTOR_PHASE1: // (2) load pre-calibrated camera and start calibration projector and computing extrinsics
        case CAMERA_AND_PROJECTOR_PHASE2:
            calibrationCamera.load("calibrationCamera.yml");
            
            // Attention! the load method, also load a set of image/object points! I don't need that: it will break the SIMULTANEOUS stereo calib
            calibrationCamera.deleteAllBoards();
            
            stateCalibration=CAMERA_AND_PROJECTOR_PHASE1;
            break;
            
        case AR_DEMO: // camera, projector and extrinsics are loaded from file:
            calibrationCamera.load("calibrationCamera.yml");
            // attention! load also load a set of image/object points! I don't need that:
            calibrationCamera.deleteAllBoards();
            
            calibrationProjector.load("calibrationProjector.yml");
            // attention! load also load a set of image/object points! I don't need that:
            calibrationProjector.deleteAllBoards();
            
            loadExtrinsics("CameraProjectorExtrinsics.yml");
            
#ifdef USING_LASER_PROJECTOR
            //  sendIntrinsicsMatrix_Laser();
            //  sendExtrinsicsMatrix_Laser();
            
            // Set projector in RUN mode:
            myPort.writeByte('{');
            
            // Send a text to display:
            mytext="HELLO";
            sendText(mytext);
            
#endif
            
            stateCalibration=AR_DEMO;
            
            break;
            
        default:
            break;
    }
}

void testApp::update() {
    
#ifndef USING_KINECT_CAMERA
    cam.update();
    //eyeMovie.idleMovie();
    	if(cam.isFrameNew()) {
            
            Mat camMat = toCv(cam);
#else
            kinect.update();
            
            // there is a new frame and we are connected
            if(kinect.isFrameNew()) {
                
                Mat camMat = toCv(kinect);
#endif
    

		Mat prevMat = toCv(previous);
		
        
        //(a) First, add and preprocess the images (this will threshold, color segmentation, etc as specified in the pattern calibration files):
        // Note: we don this in the update loop, without the time condition so as to see the processed image in "real time" (see draw method)
        calibrationCamera.addImageToProcess(camMat);
#ifndef USING_LASER_PROJECTOR
        calibrationProjector.addImageToProcess(camMat);
#else
        // In this case, we will "blend" camMat and prevMat before adding the image to process, or better,
        // we will process both first for grid circles (i.e., threhold, erode and dilate), and then OR them:
        Mat im1, im2;
        if(camMat.type() != CV_8UC1) {
            cvtColor(camMat, im1, CV_RGB2GRAY);
        }
        threshold(im1, im1, 210, 255, THRESH_BINARY_INV);
        erode(im1, im1, Mat());dilate(im1, im1, Mat()); dilate(im1, im1, Mat()); erode(im1, im1, Mat());
        
        if(prevMat.type() != CV_8UC1) {
            cvtColor(prevMat, im2, CV_RGB2GRAY);
        }
        threshold(im2, im2, 210, 255, THRESH_BINARY_INV);
        erode(im2, im2, Mat());dilate(im2, im2, Mat()); dilate(im2, im2, Mat()); erode(im2, im2, Mat());
        
        //OR them:
        bitwise_and(im1,im2,im1);
        calibrationProjector.addImageToProcessRaw(im1);
#endif
        
        // Compute frame difference:
        Mat diffMat = toCv(diff);
		absdiff(prevMat, camMat, diffMat);
		camMat.copyTo(prevMat);
		diffMean = mean(Mat(mean(diffMat)))[0];
		
		// Compute time difference (since last acquisition):
        float diffTime = ofGetElapsedTimef()-lastTime;
        
        
        //(b) detect the patterns, and perform calibration or stereo calibration:
        switch(stateCalibration) {
            case CAMERA_ONLY:
                
                if (!active) {
                    calibrationCamera.generateCandidateImageObjectPoints(); // this is useful to visualize the detected board in real time
                    // (note: camera is NOT calibrated, so we cannot compute the board pose, but just represent the image points on the scene)
                }
                else // this means that we are ready to acquire
                    if (diffTime > timeThreshold && diffMean < diffThreshold) {
                        
                        if (calibrationCamera.generateCandidateImageObjectPoints()) {
                            // If we are here, it means that the camera could detect the PRINTED pattern.
                            // The image points and object points have been calculated and stored as "candidates" for addition in the vectors.
                            
                            // Add the candidate image/object points to the board vector list:
                            calibrationCamera.addCandidateImagePoints();
                            calibrationCamera.addCandidateObjectPoints();
                            
                            calibrationCamera.calibrate(); // this use all the previous boards stored in vector arrays, AND recompute each board rotations and tranlastions in the board vector list.
                            cout << "Camera re-calibrated" << endl;
                            
                            // Clean the list of boards using reprojection error test:
                            if(calibrationCamera.size() > startCleaningCamera) {
                                cout<<"Cleaning board list" << endl;
                                calibrationCamera.clean(maxErrorCamera);
                            }
                            
                            // For visualization: undistort image from camera:
                            /*
                             if(calibrationCamera.size() > 0) {
                             calibrationCamera.undistort(toCv(cam), toCv(undistorted));
                             undistorted.update(); // << this is confusing for me (how the actual data is updated, etc).
                             }
                             */
                            
                            // Test end CAMERA_ONLY calibration:
                            // (note: puting this after cleaning, means we want a certain number of "good" boards before moving on)
                            if ((calibrationCamera.size()>=preCalibrateCameraTimes)) {
                                // Save latest camera calibration:
                                calibrationCamera.save("calibrationCamera.yml");
                                
                                // DELETE all the object/image points, because now we are going to get them for both the projector and camera:
                                calibrationCamera.deleteAllBoards();
                                
                                // Start stereo calibration (for camera and projector):
                                stateCalibration=CAMERA_AND_PROJECTOR_PHASE1;
                            }
                            
                            lastTime = ofGetElapsedTimef();
                            active=false;
                        }
                        
                    }
                break;
                
                // IMPORTANT: in case of camera/projector calibration, we need to proceed in TWO PHASES to give time to the projected image
                // to refresh before trying to detect it (in case of "dynamic projected pattern"). Otherwise we may be detecting the OLD projected
                // pattern, but using the new image points. Which completely breaks the calibration of course.
                
            case CAMERA_AND_PROJECTOR_PHASE1: // pre-detection of the printed pattern (to move the projected pattern around if necessary)
                // NOTE: in case of laser projection, we can send the data here ONCE (instead of sending it in the draw loop, all the time).
                
                // First, update candidate image/object points for real time visualization, without projected pattern (laser projector is disabled at init):
                if (!active) {
                    if (calibrationCamera.generateCandidateImageObjectPoints()) calibrationCamera.computeCandidateBoardPose();
                    // (both things are necessary to display the candidate image points and candidate object points)
                    // Otherwise, do nothing (we can signal if a new board has been sensed, to delete the superimposed points perhaps...)
                    
                }
                else { // this means we are in active mode (ready to acquired the data)
                    cout << " ****** PHASE 1 ********** " << endl;
                    if (calibrationCamera.generateCandidateImageObjectPoints()) {
                        cout << "Printed pattern detected" << endl;
                        
                        // We assume now that the camera is well calibrated: do NOT recalibrate again, simply compute latest board pose:
                        calibrationCamera.computeCandidateBoardPose();
                        
                        //NOTE: we don't add anything to the board vector arrays FOR THE CAMERA (image/object) because we need to be sure
                        // we also get this data for the projector calibration object before calling stereo calibration
                        // However, we can already use the candidate points to show image and reprojection for that board.
                        
                        
                        // Set the points for projector display (not adding them to the imagePoints vector yet), so the projector
                        // will project something to be detected in the draw function. The first time, this is using the recorded pattern, but later
                        // (as the projector gets calibrated) we can use some arbitrary points "closer" to the printed pattern.
                        // ATTN!! this test not ideal because CLEANING may change the number of boards in the list - BETTER TO TRACK THE CURRENT TOTAL
                        // REPROJECTION ERROR? (this may be ok, IF the next board position is closer to the current one. Indeed, a good reprojection error
                        // for the projector can be attained in ONE try, but the extrinsics are not even computed!!)
                        
                        if ((calibrationProjector.size()>startDynamicProjectorPattern)&&(enableDynamicProjection)) {
                            cout << "USING DYNAMIC PROJECTION PATTERN" << endl;
                            //Modify the candidate projector image points to follow the printed board if the projector has been partially
                            // calibrated (ie., we have an estimate of the intrinsics at least).
                            // This is important to effectively explore the "image space" for the projector, and improve the calibration.
                            // (This can be done using the computed extrinsics, or the board rot/trans from the latest projector calibration)
                            //(b) following the latest detected printed pattern (make a special function with displacement parameter):
                            vector<Point3f> auxObjectPoints;
                            Point3f posOrigin, axisX, axisY;
                            axisX=calibrationCamera.candidateObjectPoints[1]-calibrationCamera.candidateObjectPoints[0];
                            axisY=calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width]-calibrationCamera.candidateObjectPoints[0];
                            if (dynamicProjectionInside)
                                //pattern inside the printed chessboard:
                                posOrigin=calibrationCamera.candidateObjectPoints[0]+(axisX-axisY)*0.5;
                            else
                                // pattern outside the printed chessboard:
                                posOrigin=calibrationCamera.candidateObjectPoints[0]-axisY*(calibrationCamera.myPatternShape.getPatternSize().width-2);
                            
                            auxObjectPoints=Calibration::createObjectPointsDynamic(posOrigin, axisX, axisY, calibrationProjector.myPatternShape);
                            // Note: a method "setCandidateDynamicObjectPoints" is not needed, because the actual candidate OBJECT points will be computed from the camera image. But perhaps it would be better to have it, to avoid calling a static method.
                            
                            vector<Point2f> followingPatternImagePoints;
                            // We want the board pose in projector coordinates: we will use the rot/trans of the PREVIOUS BOARD as stored by the projector calibration object
                            // and the camera object, as well as the PRESENT board pose from the (well calibrated) camera. We will NOT use the not the (yet not good)
                            // extrinsics, which is what we are looking for by the way. So, since we don't use the
                            // extrinsics, we need to determine the new rot/trans from the camera "delta" motion, which presumably, is quite
                            // good (camera is well calibrated). Then, we will be able to calculate the pose of the board in projector coordinates.
                            // Note that even if the final pose in projector coordinates is not good, we don't care: we are just trying to get the projected point "closer"
                            // to the printed pattern to facilitate "exploration" of the space - points will we will precisely detected with the camera.
                            Mat Rc1, Tc1, Rc1inv, Tc1inv, Rc2, Tc2, Rp1, Tp1, Rp2, Tp2;
                            // Previous board position in projector coordinate frame:
                            Rp1=calibrationProjector.boardRotations.back();
                            Tp1=calibrationProjector.boardTranslations.back();
                            // Previous board position in camera coordinate frame:
                            Rc1=calibrationCamera.boardRotations.back();
                            Tc1=calibrationCamera.boardTranslations.back();
                            // Latest board position in camera coordiante frame (not yet in the vector list!!):
                            Rc2=calibrationCamera.candidateBoardRotation;
                            Tc2=calibrationCamera.candidateBoardTranslation;
                            
                            
                            Mat auxRinv=Mat::eye(3,3,CV_32F);
                            Rodrigues(Rc1,auxRinv);
                            auxRinv=auxRinv.inv(); // or transpose, the same since it is a rotation matrix!
                            Rodrigues(auxRinv, Rc1inv);
                            Tc1inv=-auxRinv*Tc1;
                            Mat Raux, Taux;
                            composeRT(Rc2, Tc2, Rc1inv, Tc1inv, Raux, Taux);
                            composeRT(Raux, Taux, Rp1, Tp1, Rp2, Tp2);
                            
                            
                            followingPatternImagePoints=calibrationProjector.createImagePointsFrom3dPoints(auxObjectPoints, Rp2, Tp2);
                            // Set image points to display:
                            calibrationProjector.setCandidateImagePoints(followingPatternImagePoints);
                        }
                        else  { // this means that the pattern that the projector needs to project is given by the stored pattern:
                            //(a) Set the candidate points (for projection) using the fixed (or semi-fixed) pattern:
                            if (semiFixedMode) {
                                cout << "USING SEMI-FIXED PROJECTION PATTERN" << endl;
                                unsigned short widthImagePlane=calibrationProjector.getImagerResolution().width;
                                unsigned short heightImagePlane=calibrationProjector.getImagerResolution().height;
                                unsigned short widthPattern=calibrationProjector.myPatternShape.getPatternSize().width;
                                unsigned short heightPattern=calibrationProjector.myPatternShape.getPatternSize().height;
                                //int nx=floor(1.0*widthImagePlane/widthPattern);
                                //int ny=floor(1.0*heightImagePlane/heightPattern);
                                // Generate image points at a random position:
                                calibrationProjector.setCandidateImagePoints(100+rand()%(widthImagePlane-widthPattern-200),
                                                                             100+rand()%(heightImagePlane-heightPattern-200));
                            } else {
                                cout << "USING FIXED PROJECTION PATTERN" << endl;
                                calibrationProjector.setCandidateImagePoints();
                            }
                        }
                        
#ifdef USING_LASER_PROJECTOR
                        // in case of laser projector, let's send the points to display serially:
                        // (NOTE: I prefer not to modify the calibration class, so the follwing functions belong to the testApp object (this is supposed
                        // to be work in progress for the OF community, and this particular feature is totally experimental):
                        drawCandidateImagePoints_Laser(calibrationProjector, 0,0, LASER_PROJ_WIDTH, LASER_PROJ_HEIGHT, 7); // RGB ON
                        
                        //Enable laser projector:
                        myPort.writeByte('{');
                        
                        cout << "PATTERN TO PROJECT SENT TO LASER PROJECTOR" << endl;
                        
                        // DO A PAUSE TO ENSURE THE PATTERN START TO BE PROJECTED:
                        //...
#endif
                        
                        // Go to next phase:
                        stateCalibration=CAMERA_AND_PROJECTOR_PHASE2; // or 2a...
                        // NOTE: I SHOULD DO THIS IF I THINK THE BOARD POSITION MAY CHANGE from this phase to the next (the time of seing the pattern projected
                        // by the projector)
                        active=false;
                        //lastTime=ofGetElapsedTimef();
                        
                    }
                    else {
                        cout << "PRINTED pattern not visible: MOVE THE BOARD AROUND" << endl;
                        // NOTE: we don't change the stateCalibration phase.
                    }
                }
                break;
                /*
                 case  CAMERA_AND_PROJECTOR_PHASE2a:
                 // In this phase, the projector is now projecting a pattern.
                 // We need to re-check the printed board and recompute the board pose (in case it moved).
                 
                 // recheck that we still see the board, and recompute it's pose, BUT don't change the projector image points!!!
                 // IF we don't see the pattern, we need to go back to previous phase (in particular if in dynamic mode)
                 if  (calibrationCamera.generateCandidateImageObjectPoints()) {
                 cout << "Printed pattern detected" << endl;
                 
                 // We assume now that the camera is well calibrated: do NOT recalibrate again, simply compute latest board pose:
                 calibrationCamera.computeCandidateBoardPose();
                 }
                 break;
                 */
            case CAMERA_AND_PROJECTOR_PHASE2:// detection of the PROJECTED pattern and creation of the OBJECT points for the projector (the image points
                // are already set in the previous phase. In fact, we don't really care if the board moved a little in between, since we could not only acquire the
                // PROJECTED points, but also the PRINTED pattern (and recompute the board pose to get a good estimation of the projector-object points). However,
                // for the time being we will assume that the board did not move much, and ensure this by checking that the image did not change a lot:
                
                if (active) { // otherwise do nothing, just wait...
                    cout << " ****** PHASE 2 ********** " << endl;
                    if (diffMean < diffThreshold && calibrationCamera.generateCandidateImageObjectPoints()) {
                        cout << "(1) Printed pattern detected..." << endl;
                        // recompute the board pose for more precision when backprojecting in case there was a slight motion:
                        calibrationCamera.computeCandidateBoardPose();
                        
                        if (calibrationProjector.generateCandidateObjectPoints(calibrationCamera)) {
                            // Attention: generateCandidateObjectPoints compute the candidate objectPoints (if these are detected by the camera), but not the image points. These were already set on the projector calibration object (and displayed!).
                            
                            cout << "(2) Projected pattern also detected..." << endl;
                            
                            // If the object points for the projector were detected, add those points as well as the image points to the
                            // list of boards image/object for BOTH the camera and projector calibration object, including the rotation and
                            // translation vector for the camera frame.
                            
                            // add to CAMERA board list (only for stereo calibration):
                            calibrationCamera.addCandidateImagePoints();
                            calibrationCamera.addCandidateObjectPoints();
                            calibrationCamera.addCandidateBoardPose();
                            
                            // add to PROJECTOR board list (for projector recalibration and stereo calibration):
                            calibrationProjector.addCandidateImagePoints();
                            calibrationProjector.addCandidateObjectPoints();
                            // Note: the rotation and translation vectors for the projector are not yet updated :these CANNOT
                            // properly be computed using PnP algorithm (as calibrationProjector.computeCandidateBoardPose()), because we are
                            // precisely trying to get the projector instrinsics! This will be done by the calibrateProj() method...
                            
                            // PUT THIS IN ANOTHER THREAD????
                            calibrationProjector.calibrate(); // this will recompute the projector instrinsics, as well as the board translation and rotation for all the boards (including the latest one), as well as update the reprojection error.
                            //NOTE: it will also update the candidateBoardRotation and candidateBoardTranslation just because we may want these matrices for drawing (but we don't need to "add" them to the vector lists, because this is already done by the openCV calibration method).
                            cout << "Projector re-calibrated." << endl;
                            
#ifdef USING_LASER_PROJECTOR
                            // Send the instrinsics to the laser projector here? No, no need yet. we will send everything when we get the full stereo calibration.
#endif
                            
                            // Cleaning: this needs to be done SIMULTANEOUSLY for projector and camera.
                            // However, the test is only done on the projector reprojection error if the camera intrinsics are fixed (because the
                            // stereo calibration will be done by generating image points for the camera based only on the OBJECT POINTS OF THE PROJECTOR)
                            if(calibrationProjector.size() > startCleaningProjector) {
                                calibrationProjector.simultaneousClean(calibrationCamera, maxErrorProjector);
                            }
                            
                            // Now we can run the stereo calibration that in this particular case, only compute the EXTRINSICS camera-projector (output: rotCamToProj and transCamToProj). Note: we call stereo calibration
                            // with FIXED INTRINSICS for both the camera and projector.
                            // NOTE: perhaps we can start running this after a few projector calibrations????
                            cout << "Performing stereo calibration..." << endl;
                            calibrationProjector.stereoCalibrationCameraProjector(calibrationCamera, rotCamToProj, transCamToProj);
                            cout << "Stereo Calibration performed (i.e. camera-projector instrinsics are estimated" << endl;
                            
                            // SAVE THE INTRINSICS and EXTRINSINCS when TOTAL reprojection error is lower than a certain threshold (this is
                            // automatically assured if we called simultaneousClean before this lines of code),
                            // and when we get a sufficiently large amount of boards, and move to AR_DEMO:
                            if (calibrationProjector.size()>minNumGoodBoards) {
#ifdef USING_LASER_PROJECTOR
                                calibrationProjector.save("calibrationLaserProjector.yml");
                                saveExtrinsics("CameraLaserProjectorExtrinsics.yml");
                                sendIntrinsicsMatrix_Laser();
                                sendExtrinsicsMatrix_Laser();
#else
                                calibrationProjector.save("calibrationProjector.yml");
                                saveExtrinsics("CameraProjectorExtrinsics.yml");
#endif
                                active=false;
                                stateCalibration=AR_DEMO;
                            } else {
                                // Otherwise, proceed refining the calibration: REVERT TO PHASE1 (and indicate that a "stereo board" was properly aquired)
                                stateCalibration=CAMERA_AND_PROJECTOR_PHASE1;
                                newBoardAquired=true;
                                cout << endl << "======= YOU CAN MOVE THE PRINTED PATTERN TO EXPLORE IMAGE SPACE =====" << endl;
                                active=false; // so we can prepare the pose and hit the keyboard
                                lastTime = ofGetElapsedTimef();
#ifdef USING_LASER_PROJECTOR
                                //Disable laser projector:
                                myPort.writeByte('}');
#endif
                            }
                            
                        }
                        else {
                            cout << "Projected pattern not visible" << endl;
                            // REVERT TO PHASE1 (in the meanwhile, the user moved the printed pattern, then changing the projected dynamic pattern):
                            stateCalibration=CAMERA_AND_PROJECTOR_PHASE1;
                            cout << endl << "======= YOU *NEED* TO MOVE THE BOARD / CAMERA (PROJECTED PATTERN NOT VISIBLE) =====" << endl;
                            active=false;
#ifdef USING_LASER_PROJECTOR
                            //Disable laser projector:
                            //myPort.writeByte('}');
#endif
                        }
                        
                    }
                    else { // This means that the board moved perhaps too much, if we want to have a dynamic projected pattern that "follows" the print:
                        // REVERT TO PHASE1 (in the meanwhile, the user moved the printed pattern, then changing the projected dynamic pattern):
                        stateCalibration=CAMERA_AND_PROJECTOR_PHASE1;
                        cout << endl << "======= STAY STILL IF YOU WANT TO ACQUIRE THE PROJECTOR IMAGE =====" << endl;
                        active=false;
#ifdef USING_LASER_PROJECTOR
                        //Disable laser projector:
                        // myPort.writeByte('}');
#endif
                    }
                }
                break;
                
            case AR_DEMO:
                
                // We assume here that projector and camera are calibrated, as well as extrinsics
                // We can detect things using the pattern, or something else (say, a rectangular A4 page). The important thing is to get the
                // transformation from OBJECT to CAMERA. We will use the EXTRINSICS to get the correspondance from OBJECT to PROJECTOR.
                if (calibrationCamera.generateCandidateImageObjectPoints()) { //diffMean > diffThreshold, diffTime > 0.01
                    cout << "Chessboard pattern recognized" << endl;
                    calibrationCamera.computeCandidateBoardPose();  // transformation from board to camera computed here
                }
                
                break;
        }
		
	}
    
    
#ifdef USING_LASER_PROJECTOR
    // Handshake: confirm we can send data:
    if (myPort.available()) {
        if (myPort.readByte()==13) {
            cout<<"handshake  code received" << endl;
            canSendData=true;
        }
    }
#endif
}

// =====================================================================================================

void testApp::draw() {
    
    stringstream intrinsicsProjector, intrinsicsCamera;
    
    // This is important (and it seems I need to do it here, and not in the setup!!??):
    ofSetWindowPosition(0,0); // .. but not sure it really works (problem with the window title; perhaps it should be negative on the y axis? but it
    //does not accept negative values!!)
    
    // Highlight projector area (frame) - this is optional:
    ofViewport(viewportProjector);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0,viewportProjector.width, viewportProjector.height, 0); // left, right, bottom, top
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, -TASKBAR_HEIGHT, 0); // THIS IS A HACK TO CANCEL THE PB WITH THE TASK BAR WHICH PUSHES EVERYTHING DOWNWARDS
    ofPushStyle();
    ofNoFill();
    ofSetColor(0,255,0);
    ofSetLineWidth(2);
    ofRect(0,0,viewportProjector.width, viewportProjector.height);
    ofPopStyle();
    
    // Now, prepare to display on computer display for checking:
    ofViewport(viewportComputer);
    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0,viewportComputer.width, viewportComputer.height, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //glTranslatef(0, -TASKBAR_HEIGHT, 0); // THIS IS A HACK TO CANCEL THE PB WITH THE TASK BAR WHICH PUSHES EVERYTHING DOWNWARDS
    // (not needed in the case of the computer display).
#ifdef RETINA_DISPLAY
    // ARG!!! horrible hack for get it working on retina display (GLUT cannot handle such resolution)
    // NOTE: this needs to be done ONLY for the COMPUTER SCREEN
    if (COMPUTER_DISP_WIDTH>1440) {
        //To get things in real pixels:
        //glScaled(1440.0/COMPUTER_DISP_WIDTH, 900.0/COMPUTER_DISP_HEIGHT, 1.0);
    }
#endif
    
    
    int posTextY=CAM_HEIGHT+20, posTextX=10;
    
    // Draw current acquired image:
    ofSetColor(255);
#ifndef USING_KINECT_CAMERA
    cam.draw(0,0, CAM_WIDTH, CAM_HEIGHT);
#else
    kinect.draw(0,0, CAM_WIDTH, CAM_HEIGHT);
#endif
    
    // Draw preprocessed images for camera and projector board detection (we need to do the preprocessing BEFORE calling the detection functions):
    calibrationCamera.drawPreprocessedImage(CAM_WIDTH, 0, CAM_WIDTH/2, CAM_HEIGHT/2);
    calibrationProjector.drawPreprocessedImage(CAM_WIDTH, CAM_HEIGHT/2, CAM_WIDTH/2, CAM_HEIGHT/2);
    
    if (calibrationCamera.candidatePatternDetected()) {
        // Draw detected points (for CAMERA CALIBRATION BOARD):
        calibrationCamera.drawCandidateImagePoints(0,0, CAM_WIDTH, CAM_HEIGHT, ofColor(255,0,0));
    }
    
    if (calibrationProjector.candidatePatternDetected()) { // this is, the projected pattern was visible and detected
        // Draw the image points as SEEN BY THE CAMERA (this is to check that the detection went well):
        calibrationProjector.drawProjectorPointsAsSeenByCamera(calibrationCamera,0,0, CAM_WIDTH, CAM_HEIGHT, ofColor(0,0,255));
    }
    
    if (calibrationCamera.isReady()) {
        // Draw reprojected points (for CAMERA CALIBRATION BOARD):
        calibrationCamera.drawCandidateReprojection(0,0, CAM_WIDTH, CAM_HEIGHT, ofColor(255,255,0));
        // Draw axis of detected board:
        calibrationCamera.drawCandidateAxis(0,0, CAM_WIDTH, CAM_HEIGHT);
        
        // Draw undistorted image:
        //undistorted.draw(CAM_WIDTH, 0);
        
        intrinsicsCamera << "Camera fov: " << toOf(calibrationCamera.getDistortedIntrinsics().getFov());
        drawHighlightString(intrinsicsCamera.str(), posTextX, posTextY, yellowPrint, ofColor(0));
        drawHighlightString("Total Reproj error camera: " + ofToString(calibrationCamera.getTotalReprojectionError()) + " from " + ofToString(calibrationCamera.size()), posTextX, posTextY+20, magentaPrint);
        
    }
    
    if (calibrationProjector.isReady()) {
        intrinsicsProjector << "Projector fov: " << toOf(calibrationProjector.getDistortedIntrinsics().getFov()) ;//<< " distCoeffs: " << calibrationProjector.getDistCoeffs();
        drawHighlightString(intrinsicsProjector.str(), posTextX, posTextY+50, yellowPrint, ofColor(0));
        drawHighlightString("Current board error projector: " + ofToString(calibrationProjector.getLastReprojectionError()), posTextX, posTextY+70, magentaPrint);
        drawHighlightString("Total reproj error projector: " + ofToString(calibrationProjector.getTotalReprojectionError()) + " from " + ofToString(calibrationProjector.size()), posTextX, posTextY+90, magentaPrint);
    }
    
    if (!active) {
        drawHighlightString("  PRESS SPACE TO PROCEED ", CAM_WIDTH+CAM_WIDTH/2+20, 60, yellowPrint, ofColor(0));
    }
    
    if (displayAR) {
        drawHighlightString(" PRE-AR ACTIVE ", CAM_WIDTH+CAM_WIDTH/2+20, 70, ofColor(255), ofColor(0));
    }
    
    switch(stateCalibration) {
        case CAMERA_ONLY:
            drawHighlightString(" *** CALIBRATING CAMERA ***", CAM_WIDTH+CAM_WIDTH/2+20, 40, cyanPrint,  ofColor(255));
            break;
        case CAMERA_AND_PROJECTOR_PHASE1:
        case CAMERA_AND_PROJECTOR_PHASE2:
            drawHighlightString(" *** CALIBRATING CAMERA + PROJECTOR ***", CAM_WIDTH+CAM_WIDTH/2+20, 40, cyanPrint,  ofColor(255));
            
            // Extrinsics camera-projector:
            // NOTE: in the future, the object STEREO should have a flag "isReady" to test if it is possible to proceed with some things...
            // For the time being, I assume that if BOTH camera and projector have been calibrated once, then we also called the stereo calibration...
            if (calibrationCamera.isReady()&&calibrationProjector.isReady()&&(calibrationProjector.size()>0)) {
                
                extrinsics="";
                Mat rotCamToProj3x3;
                Rodrigues(rotCamToProj, rotCamToProj3x3);
                for (int i=0; i<3; i++) {
                    for (int j=0; j<3; j++)  extrinsics += ofToString(rotCamToProj3x3.at<double>(i,j),1) + "  ";
                    extrinsics += ofToString(transCamToProj.at<double>(0,i),4)+" \n";
                }
                drawHighlightString("Extrinsics:", posTextX, posTextY+110, yellowPrint, ofColor(0));
                drawHighlightString(extrinsics, posTextX, posTextY+160, yellowPrint, ofColor(0));
                
                if (displayAR) {
                    // ============================ DRAW ON THE PROJECTOR DISPLAY (SET AS CONTIGUOUS SCREEN) ===================================
                    // NOTE: we can use the position parameter on the draw functions, or put (0,0) and do a translation here, or use two viewports...
                    //ofPushMatrix();
                    //ofTranslate(COMPUTER_DISP_WIDTH,0); // width of the computer screen (not the window spanning computer+projector display)
                    //...
                    // ofPopMatrix();
                    
                    ofViewport(viewportProjector);
                    glMatrixMode(GL_PROJECTION);
                    glLoadIdentity();
                    gluOrtho2D(0,viewportProjector.width, viewportProjector.height, 0);
                    glMatrixMode(GL_MODELVIEW);
                    glLoadIdentity();
                    glTranslatef(0,-TASKBAR_HEIGHT,0); // THIS IS A HACK TO CANCEL THE PB WITH THE TASK BAR WHICH PUSHES EVERYTHING DOWNWARDS
                    
                    vector<Point2f> testPoints;
                    // (a) Project over the printed chessboard using the board position directly in projector reference frame (this assumes projector calibration was possible, and also that the pose is computed FROM THE PROJECTOR POINT OF VIEW, which is ONLY when performing a "calibrate" method!!):
                    //        testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationCamera.candidateObjectPoints,  calibrationProjector.candidateBoardRotation, calibrationProjector.candidateBoardTranslation); // no final Re and te means identity transform
                    //        calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255,0,0), 3); // make static function
                    //        calibrationProjector.drawCandidateAxis(0,0, PROJ_WIDTH, PROJ_HEIGHT);
                    
                    
                    // (b) Project over the projector points (not directly, but using object points and board transformation from the projector:
                    //        testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationProjector.candidateObjectPoints,  calibrationProjector.candidateBoardRotation, calibrationProjector.candidateBoardTranslation);// no final Re and te means identity transform
                    //        calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(0,255,0), 9);
                    
                    
                    // Now, the REAL tests - i.e. true AR using camera/projector calibration:
                    // (a) Project over the printed chessboard using EXTRINSICS (this assumes projector calibration as well as stereo calibration):
                    testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationCamera.candidateObjectPoints,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation, rotCamToProj, transCamToProj);
                    calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255, 100,0,100), 8);
                    
                    // (b) Project over the projector pattern, but using the points seen by the camera, and the EXTRINSICS:
                    //     testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationProjector.candidateObjectPoints,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation,rotCamToProj, transCamToProj);
                    //     calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(0,255,0,100), 8);
                }
            }
            
            //... And ALWAYS project pattern points (I do it at last, to avoid having occlusion):
            // NOTE: we draw disks in IMAGE plane, meaning that they won't look like circles but ellipses in the real world... this is not a problem
            // here, and may be preferable to use openGL rendering (which would render real circles in 3d world), because the camera is supposed to
            // be somehow near the projector: then the circles would look like circles whatever the plane inclination, which is better for running
            // the "detect circles" openCV routine.
            ofViewport(viewportProjector);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluOrtho2D(0,viewportProjector.width, viewportProjector.height, 0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glTranslatef(0, -TASKBAR_HEIGHT, 0); // THIS IS A HACK TO CANCEL THE PB WITH THE TASK BAR WHICH PUSHES EVERYTHING DOWNWARDS
            if (newBoardAquired==false) {
                // NOTE: the size of the dots depends on the distance if we use openCV circles... if we have an estimate of the extrinsics, it is
                // better to do a good back-projection and use OpenGL to draw circles, then mantaining their real size.
                // Each "dot" is white here, with a fixed size:
                calibrationProjector.drawCandidateProjectorPattern(0,0, PROJ_WIDTH, PROJ_HEIGHT, ofColor(255,255,255,255), 6);//calibrationProjector.myPatternShape.squareSize/4);
                
            }
            else  { // just indicate that the board was acquired (short "flash" with red points, with a much larger size - equal to the pattern period);
                calibrationProjector.drawCandidateProjectorPattern(0,0, PROJ_WIDTH, PROJ_HEIGHT, ofColor(255,100,0,255),calibrationProjector.myPatternShape.squareSize/2);
                newBoardAquired=false;
                
#ifdef USING_LASER_PROJECTOR
                // FLASH THE PATTERN:
                // CHANGE color of current laser scene (not state machine value):
                myPort.writeByte(5+48); // only RED and BLUE (attention! it must be the ascii code, not the actual value (48 is '0'))
                myPort.writeByte(')');
                ofSleepMillis(500);
                // Go back to white:
                myPort.writeByte(7+48); // RGB (WHITE) (attention! it must be the ascii code, not the actual value (48 is '0'))
                myPort.writeByte(')');
#endif
                
            }
            
            break;
            
        case AR_DEMO:
            // Two ways to do this: either use openCV to compute the projection or transformation of images, or faster, using OPENGL and properly setting the
            // perspective and modelview matrix.
            drawHighlightString(" *** AR DEMO MODE ***", CAM_WIDTH+CAM_WIDTH/2+20, 40, cyanPrint,  ofColor(255));
            
            extrinsics="";
            Mat rotCamToProj3x3;
            Rodrigues(rotCamToProj, rotCamToProj3x3);
            for (int i=0; i<3; i++) {
                for (int j=0; j<3; j++)  extrinsics += ofToString(rotCamToProj3x3.at<double>(i,j),1) + "  ";
                extrinsics += ofToString(transCamToProj.at<double>(0,i),4)+" \n";
            }
            drawHighlightString(extrinsics, posTextX, posTextY+100, yellowPrint, ofColor(0));
            
            if (calibrationCamera.candidatePatternDetected()) { // note: this gives true when we found a printed board (if called using the
                //calibrationCamera object. However, the board "candidate" pose is computed by calling computeCandidateBoardPose() (see update, in AR mode).
                
#ifndef USING_LASER_PROJECTOR
                
                // First, get the 3d coordinates of the FOUR conrners in CHESSBOARD coordinates:
                vector<Point3f> corners; // define the four corners of the chessboard:
                corners.push_back(calibrationCamera.candidateObjectPoints[0]);
                corners.push_back(calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width-1]);
                corners.push_back(calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width*
                                                                          calibrationCamera.myPatternShape.getPatternSize().height-1]);
                corners.push_back(calibrationCamera.candidateObjectPoints[calibrationCamera.myPatternShape.getPatternSize().width*
                                                                          (calibrationCamera.myPatternShape.getPatternSize().height-1)]);
                
                
                //(a) ========================= Draw using OpenGL =========================
                // PROBLEM: For some unknown reason, the settings using openGL places the z=0 plane somehow below the real board plane. The
                // difference is noticeable (projecting using OpenCV projection works very well, but using openGL the points/images are not
                // properly on the plane. One possible explanation is the problem with the task bar in OF_FULLSCREEN mode...
                // (1) Set viewport:
                ofViewport(viewportProjector);
                // (2) Set perspective matrix (using the projector intrinsics):
                calibrationProjector.setOpenGLProjectionMatrix();
                // (3) Set the proper modelview:
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();
                glTranslatef(0, -TASKBAR_HEIGHT, 0); // THIS IS A HACK TO CANCEL THE PB WITH THE TASK BAR WHICH PUSHES EVERYTHING DOWNWARDS
                gluLookAt(0, 0, 0,   // position of camera
                          0, 0, 1,   // looking towards the point (0,0,1)
                          0, -1, 0); // orientation
                
                Mat finalR, finalT;
                composeRT(calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation, // from model to camera
                          rotCamToProj, transCamToProj, // extrinsics (from camera to projector)
                          finalR, finalT);
                applyMatrix(makeMatrix(finalR, finalT));
                ofScale(calibrationCamera.myPatternShape.squareSize, calibrationCamera.myPatternShape.squareSize, 0);
                //Draw something (image, whatever):
                ofSetColor(0,255,0); ofNoFill();
                ofSetLineWidth(2);
                ofRect(0,0,calibrationCamera.myPatternShape.getPatternSize().width-1, calibrationCamera.myPatternShape.getPatternSize().height-1);
                
                
                // Draw small images on the white squares of the chessboard:
                /*
                 ofSetColor(255);
                 for(int i = 0; i < calibrationCamera.myPatternShape.patternSize.height-1; i++)
                 for(int j = 0; j < calibrationCamera.myPatternShape.patternSize.width/2-1+i%2; j++) {
                 eyeMovie.draw(j*2+(i+1)%2,i,1,1); // note: ofScale ensures that each square is normalized
                 }
                 */
                
                //(b) ========================= Draw using OpenCV =========================
                // (just for checking compatibility). Note: if we draw circles, the circles would NOT be in perspective here!
                ofViewport(viewportProjector);
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                gluOrtho2D(0,viewportProjector.width, viewportProjector.height, 0);
                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();
                glTranslatef(0, -TASKBAR_HEIGHT, 0); // THIS IS A HACK TO CANCEL THE PB WITH THE TASK BAR WHICH PUSHES EVERYTHING DOWNWARDS
                // Project all corners of the printed chessboard using EXTRINSICS:
                vector<Point2f> testPoints;
                testPoints=calibrationProjector.createImagePointsFrom3dPoints(calibrationCamera.candidateObjectPoints,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation, rotCamToProj, transCamToProj);
                calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255, 0,0,255), 5);
                
                // Project the FOUR corners using the points seen by the camera, and the EXTRINSICS:
                testPoints=calibrationProjector.createImagePointsFrom3dPoints(corners,  calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation,rotCamToProj, transCamToProj);
                calibrationProjector.drawArbitraryImagePoints(0,0, PROJ_WIDTH, PROJ_HEIGHT, testPoints, ofColor(255,255,0,255), 8);
                
                // Draw axis (and indicate (0,0)):
                ofSetColor(255, 0, 255);
                ofLine(testPoints[0].x, testPoints[0].y, testPoints[1].x, testPoints[1].y);
                ofSetColor(0, 255, 255);
                ofLine(testPoints[0].x, testPoints[0].y, testPoints[3].x, testPoints[3].y);
                
#else
                //(c) Draw using the laser and raw points (equivalent to the openCV method, because the software in the computer does the projection):
                
                
                //(d) Draw using the laser and POSE/EXTRINSIC matrix (equivalent to the OpenGL method, because the hardware engine (here the laser projector with
                // my mbed code) does the calculations. <-- THIS IS THE FINAL, REAL TEST!!
                // NOTE: we assume that the EXTRINSICS and INTRINSICS are already loaded into the laser projector; we only need to send the POSE (and by the way,
                // we should not send it ALL the time, only when we detect the board, AND presumably at a maximum, acceptable frame rate.
                
                // SEND POSE DATA to the projector:
                //  if (diffTime>.01) {// Important to avoid overloading the serial communication (value in seconds)
                // sendPoseMatrix_Laser();
                // lastTime = ofGetElapsedTimef();
                //}
                // or using handshake (timer is optional - it may be slower to redraw the image too often!)
                if (canSendData) {// && (ofGetElapsedTimef()-lastTimeSent) >.06) {
                    sendPoseMatrix_Laser();
                    canSendData=false;
                    cout << "Sending period: " << ofGetElapsedTimef()-lastTimeSent << endl;
                    lastTimeSent = ofGetElapsedTimef();
                }
                
#endif
            }
            
            break;
            
    }
    
}


// Save calibration parameters (along with the detected feature image points):
// NOTE: this should be a method of the STEREO CALIBRATION CLASS!
void testApp::saveExtrinsics(string filename, bool absolute) const {
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::WRITE);
    fs << "Rotation_Vector" << rotCamToProj;
    Mat rotCamToProj3x3;
    Rodrigues(rotCamToProj, rotCamToProj3x3);
    fs << "Rotation_Matrix" << rotCamToProj3x3;
    fs << "Translation_Vector" << transCamToProj;
    // Saved as a 4x4 openGL like modelviw matrix:
    ofMatrix4x4 mat4x4=makeMatrix(rotCamToProj, transCamToProj);
    fs << "OpenGL_Mat" << "[";
    for(int i = 0; i < 4; i++) {
        fs << "{:" << "row" << "[:";
        for( int j = 0; j < 4; j++ ) fs << mat4x4(i,j);
        fs << "]" << "}";
    }
    fs << "]";
}


void testApp::loadExtrinsics(string filename, bool absolute) {
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::READ);
    fs["Rotation_Vector"] >> rotCamToProj;
    fs["Translation_Vector"] >> transCamToProj;
}


// =========== THINGS THAT WILL BELONG TO THE STEREO-CALIBRATION OBJECT ====================

void testApp::keyPressed(int key) {
	if(key == OF_KEY_DOWN) active = !active;
    else if (key== OF_KEY_UP) displayAR=!displayAR; // this is just for test to see how it is going. But better not to use during
    // calibration, because it interferes with the detection.
    
    else if (key=='p') enableDynamicProjection=!enableDynamicProjection; // enables dynamic projection (possible after a certain number of boards)
    else if (key=='o') dynamicProjectionInside=!dynamicProjectionInside;
    
#ifdef USING_LASER_PROJECTOR
    else if (key=='t') sendText("HELLO");
    else if (key=='g') {
        int nx=5, ny=5;
        int sizeX=500, sizeY=500; // this is the size of the WHOLE pattern - not between points!
        int posX=(LASER_PROJ_WIDTH-sizeX)/2;
        int posY=(LASER_PROJ_HEIGHT-sizeY)/2;
        displayLaserGrid(posX, posY, sizeX, sizeY, nx, ny);  // activate pre-defined grid. note: posX, posY and sizeX and sizeY are in "projector pixels"
    }
    else if (key=='s') { // change sensing mode
        currentSensingMode=(currentSensingMode+1)%3;
        changeSensingMode(currentSensingMode);
    }
#endif
    
    
}



// ===== THINGS THAT ARE JUST FOR THE EXPERIMENTAL LASER PROJECTOR using MBED ==============
#ifdef USING_LASER_PROJECTOR

void testApp::changeSensingMode(int sensingMode) {
    char auxstring[40];
    sprintf(auxstring,"%d+,",sensingMode); // terminator "+" for sensing mode change
    int j=0;
    while(auxstring[j]!=',') {
        myPort.writeByte(auxstring[j]);
        j++;
    }
}



void testApp::displayLaserGrid(int posX, int posY, int sizeX, int sizeY, int nx, int ny) {
    // First, send the sequence: " posX#posY#sizeX#sizeY#nx#ny#: ", corresponding to grid(sizeX, sizeY, nx, ny, 1) function in laser projector
    char auxstring[40];
    // Send using the proper protocol format (check mbed code): x#y#... and then the command (here ":")
    // Each float number must be followed by '#':
    sprintf(auxstring,"%d#%d#%d#%d#%d#%d#:,",posX, posY, sizeX, sizeY, nx, ny);// converts to decimal base and add '#' terminator (the ',' is an horrible hack for sending all the characters). There must be a better way to send this...
    int j=0;
    while(auxstring[j]!=',') {
        //myPort.writeBytes(unsigned char * buffer, int length);
        myPort.writeByte(auxstring[j]);
        //cout << (char)auxstring[j] << endl;
        j++;
    }
}


void testApp::sendIntegerNumber_Laser(unsigned short num) {
    char auxstring[40];
    // Send using the proper protocol format (check mbed code): x#y#
    // Each float number must be followed by '#':
    sprintf(auxstring,"%d#,",num);// converts to decimal base and add '#' terminator (the ',' is an horrible hack for sending all the characters). There must be a better way to send this...
    int j=0;
    while(auxstring[j]!=',') {
        //myPort.writeBytes(unsigned char * buffer, int length);
        myPort.writeByte(auxstring[j]);
        //cout << (char)auxstring[j] << endl;
        j++;
    }
}

void testApp::sendFloatNumber_Laser(float num) {
    char auxstring[40];
    // Send using the proper protocol format (check mbed code): x#y#
    // Each float number must be followed by '#':
    sprintf(auxstring,"%1.3f#,",num);// converts to decimal base and add '#' terminator (the ',' is an horrible hack for sending all the characters). There must be a better way to send this...
    int j=0;
    while(auxstring[j]!=',') {
        //myPort.writeBytes(unsigned char * buffer, int length);
        myPort.writeByte(auxstring[j]);
        //cout << (char)auxstring[j] << endl;
        j++;
    }
}

void testApp::sendText(string _text) {
    myPort.writeBytes((unsigned char *)_text.c_str(), _text.length());
    //for (int i=0; i<_text.length(); i++) {
    //    myPort.writeByte(_text[i]);
    //}
    myPort.writeByte('"'); // this is the "code" for text (in the mbed server).
}

void testApp::sendPoseMatrix_Laser() {
    // (1) send POSE MATRIX DATA:
    ofMatrix4x4 matrix=makeMatrix(calibrationCamera.candidateBoardRotation, calibrationCamera.candidateBoardTranslation);
    
    // ATTENTION: makeMatrix(r, t) produces an ouput of the form (after applying Rodriguez to the rotation vector):
    // return ofMatrix4x4(rm[0], rm[3], rm[6], 0.0f,
    //                    rm[1], rm[4], rm[7], 0.0f,
    //                    rm[2], rm[5], rm[8], 0.0f,
    //                    tm[0], tm[1], tm[2], 1.0f);
    // which is TRASPOSED with respect to the matrices I am using in the laser projector!!!
    
    // We need to send the data serially, in a  ROW-COLUMN order (meaning, we fill rows by rows, or "row first")
    char auxstring[40];
    for (int row=0; row<3 ; row++)
        for (int col=0; col<4; col++)
            sendFloatNumber_Laser(matrix(col,row));
    // and finally, the indicator for "pose matrix":
    myPort.writeByte('$');
}

void testApp::sendExtrinsicsMatrix_Laser(){
    // (1) send POSE MATRIX DATA:
    ofMatrix4x4 matrix=makeMatrix(rotCamToProj, transCamToProj);
    
    // ATTENTION: makeMatrix(r, t) produces an ouput of the form (after applying Rodriguez to the rotation vector):
    // return ofMatrix4x4(rm[0], rm[3], rm[6], 0.0f,
    //                    rm[1], rm[4], rm[7], 0.0f,
    //                    rm[2], rm[5], rm[8], 0.0f,
    //                    tm[0], tm[1], tm[2], 1.0f);
    // which is TRASPOSED with respect to the matrices I am using in the laser projector!!!
    
    // We need to send the data serially, in a  ROW-COLUMN order (meaning, we fill rows by rows, or "row first")
    char auxstring[40];
    for (int row=0; row<3 ; row++)
        for (int col=0; col<4; col++)
            sendFloatNumber_Laser(matrix(col,row));
    // and finally, the indicator for "extrinsics":
    myPort.writeByte('&');
}

void testApp::sendIntrinsicsMatrix_Laser(){
    // (1) send POSE MATRIX DATA:
    
    // The data to send is here:
    // calibrationProjector.myCameraModel.distortedIntrinsics.getCameraMatrix()
    //... it's private data. We need to make a method to get it. /////// TO DO ///////////
    Mat K=calibrationProjector.getDistortedIntrinsics().getCameraMatrix();
    
    // We need to send the data serially, in a  ROW-COLUMN order (meaning, we fill rows by rows, or "row first")
    char auxstring[40];
    for (int col=0; col<3 ; col++)
        for (int row=0; row<4; row++)
            sendFloatNumber_Laser(K.at<double>(col, row));
    // and finally, the indicator for "intrinsics":
    myPort.writeByte('%');
}




bool testApp::drawCandidateImagePoints_Laser(ofxCv::Calibration &calibrationProjector, int px, int py, int sx, int sy, unsigned char color) {
    unsigned short x, y;
    if (calibrationProjector.candidateImagePoints.size()>0) {
        // TO DO: switch case for the patter type (circles, chessboard...)
        // NOTE: I will send raw points, assuming the projector is set to orthographic projection with identity pose
        
        // set pose: (for the time being, this is set to the identity in the mbed code).
        //...
        
        // Set color (state machine):
        myPort.writeByte(color+48); // attention! it must be the ascii code, not the actual value (48 is '0')
        ofSleepMillis(10);
        myPort.writeByte('(');
        ofSleepMillis(10);
        
        // Send raw points, after proper scaling (really needed?) and translation:
        float ratx=sx/calibrationProjector.getImagerResolution().width;
        float raty=sy/calibrationProjector.getImagerResolution().height;
        
        for(int j = 0; j < calibrationProjector.candidateImagePoints.size(); j++) {
            
            // these are projected "image" points, therefore the value can be safely cast to an
            // unsigned integer (and if the laser projector has a resolution of 12 bits, this means 16 bits are enough):
            x=(unsigned short)(px+ratx*calibrationProjector.candidateImagePoints[j].x);
            y=(unsigned short)(py+raty*calibrationProjector.candidateImagePoints[j].y);
            //z=calibrationProjector.candidateImagePoints[j].z; // this is 0
            
            //cout << x << " " << y << endl;
            
            sendIntegerNumber_Laser(x);//j%6*100+1000);
            ofSleepMillis(10);
            sendIntegerNumber_Laser(y);//(j-6*j%6)*100+1000);
            ofSleepMillis(10);
            
        }
        // Finally, inform the mbed that this data is for creating an array of 2d points:
        myPort.writeByte('=');
        ofSleepMillis(10);
        
        return true;
    }else
        return false;
}

#endif


