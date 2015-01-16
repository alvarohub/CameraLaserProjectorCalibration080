#include "testApp.h"
#include "ofAppGlutWindow.h"


int main() {
	ofAppGlutWindow window;
    
#ifndef USING_LASER_PROJECTOR
    ofSetupOpenGL(&window, COMPUTER_DISP_WIDTH+PROJ_WIDTH, PROJ_HEIGHT, OF_WINDOW);
    // width computer screen + width projector screen / height projector
#else
    // In this case, there should not be a secondary viewport for the projector, but I know that if I go into game mode, only the main screen is active. So, this works for the time being:
     ofSetupOpenGL(&window, COMPUTER_DISP_WIDTH, COMPUTER_DISP_HEIGHT,  OF_FULLSCREEN);//OF_GAME_MODE);
#endif
    
    ofRunApp(new testApp());
}
