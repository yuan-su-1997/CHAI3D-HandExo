//
// Created by SuYuan(yuansu97@yeah.net) on 2022/11/17 
// Hand motion model for HandExoskeleton or other devices.
//


#include <WinSock2.h>
#include <Windows.h>
#include <iostream>
#include <fstream>
#pragma comment(lib , "ws2_32.lib")

#define ADDRESS_STM32 "192.168.1.30"
#define PORT_STM32 8089
#define ADDRESS_PC "192.168.1.10"
#define PORT_PC 8089

#include "chai3d.h"
#include <GLFW/glfw3.h>
#include <GLFW/FingerKinematics.h>

#include <chrono>

//------------------------------------------------------------------------------
// For One Euro Filter
//------------------------------------------------------------------------------

#include<stdexcept>
#include<cmath>
#include<ctime>

//------------------------------------------------------------------------------
using namespace chai3d;
//------------------------------------------------------------------------------

#define M_PI 3.1415926

//------------------------------------------------------------------------------
// GENERAL SETTINGS 
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES 
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight* light;

// a small sphere (cursor) representing the haptic device 
//cShapeSphere* hand_base;
cShapeSphere* hand_arm0;
cShapeSphere* hand_arm1;
cShapeSphere* arm0;
cShapeSphere* arm1;

cShapeSphere* thumb_0;  //base
cShapeSphere* thumb_1;
cShapeSphere* thumb_2;  //fingertip

cShapeSphere* index_0;
cShapeSphere* index_1;
cShapeSphere* index_2;
cShapeSphere* index_3;

cShapeSphere* middle_0;
cShapeSphere* middle_1;
cShapeSphere* middle_2;
cShapeSphere* middle_3;

cShapeSphere* forth_0;
cShapeSphere* forth_1;
cShapeSphere* forth_2;
cShapeSphere* forth_3;

cShapeSphere* little_0;
cShapeSphere* little_1;
cShapeSphere* little_2;
cShapeSphere* little_3;

//a line from world_frame to thumb_cursor
cShapeLine* T01;
cShapeLine* T12;

cShapeLine* I01;
cShapeLine* I12;
cShapeLine* I23;

cShapeLine* M01;
cShapeLine* M12;
cShapeLine* M23;

cShapeLine* F01;
cShapeLine* F12;
cShapeLine* F23;

cShapeLine* L01;
cShapeLine* L12;
cShapeLine* L23;

cShapeLine* IM;
cShapeLine* MF;
cShapeLine* FL;
cShapeLine* LH2;
cShapeLine* H21;
cShapeLine* H1I;
cShapeLine* H1T;

cShapeLine* arm00;
cShapeLine* arm11;


// a line representing the velocity vector of the haptic device
cShapeLine* velocity;
// a haptic device handler
cHapticDeviceHandler* handler;
// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;
// a label to display the haptic device model
cLabel* labelHapticDeviceModel;
// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;
cLabel* labelHapticDeviceRotation;
// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;
cMatrix3d hapticDeviceRotation;
// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;
cLabel* labelrotation;

// a flag for using damping (ON/OFF)
bool useDamping = false;
// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;
// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

//==========================================================================
/*
    DEMO:   Hand Motion Model
*/
//==========================================================================
 
//--------------------------------------------------------------------------
// SOCKET
// For Communication To STM32
//--------------------------------------------------------------------------

WSADATA	d1;
WORD w1;
int wsa_return;
SOCKET s1;
sockaddr_in addr_local;
sockaddr_in addr_remote;
float angle[18] = { 0 };
int socklen;

//--------------------------------------------------------------------------
// For Auto Motion
//--------------------------------------------------------------------------

double T = 100000;
double time_initial_;
double time_cur_;
bool flag_1 = false;
bool flag_2 = true;

//--------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------

Eigen::Matrix4f finger1_pose;	//thumb
Eigen::Matrix4f finger2_pose;	//index
Eigen::Matrix4f finger3_pose;	//middle

Eigen::Matrix4f aux_finger1_pose;

Eigen::Vector4f finger1_joint;
Eigen::Vector4f finger2_joint;

//--------------------------------------------------------------------------
// One Euro Filter
//--------------------------------------------------------------------------

typedef double TimeStamp; // in seconds

static const TimeStamp UndefinedTime = -1.0;

// -----------------------------------------------------------------

class LowPassFilter {               

    double y, a, s;
    bool initialized;

    void setAlpha(double alpha) {
        if (alpha <= 0.0 || alpha > 1.0)
            throw std::range_error("alpha should be in (0.0., 1.0]");
        a = alpha;
    }

public:

    LowPassFilter(double alpha, double initval = 0.0) {
        y = s = initval;  //initial test time y=s=0
        setAlpha(alpha);        
        initialized = false;
    }

    double filter(double value) {        
        double result;
        if (initialized)
            result = a * value + (1.0 - a) * s;     
        else {
            result = value;
            initialized = true;
        }
        y = value;
        s = result;
        return result;
    }

    double filterWithAlpha(double value, double alpha) {
        setAlpha(alpha);
        return filter(value);
    }

    bool hasLastRawValue(void) {
        return initialized;
    }

    double lastRawValue(void) {     
        return y;
    }

};

// -----------------------------------------------------------------

class OneEuroFilter {

    double freq;         //Sampling frequency
    double mincutoff;    //Minimum cut-off frequency
    double beta_;
    double dcutoff;
    LowPassFilter* x;
    LowPassFilter* dx;
    TimeStamp lasttime;

    double alpha(double cutoff) {                                        
        double te = 1.0 / freq;
        double tau = 1.0 / (2 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / te);
    }

    void setFrequency(double f) {                                        
        if (f <= 0) throw std::range_error("freq should be >0");
        freq = f;
    }

    void setMinCutoff(double mc) {                                       
        if (mc <= 0) throw std::range_error("mincutoff should be >0");
        mincutoff = mc;
    }

    void setBeta(double b) {                                            
        beta_ = b;
    }

    void setDerivateCutoff(double dc) {
        if (dc <= 0) throw std::range_error("dcutoff should be >0");
        dcutoff = dc;
    }

public:

    OneEuroFilter(double freq,                                                  
        double mincutoff = 1.0, double beta_ = 0.0, double dcutoff = 1.0) {
        setFrequency(freq);
        setMinCutoff(mincutoff);
        setBeta(beta_);
        setDerivateCutoff(dcutoff);
        x = new LowPassFilter(alpha(mincutoff));
        dx = new LowPassFilter(alpha(dcutoff));
        lasttime = UndefinedTime;
    }

    double filter(double value, TimeStamp timestamp = UndefinedTime) {
        // update the sampling frequency based on timestamps
        if (lasttime != UndefinedTime && timestamp != UndefinedTime)
            freq = 1.0 / (timestamp - lasttime);
        lasttime = timestamp;
        // estimate the current variation per second    每秒估计当前的变化
        double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
        double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));
        // use it to update the cutoff frequency
        double cutoff = mincutoff + beta_ * fabs(edvalue);
        // filter the given value
        return x->filterWithAlpha(value, alpha(cutoff));
    }

    ~OneEuroFilter(void) {
        delete x;
        delete dx;
    }

};

//--------------------------------------------------------------------------
// Logger
// Created by SuYuan(yuansu97@yeah.net) on 2023/2/24
// You can find logger.txt under the folder 
//--------------------------------------------------------------------------

void logger(double para1, double para2, double para3) {

    std::ofstream outfile;
    std::string filePath = "logger.txt";
    outfile.open(filePath, std::ios::app);
    if (!outfile) {
        std::cout << "open logger.txt filed" << std::endl;
        exit(1);
    }
    outfile << para1 << " " << para2 << " " << para3 <<std::endl;

}


int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    std::cout << std::endl;
    std::cout << "hello world !" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "CHAI3D" << std::endl;
    std::cout << "Device: HandExoskeleton" << std::endl;
    std::cout << "Created by SuYuan(yuan_sue@yeah.net) on 2022" << std::endl;
    std::cout << "-----------------------------------" << std::endl << std::endl << std::endl;
    std::cout << std::endl << std::endl;

    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        std::cout << "failed initialization" << std::endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 1.2 * mode->height;
    int h = 0.8 * mode->height;
    int x = 0.8 * (mode->width - w);
    int y = 0.8 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        std::cout << "failed to create window" << std::endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        std::cout << "failed to initialize GLEW library" << std::endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();
 
    //set background color
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);
  
    camera->set(cVector3d(2, 2, 0),           // camera position (eye)              
                cVector3d(0, 0, 0),            // look at position (target)          
                cVector3d(0.0, 0.0, 1.0));       // direction of the (up) vector     
    
    /*
    camera->set(cVector3d(-2, 1.5, -3),           // camera position (eye)        
                cVector3d(0.5, 1, 0),            // look at position (target)       
                cVector3d(0.0, 0.0, 1.0));       // direction of the (up) vector    
    */


    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    //light = new cSpotLight(world);
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    //light->setLocalPos(0.0, 0.5, 0);
    //light->setLocalPos(-0.3, -0.3, 0.0);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // enable this light source to generate shadows
    //light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    //light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityHigh();

    // set shadow factor
    world->setShadowIntensity(0.3);

    // set light cone half angle
    //light->setCutOffAngleDeg(30);

    // create a sphere (cursor) to represent the haptic device
    hand_arm0 = new cShapeSphere(0.05);
    hand_arm1 = new cShapeSphere(0.05);
    arm0 = new cShapeSphere(0.05);
    arm1 = new cShapeSphere(0.05);
    
    thumb_0 = new cShapeSphere(0.05);
    thumb_1 = new cShapeSphere(0.05);
    thumb_2 = new cShapeSphere(0.05);

    index_0 = new cShapeSphere(0.05);
    index_1 = new cShapeSphere(0.05);
    index_2 = new cShapeSphere(0.05);
    index_3 = new cShapeSphere(0.05);

    middle_0 = new cShapeSphere(0.05);
    middle_1 = new cShapeSphere(0.05);
    middle_2 = new cShapeSphere(0.05);
    middle_3 = new cShapeSphere(0.05);

    forth_0 = new cShapeSphere(0.05);
    forth_1 = new cShapeSphere(0.05);
    forth_2 = new cShapeSphere(0.05);
    forth_3 = new cShapeSphere(0.05);

    little_0 = new cShapeSphere(0.05);
    little_1 = new cShapeSphere(0.05);
    little_2 = new cShapeSphere(0.05);
    little_3 = new cShapeSphere(0.05);

    // create lines
    IM = new cShapeLine(cVector3d(0, 0.23, -0.05), cVector3d(0, 0, 0));
    MF = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, -0.20, -0.04));
    FL = new cShapeLine(cVector3d(0, -0.2,-0.04), cVector3d(0, -0.38,-0.16));
    LH2 = new cShapeLine(cVector3d(0, -0.38, -0.16), cVector3d(0, -0.16, -0.72));
    H21 = new cShapeLine(cVector3d(0, -0.16, -0.72), cVector3d(0, 0.19, -0.72));
    H1I = new cShapeLine(cVector3d(0, 0.19, -0.72), cVector3d(0, 0.23, -0.05));
    H1T = new cShapeLine(cVector3d(0, 0.19, -0.72), cVector3d(0.03, 0.39, -0.40));
    arm00 = new cShapeLine(cVector3d(0, 0.19, -0.72), cVector3d(0, 0.19, -1.00));
    arm11 = new cShapeLine(cVector3d(0, -0.16, -0.72), cVector3d(0, -0.16, -1.00));

    T01 = new cShapeLine(cVector3d(0.00, 0.39, -0.40), cVector3d(0.00, 0.53, -0.06));
    T12 = new cShapeLine(cVector3d(0.00, 0.53, -0.06),   cVector3d(0.00, 0.64, 0.24));

    I01 = new cShapeLine(cVector3d(0, 0.23, -0.05),   cVector3d(0.00, 0.27, 0.40));
    I12 = new cShapeLine(cVector3d(0.00, 0.27, 0.40),  cVector3d(0.00, 0.28, 0.68));
    I23 = new cShapeLine(cVector3d(0.00, 0.28, 0.68),  cVector3d(0, 0.28, 0.89));

    M01 = new cShapeLine(cVector3d(0, 0, 0),    cVector3d(0.00, 0, 0.48));
    M12 = new cShapeLine(cVector3d(0.00, 0, 0.48),   cVector3d(0.00, 0, 0.80));
    M23 = new cShapeLine(cVector3d(0.00, 0, 0.80),   cVector3d(0, 0, 1.01));

    F01 = new cShapeLine(cVector3d(0, -0.20, -0.04),  cVector3d(0.00, -0.25, 0.36));
    F12 = new cShapeLine(cVector3d(0.00, -0.25, 0.36), cVector3d(0.00, -0.30, 0.67));
    F23 = new cShapeLine(cVector3d(0.00, -0.30, 0.67), cVector3d(0, -0.32, 0.88));

    L01 = new cShapeLine(cVector3d(0, -0.38, -0.16),  cVector3d(0.00, -0.47, 0.20));
    L12 = new cShapeLine(cVector3d(0.00, -0.47, 0.20), cVector3d(0.00, -0.52, 0.39));
    L23 = new cShapeLine(cVector3d(0.00, -0.52, 0.39), cVector3d(0, -0.55, 0.59));


    // insert cursor inside world
    world->addChild(hand_arm0);
    world->addChild(hand_arm1);
    world->addChild(arm0);
    world->addChild(arm1);

    world->addChild(thumb_0);
    world->addChild(thumb_1);
    world->addChild(thumb_2);

    world->addChild(index_0);
    world->addChild(index_1);
    world->addChild(index_2);
    world->addChild(index_3);

    world->addChild(middle_0);
    world->addChild(middle_1);
    world->addChild(middle_2);
    world->addChild(middle_3);

    world->addChild(forth_0);
    world->addChild(forth_1);
    world->addChild(forth_2);
    world->addChild(forth_3);

    world->addChild(little_0);
    world->addChild(little_1);
    world->addChild(little_2);
    world->addChild(little_3);

    world->addChild(T01);
    world->addChild(T12);

    world->addChild(I01);
    world->addChild(I12);
    world->addChild(I23);

    world->addChild(M01);
    world->addChild(M12);
    world->addChild(M23);

    world->addChild(F01);
    world->addChild(F12);
    world->addChild(F23);

    world->addChild(L01);
    world->addChild(L12);
    world->addChild(L23);

    world->addChild(arm00);
    world->addChild(arm11);
    world->addChild(IM);
    world->addChild(MF);
    world->addChild(FL);
    world->addChild(LH2);
    world->addChild(H21);
    world->addChild(H1I);
    world->addChild(H1T);

    //set color
    thumb_0->m_material->setBlue();
    thumb_1->m_material->setBlue();
    thumb_2->m_material->setBlue();

    index_0->m_material->setBlue();
    index_1->m_material->setBlue();
    index_2->m_material->setBlue();
    index_3->m_material->setBlue();

    middle_0->m_material->setBlue();
    middle_1->m_material->setBlue();
    middle_2->m_material->setBlue();
    middle_3->m_material->setBlue();

    forth_0->m_material->setBlue();
    forth_1->m_material->setBlue();
    forth_2->m_material->setBlue();
    forth_3->m_material->setBlue();

    little_0->m_material->setBlue();
    little_1->m_material->setBlue();
    little_2->m_material->setBlue();
    little_3->m_material->setBlue();

    hand_arm0->setLocalPos(0,0.19,-0.72);
    hand_arm1->setLocalPos(0,-0.16,-0.72);
    arm0->setLocalPos(0, 0.19, -1.0);
    arm1->setLocalPos(0, -0.16, -1.0);

    thumb_0->setLocalPos(0.00,0.39,-0.4);
    thumb_1->setLocalPos(0.00,0.53,-0.06);
    thumb_2->setLocalPos(0.00,0.64,0.24);

    index_0->setLocalPos(0,0.23,-0.05);
    index_1->setLocalPos(0.00,0.27,0.40);
    index_2->setLocalPos(0.00,0.28,0.68);
    index_3->setLocalPos(0,0.28,0.89);

    middle_0->setLocalPos(0,0,0);
    middle_1->setLocalPos(0.00, 0, 0.48);
    middle_2->setLocalPos(0.00, 0, 0.80);
    middle_3->setLocalPos(0, 0, 1.01);

    forth_0->setLocalPos(0,-0.20,-0.04);
    forth_1->setLocalPos(0.00,-0.25,0.36);
    forth_2->setLocalPos(0.00,-0.30,0.67);
    forth_3->setLocalPos(0,-0.32,0.88);

    little_0->setLocalPos(0,-0.38,-0.16);
    little_1->setLocalPos(0.00,-0.47,0.20);
    little_2->setLocalPos(0.00,-0.52,0.39);
    little_3->setLocalPos(0,-0.55,0.59);
   
    //middle_3->setShowFrame(true);
    //middle_3->setFrameSize(0.2);

    T01->setLineWidth(20);
    T12->setLineWidth(20);

    I01->setLineWidth(20);
    I12->setLineWidth(20);
    I23->setLineWidth(20);

    M01->setLineWidth(20);
    M12->setLineWidth(20);
    M23->setLineWidth(20);

    F01->setLineWidth(20);
    F12->setLineWidth(20);
    F23->setLineWidth(20);

    L01->setLineWidth(20);
    L12->setLineWidth(20);
    L23->setLineWidth(20);

    IM->setLineWidth(20);
    MF->setLineWidth(20);
    FL->setLineWidth(20);
    LH2->setLineWidth(20);
    H21->setLineWidth(20);
    H1I->setLineWidth(20);
    H1T->setLineWidth(20);
    arm00->setLineWidth(20);
    arm11->setLineWidth(20);

    //BT0->m_colorPointA.setBlack();
    //BT0->m_colorPointB.setBlack();
    
    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------
    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
        // display reference frame
    }

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);
    labelHapticDeviceRotation = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceRotation);
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, height - 60, 0);
}


void errorCallback(int a_error, const char* a_description)
{
    std::cout << "Error: " << a_description << std::endl;
}


void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable force field
    else if (a_key == GLFW_KEY_1)
    {
        useForceField = !useForceField;
        if (useForceField)
            std::cout << "> Enable force field     \r";
        else
            std::cout << "> Disable force field    \r";
    }

    // option - enable/disable damping
    else if (a_key == GLFW_KEY_2)
    {
        useDamping = !useDamping;
        if (useDamping)
            std::cout << "> Enable damping         \r";
        else
            std::cout << "> Disable damping        \r";
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}


void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;

}


void updateGraphics(void)
{

    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));
    labelHapticDeviceRotation->setText(hapticDeviceRotation.str(3));
    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
    //labelrotation->getRotation(rotation);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) std::cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{

    w1 = MAKEWORD(2, 2);
    wsa_return = WSAStartup(w1, &d1);
    s1 = socket(AF_INET, SOCK_DGRAM, 0);

    addr_local.sin_family = AF_INET;
    addr_local.sin_addr.S_un.S_addr = inet_addr(ADDRESS_PC);
    addr_local.sin_port = htons(PORT_PC);
    addr_remote.sin_family = AF_INET;
    addr_remote.sin_addr.S_un.S_addr = inet_addr(ADDRESS_STM32);
    addr_remote.sin_port = htons(PORT_STM32);

    bind(s1, (sockaddr*)&addr_local, sizeof(addr_local));

    connect(s1, (sockaddr*)&addr_remote, sizeof(addr_remote));

    socklen = sizeof(sockaddr_in);

    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    //record current time
    if (!flag_1) {

        flag_2 = false;
        std::chrono::time_point<std::chrono::high_resolution_clock> t_cur = std::chrono::high_resolution_clock::now();
        auto t_cur_milli = std::chrono::time_point_cast<std::chrono::microseconds>(t_cur);
        time_initial_ = t_cur_milli.time_since_epoch().count();
    }

    while (simulationRunning)
    {
        
        cVector3d index_1_position;
        cVector3d index_2_position;
        cVector3d index_3_position;

        Eigen::Vector3f aux_index_1_position;
        Eigen::Vector3f aux_index_2_position;
        Eigen::Vector3f aux_index_3_position;

        float index_theta1_;
        float index_theta2_;
        float index_theta3_;
        float index_theta4_;

        //
        cVector3d middle_1_position;
        cVector3d middle_2_position;
        cVector3d middle_3_position;

        Eigen::Vector3f aux_middle_1_position;
        Eigen::Vector3f aux_middle_2_position;
        Eigen::Vector3f aux_middle_3_position;

        float middle_theta1_;
        float middle_theta2_;
        float middle_theta3_;
        float middle_theta4_;

        //
        cVector3d thumb_1_position;
        cVector3d thumb_2_position;
        cVector3d thumb_3_position;

        Eigen::Vector3f aux_thumb_1_position;
        Eigen::Vector3f aux_thumb_2_position;
        Eigen::Vector3f aux_thumb_3_position;

        float thumb_theta1_;
        float thumb_theta2_;
        float thumb_theta3_;
        float thumb_theta4_;


        //sock receive
        int recvLen = recvfrom(s1, (char*)angle, sizeof(angle), 0, (sockaddr*)&addr_local, &socklen);

        //------------------------------------------------------------------------------
        // One Euro Filter
        //------------------------------------------------------------------------------

        double duration = 1000000000;     // seconds

        double frequency = 120;           // Hz
        double mincutoff = 0.1;           // FIXME
        double beta = 0.007;              // FIXME
        double dcutoff = 1.0;             // this one should be ok

        OneEuroFilter angle0(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle1(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle2(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle3(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle4(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle5(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle6(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle7(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle8(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle9(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle10(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle11(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle12(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle13(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle14(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle15(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle16(frequency, mincutoff, beta, dcutoff);
        OneEuroFilter angle17(frequency, mincutoff, beta, dcutoff);
               
        float filtered_angle0 = angle0.filter(angle[0], -1);
        float filtered_angle1 = angle1.filter(angle[1], -1);
        float filtered_angle2 = angle2.filter(angle[2], -1);
        float filtered_angle3 = angle3.filter(angle[3], -1);
        float filtered_angle4 = angle4.filter(angle[4], -1);
        float filtered_angle5 = angle5.filter(angle[5], -1);

        float filtered_angle6 = angle6.filter(angle[6], -1);
        float filtered_angle7 = angle7.filter(angle[7], -1);
        float filtered_angle8 = angle8.filter(angle[8], -1);
        float filtered_angle9 = angle9.filter(angle[9], -1);
        float filtered_angle10 = angle10.filter(angle[10], -1);
        float filtered_angle11 = angle11.filter(angle[11], -1);

        float filtered_angle12 = angle12.filter(angle[12], -1);
        float filtered_angle13 = angle13.filter(angle[13], -1);
        float filtered_angle14 = angle14.filter(angle[14], -1);
        float filtered_angle15 = angle15.filter(angle[15], -1);
        float filtered_angle16 = angle16.filter(angle[16], -1);
        float filtered_angle17 = angle17.filter(angle[17], -1);

       
        //------------------------------------------------------------------------------
        // Get finger end-effector pose 
        //------------------------------------------------------------------------------
        aux_finger1_pose = FingerMath::auxF1TransMatrix(filtered_angle0, filtered_angle1, filtered_angle2, filtered_angle3, filtered_angle4, filtered_angle5);
        //std::cout << aux_finger1_pose(0, 3) << "," << aux_finger1_pose(1, 3) << "," << aux_finger1_pose(2, 3) << std::endl;

        //finger1_pose = FingerMath::getF1TransMatrix(filtered_angle0, filtered_angle1, filtered_angle2, filtered_angle3, filtered_angle4, filtered_angle5);

        finger2_pose = FingerMath::getF2TransMatrix(filtered_angle6, filtered_angle7, filtered_angle8, filtered_angle9, filtered_angle10, filtered_angle11);

        finger3_pose = FingerMath::getF3TransMatrix(filtered_angle12, filtered_angle13, filtered_angle14, filtered_angle15, filtered_angle16, filtered_angle17);

        //------------------------------------------------------------------------------
        // Logger for Arc Fitting
        // (x,z,y)
        //------------------------------------------------------------------------------
             
        //logger(finger2_pose(0, 3), finger2_pose(2, 3), finger2_pose(1, 3));

        //logger(finger3_pose(0, 3), finger3_pose(2, 3), finger3_pose(1, 3));

        //logger(aux_finger1_pose(0, 3), aux_finger1_pose(2, 3), aux_finger1_pose(1, 3));

    


        //参数标定
        float thumb_d = 74.006;
        float index_d = 80;
        float middle_d = 86.599;

        float thumb_base_x = 36.9497;
        float thumb_base_y = -45;
        float thumb_base_z = -21.857;

        float index_base_x = 75.5823;
        float index_base_y = 20.974;
        float index_base_z = -3.8707;

        float middle_base_x = 84.08;
        float middle_base_y = -0.144;
        float middle_base_z = -11.9169;


        ///--- get finger joint ---///
        // finger_2_pose
        Eigen::Matrix4f F2_T_tip_w = finger2_pose;        
        Eigen::Matrix4f F2_0_W;
        F2_0_W(0, 0) = 1;   F2_0_W(0, 1) = 0;   F2_0_W(0, 2) = 0;    F2_0_W(0, 3) = index_base_x;     //61.33794   //40
        F2_0_W(1, 0) = 0;   F2_0_W(1, 1) = 1;   F2_0_W(1, 2) = 0;    F2_0_W(1, 3) = index_base_y;       //11.5
        F2_0_W(2, 0) = 0;   F2_0_W(2, 1) = 0;   F2_0_W(2, 2) = 1;    F2_0_W(2, 3) = index_base_z;     //-17.8601           
        F2_0_W(3, 0) = 0;   F2_0_W(3, 1) = 0;   F2_0_W(3, 2) = 0;    F2_0_W(3, 3) = 1;
        Eigen::Matrix4f F2_0_W_inv = F2_0_W.inverse();
        Eigen::Matrix4f F2_T_tip_0 = F2_0_W_inv * F2_T_tip_w;

        //std::cout << "index finger pose T_tip_0: " << std::endl;
        //std::cout << F2_T_tip_0(0, 0) << "," << F2_T_tip_0(0, 1) << "," << F2_T_tip_0(0, 2) << "," << F2_T_tip_0(0, 3) << std::endl;
        //std::cout << F2_T_tip_0(1, 0) << "," << F2_T_tip_0(1, 1) << "," << F2_T_tip_0(1, 2) << "," << F2_T_tip_0(1, 3) << std::endl;
        //std::cout << F2_T_tip_0(2, 0) << "," << F2_T_tip_0(2, 1) << "," << F2_T_tip_0(2, 2) << "," << F2_T_tip_0(2, 3) << std::endl;
        //std::cout << F2_T_tip_0(3, 0) << "," << F2_T_tip_0(3, 1) << "," << F2_T_tip_0(3, 2) << "," << F2_T_tip_0(3, 3) << std::endl;

        // finger_3_pose
        Eigen::Matrix4f F3_T_tip_w = finger3_pose;
        Eigen::Matrix4f F3_0_W;
        F3_0_W(0, 0) = 1;   F3_0_W(0, 1) = 0;   F3_0_W(0, 2) = 0;   F3_0_W(0, 3) = middle_base_x;         //57.0968     //40
        F3_0_W(1, 0) = 0;   F3_0_W(1, 1) = 1;   F3_0_W(1, 2) = 0;   F3_0_W(1, 3) = middle_base_y;         //-25
        F3_0_W(2, 0) = 0;   F3_0_W(2, 1) = 0;   F3_0_W(2, 2) = 1;   F3_0_W(2, 3) = middle_base_z;
        F3_0_W(3, 0) = 0;   F3_0_W(3, 1) = 0;   F3_0_W(3, 2) = 0;   F3_0_W(3, 3) = 1;
        Eigen::Matrix4f F3_0_W_inv = F3_0_W.inverse();
        Eigen::Matrix4f F3_T_tip_0 = F3_0_W_inv * F3_T_tip_w;


        // finger_1_pose
        Eigen::Matrix4f F1_T_tip_w = aux_finger1_pose;
        Eigen::Matrix4f F1_0_W;

        Eigen::Matrix3f Rot_y;
        Rot_y(0, 0) = cos((-1) * 45 * PI / 180);
        Rot_y(0, 1) = 0;
        Rot_y(0, 2) = sin((-1) * 45 * PI / 180);
        Rot_y(1, 0) = 0;
        Rot_y(1, 1) = 1;
        Rot_y(1, 2) = 0;
        Rot_y(2, 0) = (-1) * sin((-1) * 45 * PI / 180);
        Rot_y(2, 1) = 0;
        Rot_y(2, 2) = cos((-1) * 45 * PI / 180);

        Eigen::Matrix3f Rot_z;
        Rot_z(0, 0) = cos(-45*PI/180);
        Rot_z(0, 1) = (-1) * sin(-45*PI/180);
        Rot_z(0, 2) = 0;
        Rot_z(1, 0) = sin(-45*PI/180);
        Rot_z(1, 1) = cos(-45*PI/180);
        Rot_z(1, 2) = 0;
        Rot_z(2, 0) = 0;
        Rot_z(2, 1) = 0;
        Rot_z(2, 2) = 1;

        Eigen::Matrix3f Rot_xyz = Rot_y * Rot_z;

        F1_0_W(0, 0) = Rot_xyz(0, 0);   F1_0_W(0, 1) = Rot_xyz(0, 1);   F1_0_W(0, 2) = Rot_xyz(0, 2);   F1_0_W(0, 3) = thumb_base_x;
        F1_0_W(1, 0) = Rot_xyz(1, 0);   F1_0_W(1, 1) = Rot_xyz(1, 1);   F1_0_W(1, 2) = Rot_xyz(1, 2);   F1_0_W(1, 3) = thumb_base_y;
        F1_0_W(2, 0) = Rot_xyz(2, 0);   F1_0_W(2, 1) = Rot_xyz(2, 1);   F1_0_W(2, 2) = Rot_xyz(2, 2);   F1_0_W(2, 3) = thumb_base_z;//-20
        F1_0_W(3, 0) = 0;   F1_0_W(3, 1) = 0;   F1_0_W(3, 2) = 0;   F1_0_W(3, 3) = 1;
        Eigen::Matrix4f F1_0_W_inv = F1_0_W.inverse();
        Eigen::Matrix4f F1_T_tip_0 = F1_0_W_inv * F1_T_tip_w;

        /*
        // finger_1_pose
        Eigen::Matrix4f F1_T_tip_w = finger1_pose;  
        Eigen::Matrix4f F1_0_W;
        Eigen::Matrix3f Rot_x;
        Rot_x(0, 0) = 1;
        Rot_x(0, 1) = 0;
        Rot_x(0, 2) = 0;
        Rot_x(1, 0) = 0;
        Rot_x(1, 1) = cos((-1) * 180 * PI / 180);
        Rot_x(1, 2) = (-1) * sin((-1) * 180 * PI / 180);
        Rot_x(2, 0) = 0;
        Rot_x(2, 1) = sin((-1) * 180 * PI / 180);
        Rot_x(2, 2) = cos((-1) * 180 * PI / 180);
        Eigen::Matrix3f Rot_y;
        Rot_y(0, 0) = cos((-1) * 45 * PI / 180);
        Rot_y(0, 1) = 0;
        Rot_y(0, 2) = sin((-1) * 45 * PI / 180);
        Rot_y(1, 0) = 0;
        Rot_y(1, 1) = 1;
        Rot_y(1, 2) = 0;
        Rot_y(2, 0) = (-1) * sin((-1) * 45 * PI / 180);
        Rot_y(2, 1) = 0;
        Rot_y(2, 2) = cos((-1) * 45 * PI / 180);
        Eigen::Matrix3f Rot_xyz;
        Rot_xyz = Rot_x * Rot_y;

        F1_0_W(0, 0) = Rot_xyz(0, 0);   F1_0_W(0, 1) = Rot_xyz(0, 1);   F1_0_W(0, 2) = Rot_xyz(0, 2);   F1_0_W(0, 3) = -45;
        F1_0_W(1, 0) = Rot_xyz(1, 0);   F1_0_W(1, 1) = Rot_xyz(1, 1);   F1_0_W(1, 2) = Rot_xyz(1, 2);   F1_0_W(1, 3) = 15;
        F1_0_W(2, 0) = Rot_xyz(2, 0);   F1_0_W(2, 1) = Rot_xyz(2, 1);   F1_0_W(2, 2) = Rot_xyz(2, 2);   F1_0_W(2, 3) = -31;//-20
        F1_0_W(3, 0) = 0;               F1_0_W(3, 1) = 0;               F1_0_W(3, 2) = 0;               F1_0_W(3, 3) = 1;
        Eigen::Matrix4f F1_0_W_inv = F1_0_W.inverse();
        Eigen::Matrix4f aux_F1_T_tip_0 = F1_0_W_inv * F1_T_tip_w;

        Eigen::Matrix4f Rota_x;
        Rota_x(0, 0) = 1;
        Rota_x(0, 1) = 0;
        Rota_x(0, 2) = 0;
        Rota_x(0, 3) = 0;
        Rota_x(1, 0) = 0;
        Rota_x(1, 1) = cos(-90 * PI / 180);
        Rota_x(1, 2) = (-1) * sin(-90 * PI / 180);
        Rota_x(1, 3) = 0;
        Rota_x(2, 0) = 0;
        Rota_x(2, 1) = sin(-90 * PI / 180);
        Rota_x(2, 2) = cos(-90 * PI / 180);
        Rota_x(2, 3) = 0;   
        Rota_x(3, 0) = 0;
        Rota_x(3, 1) = 0;
        Rota_x(3, 2) = 0;
        Rota_x(3, 3) = 1;

        Eigen::Matrix4f F1_T_tip_0 = Rota_x * aux_F1_T_tip_0;

        std::cout << F1_T_tip_0(0, 3) << "," << F1_T_tip_0(1, 3) << "," << F1_T_tip_0(2, 3) << std::endl;
        */


       
        //------------------------------------------------------------------------------
        // Auto Motion Mode
        //------------------------------------------------------------------------------
        
        /*
        if (!flag_2) {
            std::chrono::time_point<std::chrono::high_resolution_clock> t_cur = std::chrono::high_resolution_clock::now();
            auto t_cur_milli = std::chrono::time_point_cast<std::chrono::microseconds>(t_cur);
            time_cur_ = t_cur_milli.time_since_epoch().count();
        }
        if (time_cur_ - time_initial_ < T) {
            index_theta2_ = -40;
            index_theta3_ = 10;
            index_theta4_ = 10;
            middle_theta2_ = 40;
            middle_theta3_ = 70;
            middle_theta4_ = 50;
            thumb_theta2_ = -15;
            thumb_theta3_ = -25;
            thumb_theta4_ = -10;
        }
        if ((time_cur_ - time_initial_ > T || time_cur_ - time_initial_ == T) && (time_cur_ - time_initial_ < 2 * T))
        {
            index_theta2_ = -10;
            index_theta3_ = 30;
            index_theta4_ = 20;
            middle_theta2_ = 20;
            middle_theta3_ = 50;
            middle_theta4_ = 30;
            thumb_theta2_ = 0;
            thumb_theta3_ = -10;
            thumb_theta4_ = 10;
        }
        if ((time_cur_ - time_initial_ > 2 * T || time_cur_ - time_initial_ == 2 * T) && (time_cur_ - time_initial_ < 3 * T))
        {
            index_theta2_ = 20;
            index_theta3_ = 50;
            index_theta4_ = 30;
            middle_theta2_ = -10;
            middle_theta3_ = 30;
            middle_theta4_ = 20;
            thumb_theta2_ = 15;
            thumb_theta3_ = 15;
            thumb_theta4_ = 30;
        }
        if ((time_cur_ - time_initial_ > 3 * T || time_cur_ - time_initial_ == 3 * T) && (time_cur_ - time_initial_ < 4 * T))
        {
            index_theta2_ = 40;
            index_theta3_ = 70;
            index_theta4_ = 50;
            middle_theta2_ = -40;
            middle_theta3_ = 10;
            middle_theta4_ = 10;
            thumb_theta2_ = 30;
            thumb_theta3_ = 25;
            thumb_theta4_ = 60;
        }
        if ((time_cur_ - time_initial_ > 4 * T || time_cur_ - time_initial_ == 4 * T))
        {
            time_initial_ = time_cur_;
        }
      
        */


        //------------------------------------------------------------------------------
        // Solute joint angles
        //------------------------------------------------------------------------------
        
        //std::cout << "x: " << F2_T_tip_0(0,3) << "  " << "y: " << F2_T_tip_0(1, 3) << "  " << "z: " << F2_T_tip_0(2, 3) << std::endl;

        float index_d1 = index_d * 0.245/(0.245 + 0.143 + 0.097);    //46
        float index_d2 = index_d * 0.143/(0.245 + 0.143 + 0.097);    //19
        float index_d3 = index_d * 0.097/(0.245 + 0.143 + 0.097);    //21

        //theta1
        float index_theta1 = atan2(F2_T_tip_0(1, 3), F2_T_tip_0(0, 3)) * 180 / PI;
        
        //theta3
        float index_x_4 = sqrt((F2_T_tip_0(0, 3) * F2_T_tip_0(0, 3)) + (F2_T_tip_0(1, 3) * F2_T_tip_0(1, 3))) - (index_d3 * F2_T_tip_0(2, 2));

        float index_y_4 = index_d3 * F2_T_tip_0(2, 0) - F2_T_tip_0(2, 3);

        float index_cos_3 = (-1) * (((index_d1 * index_d1) + (index_d2 * index_d2) - (index_x_4 * index_x_4) - (index_y_4 * index_y_4)) / (2 * index_d1 * index_d2));
      
        if (index_cos_3 < -1) {
            index_cos_3 = -1;
        }
        else if (index_cos_3 > 1) {
            index_cos_3 = 1;
        }

        float index_sin_3 = sqrt(1 - (index_cos_3 * index_cos_3));
       
        float index_theta3 = atan2(index_sin_3, index_cos_3) * 180 / PI;
      
        //theta2
        float index_beta = atan2(index_y_4, index_x_4) * 180 / PI;

        float index_cos_gama = (index_d1 * index_d1 + index_x_4 * index_x_4 + index_y_4 * index_y_4 - index_d2 * index_d2) / (2 * index_d1 * (sqrt((index_x_4 * index_x_4 + index_y_4 * index_y_4))));

        if (index_cos_gama < -1) {
            index_cos_gama = -1;
        }
        else if (index_cos_gama > 1) {
            index_cos_gama = 1;
        }
    
        float index_sin_gama = sqrt((1 - (index_cos_gama * index_cos_gama)));

        float index_gama = atan2(index_sin_gama, index_cos_gama) * 180 / PI;
 
        float index_theta2 = index_beta - index_gama;


        //theta4; 
        /*
        float index_x_3 = index_x_4 - index_d2 * cos((index_theta2 + index_theta3) * PI / 180);
        float index_y_3 = index_y_4 - index_d2 * sin((index_theta2 + index_theta3) * PI / 180);

        float index_cos_4 = ((index_d2 * index_d2) + (index_d3 * index_d3) - ((index_y_3 + F2_T_tip_0(2, 3)) * (index_y_3 + F2_T_tip_0(2, 3)))
            - ((index_x_3 - sqrt((F2_T_tip_0(0, 3) * F2_T_tip_0(0, 3)) + (F2_T_tip_0(1, 3) * F2_T_tip_0(1, 3)))) * (index_x_3 - sqrt((F2_T_tip_0(0, 3) * F2_T_tip_0(0, 3)) + (F2_T_tip_0(1, 3) * F2_T_tip_0(1, 3)))))) / ((-2) * index_d2 * index_d3);
        
        if (index_cos_4 < -1) {
            index_cos_4 = -1;
        }
        else if (index_cos_4 > 1) {
            index_cos_4 = 1;
        }
        
        float index_sin_4 = sqrt(1-(index_cos_4*index_cos_4));
        
        float index_theta4 = atan2(index_sin_4,index_cos_4) * 180 / PI;
        */

        //another
        float index_theta4 = 0.32 * index_theta3;



        //std::cout << "index finger joints: "<<index_theta1<<","<< index_theta2 <<","<<index_theta3<<","<<index_theta4<< std::endl;


        //------------------------------------------------------------------------------
        // solute middle joint angles
        //------------------------------------------------------------------------------     

        //std::cout << "x: " << F3_T_tip_0(0, 3) << "  " << "y: " << F3_T_tip_0(1, 3) << "  " << "z: " << F3_T_tip_0(2, 3) << std::endl;
        /*
        float middle_d1 = 50;   //50
        float middle_d2 = 20;   //30
        float middle_d3 = 20;   //25
        */

        float middle_d1 = middle_d * 0.266 / (0.266 + 0.170 + 0.108);
        float middle_d2 = middle_d * 0.170 / (0.266 + 0.170 + 0.108);
        float middle_d3 = middle_d * 0.108 / (0.266 + 0.170 + 0.108);

        //theta1       
        float middle_theta1 = atan2(F3_T_tip_0(1, 3), F3_T_tip_0(0, 3)) * 180 / PI;
      
        //theta3
        float middle_x_4 = sqrt((F3_T_tip_0(0, 3) * F3_T_tip_0(0, 3)) + (F3_T_tip_0(1, 3) * F3_T_tip_0(1, 3))) - (middle_d3 * F3_T_tip_0(2, 2));
        
        float middle_y_4 = middle_d3 * F3_T_tip_0(2, 0) - F3_T_tip_0(2, 3);
        
        float middle_cos_3 = (-1) * (((middle_d1 * middle_d1) + (middle_d2 * middle_d2) - (middle_x_4 * middle_x_4) - (middle_y_4 * middle_y_4)) / (2 * middle_d1 * middle_d2));
      
        if (middle_cos_3 < -1) {
            middle_cos_3 = -1;
        }
        else if (middle_cos_3 > 1) {
            middle_cos_3 = 1;
        }
       
        float middle_sin_3 = sqrt(1 - (middle_cos_3 * middle_cos_3));
        
        float middle_theta3 = atan2(middle_sin_3, middle_cos_3) * 180 / PI;

        //theta2
        float middle_beta = atan2(middle_y_4, middle_x_4) * 180 / PI;
 
        float middle_cos_gama = (middle_d1 * middle_d1 + middle_x_4 * middle_x_4 + middle_y_4 * middle_y_4 - middle_d2 * middle_d2) / (2 * middle_d1 * (sqrt((middle_x_4 * middle_x_4 + middle_y_4 * middle_y_4))));

        if (middle_cos_gama < -1) {
            middle_cos_gama = -1;
        }
        else if (middle_cos_gama>1) {
            middle_cos_gama = 1;
        }

        float middle_sin_gama = sqrt((1 - (middle_cos_gama * middle_cos_gama)));

        float middle_gama = atan2(middle_sin_gama, middle_cos_gama) * 180 / PI;
        
        float middle_theta2 = middle_beta - middle_gama;

        //theta4;
        /*
        float middle_x_3 = middle_x_4 - middle_d2 * cos((middle_theta2 + middle_theta3) * PI / 180);
        float middle_y_3 = middle_y_4 - middle_d2 * sin((middle_theta2 + middle_theta3) * PI / 180);

        float middle_cos_4 = ((middle_d2 * middle_d2) + (middle_d3 * middle_d3) - ((middle_y_3 + F3_T_tip_0(2, 3)) * (middle_y_3 + F3_T_tip_0(2, 3)))
            - ((middle_x_3 - sqrt((F3_T_tip_0(0, 3) * F3_T_tip_0(0, 3)) + (F3_T_tip_0(1, 3) * F3_T_tip_0(1, 3)))) * (middle_x_3 - sqrt((F3_T_tip_0(0, 3) * F3_T_tip_0(0, 3)) + (F3_T_tip_0(1, 3) * F3_T_tip_0(1, 3)))))) / ((-2) * middle_d2 * middle_d3);
        
        if (middle_cos_4 < -1) {
            middle_cos_4 = -1;
        }
        else if (middle_cos_4 > 1) {
            middle_cos_4 = 1;
        }
        
        float middle_sin_4 = sqrt(1 - (middle_cos_4 * middle_cos_4));

        float middle_theta4 = atan2(middle_sin_4, middle_cos_4) * 180 / PI;

        */

        float middle_theta4 = 0.36 * middle_theta3;

        //std::cout << "middle finger joints: " << middle_theta1 << "," << middle_theta2 << "," << middle_theta3 << "," << middle_theta4 << std::endl;
        
        //------------------------------------------------------------------------------
        //  solute thumb joint angles
        //------------------------------------------------------------------------------

        //std::cout << "x: " << F1_T_tip_0(0, 3) << "  " << "y: " << F1_T_tip_0(1, 3) << "  " << "z: " << F1_T_tip_0(2, 3) << std::endl;

        float thumb_d1 = thumb_d * 0.251 / (0.251 + 0.196 + 0.158);    //70
        float thumb_d2 = thumb_d * 0.196 / (0.251 + 0.196 + 0.158);   //40
        float thumb_d3 = thumb_d * 0.158 / (0.251 + 0.196 + 0.158);   //33

        //theta1       
        float thumb_theta1 = atan2(F1_T_tip_0(1, 3), F1_T_tip_0(0, 3)) * 180 / PI;

        //theta3
        float thumb_x_4 = sqrt((F1_T_tip_0(0, 3) * F1_T_tip_0(0, 3)) + (F1_T_tip_0(1, 3) * F1_T_tip_0(1, 3))) - (thumb_d3 * F1_T_tip_0(2, 2));

        float thumb_y_4 = thumb_d3 * F1_T_tip_0(2, 0) - F1_T_tip_0(2, 3);
       
        float thumb_cos_3 = (-1) * (((thumb_d1 * thumb_d1) + (thumb_d2 * thumb_d2) - (thumb_x_4 * thumb_x_4) - (thumb_y_4 * thumb_y_4)) / (2 * thumb_d1 * thumb_d2));

        if (thumb_cos_3 < -1) {
            thumb_cos_3 = -1;
        }
        else if (thumb_cos_3 > 1) {
            thumb_cos_3 = 1;
        }

        float thumb_sin_3 = sqrt(1 - (thumb_cos_3 * thumb_cos_3));

        float thumb_theta3 = atan2(thumb_sin_3, thumb_cos_3) * 180 / PI;

        //theta2
        float thumb_beta = atan2(thumb_y_4, thumb_x_4) * 180 / PI;

        float thumb_cos_gama = (thumb_d1 * thumb_d1 + thumb_x_4 * thumb_x_4 + thumb_y_4 * thumb_y_4 - thumb_d2 * thumb_d2) / (2 * thumb_d1 * (sqrt((thumb_x_4 * thumb_x_4 + thumb_y_4 * thumb_y_4))));

        if (thumb_cos_gama < -1) {
            thumb_cos_gama = -1;
        }
        else if (thumb_cos_gama > 1) {
            thumb_cos_gama = 1;
        }

        float thumb_sin_gama = sqrt((1 - (thumb_cos_gama * thumb_cos_gama)));

        float thumb_gama = atan2(thumb_sin_gama, thumb_cos_gama) * 180 / PI;

        float thumb_theta2 = thumb_beta - thumb_gama;

        //theta4;

        /*
        float thumb_x_3 = thumb_x_4 - thumb_d2 * cos((thumb_theta2 + thumb_theta3) * PI / 180);
        float thumb_y_3 = thumb_y_4 - thumb_d2 * sin((thumb_theta2 + thumb_theta3) * PI / 180);

        float thumb_cos_4 = ((thumb_d2 * thumb_d2) + (thumb_d3 * thumb_d3) - ((thumb_y_3 + F1_T_tip_0(2, 3)) * (thumb_y_3 + F1_T_tip_0(2, 3)))
            - ((thumb_x_3 - sqrt((F1_T_tip_0(0, 3) * F1_T_tip_0(0, 3)) + (F1_T_tip_0(1, 3) * F1_T_tip_0(1, 3)))) * (thumb_x_3 - sqrt((F1_T_tip_0(0, 3) * F1_T_tip_0(0, 3)) + (F1_T_tip_0(1, 3) * F1_T_tip_0(1, 3)))))) / ((-2) * middle_d2 * middle_d3);
       
        if (thumb_cos_4 < -1) {
            thumb_cos_4 = -1;
        }
        else if (thumb_cos_4 > 1) {
            thumb_cos_4 = 1;
        }
        
        float thumb_sin_4 = sqrt(1 - (thumb_cos_4 * thumb_cos_4));

        float thumb_theta4 = atan2(thumb_sin_4, thumb_cos_4) * 180 / PI;
        */

        float thumb_theta4 = 0.2 * thumb_theta3;

              

        //------------------------------------------------------------------------------
        // 
        //------------------------------------------------------------------------------

       
        index_theta1_ = index_theta1;
        index_theta2_ = index_theta2;
        index_theta3_ = index_theta3;
        index_theta4_ = index_theta4;

        middle_theta1_ = middle_theta1;
        middle_theta2_ = middle_theta2;
        middle_theta3_ = middle_theta3;
        middle_theta4_ = middle_theta4;

        thumb_theta1_ = thumb_theta1;
        thumb_theta2_ = thumb_theta2;
        thumb_theta3_ = thumb_theta3;
        thumb_theta4_ = thumb_theta4;


        //------------------------------------------------------------------------------
        // 
        //------------------------------------------------------------------------------

        aux_index_1_position = FingerMath::index_1_motion(index_theta1_, index_theta2_);
        aux_index_2_position = FingerMath::index_2_motion(index_theta1_, index_theta2_, index_theta3_);
        aux_index_3_position = FingerMath::index_3_motion(index_theta1_, index_theta2_, index_theta3_, index_theta4_);

        index_1_position.set(aux_index_1_position(0), aux_index_1_position(1), aux_index_1_position(2));
        index_2_position.set(aux_index_2_position(0), aux_index_2_position(1), aux_index_2_position(2));
        index_3_position.set(aux_index_3_position(0), aux_index_3_position(1), aux_index_3_position(2));
       
        aux_middle_1_position = FingerMath::middle_1_motion(middle_theta1, middle_theta2_);
        aux_middle_2_position = FingerMath::middle_2_motion(middle_theta1, middle_theta2_, middle_theta3_);
        aux_middle_3_position = FingerMath::middle_3_motion(middle_theta1, middle_theta2_, middle_theta3_, middle_theta4_);

        middle_1_position.set(aux_middle_1_position(0), aux_middle_1_position(1), aux_middle_1_position(2));
        middle_2_position.set(aux_middle_2_position(0), aux_middle_2_position(1), aux_middle_2_position(2));
        middle_3_position.set(aux_middle_3_position(0), aux_middle_3_position(1), aux_middle_3_position(2));
        
        aux_thumb_1_position = FingerMath::thumb_1_motion(thumb_theta2_);
        aux_thumb_2_position = FingerMath::thumb_2_motion(thumb_theta2_, thumb_theta3_);
        aux_thumb_3_position = FingerMath::thumb_3_motion(thumb_theta2_, thumb_theta3_, thumb_theta4_);

        thumb_1_position.set(aux_thumb_1_position(0), aux_thumb_1_position(1), aux_thumb_1_position(2));
        thumb_2_position.set(aux_thumb_2_position(0), aux_thumb_2_position(1), aux_thumb_2_position(2));
        thumb_3_position.set(aux_thumb_3_position(0), aux_thumb_3_position(1), aux_thumb_3_position(2));



        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////

        // update position and orientation of cursor
        
        index_1->setLocalPos(index_1_position);
        index_2->setLocalPos(index_2_position);
        index_3->setLocalPos(index_3_position);

        I01->m_pointB = index_1_position;
        I12->m_pointA = index_1_position;
        I12->m_pointB = index_2_position;
        I23->m_pointA = index_2_position;
        I23->m_pointB = index_3_position;

        middle_1->setLocalPos(middle_1_position);
        middle_2->setLocalPos(middle_2_position);
        middle_3->setLocalPos(middle_3_position);

        M01->m_pointB = middle_1_position;
        M12->m_pointA = middle_1_position;
        M12->m_pointB = middle_2_position;
        M23->m_pointA = middle_2_position;
        M23->m_pointB = middle_3_position;

        thumb_0->setLocalPos(thumb_1_position);
        thumb_1->setLocalPos(thumb_2_position);
        thumb_2->setLocalPos(thumb_3_position);

        H1T->m_pointB = thumb_1_position;
        T01->m_pointA = thumb_1_position;
        T01->m_pointB = thumb_2_position;
        T12->m_pointA = thumb_2_position;
        T12->m_pointB = thumb_3_position;

    }

    // exit haptics thread
    simulationFinished = true;

}




