
// Created by SuYuan on 2022/11/17 

#include <WinSock2.h>
#include <Windows.h>
#include <iostream>
#pragma comment(lib , "ws2_32.lib")

#define ADDRESS_STM32 "192.168.1.30"
#define PORT_STM32 8089
#define ADDRESS_PC "192.168.1.10"
#define PORT_PC 8089

#include "chai3d.h"
#include <GLFW/glfw3.h>
#include <GLFW/FingerKinematics.h>

#include "Eigen/Eigen"
#include <math.h>

#define PI 3.1415926
#define M_PI 3.1415926

//------------------------------------------------------------------------------
// One Euro Filter
//------------------------------------------------------------------------------

#include<stdexcept>
#include<cmath>
#include<ctime>

//------------------------------------------------------------------------------
using namespace chai3d;
//------------------------------------------------------------------------------

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
//cSpotLight* light;
cDirectionalLight* light;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* thumb_cursor;
cShapeSphere* index_cursor;
cShapeSphere* middle_cursor;
//world frame
cShapeSphere* world_frame;

//a line from world_frame to thumb_cursor
cShapeLine* lineWorldToF1;
cShapeLine* lineWorldToF2;
cShapeLine* lineWorldToF3;

cShapeSphere* test_cursor;


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

// a label to display the rate [Hz] at which the simulation is running//显示模拟运行速率[赫兹]的标签
cLabel* labelRates;
cLabel* labelrotation;

// a flag for using damping (ON/OFF)//使用阻尼的标志（开/关）
bool useDamping = false;

// a flag for using force field (ON/OFF)//使用力场的标志（开/关）
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running//指示触觉模拟当前是否正在运行的标志
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated//指示触觉模拟是否已终止的标志
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate//测量模拟图形速率的频率计数器
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate//测量模拟触觉速度的频率计数器
cFrequencyCounter freqCounterHaptics;

// haptic thread//触觉线
cThread* hapticsThread;

// a handle to window display context//窗口显示上下文的句柄
GLFWwindow* window = NULL;

// current width of window//当前窗口宽度
int width = 0;

// current height of window//当前窗口高度
int height = 0;

// swap interval for the display context (vertical synchronization)//显示上下文的交换间隔（垂直同步）
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized//调整窗口显示大小时回调
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs//发生错误GLFW时回调
void errorCallback(int error, const char* a_description);

// callback when a key is pressed//按键时的回调
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene//此函数用于渲染场景
void updateGraphics(void);

// this function contains the main haptics simulation loop//此函数包含主触觉模拟循环
void updateHaptics(void);

// this function closes the application//此函数用于关闭应用程序
void close(void);

//==============================================================================
/*
    DEMO:   01-mydevice.cpp
*/
//==============================================================================

WSADATA	d1;
WORD w1;
int wsa_return;
SOCKET s1;
sockaddr_in addr_local;
sockaddr_in addr_remote;
float angle[18] = { 0 };
int socklen;

Eigen::Matrix4f finger1_pose;	//thumb
Eigen::Matrix4f finger2_pose;	//index
Eigen::Matrix4f finger3_pose;	//middle

Eigen::Vector4f finger1_joint;
Eigen::Vector4f finger2_joint;

//--------------------------------------------------------------------------
// One Euro Filter
//--------------------------------------------------------------------------

typedef double TimeStamp; // in seconds

static const TimeStamp UndefinedTime = -1.0;

// -----------------------------------------------------------------

class LowPassFilter {           //低通滤波器    

    double y, a, s;
    bool initialized;

    void setAlpha(double alpha) {
        if (alpha <= 0.0 || alpha > 1.0)
            throw std::range_error("alpha should be in (0.0., 1.0]");
        a = alpha;
    }

public:

    LowPassFilter(double alpha, double initval = 0.0) {
        y = s = initval;        //初始时刻 y=s=0
        setAlpha(alpha);        //输入a=alpha
        initialized = false;
    }

    double filter(double value) {       //value为过滤的数据值
        double result;
        if (initialized)
            result = a * value + (1.0 - a) * s;     //s为上一时刻过滤后
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

    double lastRawValue(void) {     //最后一个没有被过滤的数字
        return y;
    }

};

// -----------------------------------------------------------------

class OneEuroFilter {

    double freq;         //采样频率
    double mincutoff;    //最小截止频率
    double beta_;
    double dcutoff;
    LowPassFilter* x;
    LowPassFilter* dx;
    TimeStamp lasttime;

    double alpha(double cutoff) {                                        //计算alpha，输入截止频率fc
        double te = 1.0 / freq;
        double tau = 1.0 / (2 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / te);
    }

    void setFrequency(double f) {                                               //设置采样频率，采样频率用来计算采样周期te
        if (f <= 0) throw std::range_error("freq should be >0");
        freq = f;
    }

    void setMinCutoff(double mc) {                                              //设置最小截止频率
        if (mc <= 0) throw std::range_error("mincutoff should be >0");
        mincutoff = mc;
    }

    void setBeta(double b) {                                                    //设置bata
        beta_ = b;
    }

    void setDerivateCutoff(double dc) {
        if (dc <= 0) throw std::range_error("dcutoff should be >0");
        dcutoff = dc;
    }

public:

    OneEuroFilter(double freq,                                                  //构造函数,会调用上述设置
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


int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION初始化
    //--------------------------------------------------------------------------

    std::cout << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "CHAI3D" << std::endl;
    std::cout << "Demo: 01-mydevice" << std::endl;
    std::cout << "Copyright 2003-2016" << std::endl;
    std::cout << "-----------------------------------" << std::endl << std::endl << std::endl;
    std::cout << "Keyboard Options:" << std::endl << std::endl;
    std::cout << "[1] - Enable/Disable potential field" << std::endl;
    std::cout << "[2] - Enable/Disable damping" << std::endl;
    std::cout << "[f] - Enable/Disable full screen mode" << std::endl;
    std::cout << "[m] - Enable/Disable vertical mirroring" << std::endl;
    std::cout << "[q] - Exit application" << std::endl;
    std::cout << std::endl << std::endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY窗口显示
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
    // set the background color of the environment
    world->m_backgroundColor.setGrayGainsboro();
    //world->m_backgroundColor.setBlack();


    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    //   camera->set( cVector3d(-0.5, 0, 0),    // camera position (eye)
    //                cVector3d(0.5, 0, 0),     // look at position (target)
    //                cVector3d(0.0, 0.0, 1.0));    // direction of the (up) vector


    camera->set(cVector3d(-2.5, 0, 0),           // camera position (eye)           相机的位置
                cVector3d(0.5, 0, 0),            // look at position (target)       期望的目标点
                cVector3d(0.0, 0.0, 1.0));       // direction of the (up) vector    定义向上位置的向量



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


    // create a sphere (cursor) to represent the haptic device创造球体表现触觉设备
    thumb_cursor  = new cShapeSphere(0.02);
    index_cursor  = new cShapeSphere(0.02);
    middle_cursor = new cShapeSphere(0.02);
    world_frame = new cShapeSphere(0.02);

    test_cursor = new cShapeSphere(0.02);
   
    // create small line to illustrate the velocity of the haptic device
    velocity = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));

    lineWorldToF1 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));
    lineWorldToF2 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));
    lineWorldToF3 = new cShapeLine(cVector3d(0, 0, 0), cVector3d(1.0, 1.0, 1.0));

   
    // insert cursor inside world在世界中插入球体
    world->addChild(thumb_cursor);
    world->addChild(index_cursor);
    world->addChild(middle_cursor);
    world->addChild(world_frame);

    world->addChild(test_cursor);

    // insert line inside world
    world->addChild(velocity);
    world->addChild(lineWorldToF1);
    world->addChild(lineWorldToF2);
    world->addChild(lineWorldToF3);

    world_frame->m_material->setBlack();
    thumb_cursor->m_material->setBlack();
    index_cursor->m_material->setBlack();
    middle_cursor->m_material->setBlack();
    

    lineWorldToF1->setLineWidth(4);
    lineWorldToF2->setLineWidth(4);
    lineWorldToF3->setLineWidth(4);

    lineWorldToF1->m_colorPointA.setBlack();
    lineWorldToF1->m_colorPointB.setBlack();
    lineWorldToF2->m_colorPointA.setBlack();
    lineWorldToF2->m_colorPointB.setBlack();
    lineWorldToF3->m_colorPointA.setBlack();
    lineWorldToF3->m_colorPointB.setBlack();



    //--------------------------------------------------------------------------
    // HAPTIC DEVICE触觉设备
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

    thumb_cursor->setShowFrame(true);
    thumb_cursor->setFrameSize(0.1);

    index_cursor->setShowFrame(true);
    index_cursor->setFrameSize(0.1);

    middle_cursor->setShowFrame(true);
    middle_cursor->setFrameSize(0.1);

    world_frame->setShowFrame(true);
    world_frame->setFrameSize(0.1);

    test_cursor->setShowFrame(true);
    test_cursor->setFrameSize(0.1);
 
    //--------------------------------------------------------------------------
    // WIDGETS小部件
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic device model//创建标签以显示触觉设备模型
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    //创建标签以显示触觉设备的位置姿态
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);
    labelHapticDeviceRotation = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceRotation);
    // create a label to display the haptic and graphic rate of the simulation
    //创建标签以显示模拟的触觉和图形速率
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION开始模拟
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop//创建启动主触觉渲染循环的线程
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits//应用程序退出时设置回调
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP主图形回路
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics更新触觉设备信息
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

//------------------------------------------------------------------------------


//关闭
void close(void)//关闭
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

//------------------------------------------------------------------------------


//更新图形
void updateGraphics(void)//更新图形
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));
    labelHapticDeviceRotation->setText(hapticDeviceRotation.str(3));
    // update haptic and graphic rate data更新触觉和图形速率数据
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label更新标签位置
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
    //labelrotation->getRotation(rotation);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE渲染场景
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


    while (simulationRunning)
    {

        cVector3d thumb_position;
        cVector3d index_position;
        cVector3d middle_position;

        cMatrix3d thumb_rotation;
        cMatrix3d index_rotation;
        cMatrix3d middle_rotation;

        double thumb_x;
        double thumb_y;
        double thumb_z;

        double index_x;
        double index_y;
        double index_z;

        double middle_x;
        double middle_y;
        double middle_z;

        double aux_thumb_rot[9];
        double aux_index_rot[9];
        double aux_middle_rot[9];

        //sock receive

        int recvLen = recvfrom(s1, (char*)angle, sizeof(angle), 0, (sockaddr*)&addr_local, &socklen);

        //------------------------------------------------------------------------------
        // One Euro Filter
        //------------------------------------------------------------------------------

        double duration = 1000000000;     // seconds

        double frequency = 120;     // Hz
        double mincutoff = 0.05;     // FIXME
        double beta = 0.0001;          // FIXME
        double dcutoff = 1.0;       // this one should be ok

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

        /*
        std::cout << "raw angles : " <<angle[0]<<","<<angle[1]<<","<<angle[2]<<","<<angle[3]<<","<<angle[4]<<","<<angle[5]<<","<<
                                       angle[6]<<","<<angle[7]<<","<<angle[8]<<","<<angle[9]<<","<<angle[10]<<","<<angle[11]<<","<<
                                       angle[12]<<","<<angle[13]<<","<<angle[14]<<","<<angle[15]<<","<<angle[16]<<","<<angle[17]<<std::endl;
        */
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

        /*
        std::cout << "filtered angles : " << filtered_angle0 <<","<< filtered_angle1<<","<< filtered_angle2<<","<< filtered_angle3<<","<<filtered_angle4<<","<< filtered_angle5<<
                                       ","<< filtered_angle6 <<","<< filtered_angle7<<","<< filtered_angle8<<","<< filtered_angle9<<","<< filtered_angle10<<","<< filtered_angle11<<
                                       ","<< filtered_angle12<<","<< filtered_angle13<<","<< filtered_angle14<<","<< filtered_angle15<<","<< filtered_angle16<<","<< filtered_angle17<<std::endl;
        */


        ///--- calibration angles ---/// 
        //  -,-,-,-,+,-
        //std::cout << angle[0] << "," << angle[1] << "," << angle[2] << "," << angle[3] << "," << angle[4] << "," << angle[5] << std::endl;
        //  -,-,-,-,+,-
        //std::cout << angle[6] << "," << angle[7] << "," << angle[8] << "," << angle[9] << "," << angle[10] << "," << angle[11] << std::endl;
        //  -,-,-,-,+,-
        //std::cout << angle[12] << "," << angle[13] << "," << angle[14] << "," << angle[15] << "," << angle[16] << "," << angle[17] << std::endl;



        finger1_pose = FingerMath::getF1TransMatrix(filtered_angle0, filtered_angle1, filtered_angle2, filtered_angle3, filtered_angle4,  filtered_angle5);

        finger2_pose = FingerMath::getF2TransMatrix(filtered_angle6, filtered_angle7, filtered_angle8, filtered_angle9, filtered_angle10, filtered_angle11);

        finger3_pose = FingerMath::getF3TransMatrix(filtered_angle12, filtered_angle13, filtered_angle14, filtered_angle15, filtered_angle16, filtered_angle17);


        ///--- print finger position ---///
        //std::cout << "thumb_position: "  << finger1_pose(0, 3) << "," << finger1_pose(1, 3) << "," << finger1_pose(2, 3) << std::endl;
        //std::cout << "index position: "  << finger2_pose(0, 3) << "," << finger2_pose(1, 3) << "," << finger2_pose(2, 3) << std::endl;
        //std::cout << "middle position: " << finger3_pose(0, 3) << "," << finger3_pose(1, 3) << "," << finger3_pose(2, 3) << std::endl;


        ///--- get finger joint ---///
        
        Eigen::Matrix4f F2_T_tip_w = finger2_pose;
        Eigen::Matrix4f F2_0_W;
        F2_0_W(0, 0) = 1;   F2_0_W(0, 1) = 0;   F2_0_W(0, 2) = 0;    F2_0_W(0, 3) = 54;
        F2_0_W(1, 0) = 0;   F2_0_W(1, 1) = 1;   F2_0_W(1, 2) = 0;    F2_0_W(1, 3) = 21;
        F2_0_W(2, 0) = 0;   F2_0_W(2, 1) = 0;   F2_0_W(2, 2) = 1;    F2_0_W(2, 3) = 0;
        F2_0_W(3, 0) = 0;   F2_0_W(3, 1) = 0;   F2_0_W(3, 2) = 0;    F2_0_W(3, 3) = 1;
        Eigen::Matrix4f F2_0_W_inv = F2_0_W.inverse();
        Eigen::Matrix4f F2_T_tip_0 = F2_0_W_inv * F2_T_tip_w;

        
        // std::cout << "check F2_T_tip_0: " << std::endl;
        // std::cout << F2_T_tip_0(0, 0) << "," << F2_T_tip_0(0, 1) << "," << F2_T_tip_0(0, 2) << "," << F2_T_tip_0(0, 3) << std::endl;
        // std::cout << F2_T_tip_0(1, 0) << "," << F2_T_tip_0(1, 1) << "," << F2_T_tip_0(1, 2) << "," << F2_T_tip_0(1, 3) << std::endl;
        // std::cout << F2_T_tip_0(2, 0) << "," << F2_T_tip_0(2, 1) << "," << F2_T_tip_0(2, 2) << "," << F2_T_tip_0(2, 3) << std::endl;
        // std::cout << F2_T_tip_0(3, 0) << "," << F2_T_tip_0(3, 1) << "," << F2_T_tip_0(3, 2) << "," << F2_T_tip_0(3, 3) << std::endl;
        
          
        /*
        cVector3d test_position;
        cMatrix3d test_rotation;
        test_position.set(F2_T_tip_0(0, 3)*0.01, F2_T_tip_0(1, 3)*0.01, F2_T_tip_0(2, 3)*0.01);
        test_rotation.set(F2_T_tip_0(0, 0), F2_T_tip_0(0, 1), F2_T_tip_0(0, 2), F2_T_tip_0(1, 0), F2_T_tip_0(1, 1), F2_T_tip_0(1, 2), F2_T_tip_0(2, 0), F2_T_tip_0(2, 1), F2_T_tip_0(2, 2));
        test_cursor->setLocalPos(test_position);
        test_cursor->setLocalRot(test_rotation);
        */


        //test finger lenth
        //std::cout << F2_T_tip_0(0, 3) << "," << F2_T_tip_0(1, 3) << "," << F2_T_tip_0(2, 3) << std::endl;
                
        float d1 = 44.4695;
        float d2 = 27.7617;
        float e1 = 18.8467;


        ///--- solute F2_theta1 ---///

        float F2_theta1 = atan2(F2_T_tip_0(1,3),F2_T_tip_0(0,3))*180/PI;
        //std::cout << "theta1: " << F2_theta1 << std::endl;
     
        ///--- solute F2_theta3 ---///

        float x_4 = sqrt((F2_T_tip_0(0,3)*F2_T_tip_0(0,3))+(F2_T_tip_0(1,3)*F2_T_tip_0(1,3))) - (e1*F2_T_tip_0(2,2));
        //std::cout <<"x4: " << x_4 << std::endl;

        float y_4 = e1 * F2_T_tip_0(2, 0) - F2_T_tip_0(2, 3);
        //std::cout <<"y4: " << y_4 << std::endl;

        float cos_3 = (-1) * (((d1 * d1) + (d2 * d2) - (x_4 * x_4) - (y_4 * y_4)) / (2 * d1 * d2));
        //std::cout << "cos3: " << cos_3 << std::endl;
      
        float sin_3 = sqrt(1 - (cos_3 * cos_3));
        //std::cout << "sin3: " << sin_3 << std::endl;
        float F2_theta3 = atan2(sin_3, cos_3)*180/PI;
        

        //std::cout << "theta3: " << F2_theta3 << std::endl;
        
        
        ///--- solute F2_theta2 ---///

        float _beta = atan2(y_4, x_4)*180/PI;
        //std::cout << beta << std::endl;

        float cos_gama = (d1 * d1 + x_4 * x_4 + y_4 * y_4 - d2 * d2) / (2*d1*(sqrt((x_4*x_4+y_4*y_4))));
        float sin_gama = sqrt((1-(cos_gama*cos_gama)));

        float gama = atan2(sin_gama,cos_gama)*180/PI;
        //std::cout << gama << std::endl;

        float F2_theta2 = _beta - gama;

        //std::cout << F2_theta2 << std::endl;









        

        //modify data
        thumb_x = finger1_pose(0, 3) * 0.01;
        thumb_y = finger1_pose(1, 3) * 0.01;
        thumb_z = finger1_pose(2, 3) * 0.01;
        aux_thumb_rot[0] = finger1_pose(0, 0);
        aux_thumb_rot[1] = finger1_pose(0, 1);
        aux_thumb_rot[2] = finger1_pose(0, 2);
        aux_thumb_rot[3] = finger1_pose(1, 0);
        aux_thumb_rot[4] = finger1_pose(1, 1);
        aux_thumb_rot[5] = finger1_pose(1, 2);
        aux_thumb_rot[6] = finger1_pose(2, 0);
        aux_thumb_rot[7] = finger1_pose(2, 1);
        aux_thumb_rot[8] = finger1_pose(2, 2);

        thumb_position.set(thumb_x, thumb_y, thumb_z);
        thumb_rotation.set(aux_thumb_rot[0], aux_thumb_rot[1], aux_thumb_rot[2],
                           aux_thumb_rot[3], aux_thumb_rot[4], aux_thumb_rot[5],
                           aux_thumb_rot[6], aux_thumb_rot[7], aux_thumb_rot[8]);

        //modify data
        index_x = finger2_pose(0, 3) * 0.01;
        index_y = finger2_pose(1, 3) * 0.01;
        index_z = finger2_pose(2, 3) * 0.01;
        aux_index_rot[0] = finger2_pose(0, 0);
        aux_index_rot[1] = finger2_pose(0, 1);
        aux_index_rot[2] = finger2_pose(0, 2);
        aux_index_rot[3] = finger2_pose(1, 0);
        aux_index_rot[4] = finger2_pose(1, 1);
        aux_index_rot[5] = finger2_pose(1, 2);
        aux_index_rot[6] = finger2_pose(2, 0);
        aux_index_rot[7] = finger2_pose(2, 1);
        aux_index_rot[8] = finger2_pose(2, 2);

        index_position.set(index_x, index_y, index_z);
        index_rotation.set(aux_index_rot[0], aux_index_rot[1], aux_index_rot[2], 
                           aux_index_rot[3], aux_index_rot[4], aux_index_rot[5], 
                           aux_index_rot[6], aux_index_rot[7], aux_index_rot[8]);

        //modify data
        middle_x = finger3_pose(0, 3) * 0.01;
        middle_y = finger3_pose(1, 3) * 0.01;
        middle_z = finger3_pose(2, 3) * 0.01;

        aux_middle_rot[0] = finger3_pose(0, 0);
        aux_middle_rot[1] = finger3_pose(0, 1);
        aux_middle_rot[2] = finger3_pose(0, 2);
        aux_middle_rot[3] = finger3_pose(1, 0);
        aux_middle_rot[4] = finger3_pose(1, 1);
        aux_middle_rot[5] = finger3_pose(1, 2);
        aux_middle_rot[6] = finger3_pose(2, 0);
        aux_middle_rot[7] = finger3_pose(2, 1);
        aux_middle_rot[8] = finger3_pose(2, 2);

        middle_position.set(middle_x, middle_y, middle_z);
        middle_rotation.set(aux_middle_rot[0], aux_middle_rot[1], aux_middle_rot[2],
                            aux_middle_rot[3], aux_middle_rot[4], aux_middle_rot[5],
                            aux_middle_rot[6], aux_middle_rot[7], aux_middle_rot[8]);
   







    // read gripper position夹持器位置
    double gripperAngle;
    hapticDevice->getGripperAngleRad(gripperAngle);

    // read linear velocity线速度
    cVector3d linearVelocity;
    hapticDevice->getLinearVelocity(linearVelocity);

    // read angular velocity角速度
    cVector3d angularVelocity;
    hapticDevice->getAngularVelocity(angularVelocity);

    // read gripper angular velocity夹持器角速度
    double gripperAngularVelocity;
    hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

    // read user-switch status (button 0)读取用户开关
    bool button0, button1, button2, button3;
    button0 = false;
    button1 = false;
    button2 = false;
    button3 = false;

    hapticDevice->getUserSwitch(0, button0);
    hapticDevice->getUserSwitch(1, button1);
    hapticDevice->getUserSwitch(2, button2);
    hapticDevice->getUserSwitch(3, button3);


    /////////////////////////////////////////////////////////////////////
    // UPDATE 3D CURSOR MODEL更新三维光标模型
    /////////////////////////////////////////////////////////////////////

    // update arrow
    velocity->m_pointA = index_position;
    velocity->m_pointB = cAdd(index_position, linearVelocity);



    // update position and orientation of cursor
    thumb_cursor->setLocalPos(thumb_position);
    thumb_cursor->setLocalRot(thumb_rotation);

    index_cursor->setLocalPos(index_position);
    index_cursor->setLocalRot(index_rotation);

    middle_cursor->setLocalPos(middle_position);
    middle_cursor->setLocalRot(middle_rotation);

    //
    lineWorldToF1->m_pointB = thumb_position;
    lineWorldToF2->m_pointB = index_position;
    lineWorldToF3->m_pointB = middle_position;




    hapticDevicePosition = index_position;
    hapticDeviceRotation = index_rotation;


 

    // signal frequency counter信号频率计数器
    freqCounterHaptics.signal(1);
}

// exit haptics thread
simulationFinished = true;

}




