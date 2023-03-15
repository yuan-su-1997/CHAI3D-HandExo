//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Your name, institution, or company name.
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================


//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CMyCustomDevice.h"

//------------------------------------------------------------------------------
#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file CMyCustomDevice.h for some initial 
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    below.
*/
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
// WIN32
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)


extern "C"
{
	//int GetAavilableNumDevices = 1;
int (__stdcall *hdEOHNumDevices)  ();

int (__stdcall *hdEOHOpen)           (const int a_deviceID);

int (__stdcall *hdEOHClose)          (const int a_deviceID);

int (__stdcall *hdEOHGetPosition)    (const int a_deviceID,
                                          double *a_posX,
                                          double *a_posY,
                                          double *a_posZ);

int (__stdcall *hdEOHGetRotation)    (const int a_deviceID,
                                          double *a_rot00,
                                          double *a_rot01,
                                          double *a_rot02,
                                          double *a_rot10,
                                          double *a_rot11,
                                          double *a_rot12,
                                          double *a_rot20,
                                          double *a_rot21,
                                          double *a_rot22);

}
//------------------------------------------------------------------------------
#endif	
//==============================================================================
/*!
    Constructor of cMyCustomDevice.
*/
//==============================================================================
cMyCustomDevice::cMyCustomDevice(unsigned int a_deviceNumber)
{
    // the connection to your device has not yet been established.未建立为false
    m_deviceReady = false;

	


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 1:

        Here you should define the specifications of your device.
        These values only need to be estimates. Since haptic devices often perform
        differently depending of their configuration withing their workspace,
        simply use average values.
		步骤1：
		您应该在此处定义设备的规格。
		这些值只需要是估计值。因为触觉设备经常执行
		根据其工作空间的配置不同，
		只需使用平均值。
    */
    ////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    // NAME:Exoskeleton of hand
    //--------------------------------------------------------------------------

    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_HAPTIC_DEVICE_CUSTOM;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "Remote control research group";

    // name of your device
    m_specifications.m_modelName                     = "Exoskeleton of hand";


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following values must be positive or equal to zero)
    //--------------------------------------------------------------------------

    // the maximum force [N] the device can produce along the x,y,z axis.
    m_specifications.m_maxLinearForce                = 0;     // [N]

    // the maximum amount of torque your device can provide arround its
    // rotation degrees of freedom.
    m_specifications.m_maxAngularTorque              = 0;     // [N*m]


    // the maximum amount of torque which can be provided by your gripper
    m_specifications.m_maxGripperForce                = 0;     // [N]

    // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
    m_specifications.m_maxLinearStiffness             = 0; // [N/m]

    // the maximum amount of angular stiffness
    m_specifications.m_maxAngularStiffness            = 0;    // [N*m/Rad]

    // the maximum amount of stiffness supported by the gripper
    m_specifications.m_maxGripperLinearStiffness      = 0;   // [N*m]

    // the radius of the physical workspace of the device (x,y,z axis)
    m_specifications.m_workspaceRadius                = 0.075;     // [m]

    // the maximum opening angle of the gripper
    m_specifications.m_gripperMaxAngleRad             = cDegToRad(0.0);


    ////////////////////////////////////////////////////////////////////////////
    /*
        DAMPING PROPERTIES:

        Start with small values as damping terms can be high;y sensitive to 
        the quality of your velocity signal and the spatial resolution of your
        device. Try gradually increasing the values by using example "01-devices" 
        and by enabling viscosity with key command "2".
		阻尼特性：
		从较小的值开始，因为阻尼项可能较高；y敏感于
		速度信号的质量和
		装置尝试使用示例“01设备”逐渐增加值
		并通过按键命令“2”启用粘度。
    */
    ////////////////////////////////////////////////////////////////////////////
    
    // Maximum recommended linear damping factor Kv最大推荐线性阻尼系数Kv
    m_specifications.m_maxLinearDamping             = 1;   // [N/(m/s)]

    //! Maximum recommended angular damping factor Kv (if actuated torques are available)
    m_specifications.m_maxAngularDamping            = 0.0;    // [N*m/(Rad/s)]

    //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
    m_specifications.m_maxGripperAngularDamping     = 0.0;    // [N*m/(Rad/s)]


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following are of boolean type: (true or false)
    //--------------------------------------------------------------------------

    // does your device provide sensed position (x,y,z axis)?您的设备是否提供感应位置（x、y、z轴）？
    m_specifications.m_sensedPosition                = true;

    // does your device provide sensed rotations (i.e stylus)?您的设备是否提供感应旋转（即手写笔）？
    m_specifications.m_sensedRotation                = true;

    // does your device provide a gripper which can be sensed?您的设备是否提供可感测的夹持器？
    m_specifications.m_sensedGripper                 = false;

    // is you device actuated on the translation degrees of freedom?设备是否根据平移自由度启动？
    m_specifications.m_actuatedPosition              = true;

    // is your device actuated on the rotation degrees of freedom?您的设备是否根据旋转自由度启动
    m_specifications.m_actuatedRotation              = false;

    // is the gripper of your device actuated?设备的抓手是否已启动？
    m_specifications.m_actuatedGripper               = false;

    // can the device be used with the left hand?该设备能否与左手配合使用
    m_specifications.m_leftHand                      = true;

    // can the device be used with the right hand?该设备可以用右手使用吗？
    m_specifications.m_rightHand                     = true;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 2:

        Here, you shall  implement code which tells the application if your
        device is actually connected to your computer and can be accessed.
        In practice this may be consist in checking if your I/O board
        is active or if your drivers are available.
		在这里，您应该实现代码，告诉应用程序
		该设备实际上已连接到您的计算机，并且可以访问。
		实际上，这可能包括检查输入/输出板
		处于活动状态或您的驱动程序可用。

        If your device can be accessed, set:
        m_systemAvailable = true;如果可以访问您的设备，请设置： m_systemAvailable = true

        Otherwise set:否则
        m_systemAvailable = false;

        Your actual code may look like:您的实际代码可能如下所示：

        bool result = checkIfMyDeviceIsAvailable()
        m_systemAvailable = result;

        If want to support multiple devices, using the method argument
        a_deviceNumber to know which device to setup如果要支持多个设备，请使用method参数知道要设置哪个设备的\u设备编号
    */  
    ////////////////////////////////////////////////////////////////////////////
        

    // *** INSERT YOUR CODE HERE ***
    m_MyVariable = 0;	

    m_deviceAvailable = false; // this value should become 'true' when the device is available.

	int avilableNumDevices=1;
    // check if such device is available
    if (avilableNumDevices)
    {
        // no, such ID does not lead to an existing device
        m_deviceAvailable = true;
    }
    else
    {
        // yes, this ID leads to an existing device
        m_deviceAvailable = false;
    }
}


//==============================================================================
/*!
    Destructor of cMyCustomDevice.
*/
//==============================================================================
cMyCustomDevice::~cMyCustomDevice()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
}


//==============================================================================
/*!
    This method opens a connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_SUCCESS);

		    // update device status
    m_deviceReady = true;
	    // return success
    return (C_SUCCESS);
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 3:

        Here you shall implement to open a connection to your
        device. This may include opening a connection to an interface board
        for instance or a USB port.

        If the connection succeeds, set the variable 'result' to true.
        otherwise, set the variable 'result' to false.

        Verify that your device is calibrated. If your device 
        needs calibration then call method calibrate() for wich you will 
        provide code in STEP 5 further below.
    */
    ////////////////////////////////////////////////////////////////////////////

//    bool result = C_SUCCESS; // this value will need to become "C_SUCCESS" for the device to be marked as ready.
//
//    // *** INSERT YOUR CODE HERE ***
//     //result = openConnectionToMyDevice();
//
//
//  /*   update device status*/
//    if (result)
//    {
//        m_deviceReady = true;
//        return (C_SUCCESS);
//    }
//    else
//    {
//        m_deviceReady = false;
//        return (C_ERROR);
//    }
}


//==============================================================================
/*!
    This method closes the connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::close()
{
     //check if the system has been opened previously//检查系统之前是否已打开
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 4:

        Here you shall implement code that closes the connection to your
        device.

        If the operation fails, simply set the variable 'result' to C_ERROR   .
        If the connection succeeds, set the variable 'result' to C_SUCCESS.
		在这里，您应该实现关闭与装置
		如果操作失败，只需将变量“result”设置为C\U ERROR。
		如果连接成功，请将变量“result”设置为C\u SUCCESS。
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.设备开着才能关闭，若关闭了再运行close就报错

    // *** INSERT YOUR CODE HERE ***
  //result = closeConnectionToMyDevice();

    // update status
    m_deviceReady = false;

    return (result);
}


//==============================================================================
/*!
    This method calibrates your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::calibrate(bool a_forceCalibration)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 5:
        
        Here you shall implement code that handles a calibration procedure of the 
        device. In practice this may include initializing the registers of the
        encoder counters for instance. 

        If the device is already calibrated and  a_forceCalibration == false,
        the method may immediately return without further action.
        If a_forceCalibration == true, then the calibrartion procedure
        shall be executed even if the device has already been calibrated.
 
        If the calibration procedure succeeds, the method returns C_SUCCESS,
        otherwise return C_ERROR.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    // error = calibrateMyDevice()

    return (result);
}


//==============================================================================
/*!
    This method returns the number of devices available from this class of device.

    \return __true__ if the operation succeeds, __false__ otherwise.
	此方法返回此类设备中可用的设备数。
	\如果操作成功，则返回\uu true\uuuu，否则返回\uu false\uuuu。
*/
//==============================================================================
unsigned int cMyCustomDevice::getNumDevices()
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 6:

        Here you shall implement code that returns the number of available
        haptic devices of type "cMyCustomDevice" which are currently connected
        to your computer.

        In practice you will often have either 0 or 1 device. In which case
        the code here below is already implemented for you.

        If you have support more than 1 devices connected at the same time,
        then simply modify the code accordingly so that "numberOfDevices" takes
        the correct value.
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE, MODIFY修改 CODE below ACCORDINGLY ***
  //numberOfDevices = getNumberOfDevicesConnectedToTheComputer();
    int avilableNumDevices =1;  // At least set to 1 if a device is available.

   

    return (avilableNumDevices);
}


//==============================================================================
/*!
    This method returns the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
	此方法返回设备的位置。单位为米【m】。
	\参数a\u位置返回值。
	\如果操作成功，则返回\uu true\uuuu，否则返回\uu false\uuuu。
*/
//==============================================================================
bool cMyCustomDevice::getPosition(cVector3d& a_position)
{
    // check if the device is read. See step 3.检查设备时候已经连接
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 7:

        Here you shall implement code that reads the position (X,Y,Z) from
        your haptic device. Read the values from your device and modify
        the local variable (x,y,z) accordingly.
        If the operation fails return an C_ERROR, C_SUCCESS otherwise

        Note:
        For consistency, units must be in meters.
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky. 
		在这里，您应该实现从以下位置读取位置（X，Y，Z）的代码
		您的触觉设备。从设备读取值并修改
		相应的局部变量（x，y，z）。
		如果操作失败，则返回C\U错误，否则返回C\U成功
		注：
		为了保持一致性，单位必须以米为单位。
		如果设备位于前方，则x轴指向
		朝向您（操作员）。y轴指向您的右侧
		手侧和z轴指向天空。
    */
    ////////////////////////////////////////////////////////////////////////////



    bool result = C_SUCCESS;
    double x,y,z;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    x = 0.05;    // x = getMyDevicePositionX()
    y = 0.05;    // y = getMyDevicePositionY()
    z = 0;    // z = getMyDevicePositionZ()

    // store new position values
    a_position.set(x, y, z);

    // estimate linear velocity估计线速度
    estimateLinearVelocity(a_position);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
	此方法返回设备末端效应器的方向帧
	\参数a\u旋转返回值。
	\如果操作成功，则返回\uu true\uuuu，否则返回\uu false\uuuu。
*/
//==============================================================================
bool cMyCustomDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 8:

        Here you shall implement code which reads the orientation frame from
        your haptic device. The orientation frame is expressed by a 3x3
        rotation matrix. The 1st column of this matrix corresponds to the
        x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
        The length of each column vector should be of length 1 and vectors need
        to be orthogonal to each other.

        Note:
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        If your device has a stylus, make sure that you set the reference frame
        so that the x-axis corresponds to the axis of the stylus.
		在这里，您将实现从
		您的触觉设备。方向框由3x3表示
		旋转矩阵。该矩阵的第一列对应于
		x轴，第二列到y轴，第三列到z轴。
		每个列向量的长度应为1，并且向量需要
		相互正交的相互正交的。
		注：
		如果设备位于前方，则x轴指向
		朝向您（操作员）。y轴指向您的右侧
		手侧和z轴指向天空。
		如果您的设备有触控笔，请确保设置了参考帧
		使x轴与手写笔的轴相对应。
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // variables that describe the rotation matrix
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    cMatrix3d frame;
    frame.identity();

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    // if the device does not provide any rotation capabilities 
    // set the rotation matrix equal to the identity matrix.
    r00 = 1.0;  r01 = 0.0;  r02 = 0.0;
    r10 = 0.0;  r11 = 1.0;  r12 = 0.0;
    r20 = 0.0;  r21 = 0.0;  r22 = 1.0;

    frame.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);

    // store new rotation matrix
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the gripper angle in radian.

    \param   a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::getGripperAngleRad(double& a_angle)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 9:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as C_ERROR for instance.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    // return gripper angle in radian
    a_angle = 0.0;  // a_angle = getGripperAngleInRadianFromMyDevice();

    // estimate gripper velocity
    estimateGripperVelocity(a_angle);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method sends a force [N] and a torque [N*m] and gripper torque [N*m] 
    to your haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 10:
        
        Here you may implement code which sends a force (fx,fy,fz),
        torque (tx, ty, tz) and/or gripper force (gf) command to your haptic device.

        If your device does not support one of more of the force, torque and 
        gripper force capabilities, you can simply ignore them. 

        Note:
        For consistency, units must be in Newtons and Newton-meters
        If your device is placed in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        For instance: if the force = (1,0,0), the device should move towards
        the operator, if the force = (0,0,1), the device should move upwards.
        A torque (1,0,0) would rotate the handle counter clock-wise around the 
        x-axis.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    // retrieve force, torque, and gripper force components in individual variables
    double fx = a_force(0);
    double fy = a_force(1);
    double fz = a_force(2);

    double tx = a_torque(0);
    double ty = a_torque(1);
    double tz = a_torque(2);

    double gf = a_gripperForce;

    // *** INSERT YOUR CODE HERE ***

    // setForceToMyDevice(fx, fy, fz);
    // setTorqueToMyDevice(tx, ty, tz);
    // setForceToGripper(fg);


    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns status of all user switches 
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 11:

        Here you shall implement code that reads the status all user switches 
        on your device. For each user switch, set the associated bit on variable
        a_userSwitches. If your device only has one user switch, then set 
        a_userSwitches to 1, when the user switch is engaged, and 0 otherwise.
		在此，您应实现读取所有用户开关状态的代码
		在您的设备上。对于每个用户开关，设置变量上的相关位
		a\u用户开关。如果您的设备只有一个用户交换机，则设置
		当用户开关接通时，a\u userSwitches为1，否则为0。
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE ***
    a_userSwitches = 0;

    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
