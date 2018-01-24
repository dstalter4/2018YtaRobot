////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobot.hpp
/// @author David Stalter
///
/// @details
/// This is the class declaration for a FRC robot derived from the WPI library
/// base classes.  The SampleRobot class is the base of a robot application that
/// will automatically call the Autonomous and OperatorControl methods at the
/// right time as controlled by the switches on the driver station or the field
/// controls.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   09-JAN-2016 Created from 2015.
/// - dts   08-JAN-2017 Ported from 2016.
/// - dts   06-JAN-2018 Ported from 2017 and adopted to CTRE Phoenix.
/// - dts   20-JAN-2018 Add support for Logitech Gamepad controllers.
/// - dts   23-JAN-2018 Add support for printing to the RioLog.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOT_HPP
#define YTAROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                        // for M_PI
#include <iostream>                     // for cout

// C INCLUDES
#include "WPILib.h"                     // for FRC library
#include "CameraServer.h"               // This group of includes is for camera support
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

// C++ INCLUDES
#include "TalonMotorGroup.hpp"          // for Talon group motor control
#include "LogitechGamepad.hpp"          // for Logitech controllers

////////////////////////////////////////////////////////////////
/// @class YtaRobot
///
/// Derived class from SampleRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class YtaRobot : public SampleRobot
{
public:

    // MEMBER FUNCTIONS
    
    // Robot init routine
    void RobotInit();
    
    // Autonomous routine
    void Autonomous();
    
    // Main operator drive control routine
    void OperatorControl();
    
    // Test mode routine
    void Test();
    
    // Called when the robot enters the disabled state
    void Disabled();
    
    // Constructor, destructor
    YtaRobot();
    virtual ~YtaRobot() {}
    
    // Do not declare or implement copy constructor or assignment
    // operator!  The base classes won't like it and most likely
    // it will crash the thread when running!
      
private:

    // TYPEDEFS
    typedef DriverStation::Alliance Alliance;
    typedef Relay::Value RelayValue;
    typedef DoubleSolenoid::Value SolenoidState;
    typedef TalonMotorGroup::MotorGroupControlMode MotorGroupControlMode;
    
    // ENUMS
    enum ControllerType
    {
        LOGITECH_EXTREME,
        LOGITECH_GAMEPAD
    };
    
    enum EncoderDirection
    {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT
    };
    
    enum GyroDirection
    {
        LEFT_TURN,
        RIGHT_TURN
    };
    
    enum SonarDriveState
    {
        NONE            = 0x00,
        LEFT_GUIDE      = 0x01,
        RIGHT_GUIDE     = 0x02,
        FORWARD_GUIDE   = 0x10,
        REVERSE_GUIDE   = 0x20
    };
    
    enum SonarDriveDirection
    {
        STOPPED,
        SONAR_FORWARD,
        SONAR_REVERSE
    };
    
    // STRUCTS
    struct TriggerChangeValues
    {
        bool bCurrentValue;
        bool bOldValue;
    };
    
    struct I2cData
    {
        uint8_t m_Header;
        uint8_t m_FrontSonarA;
        uint8_t m_FrontSonarB;
        uint8_t m_LeftSonarA;
        uint8_t m_LeftSonarB;
        uint8_t m_BackSonarA;
        uint8_t m_BackSonarB;
        uint8_t m_RightSonarA;
        uint8_t m_RightSonarB;       
        uint8_t m_Footer;
    };
    
    // Displays a message to the RioLog
    inline void DisplayMessage(const char * pMessage);
    
    // Ensures a number is between the upper and lower bounds
    inline float Limit( float num, float upper, float lower );
    
    // Trims a number to be in between the upper and lower bounds
    inline float Trim( float num, float upper, float lower );

    // Detect that a button has been pressed (rather than released)
    inline bool DetectTriggerChange(TriggerChangeValues * pTriggerVals);

    // Function to check for drive control direction swap
    inline void CheckForDriveSwap();
    
    // Get a throttle control value from a joystick
    inline float GetThrottleControl(Joystick * pJoystick);
    inline float GetThrottleControl(LogitechGamepad * pGamepad);

    // Grabs a value from a sonar sensor individually
    // inline float UpdateSonarSensor(Ultrasonic * pSensor);
   
    // Get a reading from the gyro sensor
    // inline float GetGyroAngle(AnalogGyro * pSensor);
    
    // Convert a distance in inches to encoder turns
    int GetEncoderRotationsFromInches(int inches, float diameter, bool bUseQuadEncoding = true);

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(float time);

    // Autonomous drive for a specified time
    inline void AutonomousDriveSequence(float speed, float time);
    
    // Autonomous routines to back drive the motors to abruptly stop
    inline void AutonomousBackDrive(EncoderDirection currentRobotDirection);
    inline void AutonomousBackDriveTurn(GyroDirection currentGyroDirection);
    
    // Autonomous routines
    void AutonomousRoutine1();
    void AutonomousRoutine2();
    void AutonomousRoutine3();
    void AutonomousCommon();
    void AutonomousCommonRed();
    void AutonomousCommonBlue();
    bool AutonomousGyroLeftTurn(float destAngle, float turnSpeed);
    bool AutonomousGyroRightTurn(float destAngle, float turnSpeed);
    void AutonomousEncoderDrive(float speed, float distance, EncoderDirection direction);
    bool AutonomousSonarDrive(SonarDriveDirection driveDirection, SonarDriveState driveState, uint32_t destLateralDist, uint32_t destSideDist);

    // Routine to put things in a known state
    void InitialStateSetup();

    // Main sequence for drive motor control
    void DriveControlSequence();
    void SideDriveSequence();
    
    // Functions to automate slightly moving the robot
    void DirectionalInch(float speed, EncoderDirection direction);

    // Main sequence for LED control
    void LedSequence();

    // Main sequence for updating solenoid states
    void SolenoidSequence();

    // Main sequence for grabbing values from the sonars
    void SonarSensorSequence();
    
    // Main sequence for reading gyro values and related processing
    void GyroSequence();

    // Main sequence for interaction with the serial port
    void SerialPortSequence();
    
    // Main sequence for I2C interaction
    void I2cSequence();

    // Main sequence for vision processing
    void CameraSequence();
    
    // Test routines for trying out experimental code
    void AutonomousTestCode();
    void OperatorTestCode();
    void MotorTest();
    void TankDrive();
    
    // MEMBER VARIABLES
    
    // User Controls
    DriverStation                   *m_pDriverStation;                      // Driver station object for getting selections
    Joystick                        *m_pDriveJoystick;                      // Drive control option 1
    Joystick                        *m_pControlJoystick;                    // Robot functional control option 1
    LogitechGamepad                 *m_pLogitechDriveGamepad;               // Drive control option 2
    LogitechGamepad                 *m_pLogitechControlGamepad;             // Robot functional control option 2
    
    // Motors
    TalonMotorGroup                 *m_pLeftDriveMotor;                     // Left drive motor control
    TalonMotorGroup                 *m_pRightDriveMotor;                    // Right drive motor control
    TalonMotorGroup                 *m_pFrontSideDriveMotor;                // Front side drive motor control
    TalonMotorGroup                 *m_pRearSideDriveMotor;                 // Rear side drive motor control
    
    // Spike Relays
    Relay                           *m_pLedRelay;                           // Controls whether or not the LEDs are lit up
    
    // Digital I/O
    DigitalInput                    *m_pAutonomous1Switch;                  // Switch to select autonomous routine 1
    DigitalInput                    *m_pAutonomous2Switch;                  // Switch to select autonomous routine 2
    DigitalInput                    *m_pAutonomous3Switch;                  // Switch to select autonomous routine 3
    
    // Analog I/O
    AnalogGyro                      *m_pGyro;
    
    // Solenoids
    // Note: no compressor related objects required,
    // instantiating a solenoid gets that for us.
    Solenoid                        *m_pTestSingleSolenoid;
    DoubleSolenoid                  *m_pTestDoubleSolenoid;
    
    // Servos
    // (none)
    
    // Encoders
    // (none)
    
    // Timers
    Timer                           *m_pAutonomousTimer;                    // Time things during autonomous
    Timer                           *m_pInchingDriveTimer;                  // Keep track of an inching drive operation
    Timer                           *m_pI2cTimer;                           // Keep track of how often to do I2C operations
    Timer                           *m_pCameraRunTimer;                     // Keep track of how often to do camera intense code runs
    Timer                           *m_pSafetyTimer;                        // Fail safe in case critical operations don't complete
    
    // Accelerometer
    BuiltInAccelerometer            *m_pAccelerometer;                      // Built in roborio accelerometer

    // Camera
    //cs::UsbCamera                   m_Cam0;
    //cs::UsbCamera                   m_Cam1;
    //cs::VideoSink                   m_CameraServer;
    //cs::CvSink                      *m_pSink0;
    //cs::CvSink                      *m_pSink1;
    //cv::Mat                         m_Mat;
    static const int                CAMERA_X_RES                            = 320;
    static const int                CAMERA_Y_RES                            = 240;

    // Serial port configuration
    static const int                SERIAL_PORT_BUFFER_SIZE_BYTES           = 1024;
    static const int                SERIAL_PORT_NUM_DATA_BITS               = 8;
    static const int                SERIAL_PORT_BAUD_RATE                   = 115200;
    static const int                ASCII_0_OFFSET                          = 48;
    const char *                    SERIAL_PORT_PACKET_HEADER               = "Frc120";
    const int                       SERIAL_PORT_PACKET_HEADER_SIZE_BYTES    = 6;
    char                            m_SerialPortBuffer[SERIAL_PORT_BUFFER_SIZE_BYTES];

    // On board serial port
    SerialPort                      *m_pSerialPort;
    
    // I2C configuration
    static const int                I2C_DEVICE_ADDRESS                      = 4U;
    I2cData                         m_I2cData;
    I2C                             *m_pI2cPort;

    // Misc
    Alliance                        m_AllianceColor;                        // Color reported by driver station during a match
    std::string                     m_GameData;                             // Game data reported by the driver station
    bool                            m_bDriveSwap;                           // Allow the user to push a button to change forward/reverse
    bool                            m_bLed;                                 // Keep track of turning an LED on/off
    
    // CONSTS
    
    // Joysticks/Buttons
    static const ControllerType     DRIVE_CONTROLLER_TYPE                   = LOGITECH_GAMEPAD;
    static const ControllerType     CONTROL_CONTROLLER_TYPE                 = LOGITECH_EXTREME;
    
    static const int                DRIVE_JOYSTICK_PORT                     = 0;
    static const int                CONTROL_JOYSTICK_PORT                   = 1;

    // Driver buttons
    static const int                SIDE_DRIVE_LEFT_BUTTON                  = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 7 : 5;
    static const int                SIDE_DRIVE_RIGHT_BUTTON                 = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 8 : 6;
    static const int                DRIVE_CONTROLS_FORWARD_BUTTON           = 13;
    static const int                DRIVE_CONTROLS_REVERSE_BUTTON           = 14;
    
    // Control buttons
    static const int                ESTOP_BUTTON                            = 14;

    // CAN Signals
    static const int                LEFT_MOTORS_CAN_START_ID                = 1;
    static const int                RIGHT_MOTORS_CAN_START_ID               = 3;
    static const int                FRONT_SIDE_DRIVE_MOTOR_CAN_ID           = 5;
    static const int                REAR_SIDE_DRIVE_MOTOR_CAN_ID            = 6;

    // PWM Signals
    // (none)
    
    // Relays
    static const int                LED_RELAY_ID                            = 3;
    
    // Digital I/O Signals
    static const int                AUTONOMOUS_1_SWITCH                     = 7;
    static const int                AUTONOMOUS_2_SWITCH                     = 8;
    static const int                AUTONOMOUS_3_SWITCH                     = 9;
    
    // Analog I/O Signals
    static const int                ANALOG_GYRO_CHANNEL                     = 0;
    
    // Solenoid Signals
    static const int                SINGLE_SOLENOID_TEST_CHANNEL            = 0;
    static const int                DOUBLE_SOLENOID_TEST_CHANNEL_1          = 1;
    static const int                DOUBLE_SOLENOID_TEST_CHANNEL_2          = 2;
    
    // Misc
    static const int                OFF                                     = 0;
    static const int                ON                                      = 1;
    static const int                SINGLE_MOTOR                            = 1;
    static const int                NUMBER_OF_LEFT_DRIVE_MOTORS             = 2;
    static const int                NUMBER_OF_RIGHT_DRIVE_MOTORS            = 2;
    static const int                SCALE_TO_PERCENT                        = 100;
    static const int                QUADRATURE_ENCODING_ROTATIONS           = 4096;
    static const int                GAME_DATA_NEAR_SWITCH_INDEX             = 0;
    static const int                GAME_DATA_SCALE_INDEX                   = 1;
    static const int                GAME_DATA_FAR_SWITCH_INDEX              = 2;
    static const char               FIELD_ELEMENT_LEFT_SIDE_CHARACTER       = 'L';
    static const char               FIELD_ELEMENT_RIGHT_SIDE_CHARACTER      = 'R';
    static const char               NULL_CHARACTER                          = '\0';
    static const bool               DEBUG_PRINTS                            = true;
    
    static constexpr float          JOYSTICK_TRIM_UPPER_LIMIT               =  0.10F;
    static constexpr float          JOYSTICK_TRIM_LOWER_LIMIT               = -0.10F;
    static constexpr float          CONTROL_THROTTLE_VALUE_RANGE            =  0.65F;
    static constexpr float          CONTROL_THROTTLE_VALUE_BASE             =  0.35F;
    static constexpr float          DRIVE_THROTTLE_VALUE_RANGE              =  0.65F;
    static constexpr float          DRIVE_THROTTLE_VALUE_BASE               =  0.35F;
    static constexpr float          DRIVE_WHEEL_DIAMETER_INCHES             =  4.00F;
    static constexpr float          DRIVE_MOTOR_UPPER_LIMIT                 =  1.00F;
    static constexpr float          DRIVE_MOTOR_LOWER_LIMIT                 = -1.00F;
    static constexpr float          MOTOR_BACK_DRIVE_SPEED                  =  0.05F;
    static constexpr float          INCHING_DRIVE_SPEED                     =  0.25F;
    static constexpr float          INCHING_DRIVE_DELAY_S                   =  0.10F;
    
    static constexpr float          CAMERA_RUN_INTERVAL_S                   =  1.00F;
    static constexpr float          I2C_RUN_INTERVAL_S                      =  0.10F;
    static constexpr float          SAFETY_TIMER_MAX_VALUE                  =  5.00F;
    
    static const int                SONAR_LED_WARN_DIST_INCHES              = 3;
    static const uint32_t           SONAR_DRIVE_STATE_SIDE_MASK             = 0x0F;
    static const uint32_t           SONAR_DRIVE_STATE_LATERAL_MASK          = 0xF0;
    
};  // End class



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckForDriveSwap
///
/// Updates the drive control direction.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::CheckForDriveSwap()
{
    // Check if the driver pushed the button to have
    // forward be reverse and vice versa
    if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_FORWARD_BUTTON) )
    {
        m_bDriveSwap = false;
    }
    else if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_REVERSE_BUTTON) )
    {
        m_bDriveSwap = true;
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetThrottleControl
///
/// Returns a throttle value based on input from the joystick.
///
////////////////////////////////////////////////////////////////
inline float YtaRobot::GetThrottleControl(Joystick * pJoystick)
{
    // Get throttle control
    // The z axis goes from -1 to 1, so it needs to be normalized.
    // Subtract one and negate to make it zero based to give a value
    // between zero and two.  This will be used to scale the voltage
    // to the motors.  It essentially computes the max speed value
    // that can be reached.
    if (pJoystick == m_pDriveJoystick)
    {
        return ((pJoystick->GetThrottle() - 1.0F) / -2.0F) * DRIVE_THROTTLE_VALUE_RANGE + DRIVE_THROTTLE_VALUE_BASE;
    }
    else
    {
        return ((pJoystick->GetThrottle() - 1.0F) / -2.0F) * CONTROL_THROTTLE_VALUE_RANGE + CONTROL_THROTTLE_VALUE_BASE;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetThrottleControl
///
/// Returns a throttle value based on input from a Logitech
/// Gamepad controller.
///
////////////////////////////////////////////////////////////////
inline float YtaRobot::GetThrottleControl(LogitechGamepad * pGamepad)
{
    // Gamepad throttle already comes back between 0 and +1, so no need to normalize.
    return (pGamepad->GetThrottle() * DRIVE_THROTTLE_VALUE_RANGE) + DRIVE_THROTTLE_VALUE_BASE;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetGyroAngle
///
/// This method is used to get a value from an analog gyro sensor.
///
////////////////////////////////////////////////////////////////
/*
inline float YtaRobot::GetGyroAngle(AnalogGyro * pSensor)
{
    return pSensor->GetAngle();
}
*/



////////////////////////////////////////////////////////////////
/// @method YtaRobot::UpdateSonarSensor
///
/// This method is used to get a value from the sonar sensor.
/// It is intended to be used to turn a sensor briefly on and
/// get a reading from it so as to not interfere with other
/// sensors that may need to get readings.
///
////////////////////////////////////////////////////////////////
/*
inline float YtaRobot::UpdateSonarSensor(Ultrasonic * pSensor)
{
    pSensor->SetEnabled(true);
    float sensorValue = pSensor->GetRangeInches();
    pSensor->SetEnabled(false);
    return sensorValue;
}
*/



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DisplayMessage
///
/// Displays a message to the RioLog as long as debug prints are
/// enabled.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::DisplayMessage(const char * pMessage)
{
    if (DEBUG_PRINTS)
    {
        std::cout << pMessage << std::endl;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::Limit
///
/// This method is used to prevent a value outside the range
/// specified by upper and lower from being sent to physical
/// devices.
///
////////////////////////////////////////////////////////////////
inline float YtaRobot::Limit( float num, float upper, float lower )
{
    if ( num > upper )
    {
        return upper;
    }
    else if ( num < lower )
    {
        return lower;
    }

    return num;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::Trim
///
/// This method is used to ensure a signal value is above a
/// certain threshold to ensure there is actual input to a
/// device and not just noise/jitter.
///
////////////////////////////////////////////////////////////////
inline float YtaRobot::Trim( float num, float upper, float lower )
{
    if ( (num < upper) && (num > lower) )
    {
        return 0;
    }
    
    return num;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DetectTriggerChange
///
/// This method is used to check if a button has undergone a
/// state change.  The same button can be used to reverse state
/// of a particular part of the robot (such as a motor or
/// solenoid).  If the state is reversed inside an 'if'
/// statement that is part of a loop, the final state will be
/// whatever transition just occurred, which could be back to
/// the same state started in.  The intended use case is to
/// have TriggerChangeValues variables in the code and update
/// their 'current' value each time through the loop by reading
/// the joystick input.  This input will then be checked against
/// the old input and return 'true' if it detects the button has
/// been pressed.  This method is intended to be called inside
/// 'if' statements for logic control.
///
////////////////////////////////////////////////////////////////
inline bool YtaRobot::DetectTriggerChange(TriggerChangeValues * pTriggerVals)
{
    // Only report a change if the current value is different than the old value
    // Also make sure the transition is to being pressed since we are detecting
    // presses and not releases
    if ( (pTriggerVals->bCurrentValue != pTriggerVals->bOldValue) && pTriggerVals->bCurrentValue )
    {
        // Update the old value, return the button was pressed
        pTriggerVals->bOldValue = pTriggerVals->bCurrentValue;
        return true;
    }
    
    // Otherwise update the old value
    pTriggerVals->bOldValue = pTriggerVals->bCurrentValue;
    return false;
}

#endif // YTAROBOT_HPP