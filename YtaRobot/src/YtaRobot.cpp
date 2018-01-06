////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobot.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the YtaRobot class.  This file contains the functions for
/// full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// @if Edit History
/// - dts   09-JAN-2016 Created from 2015.
/// - dts   08-JAN-2017 Ported from 2016.
/// - dts   06-JAN-2018 Ported from 2017 amd adopted to CTRE Phoenix.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>          // for nullptr
#include <cstring>          // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"    // For class declaration (and other headers)

// Do not use static initialization!  There is a bug in the
// WPI libraries that will cause an exception during object
// instantiation for the robot.


////////////////////////////////////////////////////////////////
/// @method YtaRobot::YtaRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
YtaRobot::YtaRobot()
: m_pDriverStation              (&DriverStation::GetInstance())
, m_pDriveJoystick              (new Joystick(DRIVE_JOYSTICK))
, m_pControlJoystick            (new Joystick(CONTROL_JOYSTICK))
, m_pLeftDriveMotor             (new TalonMotorGroup(NUMBER_OF_LEFT_DRIVE_MOTORS, LEFT_MOTORS_CAN_START_ID, GroupControlMode::FOLLOW, FeedbackDevice::CTRE_MagEncoder_Relative))
, m_pRightDriveMotor            (new TalonMotorGroup(NUMBER_OF_RIGHT_DRIVE_MOTORS, RIGHT_MOTORS_CAN_START_ID, GroupControlMode::FOLLOW, FeedbackDevice::CTRE_MagEncoder_Relative))
, m_pLedRelay                   (new Relay(LED_RELAY_ID))
, m_pAutonomous1Switch          (new DigitalInput(AUTONOMOUS_1_SWITCH))
, m_pAutonomous2Switch          (new DigitalInput(AUTONOMOUS_2_SWITCH))
, m_pAutonomous3Switch          (new DigitalInput(AUTONOMOUS_3_SWITCH))
, m_pGyro                       (new AnalogGyro(ANALOG_GYRO_CHANNEL))
, m_pAutonomousTimer            (new Timer())
, m_pInchingDriveTimer          (new Timer())
, m_pI2cTimer                   (new Timer())
, m_pCameraRunTimer             (new Timer())
, m_pSafetyTimer                (new Timer())
, m_pAccelerometer              (new BuiltInAccelerometer())
//, m_Cam0                        (CameraServer::GetInstance()->StartAutomaticCapture(0U))
//, m_Cam1                        (CameraServer::GetInstance()->StartAutomaticCapture(1U))
//, m_CameraServer                (CameraServer::GetInstance()->GetServer())
//, m_pSink0                      (new cs::CvSink("cam0sink"))
//, m_pSink1                      (new cs::CvSink("cam1sink"))
//, m_Mat                         ()
, m_SerialPortBuffer            ()
, m_pSerialPort                 (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One))
, m_I2cData                     ()
, m_pI2cPort                    (new I2C(I2C::kMXP, I2C_DEVICE_ADDRESS))
, m_AllianceColor               (m_pDriverStation->GetAlliance())
, m_bDriveSwap                  (false)
, m_bLed                        (false)
{
    // Reset all timers
    m_pAutonomousTimer->Reset();
    m_pInchingDriveTimer->Reset();
    m_pI2cTimer->Reset();
    m_pCameraRunTimer->Reset();
    m_pSafetyTimer->Reset();

    // Reset the serial port
    m_pSerialPort->Reset();

    // Set camera quality
    //m_Cam0.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    //m_Cam1.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    
    //m_Cam0.SetVideoMode(cs::VideoMode::kMJPEG, 160, 120, 10);
    //m_Cam1.SetVideoMode(cs::VideoMode::kMJPEG, 160, 120, 10);
    
    //m_pSink0->SetSource(m_Cam0);
    //m_pSink0->SetEnabled(true);
    //m_pSink1->SetSource(m_Cam1);
    //m_pSink1->SetEnabled(true);
    //m_CameraServer.SetSource(m_Cam0);
    //m_pCameraRunTimer->Start();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::InitialStateSetup()
{
    // Start with motors off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    // Tare encoders
    m_pLeftDriveMotor->TareEncoder();
    m_pRightDriveMotor->TareEncoder();
    
    // Stop/clear any timers, just in case
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Start I2C timer
    m_pI2cTimer->Reset();
    m_pI2cTimer->Start();
    
    // Just in case constructor was called before this was set
    m_AllianceColor = m_pDriverStation->GetAlliance();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::OperatorControl
///
/// The user control method.  This method is called once each
/// time the robot enters operator control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::OperatorControl()
{
    // Autonomous should have left things in a known state, but
    // just in case clear everything.  Timers were reset in the
    // constructor, no need to do it again
    InitialStateSetup();
    
    // Teleop will use coast mode
    //m_pLeftDriveMotor->SetCoastMode();
    //m_pRightDriveMotor->SetCoastMode();
    // Teleop will use brake mode
    m_pLeftDriveMotor->SetBrakeMode();
    m_pRightDriveMotor->SetBrakeMode();
    
    // Main tele op loop
    while ( m_pDriverStation->IsOperatorControl() )
    {
        //CheckForDriveSwap();

        DriveControlSequence();
        
        //LedSequence();

        //SolenoidSequence();

        //SonarSensorSequence();
        
        //GyroSequence();

        //SerialPortSequence();
        
        //I2cSequence();
        
        //CameraSequence();
        
        
        
        // TEST CODE
        // Recommended to only enable this in test scenarios
        // to not impact matches
        //OperatorTestCode();
        //MotorTest();
        // END TEST CODE
        
    } // End main OperatorControl loop
} // End OperatorControl



////////////////////////////////////////////////////////////////
/// @method YtaRobot::LedSequence
///
/// This method contains the main workflow for controlling
/// any LEDs on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::LedSequence()
{
    // If the target's in range, give a visual indication
    if (m_bLed)
    {
        m_pLedRelay->Set(RelayValue::kOn);
    }
    else
    {
        // Otherwise set them off
        m_pLedRelay->Set(RelayValue::kOff);
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SolenoidSequence
///
/// This method contains the main workflow for updating the
/// state of the solenoids on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SolenoidSequence()
{
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SonarSensorSequence
///
/// This method contains the main workflow for getting updates
/// from the sonar sensors.  In order to not interfere with
/// each other, each sensor is enabled/disabled and checked
/// individually.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SonarSensorSequence()
{
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GyroSequence
///
/// This method contains the main workflow for getting updates
/// from the gyro sensors and processing related to those
/// readings.
///
////////////////////////////////////////////////////////////////
void YtaRobot::GyroSequence()
{
    if (DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Gyro: ", m_pGyro->GetAngle());
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SerialPortSequence
///
/// This method contains the main workflow for interaction with
/// the serial port.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SerialPortSequence()
{
/*
    // Check for any incoming transmissions, limit it to our read buffer size
    int32_t bytesReceived = m_pSerialPort->GetBytesReceived();
    bytesReceived = (bytesReceived > SERIAL_PORT_BUFFER_SIZE_BYTES) ? SERIAL_PORT_BUFFER_SIZE_BYTES : bytesReceived;

    // If we got data, read it
    if (bytesReceived > 0)
    {
        static_cast<void>(m_pSerialPort->Read(m_SerialPortBuffer, bytesReceived));

        // See if its a packet intended for us
        if (memcmp(m_SerialPortBuffer, SERIAL_PORT_PACKET_HEADER, SERIAL_PORT_PACKET_HEADER_SIZE_BYTES) == 0)
        {
            // Next character is the command.  Array indexing starts at zero, thus no +1 on the size bytes constant
            int32_t command = static_cast<int32_t>(m_SerialPortBuffer[SERIAL_PORT_PACKET_HEADER_SIZE_BYTES]) - ASCII_0_OFFSET;

            // Sanity check it
            if (command >= 0 && command <= 9)
            {
                printf("Received a valid packet, command: %d\n", command);
            }
            else
            {
                printf("Invalid command received: %d\n", command);
            }
        }

        printf(m_SerialPortBuffer);
    }
    m_SerialPortBuffer[0] = NULL_CHARACTER;
*/
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::I2cSequence
///
/// This method contains the main workflow for interaction with
/// the I2C bus.
///
////////////////////////////////////////////////////////////////
void YtaRobot::I2cSequence()
{
    //uint8_t I2C_STRING[] = "i2c_roborio";
    std::memset(&m_I2cData, 0U, sizeof(m_I2cData));
    
    if (m_pI2cTimer->Get() > I2C_RUN_INTERVAL_S)
    {
        // Get the data from the riodiuino
        //static_cast<void>(m_pI2cPort->Transaction(I2C_STRING, sizeof(I2C_STRING), reinterpret_cast<uint8_t *>(&m_I2cData), sizeof(m_I2cData)));
        
        // Restart the timer
        m_pI2cTimer->Reset();
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CameraSequence()
{    
    // Do the camera processing
    //bool bDoFullVisionProcessing = false;
    
    // To not kill the CPU/hog this thread, only do full
    // vision processing (particle analysis) periodically.
    if (m_pCameraRunTimer->Get() >= CAMERA_RUN_INTERVAL_S)
    {
        //bDoFullVisionProcessing = true;
        m_pCameraRunTimer->Reset();
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DriveControlSequence
///
/// This method contains the main workflow for drive control.
/// It will gather input from the drive joystick and then filter
/// those values to ensure they are past a certain threshold and
/// within range to send to the speed controllers.  Lastly it
/// will actually set the speed values.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DriveControlSequence()
{
    // Computes what the maximum drive speed could be
    float throttleControl = GetThrottleControl(m_pDriveJoystick);

    // Get joystick inputs and make sure they clear a certain threshold.
    // This will help to drive straight.
    float xAxisDrive = Trim((m_pDriveJoystick->GetX() * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    float yAxisDrive = Trim((m_pDriveJoystick->GetY() * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // If the swap direction button was pressed, negate y value
    if ( m_bDriveSwap )
    {
        yAxisDrive *= -1;
    }

    // Filter motor speeds
    //float leftSpeed = Limit((xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    //float rightSpeed = Limit((xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    float leftSpeed = Limit((-xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    float rightSpeed = Limit((-xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    
    // Set motor speed
    m_pLeftDriveMotor->Set(leftSpeed);
    m_pRightDriveMotor->Set(rightSpeed);
    
    /*
    // First check for inching controls
    if (m_pDriveJoystick->GetRawButton(INCH_FORWARD_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, FORWARD);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_BACKWARD_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, REVERSE);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_LEFT_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, LEFT);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_RIGHT_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, RIGHT);
    }
    else
    {
        // Set motor speed
        m_pLeftDriveMotor->Set(leftSpeed);
        m_pRightDriveMotor->Set(rightSpeed);
    }
    */
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DirectionalInch
///
/// This method contains the main workflow for drive directional
/// inching.  Based on input direction, it will briefly move the
/// robot a slight amount in that direction.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DirectionalInch(float speed, EncoderDirection direction)
{
    // 2017 LEFT FORWARD DRIVE IS NEGATIVE
    // 2017 RIGHT FORWARD DRIVE IS POSITIVE
    float leftSpeed = speed;
    float rightSpeed = speed;
    
    // Negate appropriate motor speeds, based on direction
    switch (direction)
    {
        case FORWARD:
        {
            leftSpeed *= -1.0F;
            break;
        }
        case REVERSE:
        {
            rightSpeed *= -1.0F;
            break;
        }
        case LEFT:
        {
            break;
        }
        case RIGHT:
        {
            leftSpeed *= -1.0F;
            rightSpeed *= -1.0F;
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Start the timer
    m_pInchingDriveTimer->Reset();
    m_pInchingDriveTimer->Start();
    
    // Motors on
    m_pLeftDriveMotor->Set(leftSpeed);
    m_pRightDriveMotor->Set(rightSpeed);
    
    while (m_pInchingDriveTimer->Get() < INCHING_DRIVE_DELAY_S)
    {
    }
    
    // Motors back off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    // Stop the timer
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::Test
///
/// This method is run when entering test mode.
///
////////////////////////////////////////////////////////////////
void YtaRobot::Test()
{
    while (true)
    {
    }
}



// EXECUTION START
START_ROBOT_CLASS(YtaRobot);        // Macro to instantiate and run the class
