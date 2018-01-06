////////////////////////////////////////////////////////////////////////////////
/// @file   RobotTestCode.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the YtaRobot test functions.  This keeps official stable
/// robot code isolated.
///
/// @if Edit History
/// - dts   03-JAN-2015 Created.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"         // Robot class declaration



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousTestCode
///
/// Test code to try out for autonomous mode.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousTestCode()
{
    // Motors off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    // Done, just loop
    while ( m_pDriverStation->IsAutonomous() )
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::OperatorTestCode
///
/// Test code to try out for operator control mode.
///
////////////////////////////////////////////////////////////////
void YtaRobot::OperatorTestCode()
{
    // Test code for reading the built in accelerometer
    double x = m_pAccelerometer->GetX();
    double y = m_pAccelerometer->GetY();
    double z = m_pAccelerometer->GetZ();
    printf("x: %f, y: %f, z: %f\n", x, y, z);

    // Sample code for testing the detect trigger change code
    TriggerChangeValues testValues;
    testValues.bCurrentValue = m_pControlJoystick->GetRawButton(10);
    if ( DetectTriggerChange(&testValues) )
    {
        printf("Trigger change detected!\n");
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::MotorTest
///
/// Motor test code to make sure they aren't driving against
/// each other.
///
////////////////////////////////////////////////////////////////
Joystick * pDriveJoystick;
Joystick * pControlJoystick;
TalonSRX * pLeft1;
TalonSRX * pLeft2;
TalonSRX * pRight1;
TalonSRX * pRight2;
void YtaRobot::MotorTest()
{
    static bool bInitialized = false;
    if (!bInitialized)
    {
        pDriveJoystick = new Joystick(DRIVE_JOYSTICK);
        pControlJoystick = new Joystick(CONTROL_JOYSTICK);
        
        pLeft1 = new TalonSRX(LEFT_MOTORS_CAN_START_ID);
        pLeft2 = new TalonSRX(LEFT_MOTORS_CAN_START_ID + 1);
        pRight1 = new TalonSRX(RIGHT_MOTORS_CAN_START_ID);
        pRight2 = new TalonSRX(RIGHT_MOTORS_CAN_START_ID + 1);
        
        //pLeft1->SetControlMode(CANSpeedController::kPercentVbus);
        //pLeft2->SetControlMode(CANSpeedController::kPercentVbus);
        //pRight1->SetControlMode(CANSpeedController::kPercentVbus);
        //pRight2->SetControlMode(CANSpeedController::kPercentVbus);
        
        //pLeft1->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        //pLeft2->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        //pRight1->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        //pRight2->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        
        bInitialized = true;
    }
    
    while (pDriveJoystick->GetRawButton(6))
    {
        pLeft1->Set(ControlMode::PercentOutput, 1);
    }
    while (pDriveJoystick->GetRawButton(7))
    {
        pLeft1->Set(ControlMode::PercentOutput, -1);
    }
    while (pDriveJoystick->GetRawButton(8))
    {
        pLeft2->Set(ControlMode::PercentOutput, 1);
    }
    while (pDriveJoystick->GetRawButton(9))
    {
        pLeft2->Set(ControlMode::PercentOutput, -1);
    }
    while (pControlJoystick->GetRawButton(6))
    {
        pRight1->Set(ControlMode::PercentOutput, 1);
    }
    while (pControlJoystick->GetRawButton(7))
    {
        pRight1->Set(ControlMode::PercentOutput, -1);
    }
    while (pControlJoystick->GetRawButton(8))
    {
        pRight2->Set(ControlMode::PercentOutput, 1);
    }
    while (pControlJoystick->GetRawButton(9))
    {
        pRight2->Set(ControlMode::PercentOutput, -1);
    }
    
    pLeft1->Set(ControlMode::PercentOutput, 0);
    pLeft2->Set(ControlMode::PercentOutput, 0);
    pRight1->Set(ControlMode::PercentOutput, 0);
    pRight2->Set(ControlMode::PercentOutput, 0);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TankDrive
///
/// Test code for tank drive of the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::TankDrive()
{
    m_pLeftDriveMotor->Set(-m_pDriveJoystick->GetY());
    m_pRightDriveMotor->Set(m_pControlJoystick->GetY());
}
