////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous2.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 2 for YtaRobot.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   12-MAR-2017 Created.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // Robot class declaration
#include "YtaRobotAutonomous.hpp"       // Autonomous declarations



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousRoutine2
///
/// Autonomous routine 2.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine2()
{
    // 2018: Left forward is positive, Right forward is negative
    // Robot starting alignment is facing the wall, need to go backward
    double leftSpeed = -YtaRobotAutonomous::DRIVE_SPEED_SLOW;
    double rightSpeed = YtaRobotAutonomous::DRIVE_SPEED_SLOW;
    
    // Adjust for the slight curve around the cubes
    if (m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_LEFT_SIDE_CHARACTER)
    {
        // Since the robot is going backwards, a left side target is also a left turn
        leftSpeed -= YtaRobotAutonomous::DRIVE_FROM_CENTER_TO_LEFT_ADJUST;
    }
    else if (m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_RIGHT_SIDE_CHARACTER)
    {
        // Since the robot is going backwards, a right side target is also a right turn
        rightSpeed += YtaRobotAutonomous::DRIVE_FROM_CENTER_TO_RIGHT_ADJUST;
    }
    else
    {
        leftSpeed = OFF;
        rightSpeed = OFF;
    }
    
    // Start the drive
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);

    // Time it
    AutonomousDelay(YtaRobotAutonomous::DRIVE_FROM_CENTER_TIME_1_S);

    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    // Settle
    AutonomousDelay(YtaRobotAutonomous::DELAY_SHORT_S);
    
    // The robot is now curved around the cubes, need to straighten out towards the switch
    
    // Adjust for the slight curve around the cubes
    if (m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_LEFT_SIDE_CHARACTER)
    {
        // Left speed back to default, increase right speed to straighten out
        leftSpeed += YtaRobotAutonomous::DRIVE_FROM_CENTER_TO_LEFT_ADJUST;
        rightSpeed += YtaRobotAutonomous::DRIVE_FROM_CENTER_TO_LEFT_ADJUST;
    }
    else if (m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_RIGHT_SIDE_CHARACTER)
    {
        // Right speed back to default, increase left speed to straighten out
        leftSpeed -= YtaRobotAutonomous::DRIVE_FROM_CENTER_TO_RIGHT_ADJUST;
        rightSpeed -= YtaRobotAutonomous::DRIVE_FROM_CENTER_TO_RIGHT_ADJUST;
    }
    else
    {
        leftSpeed = OFF;
        rightSpeed = OFF;
    }
    
    // Start the drive
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);

    // Time it
    AutonomousDelay(YtaRobotAutonomous::DRIVE_FROM_CENTER_TIME_2_S);

    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    // Settle
    AutonomousDelay(YtaRobotAutonomous::DELAY_SHORT_S);
    
    // The robot is now (hopefully) angled against the switch, try and straighten out
    
    // Back up just a little bit, slowly, in case we bounced off the rail
    AutonomousDriveSequence(-YtaRobotAutonomous::DRIVE_INTO_SWITCH_SPEED, YtaRobotAutonomous::DRIVE_INTO_SWITCH_DELAY_S);
    
    // Move the cube
    
    // Turn the conveyor on
    m_pConveyorMotors->Set(CONVEYOR_MOTOR_SPEED_SLOW);
    
    // Wait briefly
    AutonomousDelay(YtaRobotAutonomous::CONVEYOR_MOVE_CUBE_TIME_S);
        
    // Motor back off
    m_pConveyorMotors->Set(OFF);
}
