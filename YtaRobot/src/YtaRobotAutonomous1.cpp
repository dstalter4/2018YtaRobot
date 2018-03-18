////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous1.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 1 for YtaRobot.
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
/// @method YtaRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine1()
{
    // volatile so when it is used in the if statement below, the compiler does not optimize it
    volatile bool bSwitchDirection = m_pAutonomousLeftRightSwitch->Get();
    
    // Back up
    AutonomousDriveSequence(-YtaRobotAutonomous::DRIVE_SPEED_SLOW, YtaRobotAutonomous::DRIVE_PAST_LINE_TIME_S);
    
    // Settle
    AutonomousDelay(YtaRobotAutonomous::DELAY_SHORT_S);
    
    // Turn toward the switch
    if (!bSwitchDirection)
    {
        DisplayMessage("Left side start");
        static_cast<void>(AutonomousGyroRightTurn(YtaRobotAutonomous::TURN_TO_SWITCH_DEGREES, YtaRobotAutonomous::TURN_SPEED));
    }
    else
    {
        DisplayMessage("Right side start");
        static_cast<void>(AutonomousGyroLeftTurn(YtaRobotAutonomous::TURN_TO_SWITCH_DEGREES, YtaRobotAutonomous::TURN_SPEED));
    }
    
    // Settle
    AutonomousDelay(YtaRobotAutonomous::DELAY_SHORT_S);
    
    // Make sure we're touching the switch
    AutonomousDriveSequence(-YtaRobotAutonomous::DRIVE_SPEED_SLOW, YtaRobotAutonomous::DRIVE_TO_TOUCH_SWITCH_TIME_S);
    
    // Settle
    AutonomousDelay(YtaRobotAutonomous::DELAY_SHORT_S);
    
    if ((!bSwitchDirection && m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_LEFT_SIDE_CHARACTER) ||
         (bSwitchDirection && m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_RIGHT_SIDE_CHARACTER))
    {
        // Turn the conveyor on
        m_pConveyorMotors->Set(CONVEYOR_MOTOR_SPEED_SLOW);
        
        // Wait briefly
        AutonomousDelay(YtaRobotAutonomous::CONVEYOR_MOVE_CUBE_TIME_S);
            
        // Motor back off
        m_pConveyorMotors->Set(OFF);
    }
}
