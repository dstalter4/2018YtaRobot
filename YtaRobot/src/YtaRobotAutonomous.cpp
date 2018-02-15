////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routines for YtaRobot.
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
#include "RobotCamera.hpp"              // for interacting with cameras


////////////////////////////////////////////////////////////////
/// @method YtaRobot::Autonomous
///
/// The autonomous control method.  This method is called once
/// each time the robot enters autonomous control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::Autonomous()
{
    DisplayMessage("Autonomous enter...");
    
    // Put everything in a stable state
    InitialStateSetup();
    
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Autonomous needs full camera processing
    RobotCamera::SetFullProcessing(true);
    
    if (m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_LEFT_SIDE_CHARACTER)
    {
        DisplayMessage("Left side.");
    }
    else if (m_GameData[GAME_DATA_NEAR_SWITCH_INDEX] == FIELD_ELEMENT_RIGHT_SIDE_CHARACTER)
    {
        DisplayMessage("Right side.");
    }
    else
    {
        DisplayMessage("Invalid side.");
    }
    
    // Change values in the header to control having an
    // autonomous routine and which is selected
    
    // Auto routine 1
    //if ( YtaRobotAutonomous::ROUTINE_1 )
    if ( m_pAutonomous1Switch->Get() )
    {
        DisplayMessage("Auto routine 1.");
        AutonomousRoutine1();
    }
    
    // Auto routine 2
    //else if ( YtaRobotAutonomous::ROUTINE_2 )
    else if ( m_pAutonomous2Switch->Get() )
    {
        DisplayMessage("Auto routine 2.");
        AutonomousRoutine2();
    }
    
    // Auto routine 3
    //else if ( YtaRobotAutonomous::ROUTINE_3 )
    else if ( m_pAutonomous3Switch->Get() )
    {
        DisplayMessage("Auto routine 3.");
        AutonomousRoutine3();
    }

    /* !!! ONLY ENABLE TEST AUTONOMOUS CODE WHEN TESTING
           SELECT A FUNCTIONING ROUTINE FOR ACTUAL MATCHES !!! */
    else if ( YtaRobotAutonomous::TEST_ENABLED )
    {
        AutonomousTestCode();
    }

    else
    {
        // No option was selected; ensure known behavior to avoid issues
    }
    
    // Idle until auto is terminated
    DisplayMessage("Auto idle loop.");
    while ( m_pDriverStation->IsAutonomous() && m_pDriverStation->IsEnabled() )
    {
    }

    DisplayMessage("Autonomous exit...");
}   // End Autonomous



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousCommon
///
/// Common autonomous behavior.  It moves away from the alliance
/// wall and to the fuel loading station.  The variance is
/// whether it shoots at the start or at the end.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousCommon()
{

    if (m_AllianceColor == Alliance::kRed)
    {
        AutonomousCommonRed();
    }
    else if (m_AllianceColor == Alliance::kBlue)
    {
        AutonomousCommonBlue();
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousCommonRed
///
/// Common autonomous behavior when on the red alliance.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousCommonRed()
{
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///                           Red/Blue Separation                            ///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////
// @method YtaRobot::AutonomousCommonBlue
///
/// Common autonomous behavior when on the blue alliance.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousCommonBlue()
{
}
