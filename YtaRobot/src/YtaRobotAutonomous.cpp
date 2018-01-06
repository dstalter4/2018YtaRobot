////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routines for YtaRobot.
///
/// @if Edit History
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
/// @method YtaRobot::Autonomous
///
/// The autonomous control method.  This method is called once
/// each time the robot enters autonomous control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::Autonomous()
{
    // Put everything in a stable state
    InitialStateSetup();
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Change values in the header to control having an
    // autonomous routine and which is selected
    
    // Auto routine 1
    //if ( YtaRobotAutonomous::ROUTINE_1 )
    if ( m_pAutonomous1Switch->Get() )
    {
        AutonomousRoutine1();
        while ( m_pDriverStation->IsAutonomous()) {}
    }
    
    // Auto routine 2
    //else if ( YtaRobotAutonomous::ROUTINE_2 )
    else if ( m_pAutonomous2Switch->Get() )
    {
        AutonomousRoutine2();
        while ( m_pDriverStation->IsAutonomous() ) {}
    }
    
    // Auto routine 3
    //else if ( YtaRobotAutonomous::ROUTINE_3 )
    else if ( m_pAutonomous3Switch->Get() )
    {
        AutonomousRoutine3();
        while ( m_pDriverStation->IsAutonomous() ) {}
    }

    /* !!! ONLY ENABLE TEST AUTONOMOUS CODE WHEN TESTING
           SELECT A FUNCTIONING ROUTINE FOR ACTUAL MATCHES !!! */
    else if ( YtaRobotAutonomous::TEST_ENABLED )
    {
        // This code will never return
        AutonomousTestCode();
    }

    else
    {
        // No option was selected; ensure known behavior to avoid issues
        while ( m_pDriverStation->IsAutonomous() ) {}
    }

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
