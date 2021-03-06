////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousGyro.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous gyroscope routines.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   12-MAR-2017 Created.
/// - dts   05-FEB-2018 Convert float -> double.
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
/// @method YtaRobot::AutonomousGyroLeftTurn
///
/// Turns the robot left based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousGyroLeftTurn(double destAngle, double turnSpeed)
{
    // 2018 LEFT FORWARD DRIVE IS POSITIVE
    // 2018 RIGHT FORWARD DRIVE IS NEGATIVE    
    // 2018 LEFT TURNS DECREASE GYRO ANGLE
    // Left turn is left motors back, right motors forward
    
    double startAngle = m_pGyro->GetAngle();
    
    // Left turns are right motors forward, left motors reverse
    m_pLeftDriveMotors->Set(-turnSpeed);
    m_pRightDriveMotors->Set(-turnSpeed);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be decreasing
    while ((m_pGyro->GetAngle() > (startAngle - destAngle)) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro angle", m_pGyro->GetAngle());
    }
    
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(LEFT_TURN);
    
    return true;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousGyroRightTurn
///
/// Turns the robot right based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousGyroRightTurn(double destAngle, double turnSpeed)
{
    // 2018 LEFT FORWARD DRIVE IS POSITIVE
    // 2018 RIGHT FORWARD DRIVE IS NEGATIVE
    // 2018 RIGHT TURNS INCREASE GYRO ANGLE
    // Right turn is left motors forward, right motors back
    
    double startAngle = m_pGyro->GetAngle();
    
    // Right turns are left motors forward, right motors reverse
    m_pLeftDriveMotors->Set(turnSpeed);
    m_pRightDriveMotors->Set(turnSpeed);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be increasing
    while ((m_pGyro->GetAngle() < (startAngle + destAngle)) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro angle", m_pGyro->GetAngle());
    }
    
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(RIGHT_TURN);
    
    return true;
}
