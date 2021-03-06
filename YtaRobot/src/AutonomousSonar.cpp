////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousSonar.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous sonar routines.
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
/// @method YtaRobot::AutonomousSonarDrive
///
/// Autonomous method to drive the robot controlled by the
/// sonar sensors.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousSonarDrive(SonarDriveDirection driveDirection, SonarDriveState driveState, uint32_t destLateralDist, uint32_t destSideDist)
{    
    // Set directions based on drive state
    uint32_t sideDirection = driveState & SONAR_DRIVE_STATE_SIDE_MASK;
    uint32_t lateralDirection = driveState & SONAR_DRIVE_STATE_LATERAL_MASK;
    
    uint32_t frontGuideSensor = 0U;
    uint32_t backGuideSensor = 0U;
    uint32_t destGuideSensorA = 0U;
    uint32_t destGuideSensorB = 0U;
    
    // Set values based on which side is guiding drive        
    switch (lateralDirection)
    {
        case FORWARD_GUIDE:
        {
            destGuideSensorA = m_I2cData.m_FrontSonarB;
            destGuideSensorB = m_I2cData.m_FrontSonarB;
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_LeftSonarB;
                    backGuideSensor = m_I2cData.m_LeftSonarA;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_RightSonarB;
                    backGuideSensor = m_I2cData.m_RightSonarB;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            break;
        }
        case REVERSE_GUIDE:
        {
            destGuideSensorA = m_I2cData.m_BackSonarA;
            destGuideSensorB = m_I2cData.m_BackSonarB;
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_LeftSonarA;
                    backGuideSensor = m_I2cData.m_LeftSonarB;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_RightSonarB;
                    backGuideSensor = m_I2cData.m_RightSonarB;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Start with defaults of off and no turning
    double leftDriveSpeed = OFF;
    double rightDriveSpeed = OFF;    
    bool bLeftTurn = false;
    bool bRightTurn = false;
    bool bCanOverrideTurn = true;
    
    // Make sure we're close enough to a guiding structure
    if (    (frontGuideSensor < YtaRobotAutonomous::SONAR_MIN_DRIVE_ENABLE_INCHES)
         && (backGuideSensor < YtaRobotAutonomous::SONAR_MIN_DRIVE_ENABLE_INCHES) )
    {
        // Start assuming a straight drive
        leftDriveSpeed = YtaRobotAutonomous::SONAR_DRIVE_LEFT_SPEED;
        rightDriveSpeed = YtaRobotAutonomous::SONAR_DRIVE_RIGHT_SPEED;
        
        // Check for turning need.  The first checks here determine
        // if we need to turn the robot left or right, and are to
        // align the robot at a (mostly) right angle.
        if (frontGuideSensor > backGuideSensor)
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            // If the robot is offset too sharply, don't allow
            // the guiding below to override what we want to do.
            if ((frontGuideSensor - backGuideSensor) > YtaRobotAutonomous::SONAR_MAX_ALLOWED_READING_DIFF)
            {
                bCanOverrideTurn = false;
            }
        }
        else if (backGuideSensor > frontGuideSensor)
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            // If the robot is offset too sharply, don't allow
            // the guiding below to override what we want to do.
            if ((backGuideSensor - frontGuideSensor) > YtaRobotAutonomous::SONAR_MAX_ALLOWED_READING_DIFF)
            {
                bCanOverrideTurn = false;
            }
        }
        else
        {
        }
        
        // Align with the destination distance.  These checks, unlike the ones
        // above, are to move towards the target distance from the wall.
        if (bCanOverrideTurn && (frontGuideSensor > destSideDist))
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        
        // Set the motor speed values
        if (bLeftTurn)
        {
            leftDriveSpeed -= YtaRobotAutonomous::SONAR_COMPENSATE_LEFT_SPEED;
            rightDriveSpeed += YtaRobotAutonomous::SONAR_COMPENSATE_RIGHT_SPEED;
        }
        else if (bRightTurn)
        {
            leftDriveSpeed += YtaRobotAutonomous::SONAR_COMPENSATE_LEFT_SPEED;
            rightDriveSpeed -= YtaRobotAutonomous::SONAR_COMPENSATE_RIGHT_SPEED;
        }
        else
        {
        }
        
        // Speeds are now set based on need to turn.  Enable motors
        // only if we have not reached the maximum distance.
        if ((destGuideSensorA < destLateralDist) && (destGuideSensorB < destLateralDist))
        {
            if (driveDirection == SONAR_FORWARD)
            {
                m_pLeftDriveMotors->Set(leftDriveSpeed);
                m_pRightDriveMotors->Set(rightDriveSpeed);
            }
            else if (driveDirection == SONAR_REVERSE)
            {
                m_pLeftDriveMotors->Set(-leftDriveSpeed);
                m_pRightDriveMotors->Set(-rightDriveSpeed);
            }
            else
            {
            }
            
            return false;
        }
    }
    
    return true;
}
