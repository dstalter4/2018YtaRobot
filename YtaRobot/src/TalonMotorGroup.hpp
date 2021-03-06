////////////////////////////////////////////////////////////////////////////////
/// @file   TalonMotorGroup.hpp
/// @author David Stalter
///
/// @details
/// A class designed to work with a group of CAN Talon speed controllers working
/// in tandem.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   03-JAN-2015 Created from 2014.
/// - dts   17-JAN-2015 Ported to CAN Talons.
/// - dts   06-FEB-2015 Support for follow and inverse control.
/// - dts   08-JAN-2017 Ported to use TalonSRX class.
/// - dts   06-JAN-2018 Adopted to CTRE Phoenix.
/// - dts   05-FEB-2018 Convert float -> double.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef TALONMOTORGROUP_HPP
#define TALONMOTORGROUP_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "ctre/Phoenix.h"               // For CTRE libraries

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @class TalonMotorGroup
///
/// Class that provides methods for interacting with a group of
/// Talon speed controllers.
///
////////////////////////////////////////////////////////////////
class TalonMotorGroup
{
public:
    
    enum MotorGroupControlMode
    {
        FOLLOW,
        INDEPENDENT,
        INVERSE,
        INDEPENDENT_OFFSET,
        INVERSE_OFFSET
    };

    // Constructor
    TalonMotorGroup( int numMotors, int firstCANId, MotorGroupControlMode controlMode, FeedbackDevice sensor = FeedbackDevice::None );
    
    // Function to set the speed of each motor in the group
    void Set( double value );
    void SetWithOffset( double group1Value, double group2Value );
    
    // Return the value of the sensor connected to the Talon
    int GetEncoderValue();
    void TareEncoder();
    
    // Change Talon mode between brake/coast
    void SetCoastMode();
    void SetBrakeMode();
    
private:
    static const int MAX_NUMBER_OF_MOTORS = 4;

    // Member variables
    int m_NumMotors;                                    // Number of motors in the group
    TalonSRX *  m_pMotors[MAX_NUMBER_OF_MOTORS];        // The motor objects
    MotorGroupControlMode m_ControlMode;                // Keep track of the configuration of this Talon group
    FeedbackDevice m_Sensor;                            // Keep track of the sensor attached to the Talon
    
    // Prevent default construction/deletion/copy/assignment
    TalonMotorGroup();
    ~TalonMotorGroup();
    TalonMotorGroup( const TalonMotorGroup& ) = delete;
    TalonMotorGroup & operator=( const TalonMotorGroup& ) = delete;
};

#endif // TALONMOTORGROUP_HPP
