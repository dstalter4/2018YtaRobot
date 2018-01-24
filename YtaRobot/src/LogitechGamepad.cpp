////////////////////////////////////////////////////////////////////////////////
/// @file   LogitechGamepad.cpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to a Logitech Gamepad controller.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   19-JAN-2018 Created.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "LogitechGamepad.hpp"      // For class declaration

// STATIC MEMBER DATA
// (none)



////////////////////////////////////////////////////////////////
/// @method LogitechGamepad::LogitechGamepad
///
/// Constructor.
///
////////////////////////////////////////////////////////////////
LogitechGamepad::LogitechGamepad(int port)
: GenericHID(port)
, m_ThrottleValue(0.0F)
{
}



////////////////////////////////////////////////////////////////
/// @method LogitechGamepad::GetX
///
/// Returns x-axis input.  This method is pure virtual in the
/// base class and must be implemented.
///
////////////////////////////////////////////////////////////////
double LogitechGamepad::GetX(JoystickHand hand) const
{
    // x-axis controls are very sensitive on this
    // controller, so scale them back.
    return (GetRawAxis(LEFT_X_AXIS) * X_AXIS_SENSITIVITY_SCALING);
}



////////////////////////////////////////////////////////////////
/// @method LogitechGamepad::GetY
///
/// Returns y-axis input.  This method is pure virtual in the
/// base class and must be implemented.
///
////////////////////////////////////////////////////////////////
double LogitechGamepad::GetY(JoystickHand hand) const
{
    // In order to keep the drive logic the same as the
    // joysticks, full forward is represented by -1 and
    // full reverse is represented by +1.
    
    // Left trigger is the 'reverse' value input.
    double leftTriggerValue = GetRawAxis(LEFT_TRIGGER);
    
    // Right trigger is the 'forward' value input, which
    // needs to be negated.
    double rightTriggerValue = -GetRawAxis(RIGHT_TRIGGER);
    
    // Hopefully only one trigger is being pushed, but in
    // case both are being pressed, the value will be combined.
    return leftTriggerValue + rightTriggerValue;
}



////////////////////////////////////////////////////////////////
/// @method LogitechGamepad::GetThrottle
///
/// Returns throttle control.  The Logitech Gamepad does not
/// have an axis that retains its position when not being
/// controlled by the user.  This requires throttle control to
/// be implemented and remembered in software.
///
////////////////////////////////////////////////////////////////
double LogitechGamepad::GetThrottle() const
{
    // Not implemented yet
    return 1.0F;
}
