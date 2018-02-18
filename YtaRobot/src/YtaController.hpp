////////////////////////////////////////////////////////////////////////////////
/// @file   YtaController.hpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types (Logitech Gamepad,
/// Xbox GameSir) with custom responses.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   19-JAN-2018 Created.
/// - dts   05-FEB-2018 Convert float -> double.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTACONTROLLER_HPP
#define YTACONTROLLER_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "WPILib.h"                     // For FRC library

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @class YtaController
///
/// Class that provides methods for interacting with a Logitech
/// Gamepad controller.  Derives from GenericHID.  Inheriting
/// from GamepadBase.h is deprecated, so GenericHID is used
/// directly.
///
////////////////////////////////////////////////////////////////
class YtaController : public GenericHID
{
public:
    
    explicit YtaController(int port);
    virtual ~YtaController() = default;
    
    virtual double GetX(JoystickHand hand = kLeftHand) const override;
    virtual double GetY(JoystickHand hand = kLeftHand) const override;
    
    double GetThrottle() const;
    
private:
    
    enum RawAxes
    {
        LEFT_X_AXIS         = 0,
        LEFT_Y_AXIS         = 1,
        LEFT_TRIGGER        = 2,
        RIGHT_TRIGGER       = 3,
        RIGHT_X_AXIS        = 4,
        RIGHT_Y_AXIS        = 5
    };
    
    enum RawButtons
    {
        A                   = 1,
        B                   = 2,
        X                   = 3,
        Y                   = 4,
        LT                  = 5,
        RT                  = 6,
        BACK                = 7,
        START               = 8,
        LEFT_STICK_CLICK    = 9,
        RIGHT_STICK_CLICK   = 10
    };
    
    double m_ThrottleValue;
    
    static constexpr double X_AXIS_SENSITIVITY_SCALING = 0.30;
    static constexpr double Y_AXIS_SENSITIVITY_SCALING = 0.80;
    
    // Prevent copying/assignment
    YtaController(const YtaController&) = delete;
    YtaController& operator=(const YtaController&) = delete;
};

#endif // YTACONTROLLER_HPP
