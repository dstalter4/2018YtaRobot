////////////////////////////////////////////////////////////////////////////////
/// @file   RobotCamera.hpp
/// @author David Stalter
///
/// @details
/// A class designed to support camera functionality on the robot.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   13-FEB-2016 Created.
/// @endif
///
/// Copyright (c) 2018 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "WPILib.h"                     // For FRC library
#include "CameraServer.h"               // This group of includes is for camera support
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types.hpp"

// C++ INCLUDES
// (none)

////////////////////////////////////////////////////////////////
/// @class RobotCamera
///
/// Class that provides methods for interacting with the camera.
///
////////////////////////////////////////////////////////////////
class RobotCamera
{
public:
    
    enum CameraType
    {
        FRONT_USB,
        BACK_USB,
        AXIS
    };
    
    // Pick a camera to use
    inline static void SetCamera(CameraType camera);
    
    // Toggle between cameras
    inline static void ToggleCamera();
    
    // Toggle between what processed image is shown on the dashboard
    static void ToggleCameraProcessedImage();

    // Main processing sequence
    static bool ProcessTarget(bool bDoFullProcessing);
    
    // The vision thread itself
    static void VisionThread();

private:
    
    // Update values on the SmartDashboard
    static void UpdateSmartDashboard();

    // Get an image and start to process it
    static void FilterImage();

    // Generate a report about the found particles in an image
    static void GenerateParticleReport();

    // Compute some useful information about the identified particle
    static void CalculateTargetParticleValues();

    // Constructor
    RobotCamera();

    // Destructor, copy constructor, assignment operator
    ~RobotCamera();

    RobotCamera(const RobotCamera &) = delete;
    RobotCamera & operator=(const RobotCamera &) = delete;

    // A structure to hold measurements of a particle
    struct ParticleReport
    {
        double m_PercentAreaToImageArea;
        double m_Area;
        double m_ConvexHullArea;
        double m_BoundingRectLeft;
        double m_BoundingRectTop;
        double m_BoundingRectRight;
        double m_BoundingRectBottom;
        double m_AspectRatio;
    };
    
    static cs::UsbCamera        m_Cam0;                     // USB camera 0
    static cs::UsbCamera        m_Cam1;                     // USB camera 1
    static cs::CvSink           m_Cam0Sink;                 // Sink for camera 0
    static cs::CvSink           m_Cam1Sink;                 // Sink for camera 1
    static cs::CvSource         m_CameraOutput;             // Output source for processed images
    static cv::Mat              m_SourceMat;                // The originating source mat from the current camera
    static cv::Mat              m_ResizeOutputMat;          // Resized source mat
    static cv::Mat              m_HsvThresholdOutputMat;    // HSV filtered mat
    static cv::Mat              m_ErodeOutputMat;           // Erode output mat
    static cv::Mat              m_MaskOutputMat;            // Masked output mat
    static cv::Mat              m_OutputMat;                // The final output mat for displaying an (optionally) processed image
    static cv::Mat *            m_pDashboardMat;            // Pointer to which mat should currently be sent to the dashboard
    static CameraType           m_Camera;                   // Keep track of the current camera to process information from
    
    //ParticleReport m_TargetReport;                          // Information about the best target particle we've found
    //ParticleReport m_IteratorParicleReport;                 // An object to iterate over objects when identifying a target
    //std::vector<ParticleReport> m_ParticleReports;          // The list of particle reports

    //int m_HeartBeat;                // Keep alive with the C++ dashboard
    //int m_NumMaskedParticles;       // Number of masked particles found
    //int m_NumFilteredParticles;     // Number of filtered particles found

    //float m_CameraDistance;         // Distance to the target from the camera
    //float m_GroundDistance;         // Actual ground distance to the target
    //float m_BoundingArea;           // Area of the target based on its bounding coordinates
    //float m_ShapeAreaPercent;       // Percentage of the bounding area the shape occupies
    //float m_TrapezoidPercent;       // Likelihood that this is a true rectangle

    //bool m_bTargetInRange;          // Remember the last result from full vision processing
    
    static const int    CAMERA_0_DEV_NUM                    = 0;
    static const int    CAMERA_1_DEV_NUM                    = 1;
    static const int    CAMERA_X_RES                        = 320;
    static const int    CAMERA_Y_RES                        = 240;
    static const char * CAMERA_0_NAME;
    static const char * CAMERA_1_NAME;
    static const char * CAMERA_OUTPUT_NAME;
    
    //static const int RED_REFLECT_MIN                    = 94;
    //static const int RED_REFLECT_MAX                    = 136;
    //static const int GREEN_REFLECT_MIN                  = 156;
    //static const int GREEN_REFLECT_MAX                  = 255;
    //static const int BLUE_REFLECT_MIN                   = 233;
    //static const int BLUE_REFLECT_MAX                   = 255;
    //static constexpr float CAMERA_FPS                   = 10.0F;
    //static constexpr float TARGET_MIN_AREA_PERCENT      = 0.0F;
    //static constexpr float TARGET_MAX_AREA_PERCENT      = 100.0F;
    //static constexpr float TARGET_SIZE                  = 20.0F;
    //static constexpr float TARGET_REFLECTOR_HEIGHT      = 84.0F;
    //static constexpr float TARGET_RANGE_MIN             = 132.0F;
    //static constexpr float TARGET_RANGE_MAX             = 192.0F;
    //static constexpr float GROUND_DISTANCE_TOLERANCE    = 6.0F;
    //static constexpr float CALIBRATED_CAMERA_ANGLE      = 21.5778173F;
    //static constexpr float RADIANS_TO_DEGREES           = M_PI / 180.0F;
    //static constexpr float DECIMAL_TO_PERCENT           = 100.0F;
};



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetCamera
///
/// This method sets which camera is active.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetCamera(CameraType camera)
{
    m_Camera = camera;
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::ToggleCamera
///
/// This method toggles between which camera is active.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::ToggleCamera()
{
    m_Camera = (m_Camera == FRONT_USB) ? BACK_USB : FRONT_USB;
}
