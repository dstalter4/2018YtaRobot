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
#include "CameraServer.h"           // For (actually) patched CameraServer instance

// C++ INCLUDES
#include "RobotCamera.hpp"          // For class declaration

// STATIC MEMBER DATA
cs::UsbCamera           RobotCamera::m_Cam0;
cs::UsbCamera           RobotCamera::m_Cam1;
cs::CvSink              RobotCamera::m_Cam0Sink;
cs::CvSink              RobotCamera::m_Cam1Sink;
cs::CvSource            RobotCamera::m_CameraOutput;
cv::Mat                 RobotCamera::m_SourceMat;
cv::Mat                 RobotCamera::m_ResizeOutputMat;
cv::Mat                 RobotCamera::m_HsvThresholdOutputMat; 
cv::Mat                 RobotCamera::m_ErodeOutputMat;
cv::Mat                 RobotCamera::m_MaskOutputMat;
cv::Mat                 RobotCamera::m_OutputMat;
cv::Mat *               RobotCamera::m_pDashboardMat;
RobotCamera::CameraType RobotCamera::m_Camera;
const char *            RobotCamera::CAMERA_0_NAME = "USB Camera 0";
const char *            RobotCamera::CAMERA_1_NAME = "USB Camera 1";
const char *            RobotCamera::CAMERA_OUTPUT_NAME = "Camera Output";



////////////////////////////////////////////////////////////////
/// @method RobotCamera::VisionThread
///
/// This method contains the workflow of the main vision thread.
///
////////////////////////////////////////////////////////////////
void RobotCamera::VisionThread()
{
    // Set the default selected camera
    m_Camera = FRONT_USB;
    
    // Start image capture
    m_Cam0 = CameraServer::GetInstance()->StartAutomaticCapture(CAMERA_0_DEV_NUM);
    m_Cam1 = CameraServer::GetInstance()->StartAutomaticCapture(CAMERA_1_DEV_NUM);
    
    // Set image resolution
    m_Cam0.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    m_Cam1.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    
    // Connect the sinks and sources
    m_Cam0Sink = CameraServer::GetInstance()->GetVideo(m_Cam0);
    m_Cam1Sink = CameraServer::GetInstance()->GetVideo(m_Cam1);
    m_CameraOutput = CameraServer::GetInstance()->PutVideo(CAMERA_OUTPUT_NAME, CAMERA_X_RES, CAMERA_Y_RES);
    
    // Set the default image to display
    m_pDashboardMat = &m_SourceMat;
    
    while (true)
    {
        // First, acquire an image from the currently selected camera
        int grabFrameResult = 0;
        if (m_Camera == FRONT_USB)
        {
            grabFrameResult = m_Cam0Sink.GrabFrame(m_SourceMat);
        }
        else if (m_Camera == BACK_USB)
        {
            grabFrameResult = m_Cam1Sink.GrabFrame(m_SourceMat);
        }
        else
        {
        }
        
        // Make sure it was successful
        if (grabFrameResult == 0)
        {
            continue;
        }
        
        // Filter the image
        FilterImage();
        
        // Display the image to the dashboard
        m_CameraOutput.PutFrame(*m_pDashboardMat);
    }
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::UpdateSmartDashboard
///
/// This method sends new data to the C++ Smart Dashboard.
///
////////////////////////////////////////////////////////////////
void RobotCamera::UpdateSmartDashboard()
{
    /*
    SmartDashboard::PutNumber("Masked particles",   m_NumMaskedParticles);
    SmartDashboard::PutNumber("Filtered particles", m_NumFilteredParticles);
    SmartDashboard::PutNumber("HeartBeat",          m_HeartBeat++);
    SmartDashboard::PutNumber("Left bound",         m_TargetReport.m_BoundingRectLeft);
    SmartDashboard::PutNumber("Right bound",        m_TargetReport.m_BoundingRectRight);
    SmartDashboard::PutNumber("Top bound",          m_TargetReport.m_BoundingRectTop);
    SmartDashboard::PutNumber("Bottom bound",       m_TargetReport.m_BoundingRectBottom);
    SmartDashboard::PutNumber("ConvexHull",         m_TargetReport.m_ConvexHullArea);
    SmartDashboard::PutNumber("Bounding area",      m_BoundingArea);
    SmartDashboard::PutNumber("Shape area",         m_TargetReport.m_Area);
    SmartDashboard::PutNumber("Shape area %",       m_ShapeAreaPercent);
    SmartDashboard::PutNumber("Trapezoid score",    m_TrapezoidPercent);
    SmartDashboard::PutNumber("Camera distance",    m_CameraDistance);
    SmartDashboard::PutNumber("Ground distance",    m_GroundDistance);
    */
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::ToggleCameraProcessedImage
///
/// Updates which stage of the image processing is sent to the
/// dashboard.
///
////////////////////////////////////////////////////////////////
void RobotCamera::ToggleCameraProcessedImage()
{
    if (m_pDashboardMat == &m_SourceMat)
    {
        // Move on to HSV threshold output
        m_pDashboardMat = &m_HsvThresholdOutputMat;
    }
    else if (m_pDashboardMat == &m_HsvThresholdOutputMat)
    {
        // Move on to eroded output
        m_pDashboardMat = &m_ErodeOutputMat;
    }
    else if (m_pDashboardMat == &m_ErodeOutputMat)
    {
        // Back to the start
        m_pDashboardMat = &m_SourceMat;
    }
    else
    {
        // Default to just the typical source mat
        m_pDashboardMat = &m_SourceMat;
    }
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::ProcessTarget
///
/// The public interface method for getting information about
/// the target.
///
////////////////////////////////////////////////////////////////
bool RobotCamera::ProcessTarget(bool bDoFullProcessing)
{
    return false;
    /*
    GetAndDisplayImage();
    
    if (bDoFullProcessing)
    {
        FilterImage();
        
        GenerateParticleReport();
        
        CalculateTargetParticleValues();
        
        // Don't call this in production code - it hogs resources
        //UpdateSmartDashboard();
    }

    return m_bTargetInRange;
    */

    // Shape drawing coordinates: Top, Left, Height, Width
    //int L = std::trunc(m_TargetReport.BoundingRectLeft);
    //int R = std::trunc(m_TargetReport.BoundingRectRight);
    //int T = std::trunc(m_TargetReport.BoundingRectTop);
    //int B = std::trunc(m_TargetReport.BoundingRectBottom);
    //int W = std::trunc(R - L);
    //int H = std::trunc(B - T);
    //m_pAxisCamera->GetImage(m_pImageFrame);
    //imaqDrawShapeOnImage(frame, frame, { 10, 10, 100, 100 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0);
    //imaqDrawShapeOnImage(m_pImageFrame, m_pImageFrame, { T, L, H, W }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 1.0);
    //CameraServer::GetInstance()->SetImage(pDrawableImage);
    //AthenaCameraServer::GetInstance()->SetImage(pDrawableImage);
}

    
    
////////////////////////////////////////////////////////////////
/// @method RobotCamera::FilterImage
///
/// Starts to filter an image.  It applies a color filter to the
/// image, then counts and filters the particles.
///
////////////////////////////////////////////////////////////////
void RobotCamera::FilterImage()
{
    // min/max values
    double hsvThresholdHue[]        = {0.0, 119.39393939393939};
    double hsvThresholdSaturation[] = {0.0, 173.43434343434345};
    double hsvThresholdValue[]      = {0.0, 141.23737373737373};
    
    // Convert to HSV and filter
    cv::cvtColor(m_SourceMat, m_HsvThresholdOutputMat, cv::COLOR_BGR2HSV);
    cv::inRange(m_HsvThresholdOutputMat,
                cv::Scalar(hsvThresholdHue[0], hsvThresholdSaturation[0], hsvThresholdValue[0]),
                cv::Scalar(hsvThresholdHue[1], hsvThresholdSaturation[1], hsvThresholdValue[1]),
                m_HsvThresholdOutputMat);
    
    // Erode image
    // @param src input image
    // @param dst output image
    // @param kernel structuring element used for erosion
    // @param anchor position of the anchor within the element; default value (-1, -1) means that the anchor is at the element center.
    // @param iterations number of times erosion is applied.
    // @param borderType pixel extrapolation method, see cv::BorderTypes
    // @param borderValue border value in case of a constant border
	cv::Mat cvErodeKernel;
	cv::erode(m_HsvThresholdOutputMat, m_ErodeOutputMat, cvErodeKernel, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(-1));
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::GenerateParticleReport
///
/// This method iterates over the found particles and performs
/// certain calculations on it.  It saves off the report of the
/// target with the largest area.
///
////////////////////////////////////////////////////////////////
void RobotCamera::GenerateParticleReport()
{
    /*
    if (m_NumFilteredParticles > 0)
    {
        double currentMaxArea = 0;

        // Measure particles and sort by particle size
        for (int particleIndex = 0; particleIndex < m_NumFilteredParticles; particleIndex++)
        {
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA,   &(m_IteratorParicleReport.m_PercentAreaToImageArea));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_AREA,                 &(m_IteratorParicleReport.m_Area));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_CONVEX_HULL_AREA,     &(m_IteratorParicleReport.m_ConvexHullArea));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP,    &(m_IteratorParicleReport.m_BoundingRectTop));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT,   &(m_IteratorParicleReport.m_BoundingRectLeft));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(m_IteratorParicleReport.m_BoundingRectBottom));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT,  &(m_IteratorParicleReport.m_BoundingRectRight));
            m_ParticleReports.push_back(m_IteratorParicleReport);

            m_IteratorParicleReport.m_AspectRatio = (m_IteratorParicleReport.m_BoundingRectRight - m_IteratorParicleReport.m_BoundingRectLeft) /
                                                    (m_IteratorParicleReport.m_BoundingRectBottom - m_IteratorParicleReport.m_BoundingRectTop);

            if (m_IteratorParicleReport.m_Area > currentMaxArea)
            {
                currentMaxArea = m_IteratorParicleReport.m_Area;
                m_TargetReport = m_IteratorParicleReport;
            }
        }
    }

    // Concerns about using too much of the heap and frequent dynamic garbage collection?
    m_ParticleReports.erase(m_ParticleReports.begin(), m_ParticleReports.end());
    */
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::GenerateParticleReport
///
/// This method performs certain calculations on the best found
/// particle report.  It will figure out distances to the target
/// and return true if it finds a particle within the expected
/// range.
///
////////////////////////////////////////////////////////////////
void RobotCamera::CalculateTargetParticleValues()
{
    /*
    int32_t xRes = 0;
    int32_t yRes = 0;
    imaqGetImageSize(m_pBinaryFrame, &xRes, &yRes);

    // Target distance: 11-16 ft. (132-192 in.)
    // Bottom of goal is 7 ft. (84 in.) from ground

    // d = (TLengthIn * xRes) / (2 * TLengthPix * tan(FOVAng)), negated
    //double distance = -1 * (20 * xRes) / (2 * (targetReport.BoundingRectRight - targetReport.BoundingRectLeft) * tan(18));//tan(21.5778173));//tan(37.4));
    m_CameraDistance = (TARGET_SIZE * xRes) /
                       (2.0 * (m_TargetReport.m_BoundingRectRight - m_TargetReport.m_BoundingRectLeft)
                            * tan(CALIBRATED_CAMERA_ANGLE*RADIANS_TO_DEGREES));

    // ground_distance = sqrt((camera_reported_distance^2) - (84^2))
    // sin(camera_angle) = (height_from_ground) / (camera_reported_distance);
    m_GroundDistance = sqrt((m_CameraDistance * m_CameraDistance) - (TARGET_REFLECTOR_HEIGHT * TARGET_REFLECTOR_HEIGHT));

    m_BoundingArea = (m_TargetReport.m_BoundingRectRight-m_TargetReport.m_BoundingRectLeft)
                   * (m_TargetReport.m_BoundingRectBottom-m_TargetReport.m_BoundingRectTop);
    m_ShapeAreaPercent = (m_TargetReport.m_Area / m_BoundingArea) * DECIMAL_TO_PERCENT;
    m_TrapezoidPercent = (m_TargetReport.m_ConvexHullArea / m_BoundingArea) * DECIMAL_TO_PERCENT;

    // At a distance of 20 feet, the minimum area for the target is about 700 pxl^2
    // Our target range is 11-16 ft. so we will use this as our starting filtering point
    if (((m_GroundDistance + GROUND_DISTANCE_TOLERANCE) >= TARGET_RANGE_MIN) && ((m_GroundDistance - GROUND_DISTANCE_TOLERANCE) <= TARGET_RANGE_MAX))
    {
        m_bTargetInRange = true;
    }
    else
    {
        m_bTargetInRange = false;
    }
    */
}
