////////////////////////////////////////////////////////////////////////////////
/// @file   RobotCamera.hpp
/// @author David Stalter
///
/// @details
/// A class designed to support camera functionality on the robot.
///
/// @if INCLUDE_EDIT_HISTORY
/// - dts   13-FEB-2016 Created.
/// - dts   15-FEB-2018 Convert to opencv processing.
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
cs::UsbCamera                                   RobotCamera::m_Cam0;
cs::CvSink                                      RobotCamera::m_Cam0Sink;
cs::UsbCamera                                   RobotCamera::m_Cam1;
cs::CvSink                                      RobotCamera::m_Cam1Sink;
cs::CvSource                                    RobotCamera::m_CameraOutput;

cv::Mat                                         RobotCamera::m_SourceMat;
cv::Mat                                         RobotCamera::m_ResizeOutputMat;
cv::Mat                                         RobotCamera::m_HsvThresholdOutputMat; 
cv::Mat                                         RobotCamera::m_ErodeOutputMat;
cv::Mat                                         RobotCamera::m_ContoursMat;
cv::Mat                                         RobotCamera::m_FilteredContoursMat;
cv::Mat                                         RobotCamera::m_VisionTargetMat;
cv::Mat *                                       RobotCamera::m_pDashboardMat;

std::vector<std::vector<cv::Point>>             RobotCamera::m_Contours;
std::vector<std::vector<cv::Point>>             RobotCamera::m_FilteredContours;

std::vector<RobotCamera::VisionTargetReport>    RobotCamera::m_ContourTargetReports;
RobotCamera::VisionTargetReport                 RobotCamera::m_VisionTargetReport;
RobotCamera::CameraType                         RobotCamera::m_Camera;
bool                                            RobotCamera::m_bDoFullProcessing;
int                                             RobotCamera::m_HeartBeat;
const char *                                    RobotCamera::CAMERA_OUTPUT_NAME = "Camera Output";



////////////////////////////////////////////////////////////////
/// @method RobotCamera::VisionThread
///
/// This method contains the workflow of the main vision thread.
///
////////////////////////////////////////////////////////////////
void RobotCamera::VisionThread()
{
    // Clear the vision target structure
    std::memset(&m_VisionTargetReport, 0, sizeof(VisionTargetReport));
    
    // Set the default selected camera
    m_Camera = FRONT_USB;
    
    // Start image capture
    m_Cam0 = CameraServer::GetInstance()->StartAutomaticCapture();//CAMERA_0_DEV_NUM);
    m_Cam1 = CameraServer::GetInstance()->StartAutomaticCapture();//CAMERA_1_DEV_NUM);
    
    // Set image resolution
    m_Cam0.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    m_Cam1.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    
    // Connect the sinks and sources
    m_Cam0Sink = CameraServer::GetInstance()->GetVideo(m_Cam0);
    m_Cam1Sink = CameraServer::GetInstance()->GetVideo(m_Cam1);
    m_CameraOutput = CameraServer::GetInstance()->PutVideo(CAMERA_OUTPUT_NAME, CAMERA_X_RES, CAMERA_Y_RES);
    
    // Set the default image to display
    m_pDashboardMat = &m_SourceMat;
    SmartDashboard::PutString("Camera Output", "Default");
    
    // Default to full processing unless the robot code says otherwise
    m_bDoFullProcessing = true;
    
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
        
        if (m_bDoFullProcessing)
        {
            // Filter the image
            ProcessImage();
            
            // Try and identify the reflective tape
            FindReflectiveTapeTarget();
            
            // Calculate some info based on the reflective tape
            CalculateReflectiveTapeValues();
        
        }
        
        // Display the image to the dashboard
        m_CameraOutput.PutFrame(*m_pDashboardMat);
        
        // Don't call this in production code - it hogs resources
        UpdateSmartDashboard();
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
    SmartDashboard::PutNumber("HeartBeat",                  m_HeartBeat++);
    
    SmartDashboard::PutNumber("Bounding rect X",            m_VisionTargetReport.m_BoundingRectX);
    SmartDashboard::PutNumber("Bounding rect Y",            m_VisionTargetReport.m_BoundingRectY);
    SmartDashboard::PutNumber("Bounding rect width",        m_VisionTargetReport.m_BoundingRectWidth);
    SmartDashboard::PutNumber("Bounding rect height",       m_VisionTargetReport.m_BoundingRectHeight);
    SmartDashboard::PutNumber("Bounding rect area",         m_VisionTargetReport.m_BoundingRectArea);
    SmartDashboard::PutNumber("Bounding rect aspect ratio", m_VisionTargetReport.m_BoundingRectAspectRatio);
    
    SmartDashboard::PutNumber("Contour area",               m_VisionTargetReport.m_Area);
    SmartDashboard::PutNumber("Contour perimeter",          m_VisionTargetReport.m_Perimeter);
    SmartDashboard::PutNumber("Contour convex hull area",   m_VisionTargetReport.m_ConvexHullArea);
    SmartDashboard::PutNumber("Contour solidity",           m_VisionTargetReport.m_Solidity);
    SmartDashboard::PutNumber("Contour vertices",           m_VisionTargetReport.m_Vertices);
    
    SmartDashboard::PutNumber("Area %",                     m_VisionTargetReport.m_PercentAreaToImageArea);
    SmartDashboard::PutNumber("Trapezoid %",                m_VisionTargetReport.m_TrapezoidPercent);
    SmartDashboard::PutNumber("Camera distance, X",         m_VisionTargetReport.m_CameraDistanceX);
    SmartDashboard::PutNumber("Camera distance, Y",         m_VisionTargetReport.m_CameraDistanceY);
    SmartDashboard::PutNumber("Ground distance",            m_VisionTargetReport.m_GroundDistance);
    SmartDashboard::PutNumber("Target in range",            m_VisionTargetReport.m_bTargetInRange);
    SmartDashboard::PutNumber("Target report valid",        m_VisionTargetReport.m_bIsValid);
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
    if (m_bDoFullProcessing)
    {
        if (m_pDashboardMat == &m_SourceMat)
        {
            // Move on to HSV threshold output
            m_pDashboardMat = &m_HsvThresholdOutputMat;
            SmartDashboard::PutString("Camera Output", "HSV Threshold");
        }
        else if (m_pDashboardMat == &m_HsvThresholdOutputMat)
        {
            // Move on to eroded output
            m_pDashboardMat = &m_ErodeOutputMat;
            SmartDashboard::PutString("Camera Output", "Eroded");
        }
        else if (m_pDashboardMat == &m_ErodeOutputMat)
        {
            // Move on to contours output
            m_pDashboardMat = &m_ContoursMat;
            SmartDashboard::PutString("Camera Output", "Contours");
        }
        else if (m_pDashboardMat == &m_ContoursMat)
        {
            // Move on to filtered contours output
            m_pDashboardMat = &m_FilteredContoursMat;
            SmartDashboard::PutString("Camera Output", "Filtered Contours");
        }
        else if (m_pDashboardMat == &m_FilteredContoursMat)
        {
            // Move on to the best candidate vision target mat
            m_pDashboardMat = &m_VisionTargetMat;
            SmartDashboard::PutString("Camera Output", "Vision Target");
        }
        else if (m_pDashboardMat == &m_VisionTargetMat)
        {
            // Back to the start
            m_pDashboardMat = &m_SourceMat;
            SmartDashboard::PutString("Camera Output", "Default");
        }
        else
        {
        }
    }
    else
    {
        // Default to just the typical source mat
        m_pDashboardMat = &m_SourceMat;
        SmartDashboard::PutString("Camera Output", "Default");
    }
}

    
    
////////////////////////////////////////////////////////////////
/// @method RobotCamera::ProcessImage
///
/// Processes an image from the camera to try and identify
/// vision targets.
///
////////////////////////////////////////////////////////////////
void RobotCamera::ProcessImage()
{
    FilterImageHsv();
    ErodeImage();
    FindContours();
    FilterContours();
}

    
    
////////////////////////////////////////////////////////////////
/// @method RobotCamera::FilterImageHsv
///
/// Applies a color filter to the image based on Hue, Saturation
/// and Value.
///
////////////////////////////////////////////////////////////////
void RobotCamera::FilterImageHsv()
{
    // min/max values
    static double hsvThresholdHue[] = {0.0, 180.0};
    static double hsvThresholdSaturation[] = {0.0, 150.0};
    static double hsvThresholdValue[] = {220.0, 255.0};
    
    hsvThresholdHue[0]          = SmartDashboard::GetNumber("H min", hsvThresholdHue[0]);
    hsvThresholdHue[1]          = SmartDashboard::GetNumber("H max", hsvThresholdHue[1]);
    hsvThresholdSaturation[0]   = SmartDashboard::GetNumber("S min", hsvThresholdSaturation[0]);
    hsvThresholdSaturation[1]   = SmartDashboard::GetNumber("S max", hsvThresholdSaturation[1]);
    hsvThresholdValue[0]        = SmartDashboard::GetNumber("V min", hsvThresholdValue[0]);
    hsvThresholdValue[1]        = SmartDashboard::GetNumber("V max", hsvThresholdValue[1]);
    
    SmartDashboard::PutNumber("H min", hsvThresholdHue[0]);
    SmartDashboard::PutNumber("H max", hsvThresholdHue[1]);
    SmartDashboard::PutNumber("S min", hsvThresholdSaturation[0]);
    SmartDashboard::PutNumber("S max", hsvThresholdSaturation[1]);
    SmartDashboard::PutNumber("V min", hsvThresholdValue[0]);
    SmartDashboard::PutNumber("V max", hsvThresholdValue[1]);
    
    // Convert to HSV and filter
    cv::cvtColor(m_SourceMat, m_HsvThresholdOutputMat, cv::COLOR_BGR2HSV);
    cv::inRange(m_HsvThresholdOutputMat,
                cv::Scalar(hsvThresholdHue[0], hsvThresholdSaturation[0], hsvThresholdValue[0]),
                cv::Scalar(hsvThresholdHue[1], hsvThresholdSaturation[1], hsvThresholdValue[1]),
                m_HsvThresholdOutputMat);
}

    
    
////////////////////////////////////////////////////////////////
/// @method RobotCamera::ErodeImage
///
/// Erodes an image.
///
////////////////////////////////////////////////////////////////
void RobotCamera::ErodeImage()
{
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
/// @method RobotCamera::FindContours
///
/// Finds the contours in an image.
///
////////////////////////////////////////////////////////////////
void RobotCamera::FindContours()
{    
    // Find contours
    // @param image Source, an 8-bit single-channel image.
    // @param contours Detected contours. Each contour is stored as a vector of points (e.g. std::vector<std::vector<cv::Point> >).
    // @param hierarchy Optional output vector (e.g. std::vector<cv::Vec4i>), containing information about the image topology.
    // @param mode Contour retrieval mode, see cv::RetrievalModes ( ? cv::RETR_EXTERNAL : cv::RETR_LIST)
    // @param method Contour approximation method, see cv::ContourApproximationModes
    // @param offset Optional offset by which every contour point is shifted.
    std::vector<cv::Vec4i> hierarchy;
    m_Contours.clear();
    cv::findContours(m_ErodeOutputMat, m_Contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    // Reset the contour mats by clearing them
    m_ContoursMat = cv::Mat::zeros(m_pDashboardMat->size(), m_pDashboardMat->type());
    m_FilteredContoursMat = cv::Mat::zeros(m_pDashboardMat->size(), m_pDashboardMat->type());
    m_VisionTargetMat = cv::Mat::zeros(m_pDashboardMat->size(), m_pDashboardMat->type());
    
    // @param image Destination image.
    // @param contours All the input contours. Each contour is stored as a point vector.
    // @param contourIdx Parameter indicating a contour to draw. If it is negative, all the contours are drawn.
    // @param color Color of the contours.
    cv::drawContours(m_ContoursMat, m_Contours, -1, cv::Scalar(255, 255, 255));
}

    
    
////////////////////////////////////////////////////////////////
/// @method RobotCamera::FilterContours
///
/// Filters the contours found by certain criteria.
///
////////////////////////////////////////////////////////////////
void RobotCamera::FilterContours()
{    
    const double FILTER_CONTOURS_MIN_WIDTH      = 0.0;
    const double FILTER_CONTOURS_MAX_WIDTH      = 1000.0;
    const double FILTER_CONTOURS_MIN_HEIGHT     = 0.0;
    const double FILTER_CONTOURS_MAX_HEIGHT     = 1000.0;
    const double FILTER_CONTOURS_MIN_AREA       = 500.0;
    const double FILTER_CONTOURS_MAX_AREA       = 100000.0;
    const double FILTER_CONTOURS_MIN_PERIMETER  = 0.0;
    const double FILTER_CONTOURS_MAX_PERIMETER  = 10000.0;
    const double FILTER_CONTOURS_SOLIDITY[]     = {85.0, 100.0};
    const double FILTER_CONTOURS_MIN_VERTICES   = 0.0;
    const double FILTER_CONTOURS_MAX_VERTICES   = 1000000.0;
    const double FILTER_CONTOURS_MIN_RATIO      = 0.0;
    const double FILTER_CONTOURS_MAX_RATIO      = 1000.0;
    
    // Filter contours
    m_FilteredContours.clear();
    m_ContourTargetReports.clear();
    for (std::vector<cv::Point> contour : m_Contours)
    {
        VisionTargetReport currentContourReport;
        
        // Bounding rectangle filtering
        cv::Rect boundingRectangle = cv::boundingRect(contour);
        if ((boundingRectangle.width) < FILTER_CONTOURS_MIN_WIDTH || (boundingRectangle.width > FILTER_CONTOURS_MAX_WIDTH))
        {
            continue;
        }
        if ((boundingRectangle.height < FILTER_CONTOURS_MIN_HEIGHT) || (boundingRectangle.height > FILTER_CONTOURS_MAX_HEIGHT))
        {
            continue;
        }
        // Fill out bounding rectangle info (done here so the loop can continue if criteria aren't met)
        currentContourReport.m_BoundingRectX = boundingRectangle.x;
        currentContourReport.m_BoundingRectY = boundingRectangle.y;
        currentContourReport.m_BoundingRectWidth = boundingRectangle.width;
        currentContourReport.m_BoundingRectHeight = boundingRectangle.height;
        currentContourReport.m_BoundingRectArea = boundingRectangle.width * boundingRectangle.height;
        
        // Max area is not a standard filtering technique in GRIP
        double area = cv::contourArea(contour);
        if ((area < FILTER_CONTOURS_MIN_AREA) || (area > FILTER_CONTOURS_MAX_AREA))
        {
            continue;
        }
        currentContourReport.m_Area = area;
        
        // Max perimeter is not a standard filtering technique in GRIP
        double perimeter = cv::arcLength(contour, true);
        if ((perimeter < FILTER_CONTOURS_MIN_PERIMETER) || (perimeter > FILTER_CONTOURS_MAX_PERIMETER))
        {
            continue;
        }
        currentContourReport.m_Perimeter = perimeter;
        
        std::vector<cv::Point> hull;
        cv::convexHull(cv::Mat(contour, true), hull);
        double hullArea = cv::contourArea(hull);
        double solidity = 100.0 * (area / hullArea);
        if ((solidity < FILTER_CONTOURS_SOLIDITY[0]) || (solidity > FILTER_CONTOURS_SOLIDITY[1]))
        {
            continue;
        }
        currentContourReport.m_ConvexHullArea = hullArea;
        currentContourReport.m_Solidity = solidity;
        
        // Number of vertices
        if ((contour.size() < FILTER_CONTOURS_MIN_VERTICES) || (contour.size() > FILTER_CONTOURS_MAX_VERTICES))
        {
            continue;
        }
        currentContourReport.m_Vertices = contour.size();
        
        // Aspect ratio
        double ratio = static_cast<double>(boundingRectangle.width) / static_cast<double>(boundingRectangle.height);
        if ((ratio < FILTER_CONTOURS_MIN_RATIO) || (ratio > FILTER_CONTOURS_MAX_RATIO))
        {
            continue;
        }
        currentContourReport.m_BoundingRectAspectRatio = boundingRectangle.width / boundingRectangle.height;
        
        // All criteria passed, add this contour
        currentContourReport.m_bIsValid = true;
        m_FilteredContours.push_back(contour);
        m_ContourTargetReports.push_back(currentContourReport);
    }
    
    // @param image Destination image.
    // @param contours All the input contours. Each contour is stored as a point vector.
    // @param contourIdx Parameter indicating a contour to draw. If it is negative, all the contours are drawn.
    // @param color Color of the contours.
    cv::drawContours(m_FilteredContoursMat, m_FilteredContours, -1, cv::Scalar(255, 255, 255));
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::FindReflectiveTapeTarget
///
/// This method iterates over the filtered contours and tries to
/// identify the reflective tape target.  It will save off the
/// appropriate contour if one that meets the criteria is found.
///
////////////////////////////////////////////////////////////////
void RobotCamera::FindReflectiveTapeTarget()
{
    if (m_ContourTargetReports.size() > 0)
    {
        int index = 0;
        int candidateIndex = 0;
        double currentMaxArea = 0.0;

        // Iterate through the contour reports, searching for the best candidate
        for (VisionTargetReport report : m_ContourTargetReports)
        {
            if (report.m_Area > currentMaxArea)
            {
                currentMaxArea = report.m_Area;
                m_VisionTargetReport = report;
                candidateIndex = index;
            }
            
            index++;
        }
        
        // Draw the candidate contour
        cv::drawContours(m_VisionTargetMat, m_FilteredContours, candidateIndex, cv::Scalar(255, 255, 255));
    }
    else
    {
        // If no contour met criteria, clear out the target report information
        std::memset(&m_VisionTargetReport, 0, sizeof(VisionTargetReport));
    }
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::CalculateReflectiveTapeValues
///
/// This method performs certain calculations on the best found
/// contour.  It will primarily compute distances related to the
/// vision target.
///
////////////////////////////////////////////////////////////////
void RobotCamera::CalculateReflectiveTapeValues()
{
    // If there is no vision target report available, don't proceed
    if (!m_VisionTargetReport.m_bIsValid)
    {
        return;
    }
    
    // d = (TargetWidthIn * CAMERA_X_RES) / (2 * TargetWidthPix * tan(1/2 * FOVAng))
    // d = (TargetHeightIn * CAMERA_Y_RES) / (2 * TargetHeightPix * tan(1/2 * FOVAng))
    m_VisionTargetReport.m_CameraDistanceX = (TARGET_WIDTH_INCHES * CAMERA_X_RES) /
                                             (2.0 * (m_VisionTargetReport.m_BoundingRectWidth) * tan(.5 * CAMERA_FOV_DEGREES * DEGREES_TO_RADIANS));
                                             //(2.0 * (m_VisionTargetReport.m_BoundingRectWidth) * tan(.5 * CALIBRATED_CAMERA_ANGLE * DEGREES_TO_RADIANS));
    
    m_VisionTargetReport.m_CameraDistanceY = (TARGET_HEIGHT_INCHES * CAMERA_Y_RES) /
                                             (2.0 * (m_VisionTargetReport.m_BoundingRectHeight) * tan(.5 * CAMERA_FOV_DEGREES * DEGREES_TO_RADIANS));
                                             //(2.0 * (m_VisionTargetReport.m_BoundingRectHeight) * tan(.5 * CALIBRATED_CAMERA_ANGLE * DEGREES_TO_RADIANS));

    // ground_distance = sqrt((camera_reported_distance^2) - (84^2))
    // sin(camera_angle) = (height_from_ground) / (camera_reported_distance);
    // Use m_CameraDistanceY since the target is taller than wide
    m_VisionTargetReport.m_GroundDistance = sqrt((m_VisionTargetReport.m_CameraDistanceY * m_VisionTargetReport.m_CameraDistanceY) - (TARGET_HEIGHT_FROM_GROUND * TARGET_HEIGHT_FROM_GROUND));

    //m_VisionTargetReport.m_PercentAreaToImageArea = ( ? / m_VisionTargetReport.m_BoundingRectArea) * DECIMAL_TO_PERCENT;
    //m_VisionTargetReport.m_TrapezoidPercent = (m_TargetReport.m_ConvexHullArea / m_VisionTargetReport.m_BoundingRectArea) * DECIMAL_TO_PERCENT;

    // At a distance of 20 feet, the minimum area for the target is about 700 pxl^2
    // Our target range is 11-16 ft. so we will use this as our starting filtering point
    /*
    if (((m_VisionTargetReport.m_GroundDistance + GROUND_DISTANCE_TOLERANCE) >= TARGET_RANGE_MIN)
        && ((m_VisionTargetReport.m_GroundDistance - GROUND_DISTANCE_TOLERANCE) <= TARGET_RANGE_MAX))
    {
        m_VisionTargetReport.m_bTargetInRange = true;
    }
    else
    {
        m_VisionTargetReport.m_bTargetInRange = false;
    }
    */
    
    m_VisionTargetReport.m_bTargetInRange = false;
}
