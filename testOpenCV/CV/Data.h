#pragma once

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>

template <class T>
class Data {
public:

    // the points of the laser on the marking
    std::vector<cv::Point_<T>> center_points;

    // the points thwere the laser hits ground zero on the LEFT side of the marking
    std::vector<cv::Point_<T>> left_points;

    // the points thwere the laser hits ground zero on the RIGHT side of the marking
    std::vector<cv::Point_<T>> right_points;

    // start location for the 3 point vectors
    cv::Vec<T, 3> points_start;

    // the camera meta data
    //std::unique_ptr<CameraData> camera_data = std::make_unique<CameraData>();

    //// the main camera pointer through the system
    //AVT::VmbAPI::CameraPtr cameraPtr;

    //// holding data to construct the actual camera
    //std::unique_ptr<VimbaData> vimbaData = std::make_unique<VimbaData>();

    //// the actual camera used
    //std::shared_ptr<GC2450MCamera> camera;

    // the name of the glob loaded (or "camera" for live feed)
    std::string glob_name;

    // the rectangle which includes the entire marking
    cv::Rect_<T> marking_rect;

    // left border vector
    cv::Vec<T, 4> left_border;

    // right border vector
    cv::Vec<T, 4> right_border;

    // the base lines where the laser hits the actual ground-zero
    cv::Vec<T, 4> base_lines;

    // the locations for where the base lines intersect with the marking border
    cv::Vec<T, 4> intersections;

    // enclusure of the laser line
    cv::Vec<T, 4> center_line;

    /**
     * \brief the points where the intersections are cut.
     * adjusted so potential unwanted information is not included in further calculations
     */
    cv::Vec<T, 2> intersection_cuts;

    // the x coordinates for the pieces that are cut out.
    cv::Vec<T, 4> middle_pieces;

    // validation results for the individual members of this structure
    //std::vector<ValidationResult> validation_results;

    //// overall validation status for this structure
    //ValidationStatus status;

    // the avg
    T left_avg;

    T left_mid_avg;

    T center_avg;

    T right_mid_avg;

    T right_avg;

    // the difference between the baseline(s) and the laser line on the marking in sub-pixels
    T difference;

    /*
            static std::string DataMemberName(DataMemberIndex index) {
                switch (index) {
                case DataMemberIndex::CenterPoints:
                    return "CenterPoints";
                case DataMemberIndex::LeftPoints:
                    return "LeftPoints";
                case DataMemberIndex::RightPoints:
                    return "RightPoints";
                case DataMemberIndex::PointsStart:
                    return "PointsStart";
                case DataMemberIndex::CameraData:
                    return "CameraData";
                case DataMemberIndex::CameraPtr:
                    return "CameraPtr";
                case DataMemberIndex::GlobName:
                    return "GlobName";
                case DataMemberIndex::MarkingRect:
                    return "MarkingRect";
                case DataMemberIndex::LeftBorder:
                    return "LeftBorder";
                case DataMemberIndex::RightBorder:
                    return "RightBorder";
                case DataMemberIndex::BaseLines:
                    return "BaseLines";
                case DataMemberIndex::Intersections:
                    return "Intersections";
                case DataMemberIndex::IntersectionCuts:
                    return "IntersectionCuts";
                case DataMemberIndex::MiddlePieces:
                    return "MiddlePieces";
                case DataMemberIndex::LeftAvg:
                    return "LeftAvg";
                case DataMemberIndex::CenterAvg:
                    return "CenterAvg";
                case DataMemberIndex::RightAvg:
                    return "RightAvg";
                case DataMemberIndex::Difference:
                    return "Difference";
                default:
                    return "feeeeek";
                }
            }*/

    //void addValidationResult(DataMemberIndex index, bool ok, ValidationStatus validation, std::string information) {
    //    validation_results.emplace_back(ValidationResult(index, ok, validation, information));
    //}

};
