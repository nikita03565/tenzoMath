#include "fanucModelExtension.h"

namespace nikita
{

FanucModelExtension::FanucModelExtension() :
    _toCamera((cv::Mat_<double>(4, 4) << 0, -1, 0, -43, 1, 0, 0, -90, 0, 0, 1, 130, 0, 0, 0, 1)),
    _toSixth((cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -190, 0, 0, 0, 1)),
    _forMovingToCamera((cv::Mat_<double>(4, 4) << 0, -1, 0, 43, 1, 0, 0, 90, 0, 0, 1, 130, 0, 0, 0, 1))
{
}

std::array<double, 3> FanucModelExtension::anglesFromMat(const cv::Mat p6)
{
    std::array<double, 3> angleVector{};
    angleVector.at(0) = atan2(p6.at<double>(2, 1), p6.at<double>(2, 2));
    angleVector.at(1) = atan2(-p6.at<double>(2, 0),
        sqrt(p6.at<double>(2, 1) * p6.at<double>(2, 1) + p6.at<double>(2, 2) * p6.at<double>(2, 2)));
    angleVector.at(2) = atan2(p6.at<double>(1, 0), p6.at<double>(0, 0));
    return angleVector;
}

std::array<double, 6> FanucModelExtension::getCoordsFromMat(cv::Mat transformMatrix)
{
    std::array<double, 3> wprAngles = anglesFromMat(transformMatrix);

    return { {
            transformMatrix.at<double>(0, 3), transformMatrix.at<double>(1, 3), transformMatrix.at<double>(2, 3),
            wprAngles.at(0), wprAngles.at(1), wprAngles.at(2)
        } };
}

cv::Mat FanucModelExtension::getToCamera() const
{
    return _toCamera;
}

cv::Mat FanucModelExtension::getToSixth() const
{
    return _toSixth;
}

cv::Mat FanucModelExtension::getforMovingToCamera() const
{
    return _forMovingToCamera;
}

cv::Mat FanucModelExtension::transMatrix(const double coord0, const double coord1, const double coord2, const double coord3,
    const double coord4, const double coord5)
{
    cv::Mat res = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
    rotMatrix(coord3 / 180.0 * PI, coord4 / 180.0 * PI, coord5 / 180.0 * PI).copyTo(res(cv::Rect(0, 0, 3, 3)));
    res.at<double>(0, 3) = coord0;
    res.at<double>(1, 3) = coord1;
    res.at<double>(2, 3) = coord2;
    res.at<double>(3, 3) = 1;
    return res;
}

} // namespace nikita