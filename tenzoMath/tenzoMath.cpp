#include <fstream>
#include <string>
#include <iostream>

#include "TenzoMath.h"


namespace nikita
{

TenzoMath::TenzoMath()
{}

std::array<double, 6> TenzoMath::gravCompensation(const cv::Mat& p6, std::array<int, 6>& rawData)
{
    cv::Mat fgmaxCurr(3, 3, CV_64F);
    cv::Mat tmaxCurr(3, 3, CV_64F);
    std::array<double, 6> newData{};

    cv::Mat currRot(3, 3, cv::DataType<double>::type);
    p6(cv::Rect(0, 0, 3, 3)).copyTo(currRot);
    cv::Mat gravProjection = currRot.t() * _g;

    if (gravProjection.at<double>(0, 0) > 0)
    {
        fgmaxCurr.at<double>(0, 0) = _fgmax.at<double>(0, 0) * gravProjection.at<double>(0, 0);
        fgmaxCurr.at<double>(0, 1) = _fgmax.at<double>(0, 1) * gravProjection.at<double>(0, 0);
        fgmaxCurr.at<double>(0, 2) = _fgmax.at<double>(0, 2) * gravProjection.at<double>(0, 0);

        tmaxCurr.at<double>(0, 0) = _tmax.at<double>(0, 0) * gravProjection.at<double>(0, 0);
        tmaxCurr.at<double>(0, 1) = _tmax.at<double>(0, 1) * gravProjection.at<double>(0, 0);
        tmaxCurr.at<double>(0, 2) = _tmax.at<double>(0, 2) * gravProjection.at<double>(0, 0);
    }
    else
    {
        fgmaxCurr.at<double>(0, 0) = -_fgmaxNeg.at<double>(0, 0) * gravProjection.at<double>(0, 0);
        fgmaxCurr.at<double>(0, 1) = -_fgmaxNeg.at<double>(0, 1) * gravProjection.at<double>(0, 0);
        fgmaxCurr.at<double>(0, 2) = -_fgmaxNeg.at<double>(0, 2) * gravProjection.at<double>(0, 0);

        tmaxCurr.at<double>(0, 0) = -_tmaxNeg.at<double>(0, 0) * gravProjection.at<double>(0, 0);
        tmaxCurr.at<double>(0, 1) = -_tmaxNeg.at<double>(0, 1) * gravProjection.at<double>(0, 0);
        tmaxCurr.at<double>(0, 2) = -_tmaxNeg.at<double>(0, 2) * gravProjection.at<double>(0, 0);
    }

    if (gravProjection.at<double>(1, 0) > 0)
    {
        fgmaxCurr.at<double>(1, 0) = _fgmax.at<double>(1, 0) * gravProjection.at<double>(1, 0);
        fgmaxCurr.at<double>(1, 1) = _fgmax.at<double>(1, 1) * gravProjection.at<double>(1, 0);
        fgmaxCurr.at<double>(1, 2) = _fgmax.at<double>(1, 2) * gravProjection.at<double>(1, 0);

        tmaxCurr.at<double>(1, 0) = _tmax.at<double>(1, 0) * gravProjection.at<double>(1, 0);
        tmaxCurr.at<double>(1, 1) = _tmax.at<double>(1, 1) * gravProjection.at<double>(1, 0);
        tmaxCurr.at<double>(1, 2) = _tmax.at<double>(1, 2) * gravProjection.at<double>(1, 0);
    }
    else
    {
        fgmaxCurr.at<double>(1, 0) = -_fgmaxNeg.at<double>(1, 0) * gravProjection.at<double>(1, 0);
        fgmaxCurr.at<double>(1, 1) = -_fgmaxNeg.at<double>(1, 1) * gravProjection.at<double>(1, 0);
        fgmaxCurr.at<double>(1, 2) = -_fgmaxNeg.at<double>(1, 2) * gravProjection.at<double>(1, 0);

        tmaxCurr.at<double>(1, 0) = -_tmaxNeg.at<double>(1, 0) * gravProjection.at<double>(1, 0);
        tmaxCurr.at<double>(1, 1) = -_tmaxNeg.at<double>(1, 1) * gravProjection.at<double>(1, 0);
        tmaxCurr.at<double>(1, 2) = -_tmaxNeg.at<double>(1, 2) * gravProjection.at<double>(1, 0);
    }

    if (gravProjection.at<double>(2, 0) > 0)
    {
        fgmaxCurr.at<double>(2, 0) = _fgmax.at<double>(2, 0) * gravProjection.at<double>(2, 0);
        fgmaxCurr.at<double>(2, 1) = _fgmax.at<double>(2, 1) * gravProjection.at<double>(2, 0);
        fgmaxCurr.at<double>(2, 2) = _fgmax.at<double>(2, 2) * gravProjection.at<double>(2, 0);

        tmaxCurr.at<double>(2, 0) = _tmax.at<double>(2, 0) * gravProjection.at<double>(2, 0);
        tmaxCurr.at<double>(2, 1) = _tmax.at<double>(2, 1) * gravProjection.at<double>(2, 0);
        tmaxCurr.at<double>(2, 2) = _tmax.at<double>(2, 2) * gravProjection.at<double>(2, 0);
    }
    else
    {
        fgmaxCurr.at<double>(2, 0) = -_fgmaxNeg.at<double>(2, 0) * gravProjection.at<double>(2, 0);
        fgmaxCurr.at<double>(2, 1) = -_fgmaxNeg.at<double>(2, 1) * gravProjection.at<double>(2, 0);
        fgmaxCurr.at<double>(2, 2) = -_fgmaxNeg.at<double>(2, 2) * gravProjection.at<double>(2, 0);

        tmaxCurr.at<double>(2, 0) = -_tmaxNeg.at<double>(2, 0) * gravProjection.at<double>(2, 0);
        tmaxCurr.at<double>(2, 1) = -_tmaxNeg.at<double>(2, 1) * gravProjection.at<double>(2, 0);
        tmaxCurr.at<double>(2, 2) = -_tmaxNeg.at<double>(2, 2) * gravProjection.at<double>(2, 0);
    }

    newData[0] = static_cast<double>(rawData[0]) - _forcesBias[0] - fgmaxCurr.at<double>(0, 0) - fgmaxCurr.at<double>(1, 0) - fgmaxCurr.at<double>(2, 0);
    newData[1] = static_cast<double>(rawData[1]) - _forcesBias[1] - fgmaxCurr.at<double>(0, 1) - fgmaxCurr.at<double>(1, 1) - fgmaxCurr.at<double>(2, 1);
    newData[2] = static_cast<double>(rawData[2]) - _forcesBias[2] - fgmaxCurr.at<double>(0, 2) - fgmaxCurr.at<double>(1, 2) - fgmaxCurr.at<double>(2, 2);
    newData[3] = static_cast<double>(rawData[3]) - _torquesBias[0] - tmaxCurr.at<double>(0, 0) - tmaxCurr.at<double>(1, 0) - tmaxCurr.at<double>(2, 0);
    newData[4] = static_cast<double>(rawData[4]) - _torquesBias[1] - tmaxCurr.at<double>(0, 1) - tmaxCurr.at<double>(1, 1) - tmaxCurr.at<double>(2, 1);
    newData[5] = static_cast<double>(rawData[5]) - _torquesBias[2] - tmaxCurr.at<double>(0, 2) - tmaxCurr.at<double>(1, 2) - tmaxCurr.at<double>(2, 2);

    return newData;
}

void TenzoMath::calculateNextPos(std::array<int, 6>& curPos, std::array<int, 6>& readings)
{
    cv::Mat currRot = FanucModel::rotMatrix(curPos[3] / 180'000.0 * FanucModel::PI, curPos[4] / 180'000.0 * FanucModel::PI,
        curPos[5] / 180'000.0 * FanucModel::PI);

    constexpr double coefForces = 0.005;
    constexpr double coefTorques = 0.001;
    constexpr double threshold = 150.0;

    cv::Mat forces(1, 3, cv::DataType<double>::type);
    cv::Mat torques(1, 3, cv::DataType<double>::type);

    std::array<double, 6> newData = gravCompensation(currRot, readings);
    forces.at<double>(0, 0) = (abs(newData[0]) < threshold ? 0 : newData[0] * coefForces);
    forces.at<double>(0, 1) = (abs(newData[1]) < threshold ? 0 : newData[1] * coefForces);
    forces.at<double>(0, 2) = (abs(newData[2]) < threshold ? 0 : newData[2] * coefForces);
    torques.at<double>(0, 0) = (abs(newData[3]) < threshold ? 0 : newData[3] * coefTorques);
    torques.at<double>(0, 1) = (abs(newData[4]) < threshold ? 0 : newData[4] * coefTorques);
    torques.at<double>(0, 2) = (abs(newData[5]) < threshold ? 0 : newData[5] * coefTorques * 5.0);

    forces *= currRot.t();
    torques *= currRot.t();

    curPos[0] += static_cast<int>(forces.at<double>(0, 0));
    curPos[1] += static_cast<int>(forces.at<double>(0, 1));
    curPos[2] += static_cast<int>(forces.at<double>(0, 2));
    curPos[3] += static_cast<int>(torques.at<double>(0, 0));
    curPos[4] -= static_cast<int>(torques.at<double>(0, 1));
    curPos[5] -= static_cast<int>(torques.at<double>(0, 2));
}

} //namespace nikita