#include <fstream>
#include <iostream>

#include "TenzoCalibration.h"
#include "TenzoMathExtension.h"

namespace nikita
{

TenzoCalibration::TenzoCalibration() :
    _positions({ {
        { 0, 0, 0, 0, -90,   0 },
        { 0, 0, 0, 0,   0,   0 },
        { 0, 0, 0, 0,   0, 180 },
        { 0, 0, 0, 0,  90, 180 },
        { 0, 0, 0, 0,   0, 270 },
        { 0, 0, 0, 0,   0,  90 },
    } })
{
    _g = cv::Mat(3, 1, cv::DataType<double>::type);
    _g.at<double>(0, 0) = 0.;
    _g.at<double>(1, 0) = 0.;
    _g.at<double>(2, 0) = -1;

    _fgmax = cv::Mat(3, 3, cv::DataType<double>::type);

    _tmax = cv::Mat(3, 3, cv::DataType<double>::type);

    _tmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

    _fgmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

    _collectedData = cv::Mat(6, 6, cv::DataType<double>::type);
}
void  TenzoCalibration::calculateCalibration()
{
    _forcesBias[0] = (_collectedData.at<double>(1, 0) + _collectedData.at<double>(2, 0)) / 2.0;
    _forcesBias[1] = (_collectedData.at<double>(4, 1) + _collectedData.at<double>(5, 1)) / 2.0;
    _forcesBias[2] = (_collectedData.at<double>(0, 2) + _collectedData.at<double>(3, 2)) / 2.0;

    _torquesBias[0] = (_collectedData.at<double>(1, 3) + _collectedData.at<double>(2, 3)) / 2.0;
    _torquesBias[1] = (_collectedData.at<double>(4, 4) + _collectedData.at<double>(5, 4)) / 2.0;
    _torquesBias[2] = (_collectedData.at<double>(0, 5) + _collectedData.at<double>(3, 5)) / 2.0;

    _fgmax.at<double>(0, 0) = _collectedData.at<double>(2, 0) - _forcesBias[0];
    _fgmax.at<double>(0, 1) = _collectedData.at<double>(2, 1) - _forcesBias[1];
    _fgmax.at<double>(0, 2) = _collectedData.at<double>(2, 2) - _forcesBias[2];

    _fgmax.at<double>(1, 0) = _collectedData.at<double>(4, 0) - _forcesBias[0];
    _fgmax.at<double>(1, 1) = _collectedData.at<double>(4, 1) - _forcesBias[1];
    _fgmax.at<double>(1, 2) = _collectedData.at<double>(4, 2) - _forcesBias[2];

    _fgmax.at<double>(2, 0) = _collectedData.at<double>(0, 0) - _forcesBias[0];
    _fgmax.at<double>(2, 1) = _collectedData.at<double>(0, 1) - _forcesBias[1];
    _fgmax.at<double>(2, 2) = _collectedData.at<double>(0, 2) - _forcesBias[2];

    _tmax.at<double>(0, 0) = _collectedData.at<double>(2, 3) - _torquesBias[0];
    _tmax.at<double>(0, 1) = _collectedData.at<double>(2, 4) - _torquesBias[1];
    _tmax.at<double>(0, 2) = _collectedData.at<double>(2, 5) - _torquesBias[2];

    _tmax.at<double>(1, 0) = _collectedData.at<double>(4, 3) - _torquesBias[0];
    _tmax.at<double>(1, 1) = _collectedData.at<double>(4, 4) - _torquesBias[1];
    _tmax.at<double>(1, 2) = _collectedData.at<double>(4, 5) - _torquesBias[2];

    _tmax.at<double>(2, 0) = _collectedData.at<double>(0, 3) - _torquesBias[0];
    _tmax.at<double>(2, 1) = _collectedData.at<double>(0, 4) - _torquesBias[1];
    _tmax.at<double>(2, 2) = _collectedData.at<double>(0, 5) - _torquesBias[2];

    _fgmaxNeg.at<double>(0, 0) = _collectedData.at<double>(1, 0) - _forcesBias[0];
    _fgmaxNeg.at<double>(0, 1) = _collectedData.at<double>(1, 1) - _forcesBias[1];
    _fgmaxNeg.at<double>(0, 2) = _collectedData.at<double>(1, 2) - _forcesBias[2];

    _fgmaxNeg.at<double>(1, 0) = _collectedData.at<double>(5, 0) - _forcesBias[0];
    _fgmaxNeg.at<double>(1, 1) = _collectedData.at<double>(5, 1) - _forcesBias[1];
    _fgmaxNeg.at<double>(1, 2) = _collectedData.at<double>(5, 2) - _forcesBias[2];

    _fgmaxNeg.at<double>(2, 0) = _collectedData.at<double>(3, 0) - _forcesBias[0];
    _fgmaxNeg.at<double>(2, 1) = _collectedData.at<double>(3, 1) - _forcesBias[1];
    _fgmaxNeg.at<double>(2, 2) = _collectedData.at<double>(3, 2) - _forcesBias[2];

    _tmaxNeg.at<double>(0, 0) = _collectedData.at<double>(1, 3) - _torquesBias[0];
    _tmaxNeg.at<double>(0, 1) = _collectedData.at<double>(1, 4) - _torquesBias[1];
    _tmaxNeg.at<double>(0, 2) = _collectedData.at<double>(1, 5) - _torquesBias[2];

    _tmaxNeg.at<double>(1, 0) = _collectedData.at<double>(5, 3) - _torquesBias[0];
    _tmaxNeg.at<double>(1, 1) = _collectedData.at<double>(5, 4) - _torquesBias[1];
    _tmaxNeg.at<double>(1, 2) = _collectedData.at<double>(5, 5) - _torquesBias[2];

    _tmaxNeg.at<double>(2, 0) = _collectedData.at<double>(3, 3) - _torquesBias[0];
    _tmaxNeg.at<double>(2, 1) = _collectedData.at<double>(3, 4) - _torquesBias[1];
    _tmaxNeg.at<double>(2, 2) = _collectedData.at<double>(3, 5) - _torquesBias[2];

    std::ofstream out("calibData.txt");
    if (!out)
    {
        std::cout << "file is not opened\n";
    }

    out << _forcesBias[0] << ' ' << _forcesBias[1] << ' ' << _forcesBias[2] << ' ' << _torquesBias[0] << ' '
        << _torquesBias[1] << ' ' << _torquesBias[2] << '\n';

    std::cout << _forcesBias[0] << ' ' << _forcesBias[1] << ' ' << _forcesBias[2] << ' ' << _torquesBias[0] << ' '
        << _torquesBias[1] << ' ' << _torquesBias[2] << '\n';

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out << _fgmax.at<double>(i, j) << ' ';
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out << _fgmaxNeg.at<double>(i, j) << ' ';
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out << _tmax.at<double>(i, j) << ' ';
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out << _tmaxNeg.at<double>(i, j) << ' ';
    out.close();
}

void  TenzoMathExtension::loadCalibData()
{
    std::ifstream input("calibData.txt");
    if (!input)
    {
        std::cout << "file is not found\n";
    }

    input >> _forcesBias[0] >> _forcesBias[1] >> _forcesBias[2] >> _torquesBias[0] >> _torquesBias[1] >> _torquesBias[2];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            input >> _fgmax.at<double>(i, j);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            input >> _fgmaxNeg.at<double>(i, j);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            input >> _tmax.at<double>(i, j);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            input >> _tmaxNeg.at<double>(i, j);
    input.close();
}
} //namespace nikita