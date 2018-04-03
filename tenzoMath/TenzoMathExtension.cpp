#include "TenzoMathExtension.h"
#include <iostream>

namespace nikita
{
TenzoMathExtension::TenzoMathExtension() : _tenzoData(L"COM13")
{
}


std::array<double, 6>  TenzoMathExtension::swapData(const std::array<double, 6>& data)
{
    return std::array<double, 6>{ data[1], -data[0], -data[2], -data[4], data[3], data[5] };
}

std::array<int, 6>  TenzoMathExtension::convertToInt(const std::array<double, 6>& coord)
{
    std::array<int, 6> convert{};
    for (std::size_t i = 0; i < coord.size(); ++i)
    {
        convert.at(i) = static_cast<int>(coord.at(i) * 1000.0);
    }
    return convert;
}

std::array<double, 6>  TenzoMathExtension::convertToDouble(const std::array<int, 6>& coord)
{
    std::array<double, 6> convert{};
    for (std::size_t i = 0; i < coord.size(); ++i)
    {
        convert.at(i) = static_cast<double>(coord.at(i)) / 1000.0;
    }
    return convert;
}

std::string  TenzoMathExtension::toString(const std::array<double, 6>& coord) const
{
    std::array<int, 6> convert = convertToInt(coord);

    std::stringstream stringStream;

    std::copy(convert.begin(), convert.end(), std::ostream_iterator<int>(stringStream, " "));

    return stringStream.str();
}

void  TenzoMathExtension::calculatePos(std::array<int, 6>& curPos)
{
    std::array<double, 6> data = swapData(_tenzoData.readComStrain());
    std::array<int, 6> newData = convertToInt(data);
    return  TenzoMath::calculateNextPos(curPos, newData);
}

std::string  TenzoMathExtension::getCoordToMove() const
{
    return _coordToMove;
}

void  TenzoMathExtension::collectData(const std::size_t index)
{
    std::getchar();

    std::array<double, 6> tmp = swapData(_tenzoData.readComStrain());
    for (std::size_t i = 0; i < 6; ++i)
    {
        std::cout << tmp[i] << ' ';
        _collectedData.at<double>(static_cast<int>(index), static_cast<int>(i)) = tmp[i];
    }
    std::cout << '\n';
}

std::array<double, 6>  TenzoMathExtension::jointsToWorld(const std::array<double, 6>& joints)
{
    cv::Mat tmp = _model.fanucForwardTask(joints);
    std::array<double, 3> tmptmp = FanucModelExtension::anglesFromMat(tmp);
    return std::array<double, 6>{
        tmp.at<double>(0, 3), tmp.at<double>(1, 3), tmp.at<double>(2, 3), tmptmp[0] * 180. / FanucModel::PI,
            tmptmp[1] * 180. / FanucModel::PI, tmptmp[2] * 180. / FanucModel::PI
    };
}
} //namespace nikita