#include "tenzoMath.h"
#include <fstream>
#include <thread>
#include <chrono>
#include "poly34.h"
#include <cmath>
#include <string>
#include <vector>
void printCvMat(cv::Mat t, const char str[] = "some matrix")
{
    std::cout << str << ":\n";
    for (int i = 0; i < t.rows; ++i)
    {
        for (int j = 0; j < t.cols; ++j)
            if (abs(t.at<double>(i, j)) < 0.001)
                std::cout << 0 << ' ';
            else
                std::cout << t.at<double>(i, j) << ' ';
        std::cout << std::endl;
    }
    std::cout << "-------------------------" << std::endl;
}

TenzoMath::TenzoMath() : _xMin (680.),
                         _yMin (-465.),
                         _zMin ( 520.),
                         _xMax ( 1380.),
                         _yMax ( 465.),
                         _zMax ( 1300.)
{
    _positions = {
        {
            { 0, 0, 0, 0, -90, 0},
            { 0, 0, 0, 0, 0, 0},
            { 0, 0, 0, 0, 0, 180},
            { 0, 0, 0, 0, 90, 180},
            { 0, 0, 0, 0, 0, 270},
            { 0, 0, 0, 0, 0, 90}
        }
    };

    _g = cv::Mat(3, 1, cv::DataType<double>::type);
    _g.at<double>(0, 0) = 0.;
    _g.at<double>(1, 0) = 0.;
    _g.at<double>(2, 0) = -1;

    _fgmax = cv::Mat(3, 3, cv::DataType<double>::type);

    _tmax = cv::Mat(3, 3, cv::DataType<double>::type);

    _tmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

    _fgmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

    _collectedData = cv::Mat(6, 6, cv::DataType<double>::type);

    _isConnectedFanuc = false;
}

void TenzoMath::doCalibration()
{
    if (!_isConnectedFanuc)
    {
        _fanuc.startWorking();
        _fanuc.setJointFrame();
        _isConnectedFanuc = true;
    }
    Tenzo tenzoData(L"COM6");

    for (int i = 0; i < 6; ++i)
    {
        //Robot.goTo and wait
        _fanuc.goToCoordinates(_positions[i][0], _positions[i][1], _positions[i][2],
                               _positions[i][3],
                               _positions[i][4],
                               _positions[i][5]);

        getchar();

        //readData and put it in arrays
        std::array<double, 6> tmp = tenzoData.readData();
        _collectedData.at<double>(i, 0) = tmp[1];
        _collectedData.at<double>(i, 1) = -tmp[0];
        _collectedData.at<double>(i, 2) = -tmp[2];
        _collectedData.at<double>(i, 3) = -tmp[4];
        _collectedData.at<double>(i, 4) = tmp[3];
        _collectedData.at<double>(i, 5) = tmp[5];
    }
    std::ofstream collectedData("collectedData.txt");
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            collectedData << _collectedData.at<double>(i, j) << ' ';
    collectedData.close();

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

    out << _forcesBias[0] << ' ' << _forcesBias[1] << ' ' << _forcesBias[2] << ' ' << _torquesBias[0] << ' ' 
        << _torquesBias[1] << ' ' << _torquesBias[2] << std::endl;
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

std::array<double, 6> TenzoMath::gravCompensation(cv::Mat p6, std::array<double, 6> rawData)
{
    //std::cout << _forcesBias[0] << '\t' << _forcesBias[1] << '\t' << _forcesBias[2] << '\t' << _torquesBias[0] << '\t' << _torquesBias[1] << '\t' << _torquesBias[2] << "\n\n";
    //std::cout << _fgmax << "\n\n" << _fgmaxNeg << "\n\n" << _tmax << "\n\n" << _tmaxNeg << "\n\n";
    cv::Mat fgmaxCurr(3, 3, CV_64F);
    cv::Mat tmaxCurr(3, 3, CV_64F);
    std::array<double, 6> newData;
    
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

    newData[0] = rawData[0] - _forcesBias[0] - fgmaxCurr.at<double>(0, 0) - fgmaxCurr.at<double>(1, 0) - fgmaxCurr.at<double>(2, 0);
    newData[1] = rawData[1] - _forcesBias[1] - fgmaxCurr.at<double>(0, 1) - fgmaxCurr.at<double>(1, 1) - fgmaxCurr.at<double>(2, 1);
    newData[2] = rawData[2] - _forcesBias[2] - fgmaxCurr.at<double>(0, 2) - fgmaxCurr.at<double>(1, 2) - fgmaxCurr.at<double>(2, 2);
    newData[3] = rawData[3] - _torquesBias[0] - tmaxCurr.at<double>(0, 0) - tmaxCurr.at<double>(1, 0) - tmaxCurr.at<double>(2, 0);
    newData[4] = rawData[4] - _torquesBias[1] - tmaxCurr.at<double>(0, 1) - tmaxCurr.at<double>(1, 1) - tmaxCurr.at<double>(2, 1);
    newData[5] = rawData[5] - _torquesBias[2] - tmaxCurr.at<double>(0, 2) - tmaxCurr.at<double>(1, 2) - tmaxCurr.at<double>(2, 2);

    return newData;
}

void TenzoMath::loadCalibData()
{
    std::ifstream input("calibData.txt");
    if (!input)
        std::cout << "file is not found\n";
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

cv::Mat TenzoMath::rotMatrix(const double& w, const double& p, const double& r) const
{
    cv::Mat mx(3, 3, CV_64F), my(3, 3, CV_64F), mz(3, 3, CV_64F);
    mx.at<double>(0, 0) = 1;
    mx.at<double>(0, 1) = mx.at<double>(0, 2) = mx.at<double>(1, 0) = mx.at<double>(2, 0) = 0;
    mx.at<double>(1, 1) = mx.at<double>(2, 2) = cos(w);
    mx.at<double>(1, 2) = -sin(w);
    mx.at<double>(2, 1) = sin(w);

    my.at<double>(1, 1) = 1;
    my.at<double>(0, 1) = my.at<double>(1, 2) = my.at<double>(1, 0) = my.at<double>(2, 1) = 0;
    my.at<double>(0, 0) = my.at<double>(2, 2) = cos(p);
    my.at<double>(0, 2) = sin(p);
    my.at<double>(2, 0) = -sin(p);

    mz.at<double>(2, 2) = 1;
    mz.at<double>(0, 2) = mz.at<double>(1, 2) = mz.at<double>(2, 0) = mz.at<double>(2, 1) = 0;
    mz.at<double>(1, 1) = mz.at<double>(0, 0) = cos(r);
    mz.at<double>(0, 1) = -sin(r);
    mz.at<double>(1, 0) = sin(r);

    cv::Mat ans = mz * my * mx;
    return ans;
}

void TenzoMath::ftControlCartesianCoord()
{
    std::array<double, 6> worldPos = {
        985.0, 0.0, 940.0, -180.0, 0.0, 0.0
    };
    Tenzo tenzoData(L"COM6");
    std::array<double, 6> collectedData;
    cv::Mat forces(1, 3, cv::DataType<double>::type);
    cv::Mat torques(1, 3, cv::DataType<double>::type);
    cv::Mat currRot(3, 3, cv::DataType<double>::type);
    constexpr double coefForces = 0.005;
    constexpr double coefTorques = 0.001;
    constexpr double threshold = 150;
    //FanucM20iA fanuc;
    _fanuc.startWorking();
    _fanuc.setWorldFrame();
    while (true)
    {
        currRot = rotMatrix(worldPos[3] / 180.0 * PI, worldPos[4] / 180.0 * PI,
                            worldPos[5] / 180.0 * PI);
        std::array<double, 6> tmp = tenzoData.readData();
        collectedData[0] = tmp[1];
        collectedData[1] = -tmp[0];
        collectedData[2] = -tmp[2];
        collectedData[3] = -tmp[4];
        collectedData[4] = tmp[3];
        collectedData[5] = tmp[5];
        std::array<double, 6> newData = gravCompensation(currRot, collectedData);
        forces.at<double>(0, 0) = (abs(newData[0]) < threshold ? 0 : newData[0] * coefForces);
        forces.at<double>(0, 1) = (abs(newData[1]) < threshold ? 0 : newData[1] * coefForces);
        forces.at<double>(0, 2) = (abs(newData[2]) < threshold ? 0 : newData[2] * coefForces);
        torques.at<double>(0, 0) = (abs(newData[3]) < threshold ? 0 : newData[3] * coefTorques);
        torques.at<double>(0, 1) = (abs(newData[4]) < threshold ? 0 : newData[4] * coefTorques);
        torques.at<double>(0, 2) = (abs(newData[5]) < threshold ? 0 : newData[5] * coefTorques * 5);

        forces *= currRot.t();
        torques *= currRot.t();

        //std::cout << forces.at<double>(0, 0) << '\t' << forces.at<double>(0, 1) << '\t' << forces.at<double>(0, 2) << '\t'
        //		<< torques.at<double>(0, 0) << '\t' << torques.at<double>(0, 1) << '\t' << torques.at<double>(0, 2) << std::endl;

        worldPos[0] += forces.at<double>(0, 0);
        worldPos[1] += forces.at<double>(0, 1);
        worldPos[2] += forces.at<double>(0, 2);
        worldPos[3] += torques.at<double>(0, 0);
        worldPos[4] -= torques.at<double>(0, 1);
        worldPos[5] -= torques.at<double>(0, 2);

        for (int i = 3; i < 6; ++i)
        {
            if (worldPos[i] > 180.0f)
                worldPos[i] -= 360.0f;
            if (worldPos[i] < -180.0f)
                worldPos[i] += 360.0f;
        }
        for (int i = 0; i < 6; ++i)
        {
            std::cout << worldPos[i] << '\t';
        }
        std::cout << std::endl;
        _fanuc.goToCoordinates(worldPos[0], worldPos[1], worldPos[2], worldPos[3], worldPos[4],
                               worldPos[5]);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

//spherical wrist coordinates: homepos 985, 0, 1040; 
cv::Mat TenzoMath::inverseTask(const std::array<double, 6> coord) const
{
    std::vector<RoboModel::DhParameters> param = _model.getDhParameters();

    const double a = 2 * param[0]._aParam * coord[0];
    const double b = 2 * param[0]._aParam * coord[1];
    const double c = 2 * param[1]._aParam * param[2]._aParam - 2 * param[1]._dParam * param[3]._dParam  * 
                     sin(param[1]._alphaParam) * sin(param[2]._alphaParam);
    const double d = 2 * param[2]._aParam * param[1]._dParam * sin(param[1]._alphaParam) + 2 * param[1]._aParam * param[3]._dParam
                     * sin(param[2]._alphaParam);
    const double e = param[1]._aParam * param[1]._aParam + param[2]._aParam * param[2]._aParam + param[1]._dParam *
                     param[1]._dParam + param[2]._dParam * param[2]._dParam + param[3]._dParam * param[3]._dParam -
                     param[0]._aParam * param[0]._aParam - coord[0] * coord[0] - coord[1] * coord[1] - (coord[2] - param[0]._dParam) * (coord[2] - param[0]._dParam) + 2 *
                     param[1]._dParam * param[2]._dParam * cos(param[1]._alphaParam) + 2 * param[1]._dParam * param[3]._dParam *
                     cos(param[1]._alphaParam) * cos(param[2]._alphaParam) + 2 * param[2]._dParam * param[3]._dParam * cos(param[2]._alphaParam);
    const double f = coord[1] * sin(param[0]._alphaParam);
    const double g = -coord[0] * sin(param[0]._alphaParam);
    const double h = -param[3]._dParam * sin(param[1]._alphaParam) * sin(param[2]._alphaParam);
    const double i = param[2]._aParam * sin(param[1]._alphaParam);
    const double j = param[1]._dParam + param[2]._dParam * cos(param[1]._alphaParam) + param[3]._dParam *
                     cos(param[1]._alphaParam) * cos(param[2]._alphaParam) - (coord[2] - param[0]._dParam) * cos(param[0]._alphaParam);
    const double r = 4. * param[0]._aParam * param[0]._aParam * (j - h) * (j - h) + sin(param[0]._alphaParam) * sin(param[0]._alphaParam) * (e - c) * (e - c) 
                   - 4. * param[0]._aParam * param[0]._aParam * sin(param[0]._alphaParam) * sin(param[0]._alphaParam) * (coord[0] * coord[0] + coord[1] * coord[1]);
    const double s = 4. * (4. * param[0]._aParam * param[0]._aParam * i * (j - h) + sin(param[0]._alphaParam) * sin(param[0]._alphaParam) * d * 
                     (e - c));
    const double t = 2. * (4. * param[0]._aParam * param[0]._aParam * (j * j - h * h + 2 * i * i) + sin(param[0]._alphaParam) * sin(param[0]._alphaParam)
                     * (e * e - c * c + 2 * d * d) - 4. * param[0]._aParam * param[0]._aParam * sin(param[0]._alphaParam) * sin(param[0]._alphaParam) *
                     (coord[0] * coord[0] + coord[1] * coord[1]) );
    const double u = 4. * (4. * param[0]._aParam * param[0]._aParam * i * (j + h) +
                     sin(param[0]._alphaParam) * sin(param[0]._alphaParam) * d * (e + c));
    const double v = 4. * param[0]._aParam * param[0]._aParam * (h + j) * (h + j) + sin(param[0]._alphaParam) * sin(param[0]._alphaParam) * 
                     (e + c) * (e + c) - 4. * param[0]._aParam * param[0]._aParam * sin(param[0]._alphaParam) * sin(param[0]._alphaParam) * 
                     (coord[0] * coord[0] + coord[1] * coord[1]);

    double x[4];
    const int numberOfRoots = SolveP4(x, s / r, t / r, u / r, v / r);

    if (numberOfRoots != 2 && numberOfRoots != 4)
        std::cout << "something is wrong with roots of equatation";

    cv::Mat theta(numberOfRoots, 3, cv::DataType<double>::type);

    for (int it = 0; it < numberOfRoots; ++it)
    {
        theta.at<double>(it, 2) = 2. * atan(x[it]);
    }
   
    double costheta, sintheta;
    for (int it = 0; it < numberOfRoots; ++it)
    {
        costheta = (-g * (c * cos(theta.at<double>(it, 2)) + d * sin(theta.at<double>(it, 2)) + e) +
                    b * (h * cos(theta.at<double>(it, 2)) + i * sin(theta.at<double>(it, 2) + j))) / (a * g - f * b);
        sintheta = (f * (c * cos(theta.at<double>(it, 2)) + d * sin(theta.at<double>(it, 2)) + e) -
                    a * (h * cos(theta.at<double>(it, 2)) + i * sin(theta.at<double>(it, 2) + j))) / (a * g - f * b);
        if (sintheta >= 0)
            theta.at<double>(it, 0) = acos(costheta);
        else
            theta.at<double>(it, 0) = -acos(costheta);
    }

    for (int it = 0; it < numberOfRoots; ++it)
    {
        const double a11 = param[1]._aParam + param[2]._aParam * cos(theta.at<double>(it, 2)) +
                           param[3]._dParam * sin(param[2]._alphaParam) * sin(theta.at<double>(it, 2));
        const double a12 = -param[2]._aParam * cos(param[1]._alphaParam) *
                           sin(theta.at<double>(it, 2)) + param[2]._dParam * sin(param[1]._alphaParam) + param[3]._dParam *
                           sin(param[2]._alphaParam) * cos(param[1]._alphaParam) * cos(theta.at<double>(it, 2)) + 
                           param[3]._dParam * sin(param[1]._alphaParam) * cos(param[2]._alphaParam);
        costheta = (a11 * (coord[0] * cos(theta.at<double>(it, 0)) + coord[1] * sin(theta.at<double>(it, 0)) - param[0]._aParam)
                   - a12 * (-coord[0] * cos(param[0]._alphaParam) * sin(theta.at<double>(it, 0)) + coord[1] * cos(param[0]._alphaParam) *
                           cos(theta.at<double>(it, 0)) + (coord[2] - param[0]._dParam) * sin(param[0]._alphaParam))) / (a11 * a11 + a12 * a12);
        sintheta = (a12 * (coord[0] * cos(theta.at<double>(it, 0)) + coord[1] * sin(theta.at<double>(it, 0)) - param[0]._aParam) + a11 * 
                   (-coord[0] * cos(param[0]._alphaParam) * sin(theta.at<double>(it, 0)) + coord[1] * cos(param[0]._alphaParam) *
                           cos(theta.at<double>(it, 0)) + (coord[2] - param[0]._dParam) * sin(param[0]._alphaParam))) / (a11 * a11 + a12 * a12);
        if (sintheta >= 0)
            theta.at<double>(it, 1) = acos(costheta);
        else
            theta.at<double>(it, 1) = -acos(costheta);
    }
    for (int it = 0; it < numberOfRoots; ++it)
    {
        theta.at<double>(it, 1) = -theta.at<double>(it, 1) + PI / 2;
        theta.at<double>(it, 2) -= theta.at<double>(it, 1);
    }

    std::vector<int> ind;

    for (int it = 0; it < numberOfRoots; ++it)
    {
        bool isOk = true;

        if (abs(theta.at<double>(it, 0)) > 170. / 180. * PI)
            isOk = false;
        if (theta.at<double>(it, 1) > 90. / 180. * PI || theta.at<double>(it, 1) < -70 / 180. * PI)
            isOk = false;
        if (theta.at<double>(it, 2) > 200. / 180. * PI || theta.at<double>(it, 2) < -70 / 180. * PI)
            isOk = false;
        
        if (!std::isnan(theta.at<double>(it, 1)) && isOk)
            ind.emplace_back(it);   
    }
    cv::Mat thetaRes(ind.size(), 6, cv::DataType<double>::type);
    for (int it = 0; it < ind.size(); ++it)
    {
        thetaRes.at<double>(it, 0) = theta.at<double>(ind[it], 0);
        thetaRes.at<double>(it, 1) = theta.at<double>(ind[it], 1);
        thetaRes.at<double>(it, 2) = theta.at<double>(ind[it], 2);
        thetaRes.at<double>(it, 3) = 0;
        thetaRes.at<double>(it, 4) = 0;
        thetaRes.at<double>(it, 5) = 0;
    }
  
    cv::Mat thetaPrefinal(thetaRes.rows * 2, 6, cv::DataType<double>::type);
    for (int it = 0; it < thetaRes.rows; ++it)
    {
        cv::Mat r36(3, 3, cv::DataType<double>::type), r03(3, 3, cv::DataType<double>::type);
        std::array<double, 6> q;
        q[0] = thetaRes.at<double>(it, 0);
        q[1] = -thetaRes.at<double>(it, 1) + PI / 2;
        q[2] = thetaRes.at<double>(it, 2) + thetaRes.at<double>(it, 1);

        r03 = qi(param[0]._alphaParam, q[0]) * qi(param[1]._alphaParam, q[1]) * qi(param[2]._alphaParam, q[2]);
        r36 = r03.inv() * rotMatrix(coord[3] / 180. * PI, coord[4] / 180. * PI, coord[5] / 180. * PI);

        for (int zt = 0; zt < 2; ++zt)
        {
            thetaPrefinal.at<double>(it * 2 + zt, 0) = thetaRes.at<double>(it, 0);
            thetaPrefinal.at<double>(it * 2 + zt, 1) = thetaRes.at<double>(it, 1);
            thetaPrefinal.at<double>(it * 2 + zt, 2) = thetaRes.at<double>(it, 2);
        }
        
        thetaPrefinal.at<double>(it * 2, 4) = acos(r36.at<double>(2, 2));
        thetaPrefinal.at<double>(it * 2 + 1, 4) = -acos(r36.at<double>(2, 2));

        thetaPrefinal.at<double>(it * 2, 3) = -(PI - asin(r36.at<double>(1, 2) / sin(thetaPrefinal.at<double>(it * 2, 4))));
        thetaPrefinal.at<double>(it * 2 + 1, 3) = -asin(r36.at<double>(1, 2) / sin(thetaPrefinal.at<double>(it * 2 + 1, 4)));

        thetaPrefinal.at<double>(it * 2, 5) = -(PI - asin(r36.at<double>(2, 1) / sin(thetaPrefinal.at<double>(it * 2, 4))));
        thetaPrefinal.at<double>(it * 2 + 1, 5) = -asin(r36.at<double>(2, 1) / sin(thetaPrefinal.at<double>(it * 2 + 1, 4)));
    }

    std::vector<int> indFinal;
    for (int it = 0; it < thetaPrefinal.rows; ++it)
    {
        bool isOk = true;
        
        if (abs(thetaPrefinal.at<double>(it, 3)) > 210. / 180. * PI) isOk = false;
        if (abs(thetaPrefinal.at<double>(it, 4)) > 140. / 180. * PI) isOk = false;
        if (abs(thetaPrefinal.at<double>(it, 5)) > 270. / 180. * PI) isOk = false;

        if (isOk)
        {
            indFinal.emplace_back(it);
        }
    }

    cv::Mat thetaFinal(indFinal.size(), 6, cv::DataType<double>::type);
    for (int it = 0; it < indFinal.size(); ++it)
    {
        thetaFinal.at<double>(it, 0) = thetaPrefinal.at<double>(indFinal[it], 0);
        thetaFinal.at<double>(it, 1) = thetaPrefinal.at<double>(indFinal[it], 1);
        thetaFinal.at<double>(it, 2) = thetaPrefinal.at<double>(indFinal[it], 2);
        thetaFinal.at<double>(it, 3) = thetaPrefinal.at<double>(indFinal[it], 3);
        thetaFinal.at<double>(it, 4) = thetaPrefinal.at<double>(indFinal[it], 4);
        thetaFinal.at<double>(it, 5) = thetaPrefinal.at<double>(indFinal[it], 5);
    }

    return thetaFinal * 180. / PI;
}

std::array<double, 6> TenzoMath::chooseNearestPose(cv::Mat res, std::array<double, 6> prevPos) const
{
    if (!res.empty())
    {
        // std::cout << res << std::endl << std::endl;
        std::vector<double> delta;
        for (int j = 0; j < res.rows; ++j)
        {
            double deltaTmp = 0;
            for (int t = 0; t < 6; ++t)
            {
                deltaTmp += abs(res.at<double>(j, t) - prevPos[t]);
            }
            delta.emplace_back(deltaTmp);
        }
        if (!delta.empty())
        {
            int num = 0;
            double min = delta[0];
            for (int j = 0; j < delta.size(); ++j)
            {
                if (delta[j] < min)
                {
                    min = delta[j];
                    num = j;
                }
            }
            if (min < 10.f)
            {
                return std::array<double, 6>{res.at<double>(num, 0), res.at<double>(num, 1), res.at<double>(num, 2),  res.at<double>(num, 3), res.at<double>(num, 4), res.at<double>(num, 5) };
            }
        }
        return prevPos;
    }
    return prevPos;
}

cv::Mat TenzoMath::qi(const double& alpha, const  double& q) const
{
    cv::Mat tmp(3, 3, CV_64F);
    tmp.at<double>(0, 0) = cos(q);
    tmp.at<double>(0, 1) = -cos(alpha) * sin(q);
    tmp.at<double>(0, 2) = sin(alpha) * sin(q);

    tmp.at<double>(1, 0) = sin(q);
    tmp.at<double>(1, 1) = cos(alpha) * cos(q);
    tmp.at<double>(1, 2) = -sin(alpha) * cos(q);

    tmp.at<double>(2, 0) = 0;
    tmp.at<double>(2, 1) = sin(alpha);
    tmp.at<double>(2, 2) = cos(alpha);

    return tmp;
}
//
//void TenzoMath::ftControlJoints()
//{
//    std::array<double, 3> posCartesian = { 985, 0, 1040 };
//    std::array<double, 6> jointPos = { 0, 0, 0, 0, -90, 0 };
//
//    std::string errorMessage;
//
//    Tenzo tenzoData(L"COM6");
//    std::array<double, 6> collectedData;
//    cv::Mat forces(1, 3, cv::DataType<double>::type);
//    cv::Mat torques(1, 3, cv::DataType<double>::type);
//    constexpr double coefForces = 0.005;
//    constexpr double coefTorques = 0.001;
//    constexpr double threshold = 150;
//    _fanuc.startWorking();
//    _fanuc.setJointFrame();
//    while (true)
//    {
//        cv::Mat p6 = _model.fanucForwardTask(jointPos);
//
//        std::array<double, 6> tmp = tenzoData.readData();
//        collectedData[0] = tmp[1];
//        collectedData[1] = -tmp[0];
//        collectedData[2] = -tmp[2];
//        collectedData[3] = -tmp[4];
//        collectedData[4] = tmp[3];
//        collectedData[5] = tmp[5];
//
//        std::array<double, 6> newData = gravCompensation(p6, collectedData);
//        forces.at<double>(0, 0) = (abs(newData[0]) < threshold ? 0 : newData[0] * coefForces);
//        forces.at<double>(0, 1) = (abs(newData[1]) < threshold ? 0 : newData[1] * coefForces);
//        forces.at<double>(0, 2) = (abs(newData[2]) < threshold ? 0 : newData[2] * coefForces);
//        torques.at<double>(0, 0) = (abs(newData[3]) < threshold ? 0 : newData[3] * coefTorques);
//        torques.at<double>(0, 1) = (abs(newData[4]) < threshold ? 0 : newData[4] * coefTorques);
//        torques.at<double>(0, 2) = (abs(newData[5]) < threshold ? 0 : newData[5] * coefTorques * 5);
//
//        cv::Mat rot(3, 3, cv::DataType<double>::type);
//        p6(cv::Rect(0, 0, 3, 3)).copyTo(rot);
//        forces *= rot.t();
//
//        if (posCartesian[0] + forces.at<double>(0, 0) > _xMax) errorMessage = "Exceeding the maximum limits  X axis";
//        if (posCartesian[0] + forces.at<double>(0, 0) < _xMin) errorMessage = "Exceeding the minimum limits  X axis";
//
//        if (posCartesian[1] + forces.at<double>(0, 1) > _yMax) errorMessage = "Exceeding the maximum limits  Y axis";
//        if (posCartesian[1] + forces.at<double>(0, 1)  < _yMin) errorMessage = "Exceeding the minimum limits  Y axis";
//
//        if (posCartesian[2] + forces.at<double>(0, 2) > _zMax) errorMessage = "Exceeding the maximum limits  Z axis";
//        if (posCartesian[2] + forces.at<double>(0, 2) < _zMin) errorMessage = "Exceeding the minimum limits  Z axis";
//
//        if (errorMessage.empty())
//        {
//            posCartesian[0] += forces.at<double>(0, 0);
//            posCartesian[1] += forces.at<double>(0, 1);
//            posCartesian[2] += forces.at<double>(0, 2);
//            //jointPos[3] -= torques.at<double>(0, 0);
//            //jointPos[4] = jointPos[4] - torques.at<double>(0, 1) + jointPos[2];
//            //jointPos[5] += torques.at<double>(0, 2);
//            //std::cout << errorMessage << std::endl;
//            cv::Mat res = inverseTask(posCartesian[0], posCartesian[1], posCartesian[2]);
//            std::array<double, 3> nearest = chooseNearestPose(res, { jointPos[0], jointPos[1], jointPos[2] });
//            jointPos[4] = jointPos[4] - (nearest[2] - jointPos[2]);
//            jointPos[0] = nearest[0];
//            jointPos[1] = nearest[1];
//            jointPos[2] = nearest[2];
//           
//            //std::cout << //posCartesian[0] << '\t' << posCartesian[1] << '\t' << posCartesian[2] << ": " <<
//            //      jointPos[3] << '\t' << jointPos[4] << '\t' << jointPos[5] << '\t';
//            std::cout << jointPos[0] << '\t' << jointPos[1] << '\t' << jointPos[2] << ": " <<
//                jointPos[3] << '\t' << jointPos[4] << '\t' << jointPos[5] << '\t';
//            std::cout << std::endl;
//            _fanuc.goToCoordinates(jointPos[0], jointPos[1], jointPos[2], jointPos[3], jointPos[4], jointPos[5]);
//            std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        }
//        else
//        {
//            errorMessage.clear();
//        }
//    }     
//}
//
//void TenzoMath::collectTestData()
//{
//    if (!_isConnectedFanuc)
//    {
//        _fanuc.startWorking();
//        _fanuc.setJointFrame();
//        _isConnectedFanuc = true;
//    }
//    Tenzo tenzoData(L"COM6");
//
//    std::array<std::array<double, 6>, 8> pos = {
//        {
//            {
//                -5,
//                -3,
//                -10,
//                -120,
//                -90,
//                66
//            },
//            {
//                -0.917,
//                -8.377,
//                -9.526,
//                -165.933,
//                -40.823,
//                19.290
//            },
//            {
//                -4.872,
//                -0.907,
//                -10.075,
//                -118.418,
//                -107.636,
//                -53.147
//            },
//            {
//                3.921,
//                1.569,
//                -10.065,
//                120.544,
//                -128.754,
//                -42.986
//            },
//            {
//                -5,
//                -3,
//                -10,
//                -120,
//                -90,
//                66
//            },
//            {
//                -0.917,
//                -8.377,
//                -9.526,
//                -165.933,
//                -40.823,
//                19.290
//            },
//            {
//                -4.872,
//                -0.907,
//                -10.075,
//                -118.418,
//                -107.636,
//                -53.147
//            },
//            {
//                3.921,
//                1.569,
//                -10.065,
//                120.544,
//                -128.754,
//                -42.986
//            }
//            //, { 5.038, -3.019, -10.013, 120.297, -90.957, -65.788 }, 
//            //{ 5.167, -0.667, -3.718, 62.598, -93.559, 46.617 }, { 0.939, -8.375, -9.528, 165.796, -40.788, -19.003 },
//            //	{ 0, 0, 0, 0, -60, 30 }, { 0, 0, 0, 0, -60, -30 },{ 0, 0, 0, 50, -60, -30 }
//        }
//    };
//
//    cv::Mat collectedData(8, 6, CV_64F);
//    for (int i = 0; i < 8; ++i)
//    {
//        //Robot.goTo and wait
//        _fanuc.goToCoordinates(pos[i][0], pos[i][1], pos[i][2], pos[i][3], pos[i][4], pos[i][5]);
//
//        getchar();
//
//        //readData and put it in arrays
//        std::array<double, 6> tmp = tenzoData.readData();
//        collectedData.at<double>(i, 0) = tmp[1];
//        collectedData.at<double>(i, 1) = -tmp[0];
//        collectedData.at<double>(i, 2) = -tmp[2];
//        collectedData.at<double>(i, 3) = -tmp[4];
//        collectedData.at<double>(i, 4) = tmp[3];
//        collectedData.at<double>(i, 5) = tmp[5];
//        std::cout << i << std::endl;
//    }
//    std::cout << "collected";
//    std::ofstream out("collectedTestData.txt");
//    for (int i = 0; i < 8; ++i)
//        for (int j = 0; j < 6; ++j)
//            out << collectedData.at<double>(i, j) << ' ';
//    out.close();
//}
//
//void TenzoMath::doTest()
//{
//    std::array<std::array<double, 6>, 8> pos = {
//        {
//            {
//                -5,
//                -3,
//                -10,
//                -120,
//                -90,
//                66
//            },
//            {
//                -0.917,
//                -8.377,
//                -9.526,
//                -165.933,
//                -40.823,
//                19.290
//            },
//            {
//                -4.872,
//                -0.907,
//                -10.075,
//                -118.418,
//                -107.636,
//                -53.147
//            },
//            {
//                3.921,
//                1.569,
//                -10.065,
//                120.544,
//                -128.754,
//                -42.986
//            },
//            {
//                -5,
//                -3,
//                -10,
//                -120,
//                -90,
//                66
//            },
//            {
//                -0.917,
//                -8.377,
//                -9.526,
//                -165.933,
//                -40.823,
//                19.290
//            },
//            {
//                -4.872,
//                -0.907,
//                -10.075,
//                -118.418,
//                -107.636,
//                -53.147
//            },
//            {
//                3.921,
//                1.569,
//                -10.065,
//                120.544,
//                -128.754,
//                -42.986
//            }
//            //, { 5.038, -3.019, -10.013, 120.297, -90.957, -65.788 }, 
//            //{ 5.167, -0.667, -3.718, 62.598, -93.559, 46.617 }, { 0.939, -8.375, -9.528, 165.796, -40.788, -19.003 },
//            //	{ 0, 0, 0, 0, -60, 30 }, { 0, 0, 0, 0, -60, -30 },{ 0, 0, 0, 50, -60, -30 }
//        }
//    };
//
//    std::array<double, 6> rawData;
//
//    std::ifstream in("collectedData.txt");
//    for (int i = 0; i < 8; ++i)
//    {
//        for (int j = 0; j < 6; ++j)
//        {
//            in >> rawData[j];
//        }
//
//        std::array<double, 6> newData = gravCompensation(_model.fanucForwardTask(pos[i]), rawData);
//
//        std::cout << "rawData:\n ";
//        for (int j = 0; j < 6; ++j)
//        {
//            std::cout << rawData[j] << '\t';
//        }
//        std::cout << std::endl;
//
//        std::cout << "newData:\n ";
//        for (int j = 0; j < 6; ++j)
//        {
//            std::cout << newData[j] << '\t';
//        }
//        std::cout << std::endl << std::endl;
//    }
//    in.close();
//}
