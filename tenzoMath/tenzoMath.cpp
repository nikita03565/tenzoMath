#include "tenzoMath.h"
#include <fstream>
#include <thread>
#include <chrono>
#include "poly34.h"
#include <cmath>
#include <string>
#include <vector>
#include <tchar.h>
#include <ctime>
#include <deque>

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
                         _zMax ( 1300.),
                         _r((cv::Mat_<double>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, -1)),
                         _g((cv::Mat_<double>(3, 1) << 0, 0, -1))
{
    _positions = {
        {
            { 0, 0, 0, 0, -90, 0 },
            { 0, 0, 0, 0, 0, 0 },
            { 0, 0, 0, 0, 0, 180 },
            { 0, 0, 0, 0, 90, 180 },
            { 0, 0, 0, 0, 0, 270 },
            { 0, 0, 0, 0, 0, 90 }
        }
    };

    _fgmax = cv::Mat(3, 3, cv::DataType<double>::type);

    _tmax = cv::Mat(3, 3, cv::DataType<double>::type);

    _tmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

    _fgmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

    _tmp = cv::Mat(6, 6, cv::DataType<double>::type);

    _isConnectedFanuc = false;
}

std::array<double, 6> TenzoMath::swapData(const std::array<double, 6> data) const
{
   /* for (int i = 0; i < 6; ++i)
    {
        std::cout << data[i] << '\t';
    }*/
    cv::Mat f = cv::Mat::zeros(1, 3, cv::DataType<double>::type);
    cv::Mat t = cv::Mat::zeros(1, 3, cv::DataType<double>::type);
    std::array<double, 6> tmp;
    f.at<double>(0, 0) = data[0];
    f.at<double>(0, 1) = data[1];
    f.at<double>(0, 2) = data[2];
    t.at<double>(0, 0) = data[3];
    t.at<double>(0, 1) = -data[4];
    t.at<double>(0, 2) = data[5];
    t *= _r;
    f *= _r;
    tmp[0] = f.at<double>(0, 0);
    tmp[1] = f.at<double>(0, 1);
    tmp[2] = f.at<double>(0, 2);
    tmp[3] = t.at<double>(0, 0);
    tmp[4] = t.at<double>(0, 1);
    tmp[5] = t.at<double>(0, 2);
    return tmp;
}

void TenzoMath::doCalibration()
{
    if (!_isConnectedFanuc)
    {
        _fanuc.startWorking();
        _fanuc.setJointFrame();
        _isConnectedFanuc = true;
    }
    Tenzo tenzoData(_T("COM6"));

    cv::Mat f = cv::Mat::zeros(1, 3, cv::DataType<double>::type);
    cv::Mat t = cv::Mat::zeros(1, 3, cv::DataType<double>::type);
    double Rtmp[9] = { 0, -1, 0, 1, 0, 0, 0, 0, -1 };
    cv::Mat R(3, 3, cv::DataType<double>::type, Rtmp);

    for (int i = 0; i < 6; ++i)
    {
        //Robot.goTo and wait
        _fanuc.goToCoordinates(_positions[i][0], _positions[i][1], _positions[i][2],
                               _positions[i][3],
                               _positions[i][4],
                               _positions[i][5]);

        getchar();

        //readData and put it in arrays
        std::array<double, 6> tmp = swapData(tenzoData.readData());
        
        _tmp.at<double>(i, 0) = tmp[0];
        _tmp.at<double>(i, 1) = tmp[1];
        _tmp.at<double>(i, 2) = tmp[2];
        _tmp.at<double>(i, 3) = tmp[3];
        _tmp.at<double>(i, 4) = tmp[4];
        _tmp.at<double>(i, 5) = tmp[5];
    }

    _fanuc.goToCoordinates(0, 0, 0, 0, -90, 0);

    std::ofstream tmp("tmp.txt");
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            tmp << _tmp.at<double>(i, j) << ' ';
    tmp.close();

    _forcesBias[0] = (_tmp.at<double>(1, 0) + _tmp.at<double>(2, 0)) / 2.0;
    _forcesBias[1] = (_tmp.at<double>(4, 1) + _tmp.at<double>(5, 1)) / 2.0;
    _forcesBias[2] = (_tmp.at<double>(0, 2) + _tmp.at<double>(3, 2)) / 2.0;

    _torquesBias[0] = (_tmp.at<double>(1, 3) + _tmp.at<double>(2, 3)) / 2.0;
    _torquesBias[1] = (_tmp.at<double>(4, 4) + _tmp.at<double>(5, 4)) / 2.0;
    _torquesBias[2] = (_tmp.at<double>(0, 5) + _tmp.at<double>(3, 5)) / 2.0;

    _fgmax.at<double>(0, 0) = _tmp.at<double>(2, 0) - _forcesBias[0];
    _fgmax.at<double>(0, 1) = _tmp.at<double>(2, 1) - _forcesBias[1];
    _fgmax.at<double>(0, 2) = _tmp.at<double>(2, 2) - _forcesBias[2];

    _fgmax.at<double>(1, 0) = _tmp.at<double>(4, 0) - _forcesBias[0];
    _fgmax.at<double>(1, 1) = _tmp.at<double>(4, 1) - _forcesBias[1];
    _fgmax.at<double>(1, 2) = _tmp.at<double>(4, 2) - _forcesBias[2];

    _fgmax.at<double>(2, 0) = _tmp.at<double>(0, 0) - _forcesBias[0];
    _fgmax.at<double>(2, 1) = _tmp.at<double>(0, 1) - _forcesBias[1];
    _fgmax.at<double>(2, 2) = _tmp.at<double>(0, 2) - _forcesBias[2];

    _tmax.at<double>(0, 0) = _tmp.at<double>(2, 3) - _torquesBias[0];
    _tmax.at<double>(0, 1) = _tmp.at<double>(2, 4) - _torquesBias[1];
    _tmax.at<double>(0, 2) = _tmp.at<double>(2, 5) - _torquesBias[2];

    _tmax.at<double>(1, 0) = _tmp.at<double>(4, 3) - _torquesBias[0];
    _tmax.at<double>(1, 1) = _tmp.at<double>(4, 4) - _torquesBias[1];
    _tmax.at<double>(1, 2) = _tmp.at<double>(4, 5) - _torquesBias[2];

    _tmax.at<double>(2, 0) = _tmp.at<double>(0, 3) - _torquesBias[0];
    _tmax.at<double>(2, 1) = _tmp.at<double>(0, 4) - _torquesBias[1];
    _tmax.at<double>(2, 2) = _tmp.at<double>(0, 5) - _torquesBias[2];

    _fgmaxNeg.at<double>(0, 0) = _tmp.at<double>(1, 0) - _forcesBias[0];
    _fgmaxNeg.at<double>(0, 1) = _tmp.at<double>(1, 1) - _forcesBias[1];
    _fgmaxNeg.at<double>(0, 2) = _tmp.at<double>(1, 2) - _forcesBias[2];

    _fgmaxNeg.at<double>(1, 0) = _tmp.at<double>(5, 0) - _forcesBias[0];
    _fgmaxNeg.at<double>(1, 1) = _tmp.at<double>(5, 1) - _forcesBias[1];
    _fgmaxNeg.at<double>(1, 2) = _tmp.at<double>(5, 2) - _forcesBias[2];

    _fgmaxNeg.at<double>(2, 0) = _tmp.at<double>(3, 0) - _forcesBias[0];
    _fgmaxNeg.at<double>(2, 1) = _tmp.at<double>(3, 1) - _forcesBias[1];
    _fgmaxNeg.at<double>(2, 2) = _tmp.at<double>(3, 2) - _forcesBias[2];

    _tmaxNeg.at<double>(0, 0) = _tmp.at<double>(1, 3) - _torquesBias[0];
    _tmaxNeg.at<double>(0, 1) = _tmp.at<double>(1, 4) - _torquesBias[1];
    _tmaxNeg.at<double>(0, 2) = _tmp.at<double>(1, 5) - _torquesBias[2];

    _tmaxNeg.at<double>(1, 0) = _tmp.at<double>(5, 3) - _torquesBias[0];
    _tmaxNeg.at<double>(1, 1) = _tmp.at<double>(5, 4) - _torquesBias[1];
    _tmaxNeg.at<double>(1, 2) = _tmp.at<double>(5, 5) - _torquesBias[2];

    _tmaxNeg.at<double>(2, 0) = _tmp.at<double>(3, 3) - _torquesBias[0];
    _tmaxNeg.at<double>(2, 1) = _tmp.at<double>(3, 4) - _torquesBias[1];
    _tmaxNeg.at<double>(2, 2) = _tmp.at<double>(3, 5) - _torquesBias[2];

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


//void TenzoMath::ftControlCartesianCoord()
//{
//    std::array<double, 6> worldPos = {
//        985.0, 0.0, 940.0, -180.0, 0.0, 0.0
//    };
//    std::array<double, 6> jointPos = {
//        0.0, 0.0, 0.0, 0.0, -90.0, 0.0
//    };
//    cv::Mat curPos = _model.fanucForwardTask(jointPos);
//    std::cout << "start:\n" << curPos << '\n';
//    worldPos = FanucModel::getCoordsFromMat(curPos);
//    worldPos[3] *= (180.0 / PI);
//    worldPos[4] *= (180.0 / PI);
//    worldPos[5] *= (180.0 / PI);
//    for (int i = 0; i < 6; ++i)
//    {
//        std::cout << worldPos[i] << '\t';
//    }
//    std::cout << "\n\n";
//    std::cin.get();
//    //Tenzo tenzoData(_T("COM6"));
//    cv::Mat forces(1, 3, cv::DataType<double>::type);
//    cv::Mat torques(1, 3, cv::DataType<double>::type);
//    cv::Mat currRot(3, 3, cv::DataType<double>::type);
//    constexpr double coefForces = 0.0025;
//    constexpr double coefTorques = 0.0005;
//    constexpr double threshold = 150;
//    forces.at<double>(0, 0) = forces.at<double>(0, 1) = forces.at<double>(0, 2) = 5;
//    torques.at<double>(0, 0) = torques.at<double>(0, 1) = torques.at<double>(0, 2) = 1;
//   // _fanuc.startWorking();
//   // _fanuc.setWorldFrame();
//    // std::ofstream delta("delta.txt");
//    // std::ofstream readings("delta3.txt");
//    // std::ofstream out("readingsOut1.txt");
//    // std::chrono::time_point<std::chrono::system_clock> start, end;
//    // start = std::chrono::system_clock::now();
//    // int deltaTime = 0;
//    /*cv::Mat f = cv::Mat::zeros(1, 3, cv::DataType<double>::type);
//    cv::Mat t = cv::Mat::zeros(1, 3, cv::DataType<double>::type);
//    double Rtmp[9] = { 0, -1, 0, 1, 0, 0, 0, 0, -1 };
//    cv::Mat R(3, 3, cv::DataType<double>::type, Rtmp);
//    std::cout << R << '\n';*/
//    while (true)
//    {
//       //std::array<double, 6> tmp = swapData(tenzoData.readData());
//
//        currRot = _model.rotMatrix(worldPos[3] / 180.0 * PI, worldPos[4] / 180.0 * PI,
//            worldPos[5] / 180.0 * PI);
//        std::array<double, 6> newData = gravCompensation(currRot, tmp);
//
//        forces.at<double>(0, 0) = (abs(newData[0]) < threshold ? 0 : newData[0] * coefForces);
//        forces.at<double>(0, 1) = (abs(newData[1]) < threshold ? 0 : newData[1] * coefForces);
//        forces.at<double>(0, 2) = (abs(newData[2]) < threshold ? 0 : newData[2] * coefForces);
//        torques.at<double>(0, 0) = (abs(newData[3]) < threshold ? 0 : newData[3] * coefTorques);
//        torques.at<double>(0, 1) = (abs(newData[4]) < threshold ? 0 : newData[4] * coefTorques);
//        torques.at<double>(0, 2) = (abs(newData[5]) < threshold ? 0 : newData[5] * coefTorques * 2.5);
//        */
//        //forces *= currRot.t();
//        //torques *= currRot.t();
//
//        double deltaLength = sqrt(forces.at<double>(0, 0) * forces.at<double>(0, 0) + forces.at<double>(0, 1) * forces.at<double>(0, 1)
//            + forces.at<double>(0, 2) * forces.at<double>(0, 2));
//        // std::cout << deltaLength << '\n';
//        if (deltaLength > 20.0)
//        {
//            forces.at<double>(0, 0) = forces.at<double>(0, 0) / deltaLength * 20.0;
//            forces.at<double>(0, 1) = forces.at<double>(0, 0) / deltaLength * 20.0;
//            forces.at<double>(0, 2) = forces.at<double>(0, 0) / deltaLength * 20.0;
//        }
//        cv::Mat deltaPos = FanucModel::transMatrix(forces.at<double>(0, 0), forces.at<double>(0, 1), forces.at<double>(0, 2),
//            torques.at<double>(0, 0), torques.at<double>(0, 1), torques.at<double>(0, 2));
//        auto deltaCoord = FanucModel::getCoordsFromMat(deltaPos);
//        deltaCoord[3] *= (180.0 / PI);
//        deltaCoord[4] *= (180.0 / PI);
//        deltaCoord[5] *= (180.0 / PI);
//        for (int i = 0; i < 6; ++i)
//        {
//            std::cout << deltaCoord[i] << '\t';
//        }
//        std::cout << std::endl;
//        curPos =  curPos * deltaPos;
//        std::cout << deltaPos << '\n' << curPos << '\n';
//        worldPos = FanucModel::getCoordsFromMat(curPos);
//        worldPos[3] *= (180.0 / PI);
//        worldPos[4] *= (180.0 / PI);
//        worldPos[5] *= (180.0 / PI);
//        /* worldPos[0] += forces.at<double>(0, 0);
//        worldPos[1] += forces.at<double>(0, 1);
//        worldPos[2] += forces.at<double>(0, 2);
//        worldPos[3] += torques.at<double>(0, 0);
//        worldPos[4] += torques.at<double>(0, 1);
//        worldPos[5] += torques.at<double>(0, 2);*/
//
//
//        //std::cout << deltaLength << '\n';
//        /*for (int i = 3; i < 6; ++i)
//        {
//        if (worldPos[i] > 180.0f)
//        worldPos[i] -= 360.0f;
//        if (worldPos[i] < -180.0f)
//        worldPos[i] += 360.0f;
//        }*/
//        for (int i = 0; i < 6; ++i)
//        {
//            std::cout << worldPos[i] << '\t';
//        }
//        std::cout << std::endl;
//        std::cin.get();
//        // end = std::chrono::system_clock::now();
//
//        // unsigned long elapsed_seconds1 = std::chrono::duration_cast<std::chrono::microseconds>
//        //     (end - start1).count();
//
//        // double velocity = deltaLength / elapsed_seconds1;
//        //// std::cout << deltaLength << ' ' << velocity << '\n';
//        // unsigned long elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>
//        //     (end - start).count();
//
//
//        //  std::cout << elapsed_seconds << ' ' << forces.at<double>(0, 0) << ' ' << forces.at<double>(0, 1) << ' ' << forces.at<double>(0, 2) << ' ' <<
//        //     torques.at<double>(0, 0) << ' ' << torques.at<double>(0, 1) << ' ' << torques.at<double>(0, 2) << '\n';
//        // out << elapsed_seconds << ' ' << forces.at<double>(0, 0) << ' ' << forces.at<double>(0, 1) << ' ' << forces.at<double>(0, 2) << ' ' <<
//        //      torques.at<double>(0, 0) << ' ' << torques.at<double>(0, 1) << ' ' << torques.at<double>(0, 2) << '\n';
//        // std::time_t end_time = std::chrono::system_clock::to_time_t(end);
//        /* std::cout << elapsed_seconds << ' ' << velocity << '\n';
//        out << elapsed_seconds << ' ' << velocity << '\n';*/
//
//       // _fanuc.goToCoordinates(worldPos[0], worldPos[1], worldPos[2], worldPos[3], worldPos[4],
//       //     worldPos[5]);
//
//       // _fanuc.getJointAngles();
//    }
//}


void TenzoMath::ftControlCartesianCoord()
{
    std::array<double, 6> worldPos = {
        985.0, 0.0, 940.0, -180.0, 0.0, 0.0
    };
    std::array<double, 6> jointPos = {
        0.0, 0.0, 0.0, 0.0, -90.0, 0.0
    };
    cv::Mat curPos = _model.fanucForwardTask(jointPos);
   
    worldPos = FanucModel::getCoordsFromMat(curPos);
    worldPos[3] *= (180.0 / PI);
    worldPos[4] *= (180.0 / PI);
    worldPos[5] *= (180.0 / PI);

    Tenzo tenzoData(_T("COM6"));
    cv::Mat forces(1, 3, cv::DataType<double>::type);
    cv::Mat torques(1, 3, cv::DataType<double>::type);
    cv::Mat currRot(3, 3, cv::DataType<double>::type);
    constexpr double coefForces = 0.0025;
    constexpr double coefTorques = 0.0005;
    constexpr double threshold = 250;

    //double cos;
    //double coef;

    cv::Mat prevF = cv::Mat::zeros(1, 3, cv::DataType<double>::type);

    _fanuc.startWorking();
    _fanuc.setWorldFrame();
   
    std::deque<double> coefsF = {0.1, 0.1, 0.1};
    std::array<double, 4> weightsF = { 1.0, 1.0, 1.0, 1.0};
    while (true)
    {
        std::array<double, 6> tmp = swapData(tenzoData.readData());
       
        currRot = _model.rotMatrix(worldPos[3] / 180.0 * PI, worldPos[4] / 180.0 * PI,
            worldPos[5] / 180.0 * PI);

        std::array<double, 6> newData = gravCompensation(currRot, tmp);
       
        forces.at<double>(0, 0) = (abs(newData[0]) < threshold ? 0 : newData[0] * coefForces);
        forces.at<double>(0, 1) = (abs(newData[1]) < threshold ? 0 : newData[1] * coefForces);
        forces.at<double>(0, 2) = (abs(newData[2]) < threshold ? 0 : newData[2] * coefForces);
        torques.at<double>(0, 0) = (abs(newData[3]) < threshold ? 0 : newData[3] * coefTorques);
        torques.at<double>(0, 1) = (abs(newData[4]) < threshold ? 0 : newData[4] * coefTorques);
        torques.at<double>(0, 2) = (abs(newData[5]) < threshold ? 0 : newData[5] * coefTorques * 2.5);

        //if (cv::countNonZero(forces) == 0)
        //{         
        //    coefsF.pop_front();
        //    coefsF.push_back(0.1);         
        //}
        //else if(cv::countNonZero(prevF) == 0)
        //{ 
        //    coefsF.pop_front();
        //    coefsF.push_back(0.1);          
        //} 
        //else
        //{
        //    cv::Mat forcesTmp = forces * currRot.t();
        //    cv::Mat prevFTmp = prevF * currRot.t();
        //    //std::cout << "cur :\n" << forcesTmp << "\nprev:\n" << prevTmp << "\n";
        //    
        //    double cos = prevFTmp.dot(forcesTmp) / (cv::norm(prevFTmp) * cv::norm(forcesTmp));
        //    double cosa2 = sqrt(0.5 + 0.5 * cos) + 1e-6;

        //    if (cosa2 < 0.2)
        //    {
        //        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
        //        for (auto it = coefsF.begin(); it != coefsF.end(); ++it)
        //        {
        //            std::cout << *it << ' ';                  
        //        }
        //        std::cout << '\n';
        //        coefsF = { 0.1, 0.1, 0.1 };
        //        for (auto it = coefsF.begin(); it != coefsF.end(); ++it)
        //        {
        //            std::cout << *it << ' ';
        //        }
        //        std::cout << '\n';
        //    }
        //    
        //    double znamen = 0;
        //    std::cout << "n: " << coefsF.size() << '\n';
        //    int i = 0;
        //    for (auto it = coefsF.begin(); it != coefsF.end(); ++it, ++i)
        //    {
        //        std::cout << *it << ' ';
        //        znamen += weightsF[i] / *it;    
        //    }
        //    std::cout << '\n';
        //    znamen += weightsF[3] / cosa2;

        //    coefsF.pop_front();
        //    coefsF.push_back(4.0 / znamen);
        //    
        //    if (isnan(coefsF.back()))
        //    {
        //        std::cout << "broken\n\a" << cv::norm(prevFTmp) << ' ' << cv::norm(forcesTmp) << '\n'
        //             << '\n' << cosa2 << '\n' << znamen << '\n';
        //        assert(true);
        //        std::cin.get();
        //    }
        //}

        cv::Mat forcesTmp = forces * currRot.t();
        cv::Mat prevFTmp = prevF * currRot.t();
        double cos = prevFTmp.dot(forcesTmp) / (cv::norm(prevFTmp) * cv::norm(forcesTmp));

        std::cout << "cos = " << cos << '\n' << forcesTmp << '\n';
       // std::cout << coefsF.back() << "\n";

        forces *= coefsF.back();
        torques *= coefsF.back();

        prevF = forces.clone();

        cv::Mat deltaPos = FanucModel::transMatrix(forces.at<double>(0, 0), forces.at<double>(0, 1), forces.at<double>(0, 2),
            torques.at<double>(0, 0), torques.at<double>(0, 1), torques.at<double>(0, 2));

        curPos = curPos * deltaPos;
        worldPos = FanucModel::getCoordsFromMat(curPos);
        worldPos[3] *= (180.0 / PI);
        worldPos[4] *= (180.0 / PI);
        worldPos[5] *= (180.0 / PI);

        _fanuc.goToCoordinates(worldPos[0], worldPos[1], worldPos[2], worldPos[3], worldPos[4], worldPos[5]);
        
       _fanuc.getJointAngles();
    }
}

void TenzoMath::newJointsControl()
{
    std::array<double, 6> worldPos = { 985.0, 0.0, 1040.0, -180.0, 0.0, 0.0 };
    std::array<double, 6> jointPos = { 0.0, 0.0, 0.0, 0.0, -90.0, 0.0 };
    Tenzo tenzoData(_T("COM6"));
    cv::Mat forces(1, 3, cv::DataType<double>::type);
    cv::Mat torques(1, 3, cv::DataType<double>::type);
    cv::Mat currRot(3, 3, cv::DataType<double>::type);
    constexpr double coefForces = 0.005;
    constexpr double coefTorques = 0.001;
    constexpr double threshold = 150;
    cv::Mat p6 = _model.fanucForwardTask(jointPos);
    _fanuc.startWorking();
    _fanuc.setJointFrame();
    while (true)
    {   
        std::array<double, 6> tmp = swapData(tenzoData.readData());
        
        std::array<double, 6> newData = gravCompensation(p6, tmp);
        forces.at<double>(0, 0) = (abs(newData[0]) < threshold ? 0 : newData[0] * coefForces);
        forces.at<double>(0, 1) = (abs(newData[1]) < threshold ? 0 : newData[1] * coefForces);
        forces.at<double>(0, 2) = (abs(newData[2]) < threshold ? 0 : newData[2] * coefForces);
        torques.at<double>(0, 0) = (abs(newData[3]) < threshold ? 0 : newData[3] * coefTorques);
        torques.at<double>(0, 1) = (abs(newData[4]) < threshold ? 0 : newData[4] * coefTorques);
        torques.at<double>(0, 2) = (abs(newData[5]) < threshold ? 0 : newData[5] * coefTorques * 5);
       // std::cout << "Data: " << forces.at<double>(0, 0) << '\t' << forces.at<double>(0, 1) << '\t' << forces.at<double>(0, 2) << '\t' <<
        //    torques.at<double>(0, 0) << '\t' << torques.at<double>(0, 1) << '\t' << torques.at<double>(0, 2) << '\n';

        /*if (abs(forces.at<double>(0, 0)) > 0.0 || abs(forces.at<double>(0, 1)) > 0.0 || abs(forces.at<double>(0, 2)) > 0.0 ||
            abs(torques.at<double>(0, 0)) > 0.0 || abs(torques.at<double>(0, 1)) > 0.0 || abs(torques.at<double>(0, 2)) > 0.0)*/ // != 0
        if (forces.at<double>(0, 0) != 0.0 || forces.at<double>(0, 1) != 0.0 || forces.at<double>(0, 2) != 0.0 ||
            torques.at<double>(0, 0) != 0.0 || torques.at<double>(0, 1) != 0.0 || torques.at<double>(0, 2) != 0.0) 
        {
            //double t0 = torques.at<double>(0, 0);
            //double t1 = torques.at<double>(0, 1);
            //torques.at<double>(0, 0) = t0 * cos(worldPos[5] / 180. * PI) - t1 * sin(worldPos[5] / 180. * PI);
            //torques.at<double>(0, 1) = -t0 * sin(worldPos[5] / 180. * PI) + t1 * cos(worldPos[5] / 180. * PI);
            //torques.at<double>(0, 2) = (sin(worldPos[5] / 180. * PI) > 0 ? torques.at<double>(0, 2) : -torques.at<double>(0, 2));

            p6(cv::Rect(0, 0, 3, 3)).copyTo(currRot);
            forces *= currRot.t();
            torques *= currRot.t(); 

            worldPos[0] += forces.at<double>(0, 0);
            worldPos[1] += forces.at<double>(0, 1);
            worldPos[2] += forces.at<double>(0, 2);
            worldPos[3] += torques.at<double>(0, 0);
            worldPos[4] -= torques.at<double>(0, 1);
            worldPos[5] -= torques.at<double>(0, 2);

            for (int i = 3; i < 6; ++i)
            {
                if (worldPos[i] > 180.0)
                    worldPos[i] -= 360.0;
                if (worldPos[i] < -180.0)
                    worldPos[i] += 360.0;
            }

            for (int i = 0; i < 6; ++i)
            {
                std::cout << worldPos[i] << '\t';
            }
            std::cout << std::endl;

            std::cout << _model.fanucInverseTask(worldPos) << std::endl;

            jointPos = chooseNearestPose(_model.fanucInverseTask(worldPos), jointPos);
            p6 = _model.fanucForwardTask(jointPos);
           /* for (int i = 0; i < 6; ++i)
            {
                std::cout << jointPos[i] << '\t';
            }
            std::cout << std::endl;*/

            _fanuc.goToCoordinates(jointPos[0], jointPos[1], jointPos[2], jointPos[3], jointPos[4], jointPos[5]);
            _fanuc.getJointAngles(); //????падает
        }
       // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::array<double, 6> TenzoMath::chooseNearestPose(cv::Mat res, std::array<double, 6> prevPos)
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
            if (abs(min) < 50.f) // обосновать или убрать
            {
                std::cout << "\nchosen\n";
                return std::array<double, 6>{res.at<double>(num, 0), res.at<double>(num, 1), res.at<double>(num, 2),  res.at<double>(num, 3), res.at<double>(num, 4), res.at<double>(num, 5) };
            }
        }
        return prevPos;
    }
    return prevPos;
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
//        std::array<double, 6> tmp = swapData(tenzoData.readData());
//  
//        std::array<double, 6> newData = gravCompensation(p6, tmp);
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
//        torques *= rot.t();
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
//    cv::Mat tmp(8, 6, CV_64F);
//    for (int i = 0; i < 8; ++i)
//    {
//        //Robot.goTo and wait
//        _fanuc.goToCoordinates(pos[i][0], pos[i][1], pos[i][2], pos[i][3], pos[i][4], pos[i][5]);
//
//        getchar();
//
//        //readData and put it in arrays
//        std::array<double, 6> tmp = tenzoData.readData();
//        tmp.at<double>(i, 0) = tmp[1];
//        tmp.at<double>(i, 1) = -tmp[0];
//        tmp.at<double>(i, 2) = -tmp[2];
//        tmp.at<double>(i, 3) = -tmp[4];
//        tmp.at<double>(i, 4) = tmp[3];
//        tmp.at<double>(i, 5) = tmp[5];
//        std::cout << i << std::endl;
//    }
//    std::cout << "collected";
//    std::ofstream out("collectedTestData.txt");
//    for (int i = 0; i < 8; ++i)
//        for (int j = 0; j < 6; ++j)
//            out << tmp.at<double>(i, j) << ' ';
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
//    std::ifstream in("tmp.txt");
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
/*void TenzoMath::ftControlCartesianCoord()
{
    std::array<double, 6> worldPos = {
        985.0, 0.0, 940.0, -180.0, 0.0, 0.0
    };
    Tenzo tenzoData(_T("COM6"));
    cv::Mat forces(1, 3, cv::DataType<double>::type);
    cv::Mat torques(1, 3, cv::DataType<double>::type);
    cv::Mat currRot(3, 3, cv::DataType<double>::type);
    constexpr double coefForces = 0.005;
    constexpr double coefTorques = 0.001;
    constexpr double threshold = 150;
    _fanuc.startWorking();
    _fanuc.setWorldFrame();
    std::chrono::time_point<std::chrono::steady_clock> start;
    while (true)
    {
        
       /* start = std::chrono::system_clock::now();

std::array<double, 6> tmp = swapData(tenzoData.readData());

/* end = std::chrono::system_clock::now();
int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
std::time_t end_time = std::chrono::system_clock::to_time_t(end);

std::cout << "read time: " << elapsed_seconds << "ms\n";

start = std::chrono::system_clock::now();

currRot = _model.rotMatrix(worldPos[3] / 180.0 * PI, worldPos[4] / 180.0 * PI,
    worldPos[5] / 180.0 * PI);

std::array<double, 6> newData = gravCompensation(currRot, tmp);
forces.at<double>(0, 0) = (abs(newData[0]) < threshold ? 0 : newData[0] * coefForces);
forces.at<double>(0, 1) = (abs(newData[1]) < threshold ? 0 : newData[1] * coefForces);
forces.at<double>(0, 2) = (abs(newData[2]) < threshold ? 0 : newData[2] * coefForces);
torques.at<double>(0, 0) = (abs(newData[3]) < threshold ? 0 : newData[3] * coefTorques);
torques.at<double>(0, 1) = (abs(newData[4]) < threshold ? 0 : newData[4] * coefTorques);
torques.at<double>(0, 2) = (abs(newData[5]) < threshold ? 0 : newData[5] * coefTorques * 5);

forces *= currRot.t();
torques *= currRot.t();

/* std::cout << forces.at<double>(0, 0) << '\t' << forces.at<double>(0, 1) << '\t' << forces.at<double>(0, 2) << '\t'
<< torques.at<double>(0, 0) << '\t' << torques.at<double>(0, 1) << '\t' << torques.at<double>(0, 2) << std::endl;

worldPos[0] += forces.at<double>(0, 0);
worldPos[1] += forces.at<double>(0, 1);
worldPos[2] += forces.at<double>(0, 2);
worldPos[3] += torques.at<double>(0, 0);
worldPos[4] -= torques.at<double>(0, 1);
worldPos[5] -= torques.at<double>(0, 2);

/* for (int i = 3; i < 6; ++i)
{
if (worldPos[i] > 180.0f)
worldPos[i] -= 360.0f;
if (worldPos[i] < -180.0f)
worldPos[i] += 360.0f;
}*/
/*for (int i = 0; i < 6; ++i)
{
std::cout << worldPos[i] << '\t';
}
std::cout << std::endl;*/
/* end = std::chrono::system_clock::now();
elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
end_time = std::chrono::system_clock::to_time_t(end);
std::cout << "calc time: " << elapsed_seconds << "ms\n";

start = std::chrono::steady_clock::now();
_fanuc.goToCoordinates(worldPos[0], worldPos[1], worldPos[2], worldPos[3], worldPos[4],
    worldPos[5]);

std::array<double, 6> rec = _fanuc.getJointAngles();
//_fanuc.getJointAngles();
std::chrono::duration<double, std::ratio<1, 1000>> end = std::chrono::steady_clock::now() - start;
//int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//std::time_t end_time = std::chrono::steady_clock::to_time_t(end);
std::cout << "work time: " << end.count() << "ms\n";// << std::ctime(&end_time);

                                                    /*std::cout << "send: " << worldPos[0] << '\t' << worldPos[1] << '\t' << worldPos[2] << '\t'
                                                    << worldPos[3] << '\t' << worldPos[4] << '\t' << worldPos[5] << std::endl;

                                                    std::cout << "rec: " << rec[0] << '\t' << rec[1] << '\t' << rec[2] << '\t'
                                                    << rec[3] << '\t' << rec[4] << '\t' << rec[5] << std::endl;*/

/*  end = std::chrono::system_clock::now();
                                                    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>
                                                    (end - start).count();
                                                    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
                                                    std::cout << "time: " << elapsed_seconds << "ms\n";
                                                    // std::this_thread::sleep_for(std::chrono::milliseconds(55));
    }
}
*/