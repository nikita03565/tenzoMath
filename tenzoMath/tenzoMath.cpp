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



TenzoMath::TenzoMath() : _g((cv::Mat_<double>(3, 1) << 0, 0, -1)),
                         _r((cv::Mat_<double>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, -1)),
                         _xMin (680.),
                         _yMin (-465.),
                         _zMin ( 520.),
                         _xMax ( 1380.),
                         _yMax ( 465.),
                         _zMax ( 1300.)
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
    constexpr double coefForces = 0.005;
    constexpr double coefTorques = 0.001;
    constexpr double threshold = 250;

    

    double sumDeltaF = 0.0;
    double sumDeltaT = 0.0;

    double defaultValue = 0.01;

    double coefF = defaultValue;
    double coefT = defaultValue;

    cv::Mat prevF = cv::Mat::zeros(1, 3, cv::DataType<double>::type);
    cv::Mat prevT = cv::Mat::zeros(1, 3, cv::DataType<double>::type);

    _fanuc.startWorking();
    _fanuc.setWorldFrame();
    std::chrono::time_point<std::chrono::system_clock> start;
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
     
        ///////////////////////////////////////////////////////////////////////////////////////
        if (cv::countNonZero(forces) == 0 || cv::countNonZero(prevF) == 0)
        {
            coefF = defaultValue;
            sumDeltaF = 0.0;
        } else
        {
            cv::Mat forcesTmp = forces * currRot.t();
            cv::Mat prevFTmp = prevF * currRot.t();
            //std::cout << "cur :\n" << forcesTmp << "\nprev:\n" << prevTmp << "\n";

            double cos = prevFTmp.dot(forcesTmp) / (cv::norm(prevFTmp) * cv::norm(forcesTmp));
            double cosa2 = sqrt(0.5 + 0.5 * cos);


            if (cosa2 < 0.7)
            {
                std::cout << "forces!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
            
                coefF = 0.001;
                sumDeltaF = 0.0;
                coefT = 0.001;
                sumDeltaT = 0.0;
            }
            else
            {      
                if (coefF < 1.0)
                {
                    sumDeltaF += cv::norm(forces) * coefF;
                    if (sumDeltaF < 30)
                    {
                        coefF += 0.001;                     
                    }
                    else if (sumDeltaF < 100)
                    {
                        coefF += 0.01;
                    } 
                    else
                    {
                        coefF += 0.04;
                    }                   
                } else
                {
                    coefF = 1.0;
                }
            }
        }
  
        //////////////////////////////////////////////////////////////
        if (cv::countNonZero(torques) == 0 || cv::countNonZero(prevT) == 0)
        {
            coefT = defaultValue;
            sumDeltaT = 0.0;
        }
        else
        {
            cv::Mat torquesTmp = torques * currRot.t();
            cv::Mat prevTTmp = prevT * currRot.t();
            //std::cout << "cur :\n" << torquesTmp << "\nprev:\n" << prevTmp << "\n";

            double cos = prevTTmp.dot(torquesTmp) / (cv::norm(prevTTmp) * cv::norm(torquesTmp));
            double cosa2 = sqrt(0.5 + 0.5 * cos);


            if (cosa2 < 0.7)
            {
                std::cout << "torques!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
                coefT = 0.001;
                sumDeltaT = 0.0;
                coefF = 0.001;
                sumDeltaF = 0.0;
            }
            else
            {
                if (coefT < 1.0)
                {
                    sumDeltaT += cv::norm(torques) * coefT;
                    if (sumDeltaT < 20)
                    {
                        coefT += 0.001;
                    }
                    else if (sumDeltaT < 80)                 
                    {
                        coefT += 0.01;
                    }
                    else
                    {
                        coefT += 0.04;
                    }
                }
                else
                {
                    coefT = 1.0;
                }
            }
        }

       // std::cout << coefF << ' ' << coefT << '\n';

        forces *= coefF;
        torques *= coefT;

        prevF = forces.clone();
        prevT = torques.clone();
        cv::Mat deltaPos = FanucModel::transMatrix(forces.at<double>(0, 0), forces.at<double>(0, 1), forces.at<double>(0, 2),
            torques.at<double>(0, 0), torques.at<double>(0, 1), torques.at<double>(0, 2));

        curPos = curPos * deltaPos;
        worldPos = FanucModel::getCoordsFromMat(curPos);
        worldPos[3] *= (180.0 / PI);
        worldPos[4] *= (180.0 / PI);
        worldPos[5] *= (180.0 / PI);

        
        start = std::chrono::system_clock::now();
        _fanuc.goToCoordinates(worldPos[0], worldPos[1], worldPos[2], worldPos[3], worldPos[4], worldPos[5]);
        
       _fanuc.getJointAngles();
       int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>
           (std::chrono::system_clock::now() - start).count();

       std::cout << coefF << ' ' << coefT << ' ' << elapsed_seconds << " ms\n";
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
            _fanuc.getJointAngles(); //????������
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
            if (abs(min) < 50.f) // ���������� ��� ������
            {
                std::cout << "\nchosen\n";
                return std::array<double, 6>{res.at<double>(num, 0), res.at<double>(num, 1), res.at<double>(num, 2),  res.at<double>(num, 3), res.at<double>(num, 4), res.at<double>(num, 5) };
            }
        }
        return prevPos;
    }
    return prevPos;
}