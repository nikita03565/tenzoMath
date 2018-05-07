#include "fanucModel.h"
#include <iostream>
#include "tenzoMath.h"
#include <algorithm>
#include <fstream>
#include <tchar.h>
#include <chrono>
#include "Fanuc.h"
#include <thread>

int main()
{  
    TenzoMath math;
    std::cout << "1 - calibration, 2 - control: ";
    int i;
    std::cin >> i;
    if (i == 1)
    {
        math.doCalibration();
    } else if (i == 2)
    {
        math.loadCalibData();
        math.ftControlCartesianCoord();
    }
    /*std::chrono::time_point<std::chrono::system_clock> start, end;
    Tenzo tenzo(_T("COM6"));
    while (true)
    {
        start = std::chrono::system_clock::now();
        std::array<double, 6> _collectedData = tenzo.readData();
        end = std::chrono::system_clock::now();
        unsigned long elapsed_seconds = std::chrono::duration_cast<std::chrono::microseconds>
            (end - start).count();
        std::cout << elapsed_seconds << '\n';
    }*/
    //Tenzo tenzo(_T("COM6"));
    //constexpr int Ndata = 15;
    //std::array<std::array<double, Ndata>, 6> collectedData;
    //std::array<double, 6> filteredData;
    //while (true)
    //{
    //    for (int i = 0; i < Ndata; ++i)
    //    {
    //        std::array<double, 6> tmp = tenzo.readData();
    //        for (int j = 0; j < 6; ++j)
    //        {
    //            collectedData[j][i] = tmp[j];
    //        }
    //    }
    //    for (int i = 0; i < 6; ++i)
    //    {
    //        std::nth_element(collectedData[i].begin(), collectedData[i].begin() + collectedData[i].size() / 2, collectedData[i].end());
    //        filteredData[i] = collectedData[i][collectedData[i].size() / 2];
    //    }
    //   
    //    std::cout << filteredData[0] << '\t' << filteredData[1] << '\t' << filteredData[2] << '\t' << filteredData[3] << '\t' << -filteredData[4] << '\t' << filteredData[5] << '\n';
    //  

    //   /* std::array<double, 6> tmp = tenzo.readData();
    //    std::cout << tmp[0] << '\t' << tmp[1] << '\t' << tmp[2] << '\t' << tmp[3] << '\t' << -tmp[4] << '\t' << tmp[5] << '\n';*/


    //    /* for (int i = 0; i < 6; ++i)
    //        std::cout << filteredData[i] << '\t';
    //    std::cout << std::endl;*/
    //    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //}
    getchar();
    return 0;
}




/*std::array<double, 6> worldPos = {
985.0, 0.0, 940.0, -180.0, 0.0, 0.0
};
std::chrono::time_point<std::chrono::steady_clock> start;
std::chrono::duration<double, std::ratio<1, 1000>> end;
std::array<double, 6> rec;
FanucM20iA _fanuc;
_fanuc.startWorking();
_fanuc.setWorldFrame();
while (true)
{
start = std::chrono::steady_clock::now();

_fanuc.goToCoordinates(worldPos[0], worldPos[1], worldPos[2], worldPos[3], worldPos[4],
worldPos[5]);

rec = _fanuc.getJointAngles();
end = std::chrono::steady_clock::now() - start;
std::cout << "work time: " << end.count() << "ms\n";
}*/

//math.loadCalibData();
//math.newJointsControl();
//	math.ftControlCartesianCoord();
//math.collectTestData();
//math.doTest();

//math.loadCalibData();
/* Tenzo tenzo(_T("COM6"));
while(true)
{
std::array<double, 6> tmp = tenzo.readData();
std::array<double, 6> _collectedData;
_collectedData[0] = tmp[1];
_collectedData[1] = -tmp[0];
_collectedData[2] = -tmp[2];
_collectedData[3] = tmp[4];
_collectedData[4] = -tmp[3];
_collectedData[5] = tmp[5];
for (int i = 0; i < 6; ++i)
std::cout << _collectedData[i] << '\t';
std::cout << std::endl;
}*/
/*
//	FanucM20iA _fanuc;
/*FanucModel _model;
//_fanuc.setJointFrame();
Tenzo tenzoData(L"COM6");
cv::Mat currRot(3, 3, CV_64F);

std::array<double, 6> tmp;
std::array<double, 3> newData;
cv::Mat _g (3, 1, CV_64F);
_g.at<double>(0, 0) = 0.;
_g.at<double>(1, 0) = 0.;
_g.at<double>(2, 0) = -1;

_model.fanucForwardTask({ 0,0,0,0,-90,0 })(cv::Rect(0, 0, 3, 3)).copyTo(currRot);

cv::Mat gravProjection = currRot.t() * _g;

while (true) {

tmp = tenzoData.readData();
std::array<double, 3> rawData{ tmp[1] - 128, -tmp[0] + 140, -tmp[2] + 5492 };
newData[0] = rawData[0] - gravProjection.at<double>(0, 0) * 1181;
newData[1] = rawData[1] - gravProjection.at<double>(1, 0) * 982;
newData[2] = rawData[2] - gravProjection.at<double>(2, 0) * 512.5;

std::cout << "New data: " << newData[0] << '\t' << newData[1] << '\t' << newData[2] << std::endl;
}*/