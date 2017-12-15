#include "fanucModel.h"
#include <iostream>
#include "tenzoMath.h"
#include <algorithm>




int main()
{
    TenzoMath math;
   // FanucModel model;
   // std::cout << model.fanucForwardTask({ 0, 0, 0, 0, -90, 0 }) << std::endl;
    std::cout << math.inverseTask({ 985, 0, 1040, -PI, 0, 0 });

    //math.doCalibration();
   // math.loadCalibData();
   // math.ftControlJoints();
    //	math.ftControlCartesianCoord();
    //math.collectTestData();
    //math.doTest();

    //math.loadCalibData();
    /*	Tenzo tenzo(L"COM6");
        while(true)
        {
            std::array<double, 6> _collectedData = tenzo.readData();
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
    getchar();
    return 0;
}
