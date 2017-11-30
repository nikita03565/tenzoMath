#include "fanucModel.h"
#include <iostream>
#include "tenzoMath.h"

void printCvMat(cv::Mat t)
{
	for (int i = 0; i < t.rows; ++i)
	{
		for (int j = 0; j < t.cols; ++j)
			if (abs(t.at<double>(i, j)) < 0.001) std::cout << 0 << ' ';
			else std::cout << t.at<double>(i, j) << ' ';
			std::cout << std::endl;
	}
	std::cout << "-------------------------" << std::endl;
}

int main()
{	
	TenzoMath math;
	math.biasAndFgmaxEstimation();
	/*Tenzo tenzo(L"COM6");
	while(true)
	{
		std::array<double, 6> tmp = tenzo.readData();
		std::cout << tmp[1] << '\t' << -tmp[0] << '\t' << -tmp[2] + 5492 << std::endl;
	}*/

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