#include "tenzoMath.h"

TenzoMath::TenzoMath()
{
	_positions = {
		{
			{0, 0, 0, 0, -90, 0},{0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 180},{0, 0, 0, 0, 90, 180},
			{0, 0, 0, 0, 0, 270},{0, 0, 0, 0, 0, 90}
		}
	};

	_g = cv::Mat(3, 1, cv::DataType<double>::type);
	_g.at<double>(0, 0) = 0.;
	_g.at<double>(1, 0) = 0.;
	_g.at<double>(2, 0) = -1;

	_bias = cv::Mat(3, 3, cv::DataType<double>::type);

	__collectedData = cv::Mat(6, 6, cv::DataType<double>::type);
}

void TenzoMath::biasAndFgmaxEstimation()
{
	//_fanuc.startWorking();
	//_fanuc.setJointFrame();
	//Tenzo tenzoData(L"COM6");

	//for (int i = 0; i < 6; ++i)
	//{
	//	//Robot.goTo and wait
	//	_fanuc.goToCoordinates(_positions[i][0], _positions[i][1], _positions[i][2], _positions[i][3], _positions[i][4],
	//	                       _positions[i][5]);

	//	_fanuc.getJointAngles();

	//	//readData and put it in arrays
	//	std::array<double, 6> tmp = tenzoData.readData();
	//	_collectedData.at<double>(i, 0) = tmp[1];
	//	_collectedData.at<double>(i, 1) = -tmp[0];
	//	_collectedData.at<double>(i, 2) = -tmp[2];
	//	_collectedData.at<double>(i, 3) = tmp[4];
	//	_collectedData.at<double>(i, 4) = -tmp[3];
	//	_collectedData.at<double>(i, 5) = tmp[5];
	//}

	std::array<std::array<double, 6>, 6> tmpdata =
	{ {
		{101, -170, -4980, -61, -1198, -36},
		{-1053, -131, -5403, -88, -1749, -54},
		{1309, -229, -5565, -118, -486, -73},
		{137, -166, -6005, -122, -1087, -41},
		{53, 842, -5439, 623, -1117, -113},
		{81, -1122, -5499, -803, -1066, -22}
	} };
	double tmptmpdata[6][6] = { 
		{ 101, -170, -4980, -61, -1198, -36 },
		{ -1053, -131, -5403, -88, -1749, -54 },
		{ 1309, -229, -5565, -118, -486, -73 },
		{ 137, -166, -6005, -122, -1087, -41 },
		{ 53, 842, -5439, 623, -1117, -113 },
		{ 81, -1122, -5499, -803, -1066, -22 }
		 };
	cv::Mat _collectedData(6, 6, CV_64F, tmptmpdata);
	std::cout << _collectedData;
	_bias.at<double>(0, 0) = (_collectedData.at<double>(1, 0) + _collectedData.at<double>(2, 0)) / 2.0;
	_bias.at<double>(0, 1) = (_collectedData.at<double>(1, 1) + _collectedData.at<double>(2, 1)) / 2.0;
	_bias.at<double>(0, 2) = (_collectedData.at<double>(1, 2) + _collectedData.at<double>(2, 2)) / 2.0;

	_bias.at<double>(1, 0) = (_collectedData.at<double>(4, 0) + _collectedData.at<double>(5, 0)) / 2.0;
	_bias.at<double>(1, 1) = (_collectedData.at<double>(4, 1) + _collectedData.at<double>(5, 1)) / 2.0;
	_bias.at<double>(1, 2) = (_collectedData.at<double>(4, 2) + _collectedData.at<double>(5, 2)) / 2.0;

	_bias.at<double>(2, 0) = (_collectedData.at<double>(0, 0) + _collectedData.at<double>(3, 0)) / 2.0;
	_bias.at<double>(2, 1) = (_collectedData.at<double>(0, 1) + _collectedData.at<double>(3, 1)) / 2.0;
	_bias.at<double>(2, 2) = (_collectedData.at<double>(0, 2) + _collectedData.at<double>(3, 2)) / 2.0;

	std::cout << "\nBias:\n " << _bias << std::endl;

	//_collectedData(cv::Rect(0, 0, 3, 3)).copyTo(currRot);

	_fgmax[0] = _collectedData.at<double>(2, 0) - _bias.at<double>(0, 0);
	_fgmax[1] = _collectedData.at<double>(2, 0) - _bias.at<double>(1, 1);
	_fgmax[2] = _collectedData.at<double>(2, 0) - _bias.at<double>(2, 2);
	//std::cout << "\nfgmax: " << _fgmax << std::endl;

	//----------------------------------------------------
	cv::Mat currRot(3, 3, CV_64F);
	std::array<double, 3> rawData;
	std::array<double, 3> newData;
	for (int i = 0; i < 6; ++i)
	{
		_model.fanucForwardTask(_positions[i])(cv::Rect(0, 0, 3, 3)).copyTo(currRot);

		cv::Mat gravProjection = currRot.t() * _g;

		for (int j = 0; j < 3; ++j)
		{
			rawData[j] = _collectedData.at<double>(i, j);
		}

		/*newData[0] = rawData[0] - gravProjection.at<double>(0, 0) * _fgmax.at<double>(0, 0) - gravProjection.at<double>(1, 0)
			* _fgmax.at<double>(0, 1) - gravProjection.at<double>(2, 0) * _fgmax.at<double>(0, 2);
		newData[1] = rawData[1] - gravProjection.at<double>(1, 0) * _fgmax.at<double>(1, 1) - gravProjection.at<double>(0, 0)
			* _fgmax.at<double>(1, 0) - gravProjection.at<double>(2, 0) * _fgmax.at<double>(1, 2);
		newData[2] = rawData[2] - gravProjection.at<double>(2, 0) * _fgmax.at<double>(2, 2) - gravProjection.at<double>(0, 0)
			* _fgmax.at<double>(2, 0) - gravProjection.at<double>(2, 0) * _fgmax.at<double>(2, 2);
*/
		std::cout << "New data: " << newData[0] << '\t' << newData[1] << '\t' << newData[2] << std::endl;
	}
}

void TenzoMath::gravCompensation()
{
	/**
	* \brief object for connecting with fanuc
	*/
	FanucM20iA _fanuc;
	_fanuc.setJointFrame();
	Tenzo tenzoData(L"COM8");
	std::array<double, 6> currJoints;
	cv::Mat currRot(3, 3, CV_64F);
	std::array<double, 6> rawData;
	std::array<double, 3> newData;
	while (true)
	{
		currJoints = _fanuc.getJointAngles();

		for (int i = 0; i < 6; ++i)
			currJoints[i] /= 1000.0;

		std::cout << "Current joints: " << currJoints[0] << '\t' << currJoints[1] << '\t' << currJoints[2] << '\t' <<
			currJoints[3] << '\t' << currJoints[4] << '\t' << currJoints[5] << std::endl;

		_model.fanucForwardTask(currJoints)(cv::Rect(0, 0, 3, 3)).copyTo(currRot);

		cv::Mat gravProjection = currRot.t() * _g;

		rawData = tenzoData.readData();

	/*	newData[0] = rawData[0] - gravProjection.at<double>(0, 0) * _fgmax.at<double>(0, 0) - gravProjection.at<double>(1, 0)
			* _fgmax.at<double>(0, 1) - gravProjection.at<double>(2, 0) * _fgmax.at<double>(0, 2);
		newData[1] = rawData[1] - gravProjection.at<double>(1, 0) * _fgmax.at<double>(1, 1) - gravProjection.at<double>(0, 0)
			* _fgmax.at<double>(1, 0) - gravProjection.at<double>(2, 0) * _fgmax.at<double>(1, 2);
		newData[2] = rawData[2] - gravProjection.at<double>(2, 0) * _fgmax.at<double>(2, 2) - gravProjection.at<double>(0, 0)
			* _fgmax.at<double>(2, 0) - gravProjection.at<double>(2, 0) * _fgmax.at<double>(2, 2);
*/
		std::cout << "New data: " << newData[0] << '\t' << newData[1] << '\t' << newData[2] << std::endl;
	}
}
