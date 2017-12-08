#include "tenzoMath.h"
#include <fstream>

void printCvMat(cv::Mat t, const char str[] = "some matrix")
{
	std::cout << str << ":\n";
	for (int i = 0; i < t.rows; ++i)
	{
		for (int j = 0; j < t.cols; ++j)
			if (abs(t.at<double>(i, j)) < 0.001) std::cout << 0 << ' ';
			else std::cout << t.at<double>(i, j) << ' ';
			std::cout << std::endl;
	}
	std::cout << "-------------------------" << std::endl;
}

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

	_fgmax = cv::Mat(3, 3, cv::DataType<double>::type);

	_tmax = cv::Mat(3, 3, cv::DataType<double>::type);

	_tmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

	_fgmaxNeg = cv::Mat(3, 3, cv::DataType<double>::type);

	_collectedData = cv::Mat(6, 6, cv::DataType<double>::type);
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

	//	getchar();
	//	getchar();

	//	//readData and put it in arrays
	//	std::array<double, 6> tmp = tenzoData.readData();
	//	_collectedData.at<double>(i, 0) = tmp[1];
	//	_collectedData.at<double>(i, 1) = -tmp[0];
	//	_collectedData.at<double>(i, 2) = -tmp[2];
	//	_collectedData.at<double>(i, 3) = tmp[4];
	//	_collectedData.at<double>(i, 4) = -tmp[3];
	//	_collectedData.at<double>(i, 5) = tmp[5];
	//}
	//std::ofstream collectedData("collectedData.txt");
	//for (int i = 0; i < 6; ++i)
	//	for (int j = 0; j < 6; ++j)
	//		collectedData << _collectedData.at<double>(i, j) << ' ';
	//collectedData.close();
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

	_forcesBias[0] = (_collectedData.at<double>(1, 0) + _collectedData.at<double>(2, 0)) / 2.0;
	_forcesBias[1] = (_collectedData.at<double>(4, 1) + _collectedData.at<double>(5, 1)) / 2.0;
	_forcesBias[2] = (_collectedData.at<double>(0, 2) + _collectedData.at<double>(3, 2)) / 2.0;
	
	//std::cout << "\n_forcesBias: " << _forcesBias[0] << ' ' << _forcesBias[1] << ' ' << _forcesBias[2] << std::endl;

	_torquesBias[0] = (_collectedData.at<double>(1, 3) + _collectedData.at<double>(2, 3)) / 2.0;
	_torquesBias[1] = (_collectedData.at<double>(4, 4) + _collectedData.at<double>(5, 4)) / 2.0;
	_torquesBias[2] = (_collectedData.at<double>(0, 5) + _collectedData.at<double>(3, 5)) / 2.0;

	std::cout << "\n_torquesBias: " << _torquesBias[0] << ' ' << _torquesBias[1] << ' ' << _torquesBias[2] << std::endl;

	_fgmax.at<double>(0, 0) = _collectedData.at<double>(2, 0) - _forcesBias[0];
	_fgmax.at<double>(0, 1) = _collectedData.at<double>(2, 1) - _forcesBias[1];
	_fgmax.at<double>(0, 2) = _collectedData.at<double>(2, 2) - _forcesBias[2];

	_fgmax.at<double>(1, 0) = _collectedData.at<double>(4, 0) - _forcesBias[0];
	_fgmax.at<double>(1, 1) = _collectedData.at<double>(4, 1) - _forcesBias[1];
	_fgmax.at<double>(1, 2) = _collectedData.at<double>(4, 2) - _forcesBias[2];

	_fgmax.at<double>(2, 0) = _collectedData.at<double>(0, 0) - _forcesBias[0];
	_fgmax.at<double>(2, 1) = _collectedData.at<double>(0, 1) - _forcesBias[1];
	_fgmax.at<double>(2, 2) = _collectedData.at<double>(0, 2) - _forcesBias[2];
	//std::cout << "fgmax:\n" << _fgmax << std::endl;

	_tmax.at<double>(0, 0) = _collectedData.at<double>(2, 3) - _torquesBias[0];
	_tmax.at<double>(0, 1) = _collectedData.at<double>(2, 4) - _torquesBias[1];
	_tmax.at<double>(0, 2) = _collectedData.at<double>(2, 5) - _torquesBias[2];

	_tmax.at<double>(1, 0) = _collectedData.at<double>(4, 3) - _torquesBias[0];
	_tmax.at<double>(1, 1) = _collectedData.at<double>(4, 4) - _torquesBias[1];
	_tmax.at<double>(1, 2) = _collectedData.at<double>(4, 5) - _torquesBias[2];

	_tmax.at<double>(2, 0) = _collectedData.at<double>(0, 3) - _torquesBias[0];
	_tmax.at<double>(2, 1) = _collectedData.at<double>(0, 4) - _torquesBias[1];
	_tmax.at<double>(2, 2) = _collectedData.at<double>(0, 5) - _torquesBias[2];
	//std::cout << "tmax:\n" << _tmax << std::endl;
	printCvMat(_tmax, "_tmax");

	_fgmaxNeg.at<double>(0, 0) = _collectedData.at<double>(1, 0) - _forcesBias[0];
	_fgmaxNeg.at<double>(0, 1) = _collectedData.at<double>(1, 1) - _forcesBias[1];
	_fgmaxNeg.at<double>(0, 2) = _collectedData.at<double>(1, 2) - _forcesBias[2];

	_fgmaxNeg.at<double>(1, 0) = _collectedData.at<double>(5, 0) - _forcesBias[0];
	_fgmaxNeg.at<double>(1, 1) = _collectedData.at<double>(5, 1) - _forcesBias[1];
	_fgmaxNeg.at<double>(1, 2) = _collectedData.at<double>(5, 2) - _forcesBias[2];

	_fgmaxNeg.at<double>(2, 0) = _collectedData.at<double>(3, 0) - _forcesBias[0];
	_fgmaxNeg.at<double>(2, 1) = _collectedData.at<double>(3, 1) - _forcesBias[1];
	_fgmaxNeg.at<double>(2, 2) = _collectedData.at<double>(3, 2) - _forcesBias[2];
	//std::cout << "fgmaxNeg:\n" << _fgmaxNeg << std::endl;
	
	_tmaxNeg.at<double>(0, 0) = _collectedData.at<double>(1, 3) - _torquesBias[0];
	_tmaxNeg.at<double>(0, 1) = _collectedData.at<double>(1, 4) - _torquesBias[1];
	_tmaxNeg.at<double>(0, 2) = _collectedData.at<double>(1, 5) - _torquesBias[2];

	_tmaxNeg.at<double>(1, 0) = _collectedData.at<double>(5, 3) - _torquesBias[0];
	_tmaxNeg.at<double>(1, 1) = _collectedData.at<double>(5, 4) - _torquesBias[1];
	_tmaxNeg.at<double>(1, 2) = _collectedData.at<double>(5, 5) - _torquesBias[2];

	_tmaxNeg.at<double>(2, 0) = _collectedData.at<double>(3, 3) - _torquesBias[0];
	_tmaxNeg.at<double>(2, 1) = _collectedData.at<double>(3, 4) - _torquesBias[1];
	_tmaxNeg.at<double>(2, 2) = _collectedData.at<double>(3, 5) - _torquesBias[2];
	//std::cout << "tmaxNeg:\n" << _tmaxNeg << std::endl;
	printCvMat(_tmaxNeg, "_tmaxNeg");

	std::ofstream out("calibData.txt");

	out << _forcesBias[0] << ' ' << _forcesBias[1] << ' ' << _forcesBias[2] << ' '<< _torquesBias[0] << ' ' << _torquesBias[1] << ' ' << _torquesBias[2] << std::endl;
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
	//----------------------------------------------------
	cv::Mat currRot(3, 3, CV_64F);
	cv::Mat _fgmaxCurr(3, 3, CV_64F);
	cv::Mat _tmaxCurr(3, 3, CV_64F);
	std::array<double, 6> rawData;
	std::array<double, 6> newData;
	for (int i = 0; i < 6; ++i)
	{
		_model.fanucForwardTask(_positions[i])(cv::Rect(0, 0, 3, 3)).copyTo(currRot);

		cv::Mat gravProjection = currRot.t() * _g;

		//std::cout << "gravProjection:\n" << gravProjection << std::endl;
		printCvMat(gravProjection, "gravProjection");
		for (int j = 0; j < 6; ++j)
		{
			rawData[j] = _collectedData.at<double>(i, j);
		}

		if (gravProjection.at<double>(0, 0) > 0)
		{
			_fgmaxCurr.at<double>(0, 0) = _fgmax.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 1) = _fgmax.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 2) = _fgmax.at<double>(0, 2) * gravProjection.at<double>(0, 0);

			_tmaxCurr.at<double>(0, 0) = _tmax.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 1) = _tmax.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 2) = _tmax.at<double>(0, 2) * gravProjection.at<double>(0, 0);
		}
		else
		{
			_fgmaxCurr.at<double>(0, 0) = -_fgmaxNeg.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 1) = -_fgmaxNeg.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 2) = -_fgmaxNeg.at<double>(0, 2) * gravProjection.at<double>(0, 0);

			_tmaxCurr.at<double>(0, 0) = -_tmaxNeg.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 1) = -_tmaxNeg.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 2) = -_tmaxNeg.at<double>(0, 2) * gravProjection.at<double>(0, 0);
		}

		if (gravProjection.at<double>(1, 0) > 0)
		{
			_fgmaxCurr.at<double>(1, 0) = _fgmax.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 1) = _fgmax.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 2) = _fgmax.at<double>(1, 2) * gravProjection.at<double>(1, 0);

			_tmaxCurr.at<double>(1, 0) = _tmax.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 1) = _tmax.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 2) = _tmax.at<double>(1, 2) * gravProjection.at<double>(1, 0);
		}
		else
		{
			_fgmaxCurr.at<double>(1, 0) = -_fgmaxNeg.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 1) = -_fgmaxNeg.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 2) = -_fgmaxNeg.at<double>(1, 2) * gravProjection.at<double>(1, 0);

			_tmaxCurr.at<double>(1, 0) = -_tmaxNeg.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 1) = -_tmaxNeg.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 2) = -_tmaxNeg.at<double>(1, 2) * gravProjection.at<double>(1, 0);
		}

		if (gravProjection.at<double>(2, 0) > 0)
		{
			_fgmaxCurr.at<double>(2, 0) = _fgmax.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 1) = _fgmax.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 2) = _fgmax.at<double>(2, 2) * gravProjection.at<double>(2, 0);

			_tmaxCurr.at<double>(2, 0) = _tmax.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 1) = _tmax.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 2) = _tmax.at<double>(2, 2) * gravProjection.at<double>(2, 0);
		}
		else
		{
			_fgmaxCurr.at<double>(2, 0) = -_fgmaxNeg.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 1) = -_fgmaxNeg.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 2) = -_fgmaxNeg.at<double>(2, 2) * gravProjection.at<double>(2, 0);

			_tmaxCurr.at<double>(2, 0) = -_tmaxNeg.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 1) = -_tmaxNeg.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 2) = -_tmaxNeg.at<double>(2, 2) * gravProjection.at<double>(2, 0);
		}
		//std::cout << "_fgmaxCurr:\n" << _fgmaxCurr << std::endl;
		printCvMat(_fgmaxCurr, "_fgmaxCurr");
		std::cout << "RawData: " << rawData[0] << ' ' << rawData[1] << ' ' << rawData[2] << ' ' << rawData[3] << ' ' << rawData[4] << ' ' << rawData[5] << std::endl;

		newData[0] = rawData[0] - _forcesBias[0] -_fgmaxCurr.at<double>(0, 0) - _fgmaxCurr.at<double>(1, 0) - _fgmaxCurr.at<double>(2, 0);
		newData[1] = rawData[1] - _forcesBias[1] -_fgmaxCurr.at<double>(0, 1) - _fgmaxCurr.at<double>(1, 1) - _fgmaxCurr.at<double>(2, 1);
		newData[2] = rawData[2] - _forcesBias[2] -_fgmaxCurr.at<double>(0, 2) - _fgmaxCurr.at<double>(1, 2) - _fgmaxCurr.at<double>(2, 2);
		newData[3] = rawData[3] - _torquesBias[0] - _tmaxCurr.at<double>(0, 0) - _tmaxCurr.at<double>(1, 0) - _tmaxCurr.at<double>(2, 0);
		newData[4] = rawData[4] - _torquesBias[1] - _tmaxCurr.at<double>(0, 1) - _tmaxCurr.at<double>(1, 1) - _tmaxCurr.at<double>(2, 1);
		newData[5] = rawData[5] - _torquesBias[2] - _tmaxCurr.at<double>(0, 2) - _tmaxCurr.at<double>(1, 2) - _tmaxCurr.at<double>(2, 2);
		
		std::cout << "New data: ";
		for (int r = 0; r < 6; ++r)
		{
			std::cout << (newData[r] < 0.01 ? 0 : newData[r]) << '\t';
		}
		std::cout << std::endl;
		//std::cout << "New data: " << newData[0] << '\t' << newData[1] << '\t' << newData[2] << '\t' << newData[3] << '\t' << newData[4] << '\t' << newData[5] << std::endl << std::endl;
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

void TenzoMath::loadCalibData()
{
	std::ifstream input("calibData.txt");
	if (!input) std::cout << "file is not found\n";
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

void TenzoMath::check()
{
	cv::Mat currRot(3, 3, CV_64F);
	cv::Mat _fgmaxCurr(3, 3, CV_64F);
	cv::Mat _tmaxCurr(3, 3, CV_64F);
	std::array<double, 6> rawData = {63, -1186, -5294, -922, -1530, 145};
	std::array<double, 6> newData;
	std::array<double, 6> _pos = { 0, 0, 0, -30, -40, 120 };
	
		_model.fanucForwardTask(_pos)(cv::Rect(0, 0, 3, 3)).copyTo(currRot);
		
		cv::Mat gravProjection = currRot.t() * _g;

		//std::cout << "gravProjection:\n" << gravProjection << std::endl;
		printCvMat(gravProjection, "gravProjection");
		/*for (int j = 0; j < 6; ++j)
		{
			rawData[j] = _collectedData.at<double>(i, j);
		}*/
		if (gravProjection.at<double>(0, 0) > 0)
		{
			_fgmaxCurr.at<double>(0, 0) = _fgmax.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 1) = _fgmax.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 2) = _fgmax.at<double>(0, 2) * gravProjection.at<double>(0, 0);

			_tmaxCurr.at<double>(0, 0) = _tmax.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 1) = _tmax.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 2) = _tmax.at<double>(0, 2) * gravProjection.at<double>(0, 0);
		}
		else
		{
			_fgmaxCurr.at<double>(0, 0) = -_fgmaxNeg.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 1) = -_fgmaxNeg.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_fgmaxCurr.at<double>(0, 2) = -_fgmaxNeg.at<double>(0, 2) * gravProjection.at<double>(0, 0);

			_tmaxCurr.at<double>(0, 0) = -_tmaxNeg.at<double>(0, 0) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 1) = -_tmaxNeg.at<double>(0, 1) * gravProjection.at<double>(0, 0);
			_tmaxCurr.at<double>(0, 2) = -_tmaxNeg.at<double>(0, 2) * gravProjection.at<double>(0, 0);
		}

		if (gravProjection.at<double>(1, 0) > 0)
		{
			_fgmaxCurr.at<double>(1, 0) = _fgmax.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 1) = _fgmax.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 2) = _fgmax.at<double>(1, 2) * gravProjection.at<double>(1, 0);

			_tmaxCurr.at<double>(1, 0) = _tmax.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 1) = _tmax.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 2) = _tmax.at<double>(1, 2) * gravProjection.at<double>(1, 0);
		}
		else
		{
			_fgmaxCurr.at<double>(1, 0) = -_fgmaxNeg.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 1) = -_fgmaxNeg.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_fgmaxCurr.at<double>(1, 2) = -_fgmaxNeg.at<double>(1, 2) * gravProjection.at<double>(1, 0);

			_tmaxCurr.at<double>(1, 0) = -_tmaxNeg.at<double>(1, 0) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 1) = -_tmaxNeg.at<double>(1, 1) * gravProjection.at<double>(1, 0);
			_tmaxCurr.at<double>(1, 2) = -_tmaxNeg.at<double>(1, 2) * gravProjection.at<double>(1, 0);
		}

		if (gravProjection.at<double>(2, 0) > 0)
		{
			_fgmaxCurr.at<double>(2, 0) = _fgmax.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 1) = _fgmax.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 2) = _fgmax.at<double>(2, 2) * gravProjection.at<double>(2, 0);

			_tmaxCurr.at<double>(2, 0) = _tmax.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 1) = _tmax.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 2) = _tmax.at<double>(2, 2) * gravProjection.at<double>(2, 0);
		}
		else
		{
			_fgmaxCurr.at<double>(2, 0) = -_fgmaxNeg.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 1) = -_fgmaxNeg.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_fgmaxCurr.at<double>(2, 2) = -_fgmaxNeg.at<double>(2, 2) * gravProjection.at<double>(2, 0);

			_tmaxCurr.at<double>(2, 0) = -_tmaxNeg.at<double>(2, 0) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 1) = -_tmaxNeg.at<double>(2, 1) * gravProjection.at<double>(2, 0);
			_tmaxCurr.at<double>(2, 2) = -_tmaxNeg.at<double>(2, 2) * gravProjection.at<double>(2, 0);
		}
		//std::cout << "_fgmaxCurr:\n" << _fgmaxCurr << std::endl;
		printCvMat(_fgmaxCurr, "_fgmaxCurr");
		std::cout << "RawData: " << rawData[0] << ' ' << rawData[1] << ' ' << rawData[2] << ' ' << rawData[3] << ' ' << rawData[4] << ' ' << rawData[5] << std::endl;

		newData[0] = rawData[0] - _forcesBias[0] - _fgmaxCurr.at<double>(0, 0) - _fgmaxCurr.at<double>(1, 0) - _fgmaxCurr.at<double>(2, 0);
		newData[1] = rawData[1] - _forcesBias[1] - _fgmaxCurr.at<double>(0, 1) - _fgmaxCurr.at<double>(1, 1) - _fgmaxCurr.at<double>(2, 1);
		newData[2] = rawData[2] - _forcesBias[2] - _fgmaxCurr.at<double>(0, 2) - _fgmaxCurr.at<double>(1, 2) - _fgmaxCurr.at<double>(2, 2);
		newData[3] = rawData[3] - _torquesBias[0] - _tmaxCurr.at<double>(0, 0) - _tmaxCurr.at<double>(1, 0) - _tmaxCurr.at<double>(2, 0);
		newData[4] = rawData[4] - _torquesBias[1] - _tmaxCurr.at<double>(0, 1) - _tmaxCurr.at<double>(1, 1) - _tmaxCurr.at<double>(2, 1);
		newData[5] = rawData[5] - _torquesBias[2] - _tmaxCurr.at<double>(0, 2) - _tmaxCurr.at<double>(1, 2) - _tmaxCurr.at<double>(2, 2);

		std::cout << "New data: ";
		for (int r = 0; r < 6; ++r)
		{
			std::cout << (newData[r] < 0.01 ? 0 : newData[r]) << '\t';
		}
		std::cout << std::endl;
}
