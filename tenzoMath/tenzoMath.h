#ifndef _TENZO_MATH_
#define _TENZO_MATH_

#include <array>
#include <iostream>
#include "fanucModel.h"
#include "Tenzo.h"
#include "Fanuc.h"


class TenzoMath
{	
	/**
	 * \brief bias of forces
	 */
	std::array<double, 3> _forcesBias;
	
	/**
	* \brief bias of torques
	*/
	std::array<double, 3> _torquesBias;

	/**
	 * \brief weigth of end-effector on every axis in positive direction
	 */
	cv::Mat _fgmax;

	cv::Mat _tmax;

	cv::Mat _tmaxNeg;
	/**
	* \brief weigth of end-effector on every axis in negative direction
	*/
	cv::Mat _fgmaxNeg;

	/**
	 * \brief 6 different positions of end-effector
	 */
	std::array<std::array<double, 6>, 6> _positions;

	/**
	 * \brief local gravity acceleration
	 */
	cv::Mat _g;

	/**
	 * \brief collected forces and torques measurements
	 */
	cv::Mat __collectedData;

	/**
	* \brief object for connecting with fanuc
	*/
	FanucM20iA _fanuc;

	/**
	 * \brief object for math model of fanuc
	 */
	FanucModel _model;

	cv::Mat _collectedData;
public:
	TenzoMath();

	~TenzoMath() = default;
	/**
	 * \brief estimates bias for forces and max weight of end-effector on every axis
	 */
	void biasAndFgmaxEstimation();

	/**
	 * \brief calculates gravity compensation matrix for forces
	 */
	void gravCompensation();

	void loadCalibData();

	void check();
};

#endif //_TENZO_MATH_
