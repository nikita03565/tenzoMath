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
	cv::Mat _bias;

	/**
	 * \brief weigth of end-effector on every axis
	 */
	std::array<double, 3> _fgmax;

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
};

#endif //_TENZO_MATH_
