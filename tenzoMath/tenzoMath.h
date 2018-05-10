#ifndef _TENZO_MATH_
#define _TENZO_MATH_

#include <array>
#include <iostream>
#include "fanucModelExtension.h"
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

    /**
    * \brief torque of end-effector on every axis in positive direction
    */
    cv::Mat _tmax;

    /**
    * \brief torque of end-effector on every axis in negative direction
    */
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
    * \brief object for connecting with fanuc
    */
    FanucM20iA _fanuc;

    /**
     * \brief is this app connected to robot
     */
    bool _isConnectedFanuc;

    /**
     * \brief object for math model of fanuc
     */
    nikita::FanucModelExtension _model;

    /**
    * \brief collected forces and torques measurements
    */
    cv::Mat _collectedData;

    cv::Mat _tmp;

    cv::Mat _r;

    std::array<double, 6> swapData(const std::array<double, 6> data) const;

    const double _xMin;
    const double _yMin;
    const double _zMin;
    const double _xMax;
    const double _yMax; 
    const double _zMax;
public:
    TenzoMath();

    ~TenzoMath() = default;

    /**
     * \brief estimates bias for forces and max weight of end-effector on every axis
     */
    void doCalibration();

    /**
     * \brief calculates gravity compensation matrix for forces
     */
    std::array<double, 6> gravCompensation(cv::Mat currRot, std::array<double, 6> rawData);

    /**
     * \brief loads calibration data from file
     */
    void loadCalibData();

    /**
     * \brief provides force-torque control for robot
     */
    void ftControlCartesianCoord();

    void newJointsControl();

    static std::array<double, 6> chooseNearestPose(cv::Mat res, std::array<double, 6> prevPos);
};

#endif //_TENZO_MATH_
