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
    FanucModel _model;

    /**
    * \brief collected forces and torques measurements
    */
    cv::Mat _collectedData;

    /**
     * \brief calculates rotation matrix of end-effector. input angles given in radians
     * \param[in] w angle of rotation around x axis
     * \param[in] p angle of rotation around y axis
     * \param[in] r angle of rotation around z axis
     * \return rotation matrix 3*3
     */
    cv::Mat TenzoMath::rotMatrix(const double& w, const double& p, const double& r) const;

    std::array<double, 3> chooseNearestPose(cv::Mat res, std::array<double, 3> prevPos);

    cv::Mat qi(const double& alpha, const double& q) const;

    const double _xMin;
    const double _yMin;
    const double _zMin;
    const double _xMax;
    const double _yMax; 
    const double _zMax;
public:

    cv::Mat inverseTask(std::array<double, 6> coord) const;

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

    void ftControlJoints();

    void collectTestData();

    void doTest();
};

#endif //_TENZO_MATH_
