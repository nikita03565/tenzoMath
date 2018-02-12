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
    std::array<std::array<double, 6>, 6> _posCartesian;

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
     * \brief stored data from force-torque sensor
     */
    cv::Mat _tmp;

    /**
     * \brief string of cartesian coords
     */
    std::string _coordToMove;

    /**
     * \brief swaps x, y, z axes so sensor frame is the same as end-effector frame
     * \param[in] data raw readings from sensor
     * \return new data
     */
    std::array<double, 6> swapData(const std::array<double, 6> data) const;


    /**
     * \brief converts array to of cartesian coords to string
     * \param[in] coord input array
     * \return string ready to sending
     */
    std::string toString(std::array<double, 6> coord) const;

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

    /**
     * \brief return _coordToMove
     * \return string of cartesian coords
     */
    std::string getCoordToMove() const;

    /**
     * \brief calculates new position in cartesian coords and stores it in string
     * \param[in] curPos current position in cartesian coords
     * \param[in] ftReadings force-torque readings from sensor
     */
    void TenzoMath::calculatePos(std::array<double, 6>& curPos, std::array<double, 6> ftReadings);

    std::array<double, 6> jointsToWorld(const std::array<double, 6>& joints);

    void newJointsControl();

    static std::array<double, 6> chooseNearestPose(cv::Mat res, std::array<double, 6> prevPos);
};

#endif //_TENZO_MATH_
