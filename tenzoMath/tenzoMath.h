#ifndef _TENZO_MATH_
#define _TENZO_MATH_

#include <array>
#include "fanucModel.h"
#include "Tenzo.h"

namespace nikita
{
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
    * \brief 6 different positions of end-effector in generalized coordinates
    */
    std::array<std::array<double, 6>, 6> _positions;

    /**
    * \brief 6 different positions of end-effector in cartesian coordinates
    */
    std::array<std::array<int, 6>, 6> _posCartesian;

    /**
    * \brief local gravity acceleration
    */
    cv::Mat _g;

    /**
    * \brief object for math model of fanuc
    */
    FanucModel _model;

    /**
    * \brief object for reading from force-torque sensor
    */
    Tenzo _tenzoData;

    /**
    * \brief collected forces and torques measurements
    */
    cv::Mat _collectedData;

    /**
    * \brief string of cartesian coords
    */
    std::string _coordToMove;

    /**
    * \brief swaps x, y, z axes so sensor frame is the same as end-effector frame
    * \param[in] data raw readings from sensor
    * \return new data
    */
    std::array<double, 6> swapData(const std::array<double, 6>& data) const;


    /**
    * \brief converts array to of cartesian coords to string
    * \param[in] coord input array
    * \return string ready to sending
    */
    std::string toString(const std::array<double, 6>& coord) const;

public:
    TenzoMath();

    ~TenzoMath() = default;

    /**
     * \brief convertes double array into int array * 1000.0
     */
    static std::array<int, 6> convertToInt(const std::array<double, 6>& coord);

    /**
    * \brief convertes int array * 1000.0 into double array
    */
    static std::array<double, 6> convertToDouble(const std::array<int, 6>& coord);

    /**
    * \brief estimates bias for forces and max weight of end-effector on every axis
    */
    void doCalibration(); 

    /**
    * \brief calculates gravity compensation matrix for forces
    */
    std::array<double, 6> gravCompensation(const cv::Mat currRot, std::array<double, 6>& rawData);

    /**
    * \brief loads calibration data from file
    */
    void loadCalibData(); 

    /**
    * \brief return _coordToMove
    * \return string of cartesian coords
    */
    std::string getCoordToMove() const;

    /**
     * \brief 
     * \param[in] index 
     */
    void collectData(const std::size_t index);

    std::array<int, 6> getPosition(const std::size_t index) const;

    /**
    * \brief calculates new position in cartesian coords and stores it in string
    * \param[in] curPos current position in cartesian coords in int * 1000.0
    */
    void calculatePos(std::array<int, 6>& curPos); 

    std::array<double, 6> jointsToWorld(const std::array<double, 6>& joints);
};
} //namespace nikita
#endif //_TENZO_MATH_
