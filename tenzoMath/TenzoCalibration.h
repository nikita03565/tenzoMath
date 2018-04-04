#ifndef TENZO_CALIBRATION
#define TENZO_CALIBRATION

#include <array>
#include <opencv2/core/mat.hpp>


namespace nikita
{

/**
* \brief Class for calculating calibration.
* Contains data collected from sensor, calibration algorythm, calibration parameters: bias and matrices.
*/
class TenzoCalibration
{
protected:
    TenzoCalibration();

    /**
    * \brief Collected forces and torques measurements.
    */
    cv::Mat _collectedData;

    /**
    * \brief Bias of forces.
    */
    std::array<double, 3> _forcesBias{};
    /**
    * \brief Bias of torques.
    */
    std::array<double, 3> _torquesBias{};

    /**
    * \brief Weigth of end-effector on every axis in positive direction.
    */
    cv::Mat _fgmax;

    /**
    * \brief Torque of end-effector on every axis in positive direction.
    */
    cv::Mat _tmax;

    /**
    * \brief Torque of end-effector on every axis in negative direction.
    */
    cv::Mat _tmaxNeg;

    /**
    * \brief Weigth of end-effector on every axis in negative direction.
    */
    cv::Mat _fgmaxNeg;

    /**
    * \brief 6 different positions of end-effector in generalized coordinates.
    */
    std::array<std::array<double, 6>, 6> _positions;

    /**
    * \brief Local gravity acceleration.
    */
    cv::Mat _g;

public:  
    
    /**
     * \brief calculate calubration using collected data.
     */
    void calculateCalibration();
};

} //namespace nikita

#endif // TENZO_CALIBRATION


