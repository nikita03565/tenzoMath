#ifndef TENZO_MATH_EXTENSION
#define TENZO_MATH_EXTENSION

#include "fanucModelExtension.h"
#include "StrainGauge.h"
#include "tenzoMath.h"


namespace nikita
{

/**
* \brief Class for communication with Vasily's client.
*/
class TenzoMathExtension : public TenzoMath
{
    /**
    * \brief Object for reading from force-torque sensor.
    */
    StrainGauge _tenzoData;

    /**
    * \brief String of cartesian coords.
    */
    std::string _coordToMove;

public:
    TenzoMathExtension();
    ~TenzoMathExtension() = default;
    TenzoMathExtension(const TenzoMathExtension&) = default;
    TenzoMathExtension(TenzoMathExtension&&) = default;
    TenzoMathExtension& operator=(const TenzoMathExtension&) = delete;
    TenzoMathExtension& operator=(TenzoMathExtension&&) = delete;

    /**
    * \brief Convertes double array into int array * 1000.
    */
    static std::array<int, 6> convertToInt(const std::array<double, 6>& coord);

    /**
    * \brief Convertes int array * 1000.0 into double array.
    */
    static std::array<double, 6> convertToDouble(const std::array<int, 6>& coord);

    /**
    * \brief Loads calibration data from file.
    */
    void loadCalibData();

    /**
    * \brief Return _coordToMove.
    * \return String of cartesian coords.
    */
    std::string getCoordToMove() const;

    /**
    * \brief Do one force-torque measurement and store data in matrix.
    * \param[in] index Number of your measurement.
    */
    void collectData(const std::size_t index);

    /**
    * \brief Get position for calibration.
    * \param[in] index Number of position.
    * \return Coordinates in int * 1000.
    */
    std::array<int, 6> getPosition(const std::size_t index) const;

    /**
    * \brief Forward kinematic task.
    * \param[in] joints Joints angles.
    * \return Coordinates in int * 1000.
    */
    std::array<double, 6> jointsToWorld(const std::array<double, 6>& joints);

    /**
    * \brief Converts array to of cartesian coords to string.
    * \param[in] coord Input array.
    * \return String ready to sending.
    */
    std::string toString(const std::array<double, 6>& coord) const;

    /**
     * \brief Calculates new position based on force-torque measurements that are read inside of this method.
     * \param[in] curPos Current posiotion in cartesian coordianes.
     */
    void calculatePos(std::array<int, 6>& curPos);

    /**
    * \brief Swaps x, y, z axes so sensor frame is the same as end-effector frame.
    * \param[in] data Raw readings from sensor.
    * \return New data.
    */
    static std::array<double, 6> swapData(const std::array<double, 6>& data);
};

} //namespace nikita

#endif //TENZO_MATH_EXTENSION


