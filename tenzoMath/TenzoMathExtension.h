#ifndef TENZO_MATH_EXTENSION
#define TENZO_MATH_EXTENSION

#include "fanucModelExtension.h"
#include "StrainGauge.h"
#include "tenzoMath.h"

namespace nikita
{

/**
* \brief class for communication with Vasily's client
*/
class TenzoMathExtension : public TenzoMath
{
    /**
    * \brief object for reading from force-torque sensor
    */
    StrainGauge _tenzoData;

    /**
    * \brief string of cartesian coords
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
    * \brief convertes double array into int array * 1000
    */
    static std::array<int, 6> convertToInt(const std::array<double, 6>& coord);

    /**
    * \brief convertes int array * 1000.0 into double array
    */
    static std::array<double, 6> convertToDouble(const std::array<int, 6>& coord);

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
    * \brief do one force-torque measurement and store data in matrix
    * \param[in] index number of your measurement
    */
    void collectData(const std::size_t index);

    /**
    * \brief get position for calibration
    * \param[in] index number of position
    * \return coordinates in int * 1000
    */
    std::array<int, 6> getPosition(const std::size_t index) const;

    /**
    * \brief forward kinematic task
    * \param[in] joints joints angles
    * \return coordinates in int * 1000
    */
    std::array<double, 6> jointsToWorld(const std::array<double, 6>& joints);

    /**
    * \brief converts array to of cartesian coords to string
    * \param[in] coord input array
    * \return string ready to sending
    */
    std::string toString(const std::array<double, 6>& coord) const;

    /**
     * \brief calculates new position based on force-torque measurements
     * \param[in] curPos current posiotion in cartesian coordianes
     */
    void calculatePos(std::array<int, 6>& curPos);

    /**
    * \brief swaps x, y, z axes so sensor frame is the same as end-effector frame
    * \param[in] data raw readings from sensor
    * \return new data
    */
    static std::array<double, 6> swapData(const std::array<double, 6>& data);
};

} //namespace nikita

#endif //TENZO_MATH_EXTENSION


