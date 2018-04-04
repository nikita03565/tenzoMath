#ifndef TENZO_MATH
#define TENZO_MATH

#include <array>

#include "FanucModelExtension.h"
#include "StrainGauge.h"
#include "TenzoCalibration.h"


namespace nikita
{

/**
 * \brief Class for gravity compensation and calculating next position in cartesian coordinates to provide force-torque control.
 */
class TenzoMath : public TenzoCalibration
{
   
protected:
    TenzoMath();
    ~TenzoMath() = default;
    TenzoMath(const TenzoMath&) = default;
    TenzoMath(TenzoMath&&) = default;
    TenzoMath& operator=(const TenzoMath&) = delete;
    TenzoMath& operator=(TenzoMath&&) = delete;

    /**
    * \brief Object for math model of fanuc.
    */
    FanucModelExtension _model;
public:
    /**
    * \brief Calculates gravity compensation matrix for forces.
    */
    std::array<double, 6> gravCompensation(const cv::Mat& p6, std::array<int, 6>& rawData);

    /**
    * \brief Calculates new position in cartesian coords and stores it in string.
    * \param[in] curPos Current position in cartesian coords in int * 1000.
    */
    void calculateNextPos(std::array<int, 6>& curPos, std::array<int, 6>& readings);
};

} //namespace nikita

#endif // TENZO_MATH
