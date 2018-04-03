#ifndef FANUC_MODEL_EXTENSION
#define FANUC_MODEL_EXTENSION

#include "fanucModel.h"

namespace nikita
{
/**
* \brief FanucModel with added transformation matrices and couple of other methods
*/
class FanucModelExtension : public FanucModel
{
    /**
    * \brief transformation matrix from 6-th joint to camera
    */
    const cv::Mat _toCamera;

    /**
    * \brief transformation matrix from camera to 6-th joint
    */
    const cv::Mat _toSixth;

    /**
    * \brief transformation matrix that provides moving to object with camera instead of end-effector
    */
    const cv::Mat _forMovingToCamera;
public:
    FanucModelExtension();

    /**
    * \brief returns _ToCamera transformation matrix
    * \return 4x4 transformation matrix
    */
    cv::Mat getToCamera() const;

    /**
    * \brief returns _ToSixth transformation matrix
    * \return 4x4 transformation matrix
    */
    cv::Mat getToSixth() const;

    /**
    * \brief returns _forMovingToCamera transformation matrix
    * \return 4x4 transformation matrix
    */
    cv::Mat getforMovingToCamera() const;

    /**
    * \brief calculates 6 Cartesian coordinates from transformation matrix
    * \param[in] transformMatrix 4x4 matrix
    * \return 3 coordinates x, y, z and 3 angles w, p, r
    */
    static std::array<double, 6> getCoordsFromMat(cv::Mat transformMatrix);

    /**
    * \brief function to calculate three rotation angles from transformation matrix
    * \param[in] p6 transofrmation matrix(4x4) or rotation matrix(3x3)
    * \return three Tait-Bryan angles
    */
    static std::array<double, 3> anglesFromMat(const cv::Mat p6);
};
} // namespace nikita
#endif // FANUC_MODEL_EXTENSION