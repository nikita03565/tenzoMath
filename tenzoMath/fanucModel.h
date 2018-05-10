#ifndef FANUC_MODEL
#define FANUC_MODEL

#include "RoboModel.h"


namespace nikita
{
/**
* \brief Class for matematics of robot manipulator Fanuc M20ia based on Denavit-Hartenberg parameters.
*/
class FanucModel : RoboModel
{
    /**
    * \brief Function for conversion joints angles to Denavit-Hartenberg generalized angles.
    * \param[in] j Joints angles.
    * \return Denavit-Hartenberg generalized angles.
    */
    static std::vector<double> jointsToQ(const std::array<double, 6>& j);

    /**
     * \brief Rotation matrix from i-th frame to (i + 1)-th frame.
     * \param[in] alpha One of D-H parameters.
     * \param[in] q Generalized D-H angle.
     * \return Rotation matrix 3x3.
     */
    cv::Mat qi(const double& alpha, const double& q) const;
public:

    /**
     * \brief Constant Pi.
     */
    static constexpr double PI = 3.14159265;

    /**
    * \brief Default constructor with Fanuc M20ia parameters.
    */
    FanucModel();

    ~FanucModel() = default;

    /**
    * \brief Function for solving forward kinematic task for Fanuc M20ia.
    * \param[in] inputjoints Joints angles.
    * \return Coordinates of end-effector in world frame: x, y, z in mm and w, p, r in radians.
    */
    cv::Mat fanucForwardTask(const std::array<double, 6>& inputjoints);

    /**
     * \brief Solves inverse kinematic task.
     * \param[in] coord 6 Cartesian coordinates x, y, z, w, p, r.
     * \return Matrix of solutions.
     */
    cv::Mat fanucInverseTask(const std::array<double, 6>& coord) const;

    /**
    * \brief Solves inverse kinematic task.
    * \param[in] coord 6 Cartesian coordinates x, y, z, w, p, r.
    * \return Matrix of solutions.
    */
    cv::Mat fanucInverseTaskNew(const std::array<double, 6>& coord) const;

    /**
    * \brief Calculates rotation matrix of end-effector. input angles given in radians.
    * \param[in] w Angle of rotation around x axis.
    * \param[in] p Angle of rotation around y axis.
    * \param[in] r Angle of rotation around z axis.
    * \return Rotation matrix 3*3.
    */
    static cv::Mat rotMatrix(const double& w, const double& p, const double& r);
};

} // nampespace nikita

#endif // FANUC_MODEL
