#ifndef ROBOMODEL_H
#define ROBOMODEL_H

#include <array>
#include <vector>

#include <opencv2/core.hpp>

namespace nikita
{
/**
* \brief class for matematics of robot manipulator based on Denavit-Hartenberg parameters
*/
class RoboModel
{
public:
    /**
    * \brief struct of Denavit-Hartenberg parameters
    */
    struct DhParameters
    {
        /**
         * \brief offset along previous z to the common normal
         */
        double _dParam;

        /**
        * \brief angle about previous  z, from old  x to new  x
        */
        double _qParam;

        /**
        * \brief offset along x in current frame
        */
        double _aParam;

        /**
        * \brief angle about x in current frame
        */
        double _alphaParam;

        /**
         * \brief constructor with parameters
         * \param[in] d D-H parameter
         * \param[in] q D-H parameter
         * \param[in] a D-H parameter
         * \param[in] alpha D-H parameter
         */
        DhParameters(const double d, const double q, const double a, const double alpha);
    };

protected:
    /**
    * \brief vector of parameters for each joint
    */
    std::vector<DhParameters> _kinematicChain;

    /**
    * \brief function to calculate a transform matrix from i-th frame to (i-1)-th
    * \param[in] i number of frame
    * \return transform matrix (4x4)
    */
    cv::Mat prevMatTransform(const std::size_t i);

    /**
    * \brief constructor with parameters for any robot
    * \param[in] input vector of secuences d, q, a, alpha
    */
    explicit RoboModel(std::vector<std::array<double, 4>> input);

    ~RoboModel() = default;

    /**
    * \brief function for solving forward kinematic task
    * \param[in] inputq generalized D-H coordinates
    * \return coordinates of end-effector in world frame: x, y, z in mm and w, p, r in radians
    */
    cv::Mat forwardTask(std::vector<double> inputq);
};
} //namespace nikita
#endif // ROBOMODEL_H
