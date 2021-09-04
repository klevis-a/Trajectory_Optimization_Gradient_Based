#ifndef M20IA_TRAJ_OPT_GEOMETRYUTILS_H
#define M20IA_TRAJ_OPT_GEOMETRYUTILS_H

#include <eigen3/Eigen/Dense>

class GeometryUtils
{
public:
    static void pseudoinverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv, double tolerance);
    static Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &M, double tolerance=1e-4);
    static Eigen::MatrixXd frameInverse(const Eigen::MatrixXd &input);
    static Eigen::VectorXd poseDiffIntrinsic(const Eigen::MatrixXd &start,const Eigen::MatrixXd &stop);
    static Eigen::VectorXd poseDiffIntrinsic(const Eigen::MatrixXd &diff);
    static Eigen::VectorXd poseDiffExtrinsic(const Eigen::MatrixXd &start,const Eigen::MatrixXd &stop);
    static Eigen::MatrixXd zRotFrame(const double x, const double y, const double z, const double zrot);
private:
    GeometryUtils();
};

#endif //M20IA_TRAJ_OPT_GEOMETRYUTILS_H
