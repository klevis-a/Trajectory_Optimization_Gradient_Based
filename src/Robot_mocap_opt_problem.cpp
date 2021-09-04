//
// Created by klevis on 11/2/17.
//

#include "Robot_mocap_opt_problem.h"
#include "GeometryUtils.h"

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

double logisticFun(double velPer)
{
    double k=10.0;
    double x0=1.2;
    return 1/(1+exp(-k*(velPer-x0)));
}

Robot_mocap_opt_problem::Robot_mocap_opt_problem(int dummy)
{
    std::cerr<<"Robot_mocap_opt_problem dummy constructor is initialized, fix me!!!"<<std::endl;
}

Robot_mocap_opt_problem::Robot_mocap_opt_problem(const RobotKDL robot_kdl, const vector<MatrixXd> frames, const VectorXd dt, const VectorXd vel_limits, const vector<double> &posLB, 
                                                const vector<double> &posUB, const double rotLB, const double rotUB):_robot_kdl(robot_kdl),_frames(frames),_dt(dt), _vel_limits(vel_limits), 
                                                _timesteps(frames.size()), _posLB(posLB), _posUB(posUB), _rotLB(rotLB), _rotUB(rotUB)
{
    // Get optimization dimensions from robot kdl and trajectory data
    //robot degree of freedom * number of frames + 4 variables for optimizing starting position and orientation
    m_dim=robot_kdl.dof*_timesteps+4;
    //1 equality for making sure the position matches
    m_nec=1;
    m_nic = robot_kdl.dof * (_timesteps-1);// velocity constraints
}

double Robot_mocap_opt_problem::objFunction(const vector<double> &x) const
{
    //note that the vector x has the following format
    //0-5 - joint positions for step 0
    //6-10 - joint positions for step 1
    //
    //...
    //...
    //length-4: rotation about z
    //length-3: translation along x
    //length-2: translation along y
    //length-1: translation along z

    // Cost function is on reaching a desired cartesian pose
    double L=20.0;
    double cost_val=0.0;

    for(int i=0; i<_timesteps;++i)
    {
        for(int j=0; j<_robot_kdl.dof;++j)
        {
            double cost;
            if(i==0)
            {
                double previousTime = x[(i) * _robot_kdl.dof + j];
                double nextTime = x[(i + 1) * _robot_kdl.dof + j];
                double vel = abs(nextTime - previousTime) / (_dt[i + 1]);
                double velPer = vel / _vel_limits[j];
                cost = L * logisticFun(velPer);
            }
            else if(i==_timesteps-1)
            {
                double previousTime = x[(i-1) * _robot_kdl.dof + j];
                double nextTime = x[(i) * _robot_kdl.dof + j];
                double vel = abs(nextTime - previousTime) / (_dt[i]);
                double velPer = vel / _vel_limits[j];
                cost = L * logisticFun(velPer);
            }
            else {
                double previousTime = x[(i-1) * _robot_kdl.dof + j];
                double nextTime = x[(i + 1) * _robot_kdl.dof + j];
                double vel = abs(nextTime - previousTime) / (_dt[i]+_dt[i + 1]);
                double velPer = vel / _vel_limits[j];
                cost = L * logisticFun(velPer);
            }
            cost_val += cost;
        }
    }
    return cost_val;
}

vector<double> Robot_mocap_opt_problem::objGradient(const vector<double> &x) const
{
    //note that the vector x has the following format
    //0-5 - joint positions for step 0
    //6-10 - joint positions for step 1
    //
    //...
    //...
    //length-4: rotation about z
    //length-3: translation along x
    //length-2: translation along y
    //length-1: translation along z

    // The return value is the gradient of the objective function with respect to each of the optimization parameters
    // as described above
    double L=20.0;
    double k=10.0;
    vector<double> retval(m_dim,0.0);

    for(int i=0; i<_timesteps;++i)
    {
        for(int j=0; j<_robot_kdl.dof;++j)
        {
            if(i==0)
            {
                double previousTime = x[(i) * _robot_kdl.dof + j];
                double nextTime = x[(i + 1) * _robot_kdl.dof + j];
                double vel = abs(nextTime - previousTime) / (_dt[i + 1]);
                double velPer = vel / _vel_limits[j];
                double cost = logisticFun(velPer);
                //derivative of logistic function
                double der = L*k*cost*(1-cost)/(_dt[i+1]*_vel_limits[j]);
                //this is necessary because we are using absolute value
                if(nextTime-previousTime<0)
                {
                    der=-der;
                }
                retval[_robot_kdl.dof*(i)+j]+=-der;
                retval[_robot_kdl.dof*(i+1)+j]+=der;
            }
            else if(i==_timesteps-1)
            {
                double previousTime = x[(i-1) * _robot_kdl.dof + j];
                double nextTime = x[(i) * _robot_kdl.dof + j];
                double vel = abs(nextTime - previousTime) / (_dt[i]);
                double velPer = vel / _vel_limits[j];
                //derivative of logistic function
                double cost = logisticFun(velPer);
                double der = L*k*cost*(1-cost)/(_dt[i]*_vel_limits[j]);
                //this is necessary because we are using absolute value
                if(nextTime-previousTime<0)
                {
                    der=-der;
                }
                retval[_robot_kdl.dof*(i-1)+j]+=-der;
                retval[_robot_kdl.dof*(i)+j]+=der;
            }
            else {
                double previousTime = x[(i-1) * _robot_kdl.dof + j];
                double nextTime = x[(i + 1) * _robot_kdl.dof + j];
                double vel = abs(nextTime - previousTime) / (_dt[i]+_dt[i + 1]);
                double velPer = vel / _vel_limits[j];
                double cost = logisticFun(velPer);
                //derivative of logistic function
                double der = L*k*cost*(1-cost)/((_dt[i]+_dt[i + 1])*_vel_limits[j]);
                //this is necessary because we are using absolute value
                if(nextTime-previousTime<0)
                {
                    der=-der;
                }
                retval[_robot_kdl.dof*(i-1)+j]+=-der;
                retval[_robot_kdl.dof*(i+1)+j]+=der;
            }
        }
    }

    return retval;
}

vector<double> Robot_mocap_opt_problem::inEqConstraints(const vector<double> &x) const
{
    // limiting velocity at joints
    vector<double> vel_const(m_nic,0.0);

    //there is no velocity constraint for the first frame
    for (int i = 1; i < _timesteps; ++i)
    {
        for (int j = 0; j < _robot_kdl.dof; ++j)
        {
            vel_const[(i-1) * _robot_kdl.dof + j] =
                    abs(x[(i ) * _robot_kdl.dof + j] - x[(i-1) * _robot_kdl.dof + j]) / _dt[i] -
                    _vel_limits[j];
        }
    }

    return vel_const;
}

vector<double> Robot_mocap_opt_problem::inEqGradient(const vector<double> &x) const
{
    #if SPARSE_GRADIENT==false
    vector<double> retval(m_dim*m_nic,0.0);

    for (int i = 1; i < timesteps_; ++i) {
        for (int j = 0; j < robot_kdl_.dof; ++j) {
            if (std::abs(x[(i) * robot_kdl_.dof + j] - x[(i-1) * robot_kdl_.dof + j])/timeBetweenS[i] > vel_lim[j]) {
                if (x[(i) * robot_kdl_.dof + j] - x[(i-1) * robot_kdl_.dof + j] < 0.0) {
                    retval[((i-1) * robot_kdl_.dof + j) * m_dim + (i) * robot_kdl_.dof + j] =
                            -1.0 / timeBetweenS[i];
                    retval[((i-1) * robot_kdl_.dof + j) * m_dim + (i-1) * robot_kdl_.dof + j] =
                            1.0 / timeBetweenS[i];
                } else {
                    retval[((i-1) * robot_kdl_.dof + j) * m_dim + (i) * robot_kdl_.dof + j] =
                            1.0 / timeBetweenS[i];
                    retval[((i-1) * robot_kdl_.dof + j) * m_dim + (i-1) * robot_kdl_.dof + j] =
                            -1.0 / timeBetweenS[i];
                }
            }
        }
    }
    #endif

    #if SPARSE_GRADIENT==true
    vector<double> retval(2*(_timesteps-1)*_robot_kdl.dof,0.0);

    for(int i=1;i<_timesteps;++i)
    {
        for(int j=0;j<_robot_kdl.dof;++j)
        {
            if(x[(i)*_robot_kdl.dof+j]-x[(i-1)*_robot_kdl.dof+j]<0.0)
            {
                retval[2*(i-1)*_robot_kdl.dof+j]=1.0/_dt[i];

                retval[2*(i-1)*_robot_kdl.dof+_robot_kdl.dof+j]=-1.0/_dt[i];

            }
            else
            {
                retval[2*(i-1)*_robot_kdl.dof+j]=-1.0/_dt[i];

                retval[2*(i-1)*_robot_kdl.dof+_robot_kdl.dof+j]=1.0/_dt[i];

            }
        }
    }
    #endif

    return retval;
}

vector<double> Robot_mocap_opt_problem::EqConstraints(const vector<double> &x) const
{
    // no constraint
    vector<double> ret(m_nec,0.0);

    //note that the vector x has the following format
    //0-5 - joint positions for step 0
    //6-10 - joint positions for step 1
    //
    //...
    //...
    //length-4: rotation about z
    //length-3: translation along x
    //length-2: translation along y
    //length-1: translation along z

    // Cost function is on reaching a desired cartesian pose
    double cost_val=0.0;

    //initial position
    double zrot = x[x.size()-4];
    double x0 = x[x.size()-3];
    double y0 = x[x.size()-2];
    double z0 = x[x.size()-1];


    //initial transformation matrix
    MatrixXd init_transform=GeometryUtils::zRotFrame(x0, y0, z0,zrot);
    const VectorXd j = Eigen::Map<const VectorXd>(x.data(), x.size());

    for(int i=0; i<_timesteps; i++)
    {
        VectorXd currentJoints = j.segment(i*_robot_kdl.dof, _robot_kdl.dof);
        MatrixXd suggestedFrame = _robot_kdl.getFKFrame(currentJoints);
        VectorXd posError = GeometryUtils::poseDiffExtrinsic(suggestedFrame,init_transform*_frames[i]);
        cost_val+=posError.dot(posError);
    }

    ret[0]=cost_val;
    return ret;
}

vector<double> Robot_mocap_opt_problem::eqGradient(const vector<double> &x) const
{
    // The return value is the gradient of the objective function with respect to each of the optimization parameters
    // as described above
    vector<double> retval(m_dim*m_nec,0.0);

    //note that the vector x has the following format
    //0-5 - joint positions for step 0
    //6-10 - joint positions for step 1
    //
    //...
    //...
    //length-3: translation along x
    //length-2: translation along y
    //length-1: translation along z

    //these are the partial derivatives of the objective function but broken down with respect to the part that the
    //position error contributes and the orientation error contributes. Note that the derivative of the orientation
    //error with respect to x0,y0,z0 is zero.
    double dPhiPos=0.0;
    double dX0Pos=0.0;
    double dY0Pos=0.0;
    double dZ0Pos=0.0;
    double dPhiOrient=0.0;

    //initial position and orientation
    double zrot = x[x.size()-4];
    double x0 = x[x.size()-3];
    double y0 = x[x.size()-2];
    double z0 = x[x.size()-1];

    //initial transformation matrix
    Eigen::MatrixXd init_transform=GeometryUtils::zRotFrame(x0, y0, z0,zrot);
    const VectorXd j = Eigen::Map<const VectorXd>(x.data(), x.size());

    for(int i=0; i<_timesteps; i++)
    {
        VectorXd currentJoints = j.segment(i*_robot_kdl.dof, _robot_kdl.dof);
        MatrixXd suggestedFrame = _robot_kdl.getFKFrame(currentJoints);
        MatrixXd desiredFrame = init_transform*_frames[i];
        VectorXd posError = GeometryUtils::poseDiffExtrinsic(suggestedFrame,desiredFrame);

        VectorXd gradient=-2*posError.transpose()*_robot_kdl.getJacobian(currentJoints);

        //place the gradient in the appropriate slot in the return vector
        for (int j=0;j<_robot_kdl.dof;++j)
        {
            retval[_robot_kdl.dof*(i)+j]=gradient[j];
        }

        //now we need to compute the gradient with respect to x0,y0,z0
        //desired displacement for the 1,2,and 3 directions
        double dd1=desiredFrame(0,3);
        double dd2=desiredFrame(1,3);
        double dd3=desiredFrame(2,3);

        //suggested displacement
        double d1 = suggestedFrame(0,3);
        double d2 = suggestedFrame(1,3);
        double d3 = suggestedFrame(2,3);

        //add the derivatives for the current step
        dX0Pos += 2 * (dd1-d1);
        dY0Pos += 2 * (dd2-d2);
        dZ0Pos += 2 * (dd3-d3);
        dPhiPos += 2 * ((d2-dd2)*x0 + (dd1-d1)*y0 - dd1*d2 + d1*dd2);
        dPhiOrient += 2*posError[5];
    }

    //place the derivatives in the current location
    retval[x.size()-4]=dPhiOrient+dPhiPos;
    retval[x.size()-3]=dX0Pos;
    retval[x.size()-2]=dY0Pos;
    retval[x.size()-1]=dZ0Pos;

    return retval;
}

vector<vector<double>> Robot_mocap_opt_problem::bounds() const
{
    vector<vector<double>> bound;
    vector<double> up_bounds;
    vector<double> low_bounds;

    // Getting joint angle limits from urdf
    for(int i=0;i<_timesteps;++i)
    {
        up_bounds.insert(up_bounds.end(), _robot_kdl.up_bounds.begin(), _robot_kdl.up_bounds.end());
        low_bounds.insert(low_bounds.end(), _robot_kdl.low_bounds.begin(), _robot_kdl.low_bounds.end());
    }

    //add upper bounds for the initial position
    up_bounds.push_back(_rotUB);
    up_bounds.push_back(_posUB[0]);
    up_bounds.push_back(_posUB[1]);
    up_bounds.push_back(_posUB[2]);

    //add lower bounds for the initial position
    low_bounds.push_back(_rotLB);
    low_bounds.push_back(_posLB[0]);
    low_bounds.push_back(_posLB[1]);
    low_bounds.push_back(_posLB[2]);

    bound.resize(2);
    bound[0]=low_bounds;
    bound[1]=up_bounds;
    return bound;
}

vector<double> Robot_mocap_opt_problem::gradient(const vector<double> &x) const
{
    vector<double> grad;
    vector<double> obj_grad=objGradient(x);
    vector<double> eq_grad=eqGradient(x);
    vector<double> ineq_grad=inEqGradient(x);

    grad.resize(m_dim+m_dim*m_nec+m_dim*m_nic,0.0);

    for(int i=0;i<m_dim;i++)
    {
        grad[i]=obj_grad[i];

    }

    for(int i=m_dim;i<m_dim+m_dim*m_nec;i++)
    {
        grad[i]=eq_grad[i-m_dim];
    }


    for(int i=m_dim+m_dim*m_nec;i<m_dim+m_dim*m_nec+m_dim*m_nic;i++)
    {

        grad[i]=ineq_grad[i-m_dim-m_dim*m_nec];
    }

    return grad;
}

#if SPARSE_GRADIENT==true
vector<std::pair<vector<double>::size_type, vector<double>::size_type>> Robot_mocap_opt_problem::gradient_sparsity() const
{
    vector<std::pair<vector<double>::size_type, vector<double>::size_type>> retval;
    // Objective gradient is dense
    for (int i=0;i<m_dim;++i)
    {
        retval.emplace_back(0,i);
    }
    // Equality constraints:
    for (int i=0;i<m_nec;++i)
    {
        for(int j=0;j<m_dim; j++)
        {
            retval.emplace_back(i+1,j);
        }
    }
    // inequality constraints:
    for(int i=1;i<_timesteps;++i)
    {
        for(int j=0;j<_robot_kdl.dof;++j)
        {
            retval.emplace_back(1+m_nec+(i-1)*_robot_kdl.dof+j,(i-1)*_robot_kdl.dof+j);
            retval.emplace_back(1+m_nec+(i-1)*_robot_kdl.dof+j,(i)*_robot_kdl.dof+j);
        }
    }
    return retval;
}

vector<double> Robot_mocap_opt_problem::sparse_gradient(const vector<double> &x) const {
    vector<double> grad;
    vector<double> obj_grad = objGradient(x);
    vector<double> eq_grad=eqGradient(x);
    vector<double> ineq_grad = inEqGradient(x);

    grad.resize(m_dim, 0.0);

    for (int i = 0; i < m_dim; i++) {
        grad[i] = obj_grad[i];
    }

    grad.insert(grad.end(), eq_grad.begin(), eq_grad .end());

    for (int i = 1; i < _timesteps; ++i) {
        for (int j = 0; j < _robot_kdl.dof; ++j) {
            grad.emplace_back(ineq_grad[2 * (i - 1) * _robot_kdl.dof + j]);
            grad.emplace_back(ineq_grad[2 * (i - 1) * _robot_kdl.dof + _robot_kdl.dof + j]);
        }
    }

    return grad;
}

#endif
