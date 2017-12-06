//
// Created by klevis on 11/2/17.
//

#include <opt_problem.hpp>
#include <fstream>

using namespace opt_problems;
using namespace Eigen;

optProblem::optProblem(int dummy)
{
    std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

optProblem::optProblem(manipulator_kdl::robotKDL robot_kdl, vector<vector<double>> data_arr,vector<double> vel_limits, vector<double> timeBetween)
{
    // Get optimization dimensions from robot kdl:
    m_dim=robot_kdl.dof*data_arr.size();
    m_nec=0;
    m_nic = robot_kdl.dof * data_arr.size();// velocity constraints
    timesteps_=data_arr.size();
    robot_kdl_=robot_kdl;
    des_poses.resize(timesteps_);
    vel_lim=vel_limits;
    timeBetweenS=timeBetween;

    for(int i=0;i<timesteps_;++i)
    {
        Map<VectorXd> des_pose(&data_arr[i][0],data_arr[i].size());// desired object pose
        des_poses[i]=des_pose;
    }
}

VectorXd optProblem::positionCost(const vector<double> &x, int i) const
{
    //joint position for current timestep
    vector<double>joint_pos(robot_kdl_.dof,0.0);
    return positionCost(x, i, joint_pos);
}

VectorXd optProblem::positionCost(const vector<double> &x, int i, vector<double> &joint_pos) const
{
    for(int j=0; j<robot_kdl_.dof;++j) {
        joint_pos[j] = x[(i) * robot_kdl_.dof + j];
    }

    //now get the end effector frame - note this is the frame suggested by the optimization framework
    //convert the end effector pose to a VectorXd
    vector<double> ee_pose_vec=robot_kdl_.getFK(0,joint_pos,true);
    VectorXd ee_pose_sugg=Map<VectorXd>(&ee_pose_vec[0],robot_kdl_.dof);

    //desired pose
    const VectorXd &des_pose = des_poses[i];

    //this is the difference between the pose suggested by the optimization framework and our desired pose
    //note that the orientation error does not make sense yet because you cannot apply the l2 norm to euler angles
    //but we will fix this below
    VectorXd pos_err = des_pose-ee_pose_sugg;

    //create a vector of the desired orientation
    vector<double> orient_des = {des_pose[3], des_pose[4], des_pose[5]};
    //create a vector of the suggested orientation
    vector<double> orient_sugg = {ee_pose_sugg[3], ee_pose_sugg[4], ee_pose_sugg[5]};

    //more about the structure of this error below when the l2 norm is computed
    vector<double> orient_err=robot_kdl_.euler_diff(orient_sugg,orient_des);

    //copy the orientation error over to the overall position error vector
    pos_err[3]=orient_err[0];
    pos_err[4]=orient_err[1];
    pos_err[5]=orient_err[2];

    //the cost then is simply the l2 norm between the desired and suggested pose
    //the position is easy to see why we would use the l2 norm - euclidean distance
    //the orientation error vector contains the unit vector axis of rotation
    //scaled by the angle that will get us from the desired pose to the suggested pose
    //hence the l2 norm (or magnitude sqaured) of this vector will tell us the angle difference squared
    //between the two frames
    return pos_err;
}

double optProblem::objFunction(const vector<double> &x) const
{
    //note that the vector x has the following format
    //0-5 - joint positions for step 0
    //6-10 - joint positions for step 1
    //
    //...
    //...

    // Cost function is on reaching a desired cartesian pose
    double cost_val=0.0;

    for(int i=0; i<timesteps_; i++)
    {
        VectorXd posError=positionCost(x, i);
        cost_val+=posError.dot(posError);
    }

    cout << "Cost value: " << cost_val << endl;
    return cost_val;
}

vector<double> optProblem::objGradient(const vector<double> &x) const
{
    //note that the vector x has the following format
    //0-5 - joint positions for step 0
    //6-10 - joint positions for step 1
    //
    //...
    //...

    // The return value is the gradient of the objective function with respect to each of the optimization parameters
    // as described above
    vector<double> retval(m_dim,0.0);

    for(int i=0; i<timesteps_; i++)
    {
        vector<double> joint_pos(robot_kdl_.dof,0.0);
        VectorXd posError=positionCost(x, i);
        Eigen::VectorXd gradient=-2*posError.transpose()*robot_kdl_.getJacobian(0,joint_pos);

        //place the gradient in the appropriate slot in the return vector
        for (int j=0;j<robot_kdl_.dof;++j)
        {
            retval[robot_kdl_.dof*(i)+j]=gradient[j];
        }
    }

    return retval;
}

vector<double> optProblem::inEqConstraints(const vector<double> &x) const
{
    // limiting velocity at joints
    vector<double> vel_const(m_nic,0.0);

    for (int i = 0; i < timesteps_ - 1; ++i) {
        for (int j = 0; j < robot_kdl_.dof; ++j) {
            vel_const[(i + 1) * robot_kdl_.dof + j] =
                    std::abs(x[(i + 1) * robot_kdl_.dof + j] - x[(i) * robot_kdl_.dof + j]) / timeBetweenS[i + 1] -
                    vel_lim[j];
        }
    }

    return vel_const;
}

vector<double> optProblem::inEqGradient(const vector<double> &x) const
{
    vector<double> retval(m_dim*m_nic,0.0);

    for (int i = 0; i < timesteps_ - 1; ++i) {
        for (int j = 0; j < robot_kdl_.dof; ++j) {
            if (std::abs(x[(i + 1) * robot_kdl_.dof + j] - x[(i) * robot_kdl_.dof + j])/timeBetweenS[i+1] > vel_lim[j]) {
                if (x[(i + 1) * robot_kdl_.dof + j] - x[(i) * robot_kdl_.dof + j] < 0.0) {
                    retval[((i + 1) * robot_kdl_.dof + j) * m_dim + (i + 1) * robot_kdl_.dof + j] =
                            -1.0 / timeBetweenS[i + 1];
                    retval[((i + 1) * robot_kdl_.dof + j) * m_dim + (i) * robot_kdl_.dof + j] =
                            1.0 / timeBetweenS[i + 1];
                } else {
                    retval[((i + 1) * robot_kdl_.dof + j) * m_dim + (i + 1) * robot_kdl_.dof + j] =
                            1.0 / timeBetweenS[i + 1];
                    retval[((i + 1) * robot_kdl_.dof + j) * m_dim + (i) * robot_kdl_.dof + j] =
                            -1.0 / timeBetweenS[i + 1];
                }
            }
        }
    }

    return retval;
}

vector<double> optProblem::EqConstraints(const vector<double> &x) const
{
    // no constraint
    vector<double> ret(m_nec,0.0);
    return  ret;
}



vector<double> optProblem::eqGradient(const vector<double> &x) const
{
    vector<double> retval(m_dim*m_nec,0.0);
    return retval;
}

vector<vector<double>> optProblem::bounds() const
{
    vector<vector<double>> bound;
    vector<double> up_bounds;
    vector<double> low_bounds;

    // Getting joint angle limits from urdf
    for(int i=0;i<timesteps_;++i)
    {
        up_bounds.insert(up_bounds.end(), robot_kdl_.up_bounds.begin(), robot_kdl_.up_bounds.end());
        low_bounds.insert(low_bounds.end(), robot_kdl_.low_bounds.begin(), robot_kdl_.low_bounds.end());
    }

    bound.resize(2);
    bound[0]=low_bounds;
    bound[1]=up_bounds;
    return bound;

}

vector<double> optProblem::gradient(const vector<double> &x) const
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
std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> optProblem::gradient_sparsity() const
{
  std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> retval;
  // Objective gradient is dense
  for (int i=0;i<m_dim;++i)
  {
    retval.emplace_back(0,i);
  }
  // Equality constraints:
  for (int i=0;i<m_nec;++i)
  {
    retval.emplace_back(i+1,i);
  }
  // inequality constraints:
  for (int i=0;i<m_nic;++i)
  {
    retval.emplace_back(i+m_nec+1,i);
    retval.emplace_back(i+m_nec+1,i+robot_kdl_.dof);

  }
  return retval;
}
vector<double> optProblem::sparse_gradient(const vector<double> x) const
{
  vector<double> grad;
  vector<double> obj_grad=objGradient(x);
  vector<double> eq_grad=eqGradient(x);
  vector<double> ineq_grad=inEqGradient(x);

  grad.resize(m_dim,0.0);

  for(int i=0;i<m_dim;i++)
  {
    grad[i]=obj_grad[i];
  }

  for(int i=m_dim;i<m_dim+m_dim*m_nec;i++)
  {
    if(eq_grad[i-m_dim]!=0.0)
    {
      grad.emplace_back(eq_grad[i-m_dim]);
    }
  }


  for(int i=m_dim+m_dim*m_nec;i<m_dim+m_dim*m_nec+m_dim*m_nic;i++)
  {
    if(ineq_grad[i-m_dim-m_dim*m_nec]!=0.0)
    {
      grad.emplace_back(ineq_grad[i-m_dim-m_dim*m_nec]);
    }
  }

  return grad;
}

#endif
