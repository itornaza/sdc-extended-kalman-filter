#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
   * Constructor
   */
  Tools();

  /**
   * Destructor
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE
   */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                         const vector<VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians
   */
  MatrixXd CalculateJacobian(const VectorXd& x_state);
  
  /**
   * A helper method to calculate h(x) for the EKF
   */
  VectorXd CalculateHx(const VectorXd& x_state);
  
  /**
   * A helper method to provide an identity matrix
   */
  MatrixXd GetI(const VectorXd& x_state);
  
  /**
   * A helper method to normalize angles
   * wrap x -> [0,max)
   * From: http://stackoverflow.com/a/29871193/1321129
   */
  float wrapMax(float x, float max);
  
  /**
   * A helper method to normalize angles
   * wrap x -> [min,max)
   * From: http://stackoverflow.com/a/29871193/1321129
   */
  float wrapMinMax(float x, float min, float max);
};

#endif /* TOOLS_H_ */
