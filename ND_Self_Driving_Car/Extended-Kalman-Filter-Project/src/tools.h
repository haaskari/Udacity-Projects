#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  
  
  /*Cartesian to polar coordinates system*/
  
  Eigen::VectorXd convert_cartesian_to_polar(const Eigen::VectorXd& v);
  
   /*polar coordinates to cartesian coordinate system*/
  Eigen::VectorXd convert_polar_to_cartesian(const Eigen::VectorXd& v);

};

#endif  // TOOLS_H_
