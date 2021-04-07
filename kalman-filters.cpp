//EIGEN LIBRARY NEEDS TO BE INSTALLED BEFORE CODE COMPILATION
#include <bits/stdc++.h>
#include <Eigen/Dense>
class KalmanFilter {		//Class Kalman Filter
public:
  int m, n;		//System dimensions
  double t0, t;	//Initial  and Current time
  double dt;		//Time step to be used
  bool initialized;	//Initialization flag
  Eigen::VectorXd x_curr, x_new, input;	//estimated states
  Eigen::MatrixXd A, B, C, Q, R, P, K, P0, I; //required matrices(P0 is initial P, I is identity matrix)
  KalmanFilter(
      double dt,			//Time step
      const Eigen::MatrixXd& A,	//State transition matrix
      const Eigen::MatrixXd& B,	//Control-input matrix
      const Eigen::MatrixXd& C,	//Observation matrix(also represented by H)
      const Eigen::MatrixXd& Q,	//Covariance of process noise
      const Eigen::MatrixXd& R,	//Covariance of observation noise
      const Eigen::MatrixXd& P	//Estimated accuracy
  );
  KalmanFilter();	//blank filter
  void init();		//initializing all the variables of the class
  void init(double t0, const Eigen::VectorXd& x0, const Eigen::VectorXd& i0);
  void update(const Eigen::VectorXd& input, const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A, const Eigen::MatrixXd B);	//updating the values
  Eigen::VectorXd return_state() { return x_curr; };	//returning estimated state and time
  double return_time(){return t;};
};

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), B(B), C(C), Q(Q), R(R), P0(P), I(n,n),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false), x_curr(n), x_new(n), input(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init() {	//variable initialization
  x_curr.setZero();
//  input.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0, const Eigen::VectorXd& i0) {
  x_curr = x0;
//  input = i0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}


void KalmanFilter::update(const Eigen::VectorXd& input, const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A, const Eigen::MatrixXd B) {
  if(!initialized)
    throw std::runtime_error("Filter not initialized!");
  this->A = A;
  this->B = B;
  this->dt = dt;
  
  x_new = A * x_curr + B * input;			//PREDICTION EQUATIONS
  P = A*P*A.transpose() + Q;
  
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();	//UPDATION EQUATIONS
  x_new += K * (y - C*x_new);
  P = (I - K*C)*P;
  
  x_curr = x_new;			//Changing the current value to newly refined value
  t += dt;
}

int main(int argc, char* argv[]) {

  int n = 3; // Number of states
  int m = 1; // Number of measurements

  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd B(n, n); // Control-input matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
//  B << 0.1, 0, dt, 0, dt, 0, dt, 0.1, 0.1;
  B << 1, dt, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << .01, .0, .02, .0, .05, .02, .04, .03, .01;
  R << 3;
  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "B: \n" << B << std::endl;  
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt,A, B, C, Q, R, P);

  // Positions
  std::vector<double> measurements = {
     4.9453,82.0088,2.3885,17.0106,6.2305,
     18.8421,14.4227,68.0032,4.049,1.2386,
     68.8457,34.0079,46.2962,79.5702,8.5624,
     82.9663,46.5978,51.4124,78.9766,70.1986,
     39.0614,70.1913,7.235,70.1725,2.2537,
     39.9189,86.0197,17.4176,37.978,59.6607,
     69.3944,63.5389,96.0423,80.9638,21.7409,
     34.6954,2.6303,2.2193,99.0133,93.7819,
     4.9014,82.1409,92.4156,83.2824,97.0619
  };
  // Manual inputs
  std::vector<double> inputs = {	
      1.2719,8.1712,7.7111,8.8326,1.9098,
      5.2521,9.4109,6.9533,7.2131,4.9088,
      9.0647,4.2306,9.432,6.7861,7.0865,
      4.6899,6.4236,5.0402,1.1371,4.0227,
      7.0651,9.5832,4.7086,9.0111,9.0935,
      3.5641,6.7923,1.8308,4.1764,2.5577,
      1.5522,5.6319,6.735,6.699,3.7442,
      2.9707,8.908,4.9091,5.4408,8.5234,
      4.8706,4.5274,3.4435,9.361,9.8624,
   };
  
  Eigen::VectorXd x0(n), i0(n);
  double t = 0;
  x0 << measurements[0], 0, -9.81;
  i0 << inputs[0], 0 , 0.08;
  kf.init(t, x0, i0);

  // Feed measurements into filter, output estimated states

  Eigen::VectorXd y(m), inp(n);
  std::cout << "t = " << t << ", " << "x_curr[0]: " << kf.return_state().transpose() << std::endl;
  for(int i = 0; i < measurements.size(); i++) {
    t += dt;
    y << measurements[i];
    inp << inputs[i], 0 , 0.08;
    kf.update(inp, y, dt, A, B);
    std::cout<<i<<" updated"<<std::endl;
    std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
        << ", x_curr[" << i << "] = " << kf.return_state().transpose() << " input["<<i<<"] = "<<inp<< std::endl;
  }
  return 0;
}
