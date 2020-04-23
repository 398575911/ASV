/*
***********************************************************************
* kalmanfilter.h: observer using kalman filter
* function to simulate the kalman filter based on Eigen
* This header file can be read by C++ compilers
*
*  Kalman Filter Class Definition.
*
*  Matrix Dimension must be:
*  x[k] = A * x[k-1] + B * u[k-1] + w[k-1]
*  z[k] = H * x[k] + v[k]
*  x: n x 1, state vector
*  z: m x 1, observer vector
*  u: l x 1, input vector
*  A: n x n
*  B: n x l
*  H: m x n
*  Q: n x n
*  R: m x m
*  I: n x n
*  P: n x n
*  K: n x m
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include "common/property/include/vesseldata.h"
#include "estimatordata.h"

namespace ASV::localization {

// Kalman filtering for linear system
template <int l = 1, int m = 1, int n = 1>
class kalmanfilter {
 protected:
  using vectorld = Eigen::Matrix<double, l, 1>;
  using vectormd = Eigen::Matrix<double, m, 1>;
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnnd = Eigen::Matrix<double, n, n>;
  using matrixnld = Eigen::Matrix<double, n, l>;
  using matrixmnd = Eigen::Matrix<double, m, n>;
  using matrixnmd = Eigen::Matrix<double, n, m>;
  using matrixmmd = Eigen::Matrix<double, m, m>;

 public:
  // disable the default constructor
  kalmanfilter() = delete;
  explicit kalmanfilter(const matrixnnd &A, const matrixnld &B,
                        const matrixmnd &H, const matrixnnd &Q,
                        const matrixmmd &R) noexcept
      : A_(A),
        B_(B),
        H_(H),
        Q_(Q),
        R_(R),
        P_(matrixnnd::Identity()),
        K_(matrixnmd::Zero()),
        X_(vectornd::Zero()) {}

  virtual ~kalmanfilter() = default;
  /* Set Initial Value */
  void setInitial(const vectornd &X0, const matrixnnd &P0) {
    this->X_ = X0;
    this->P_ = P0;
  }  // setInitial

  // perform kalman filter for one step
  void linearkalman(const matrixnnd &A, const matrixnld &B,
                    const vectorld &former_U, const vectormd &Z) {
    updatesystem(A, B);
    predict(former_U);
    correct(Z);
  }  // linearkalman

  void linearkalman(const matrixnnd &A, const vectorld &former_U,
                    const vectormd &Z) {
    updatesystem(A);
    predict(former_U);
    correct(Z);
  }  // linearkalman

  void linearkalman(const vectorld &former_U, const vectormd &Z) {
    predict(former_U);
    correct(Z);
  }  // linearkalman

  // calculate the max eigenvalue of P
  double getMaxEigenP() const {
    Eigen::SelfAdjointEigenSolver<matrixnnd> eigensolver(this->P_);
    if (eigensolver.info() != Eigen::Success)
      return 100;
    else
      return eigensolver.eigenvalues().maxCoeff();
  }  // getMaxEigenP

  vectornd getState() const noexcept { return X_; }
  void setState(const vectornd &state) noexcept { X_ = state; }
  // After intialization of sensors, we can specify value to state
  void setQ(const matrixnnd &Q) noexcept { Q_ = Q; }
  void setR(const matrixmmd &R) noexcept { R_ = R; }

 protected:
  /* Fixed Matrix */
  matrixnnd A_;  // System dynamics matrix
  matrixnld B_;  // Control matrix
  matrixmnd H_;  // Mesaurement Adaptation matrix
  matrixnnd Q_;  // Process Noise Covariance matrix
  matrixmmd R_;  // Measurement Noise Covariance matrix

  /* Variable Matrix */
  matrixnnd P_;  // State Covariance
  matrixnmd K_;  // Kalman Gain matrix
  vectornd X_;   //(Current) State vector

 private:
  /* Do prediction based of physical system (No external input) */
  void predict(void) {
    this->X_ = this->A_ * this->X_;
    this->P_ = this->A_ * this->P_ * this->A_.transpose() + this->Q_;
  }  // predict

  /* Do prediction based of physical system (with external input)
   * U: Control vector
   */
  void predict(const vectorld &U) {
    this->X_ = this->A_ * this->X_ + this->B_ * U;
    this->P_ = this->A_ * this->P_ * this->A_.transpose() + this->Q_;
  }  // predict

  /* Correct the prediction, using mesaurement
   *  Z: mesaure vector */
  void correct(const vectormd &Z) {
    this->K_ =
        (this->P_ * this->H_.transpose()) *
        (this->H_ * this->P_ * this->H_.transpose() + this->R_).inverse();
    // K = (P * H.transpose()) * (H * P * H.transpose() + R).llt().solve(Im);
    this->X_ = this->X_ + this->K_ * (Z - this->H_ * this->X_);
    this->P_ = (matrixnnd::Identity() - this->K_ * this->H_) * this->P_;
  }  // correct

  /*Set Fixed Matrix(NO INPUT) */
  void updatesystem(const matrixnnd &A, const matrixnld &B) {
    this->A_ = A;
    this->B_ = B;
  }  // updatesystem

  /*Set Fixed Matrix(NO INPUT) */
  void updatesystem(const matrixnnd &A) { this->A_ = A; }
};  //  // end class kalmanfilter

// Kalman filtering for surface vessel
class ASV_kalmanfilter : public kalmanfilter<3, 6, 6> {
 public:
  explicit ASV_kalmanfilter(const common::vessel &vesselconfig,
                            const estimatordata &estimator_data) noexcept
      : kalmanfilter(matrixnnd::Zero(), matrixnld::Zero(),
                     matrixmnd::Identity(), estimator_data.Q, estimator_data.R),
        sample_time_(estimator_data.sample_time) {
    initializekalman(vesselconfig);
  }

  ~ASV_kalmanfilter() noexcept {}

  // perform kalman filter for one step
  ASV_kalmanfilter &linearkalman(const estimatorRTdata &RTdata) {
    updateKalmanA(RTdata.CTB2G);
    kalmanfilter::linearkalman(RTdata.BalphaU, RTdata.Measurement);
    return *this;
  }

 private:
  const double sample_time_;

  // initialize parameters in Kalman filter
  void initializekalman(const common::vessel &vesselconfig) {
    // copy the constant data
    Eigen::Matrix3d Mass(vesselconfig.Mass + vesselconfig.AddedMass);
    Eigen::Matrix3d Damping(vesselconfig.LinearDamping);

    // calcualte the A and B in continous equation
    matrixnld Bk = matrixnld::Zero();
    matrixnnd Ak = matrixnnd::Zero();
    Eigen::Matrix3d Inv_Mass = Mass.inverse();
    Ak.topRightCorner(3, 3) = Eigen::Matrix3d::Identity();
    Ak.bottomRightCorner(3, 3) = -Inv_Mass * Damping;
    Bk.bottomRows(3) = Inv_Mass;

    // calculate discrete time A, B, and H
    kalmanfilter::A_ = matrixnnd::Identity() + sample_time_ * Ak;
    kalmanfilter::B_ = sample_time_ * Bk;
  }

  // real time update the Kalman filter matrix using orientation
  void updateKalmanA(const Eigen::Matrix3d &CTB2G) {
    kalmanfilter::A_.topRightCorner(3, 3) = sample_time_ * CTB2G;
  }  // updateKalmanA

};  // end class ASV_kalmanfilter

}  // namespace ASV::localization

#endif /* _KALMANFILTER_H_ */