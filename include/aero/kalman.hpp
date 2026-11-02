#pragma once

namespace aero {

// One-dimensional constant-velocity Kalman filter.
//   state  x = [position, velocity]
//   model  position' = position + velocity * dt   (velocity assumed constant,
//          with random acceleration noise to cover manoeuvres / model error)
//   sensor measures position only.
//
// Because the model is separable, the tracker runs one of these per axis
// (east, north, up) instead of one big 6x6 filter. All matrices here are 2x2
// and written out by hand so every line maps to a textbook equation.
class KalmanAxis {
 public:
  void init(double pos, double posVar, double velVar);

  // Time update: x = F x,  P = F P F^T + Q
  void predict(double dt, double accelSigma);

  // Measurement update with scalar position measurement of variance measVar.
  void update(double meas, double measVar);

  double pos() const { return p_; }
  double vel() const { return v_; }
  double posVar() const { return P00_; }
  // Variance of the innovation (what update() will compare the measurement to).
  double innovationVar(double measVar) const { return P00_ + measVar; }

 private:
  double p_ = 0, v_ = 0;
  // Symmetric covariance [[P00, P01], [P01, P11]]
  double P00_ = 0, P01_ = 0, P11_ = 0;
};

}  // namespace aero
