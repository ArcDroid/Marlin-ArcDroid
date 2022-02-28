#pragma once

#include "../inc/MarlinConfig.h"


template <size_t R, size_t C>
struct Matrix {
    float x[R][C];

    void debugPrint(const char* name) {
      SERIAL_ECHOPAIR("matrix: ", name);

      SERIAL_ECHO(" {");
      for(size_t r = 0; r < R; r++) {
        SERIAL_ECHO("{");
        for(size_t c = 0; c < C; c++) {
          SERIAL_ECHO_F(x[r][c]);
          SERIAL_ECHO(", ");
        }
        SERIAL_ECHO("}, ");
      }
      SERIAL_ECHOLN("}");
    }
};

template <size_t R, size_t C>
Matrix<R, C> plus(Matrix<R, C>a, Matrix<R, C>b) {
  Matrix<R, C> ret;
  for(size_t r = 0; r < R; r++) {
    for(size_t c = 0; c < C; c++) {
      ret.x[r][c] = a.x[r][c] + b.x[r][c];
    }
  }
  return ret;
};

template <size_t R, size_t C>
Matrix<R, C> minus(Matrix<R, C>a, Matrix<R, C>b) {
  Matrix<R, C> ret;
  for(size_t r = 0; r < R; r++) {
    for(size_t c = 0; c < C; c++) {
      ret.x[r][c] = a.x[r][c] - b.x[r][c];
    }
  }
  return ret;
};

template <size_t R, size_t C, size_t M>
Matrix<R, C> times(Matrix<R, M>a, Matrix<M, C>b) {
  Matrix<R, C> ret;
  for(size_t c = 0; c < C; c++) {
    for(size_t r = 0; r < R; r++) {
      float v = 0;
      for (size_t m = 0; m < M; m++) {
        v += a.x[r][m] * b.x[m][c];
      }
      ret.x[r][c] = v;
    }
  }
  return ret;
};

template <size_t R, size_t C>
Matrix<R, C> transpose(Matrix<C, R>a) {
  Matrix<R, C> ret;
  for(size_t r = 0; r < R; r++) {
    for(size_t c = 0; c < C; c++) {
      ret.x[r][c] = a.x[c][r];
    }
  }
  return ret;
};


template <size_t C>
Matrix<C, C> identity() {
  Matrix<C, C> ret = {0};
  for(size_t c = 0; c < C; c++) {
    ret.x[c][c] = 1;
  }
  return ret;
};

Matrix<1, 1> inverse(Matrix<1, 1>a);

template<size_t N, size_t M, typename Utype>
struct Estimate
{
public:
    Matrix<N, 1> x;
    Matrix<N, N> P;
    Utype* u;
    float dt;
};

template<size_t N, size_t M, typename Utype>
class EKFModel {
public:
  virtual Matrix<M, 1> getR() = 0;

  virtual Matrix<N,N> getQ(const float dT) = 0;

  virtual Matrix<N, 1> f(const Matrix<N, 1> x_prev, const float dT, Utype* u) = 0;

  virtual Matrix<N, N> getF(const Matrix<N, 1> x_prev, const float dT, Utype* u) = 0;

  virtual Matrix<M, 1> h(const Matrix<N, 1> x) = 0;

  virtual Matrix<M, N> getH(const Matrix<N, 1> x) = 0;
};

template<size_t N, size_t M, typename Utype>
class ExtendedKalman
{
public:
  Estimate<N, M, Utype> state;

  EKFModel<N, M, Utype>* model;

  void initialize(const Estimate<N, M, Utype> state, EKFModel<N, M, Utype>* model) {
    this->state = state;
    this->model = model;
  }

  virtual Estimate<N, M, Utype> step(const Matrix<M, 1> z, const float deltaT, Utype* u, const bool skipUpdate)
  {
    //#  predict
    // estimated state
    auto xPriori = this->model->f(this->state.x, deltaT, u);
    // estimated error covariance
    auto F = this->model->getF(this->state.x, deltaT, u);
    //## Ppriori = F.times(self.state.P).times(F.transpose()).plus(self.model->getQ(deltaT))
    auto Ppriori = plus(times(times(F, this->state.P), transpose(F)), this->model->getQ(deltaT));
    this->state.x = xPriori;
    this->state.P = Ppriori;
    this->state.dt = deltaT;
    this->state.u = u;
    if (!skipUpdate)
    {
      //# update
      // difference between expected observation and actual
      auto y = minus(z, this->model->h(xPriori));
      // innovation covariance
      auto H = this->model->getH(xPriori);
      auto S = plus(times(times(H, Ppriori), transpose(H)), this->model->getR());
      // kalman gain
      auto K = times(times(Ppriori, transpose(H)), inverse(S));
      // state estimate updated with measurement
      auto x = plus(xPriori, times(K, y));
      // estimated covariance
      auto P = times(minus(identity<N>(), times(K, H)), Ppriori);
      this->state.x = x;
      this->state.P = P;
    }
    ////this->state = new Estimate(xPriori, Ppriori, this->state.u, deltaT);
    return this->state;
  }

};
