/**
 * \file LocalCartesian.cpp
 * \brief Implementation for GeographicLib::LocalCartesian class
 *
 * Copyright (c) Charles Karney (2008-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include "LocalCartesian.hpp"

namespace GeographicLib {

  using namespace std;

  void LocalCartesian::Reset(real lat0, real lon0, real h0) {
    _lat0 = Math::LatFix(lat0);
    _lon0 = Math::AngNormalize(lon0);
    _h0 = h0;
    _earth.Forward(_lat0, _lon0, _h0, _x0, _y0, _z0);
    real sphi, cphi, slam, clam;
    Math::sincosd(_lat0, sphi, cphi);
    Math::sincosd(_lon0, slam, clam);
    Geocentric::Rotation(sphi, cphi, slam, clam, _r);
  }

  void LocalCartesian::MatrixMultiply(real M[dim2_]) const {
    // M = r' . M
    real t[dim2_];
    copy(M, M + dim2_, t);
    for (size_t i = 0; i < dim2_; ++i) {
      size_t row = i / dim_, col = i % dim_;
      M[i] = _r[row] * t[col] + _r[row+3] * t[col+3] + _r[row+6] * t[col+6];
    }
  }

  void LocalCartesian::IntForward(real lat, real lon, real h,
                                  real& x, real& y, real& z,
                                  real M[dim2_]) const {
    real xc, yc, zc;
    _earth.IntForward(lat, lon, h, xc, yc, zc, M);
    xc -= _x0; yc -= _y0; zc -= _z0;
    x = _r[0] * xc + _r[3] * yc + _r[6] * zc;
    y = _r[1] * xc + _r[4] * yc + _r[7] * zc;
    z = _r[2] * xc + _r[5] * yc + _r[8] * zc;
    if (M)
      MatrixMultiply(M);
  }

  void LocalCartesian::IntReverse(real x, real y, real z,
                                  real& lat, real& lon, real& h,
                                  real M[dim2_]) const {
    real
      xc = _x0 + _r[0] * x + _r[1] * y + _r[2] * z,
      yc = _y0 + _r[3] * x + _r[4] * y + _r[5] * z,
      zc = _z0 + _r[6] * x + _r[7] * y + _r[8] * z;
    _earth.IntReverse(xc, yc, zc, lat, lon, h, M);
    if (M)
      MatrixMultiply(M);
  }

} // namespace GeographicLib
