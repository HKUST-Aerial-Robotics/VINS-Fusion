/**
 * \file Geocentric.hpp
 * \brief Header for GeographicLib::Geocentric class
 *
 * Copyright (c) Charles Karney (2008-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEOCENTRIC_HPP)
#define GEOGRAPHICLIB_GEOCENTRIC_HPP 1

#include <vector>
#include "Constants.hpp"

namespace GeographicLib {

  /**
   * \brief %Geocentric coordinates
   *
   * Convert between geodetic coordinates latitude = \e lat, longitude = \e
   * lon, height = \e h (measured vertically from the surface of the ellipsoid)
   * to geocentric coordinates (\e X, \e Y, \e Z).  The origin of geocentric
   * coordinates is at the center of the earth.  The \e Z axis goes thru the
   * north pole, \e lat = 90&deg;.  The \e X axis goes thru \e lat = 0,
   * \e lon = 0.  %Geocentric coordinates are also known as earth centered,
   * earth fixed (ECEF) coordinates.
   *
   * The conversion from geographic to geocentric coordinates is
   * straightforward.  For the reverse transformation we use
   * - H. Vermeille,
   *   <a href="https://doi.org/10.1007/s00190-002-0273-6"> Direct
   *   transformation from geocentric coordinates to geodetic coordinates</a>,
   *   J. Geodesy 76, 451--454 (2002).
   * .
   * Several changes have been made to ensure that the method returns accurate
   * results for all finite inputs (even if \e h is infinite).  The changes are
   * described in Appendix B of
   * - C. F. F. Karney,
   *   <a href="https://arxiv.org/abs/1102.1215v1">Geodesics
   *   on an ellipsoid of revolution</a>,
   *   Feb. 2011;
   *   preprint
   *   <a href="https://arxiv.org/abs/1102.1215v1">arxiv:1102.1215v1</a>.
   * .
   * Vermeille similarly updated his method in
   * - H. Vermeille,
   *   <a href="https://doi.org/10.1007/s00190-010-0419-x">
   *   An analytical method to transform geocentric into
   *   geodetic coordinates</a>, J. Geodesy 85, 105--117 (2011).
   * .
   * See \ref geocentric for more information.
   *
   * The errors in these routines are close to round-off.  Specifically, for
   * points within 5000 km of the surface of the ellipsoid (either inside or
   * outside the ellipsoid), the error is bounded by 7 nm (7 nanometers) for
   * the WGS84 ellipsoid.  See \ref geocentric for further information on the
   * errors.
   *
   * Example of use:
   * \include example-Geocentric.cpp
   *
   * <a href="CartConvert.1.html">CartConvert</a> is a command-line utility
   * providing access to the functionality of Geocentric and LocalCartesian.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT Geocentric {
  private:
    typedef Math::real real;
    friend class LocalCartesian;
    friend class MagneticCircle; // MagneticCircle uses Rotation
    friend class MagneticModel;  // MagneticModel uses IntForward
    friend class GravityCircle;  // GravityCircle uses Rotation
    friend class GravityModel;   // GravityModel uses IntForward
    friend class NormalGravity;  // NormalGravity uses IntForward
    static const size_t dim_ = 3;
    static const size_t dim2_ = dim_ * dim_;
    real _a, _f, _e2, _e2m, _e2a, _e4a, _maxrad;
    static void Rotation(real sphi, real cphi, real slam, real clam,
                         real M[dim2_]);
    static void Rotate(real M[dim2_], real x, real y, real z,
                       real& X, real& Y, real& Z) {
      // Perform [X,Y,Z]^t = M.[x,y,z]^t
      // (typically local cartesian to geocentric)
      X = M[0] * x + M[1] * y + M[2] * z;
      Y = M[3] * x + M[4] * y + M[5] * z;
      Z = M[6] * x + M[7] * y + M[8] * z;
    }
    static void Unrotate(real M[dim2_], real X, real Y, real Z,
                         real& x, real& y, real& z)  {
      // Perform [x,y,z]^t = M^t.[X,Y,Z]^t
      // (typically geocentric to local cartesian)
      x = M[0] * X + M[3] * Y + M[6] * Z;
      y = M[1] * X + M[4] * Y + M[7] * Z;
      z = M[2] * X + M[5] * Y + M[8] * Z;
    }
    void IntForward(real lat, real lon, real h, real& X, real& Y, real& Z,
                    real M[dim2_]) const;
    void IntReverse(real X, real Y, real Z, real& lat, real& lon, real& h,
                    real M[dim2_]) const;

  public:

    /**
     * Constructor for a ellipsoid with
     *
     * @param[in] a equatorial radius (meters).
     * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
     *   Negative \e f gives a prolate ellipsoid.
     * @exception GeographicErr if \e a or (1 &minus; \e f) \e a is not
     *   positive.
     **********************************************************************/
    Geocentric(real a, real f);

    /**
     * A default constructor (for use by NormalGravity).
     **********************************************************************/
    Geocentric() : _a(-1) {}

    /**
     * Convert from geodetic to geocentric coordinates.
     *
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[in] h height of point above the ellipsoid (meters).
     * @param[out] X geocentric coordinate (meters).
     * @param[out] Y geocentric coordinate (meters).
     * @param[out] Z geocentric coordinate (meters).
     *
     * \e lat should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    void Forward(real lat, real lon, real h, real& X, real& Y, real& Z)
      const {
      if (Init())
        IntForward(lat, lon, h, X, Y, Z, NULL);
    }

    /**
     * Convert from geodetic to geocentric coordinates and return rotation
     * matrix.
     *
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[in] h height of point above the ellipsoid (meters).
     * @param[out] X geocentric coordinate (meters).
     * @param[out] Y geocentric coordinate (meters).
     * @param[out] Z geocentric coordinate (meters).
     * @param[out] M if the length of the vector is 9, fill with the rotation
     *   matrix in row-major order.
     *
     * Let \e v be a unit vector located at (\e lat, \e lon, \e h).  We can
     * express \e v as \e column vectors in one of two ways
     * - in east, north, up coordinates (where the components are relative to a
     *   local coordinate system at (\e lat, \e lon, \e h)); call this
     *   representation \e v1.
     * - in geocentric \e X, \e Y, \e Z coordinates; call this representation
     *   \e v0.
     * .
     * Then we have \e v0 = \e M &sdot; \e v1.
     **********************************************************************/
    void Forward(real lat, real lon, real h, real& X, real& Y, real& Z,
                 std::vector<real>& M)
      const {
      if (!Init())
        return;
      if (M.end() == M.begin() + dim2_) {
        real t[dim2_];
        IntForward(lat, lon, h, X, Y, Z, t);
        std::copy(t, t + dim2_, M.begin());
      } else
        IntForward(lat, lon, h, X, Y, Z, NULL);
    }

    /**
     * Convert from geocentric to geodetic to coordinates.
     *
     * @param[in] X geocentric coordinate (meters).
     * @param[in] Y geocentric coordinate (meters).
     * @param[in] Z geocentric coordinate (meters).
     * @param[out] lat latitude of point (degrees).
     * @param[out] lon longitude of point (degrees).
     * @param[out] h height of point above the ellipsoid (meters).
     *
     * In general there are multiple solutions and the result which maximizes
     * \e h is returned.  If there are still multiple solutions with different
     * latitudes (applies only if \e Z = 0), then the solution with \e lat > 0
     * is returned.  If there are still multiple solutions with different
     * longitudes (applies only if \e X = \e Y = 0) then \e lon = 0 is
     * returned.  The value of \e h returned satisfies \e h &ge; &minus; \e a
     * (1 &minus; <i>e</i><sup>2</sup>) / sqrt(1 &minus; <i>e</i><sup>2</sup>
     * sin<sup>2</sup>\e lat).  The value of \e lon returned is in the range
     * [&minus;180&deg;, 180&deg;].
     **********************************************************************/
    void Reverse(real X, real Y, real Z, real& lat, real& lon, real& h)
      const {
      if (Init())
        IntReverse(X, Y, Z, lat, lon, h, NULL);
    }

    /**
     * Convert from geocentric to geodetic to coordinates.
     *
     * @param[in] X geocentric coordinate (meters).
     * @param[in] Y geocentric coordinate (meters).
     * @param[in] Z geocentric coordinate (meters).
     * @param[out] lat latitude of point (degrees).
     * @param[out] lon longitude of point (degrees).
     * @param[out] h height of point above the ellipsoid (meters).
     * @param[out] M if the length of the vector is 9, fill with the rotation
     *   matrix in row-major order.
     *
     * Let \e v be a unit vector located at (\e lat, \e lon, \e h).  We can
     * express \e v as \e column vectors in one of two ways
     * - in east, north, up coordinates (where the components are relative to a
     *   local coordinate system at (\e lat, \e lon, \e h)); call this
     *   representation \e v1.
     * - in geocentric \e X, \e Y, \e Z coordinates; call this representation
     *   \e v0.
     * .
     * Then we have \e v1 = <i>M</i><sup>T</sup> &sdot; \e v0, where
     * <i>M</i><sup>T</sup> is the transpose of \e M.
     **********************************************************************/
    void Reverse(real X, real Y, real Z, real& lat, real& lon, real& h,
                 std::vector<real>& M)
      const {
      if (!Init())
        return;
      if (M.end() == M.begin() + dim2_) {
        real t[dim2_];
        IntReverse(X, Y, Z, lat, lon, h, t);
        std::copy(t, t + dim2_, M.begin());
      } else
        IntReverse(X, Y, Z, lat, lon, h, NULL);
    }

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return true if the object has been initialized.
     **********************************************************************/
    bool Init() const { return _a > 0; }
    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const
    { return Init() ? _a : Math::NaN(); }

    /**
     * @return \e f the  flattening of the ellipsoid.  This is the
     *   value used in the constructor.
     **********************************************************************/
    Math::real Flattening() const
    { return Init() ? _f : Math::NaN(); }
    ///@}

    /**
     * A global instantiation of Geocentric with the parameters for the WGS84
     * ellipsoid.
     **********************************************************************/
    static const Geocentric& WGS84();
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GEOCENTRIC_HPP
