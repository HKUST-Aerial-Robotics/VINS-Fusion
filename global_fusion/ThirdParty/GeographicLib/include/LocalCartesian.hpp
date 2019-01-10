/**
 * \file LocalCartesian.hpp
 * \brief Header for GeographicLib::LocalCartesian class
 *
 * Copyright (c) Charles Karney (2008-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_LOCALCARTESIAN_HPP)
#define GEOGRAPHICLIB_LOCALCARTESIAN_HPP 1

#include "Geocentric.hpp"
#include "Constants.hpp"

namespace GeographicLib {

  /**
   * \brief Local cartesian coordinates
   *
   * Convert between geodetic coordinates latitude = \e lat, longitude = \e
   * lon, height = \e h (measured vertically from the surface of the ellipsoid)
   * to local cartesian coordinates (\e x, \e y, \e z).  The origin of local
   * cartesian coordinate system is at \e lat = \e lat0, \e lon = \e lon0, \e h
   * = \e h0. The \e z axis is normal to the ellipsoid; the \e y axis points
   * due north.  The plane \e z = - \e h0 is tangent to the ellipsoid.
   *
   * The conversions all take place via geocentric coordinates using a
   * Geocentric object (by default Geocentric::WGS84()).
   *
   * Example of use:
   * \include example-LocalCartesian.cpp
   *
   * <a href="CartConvert.1.html">CartConvert</a> is a command-line utility
   * providing access to the functionality of Geocentric and LocalCartesian.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT LocalCartesian {
  private:
    typedef Math::real real;
    static const size_t dim_ = 3;
    static const size_t dim2_ = dim_ * dim_;
    Geocentric _earth;
    real _lat0, _lon0, _h0;
    real _x0, _y0, _z0, _r[dim2_];
    void IntForward(real lat, real lon, real h, real& x, real& y, real& z,
                    real M[dim2_]) const;
    void IntReverse(real x, real y, real z, real& lat, real& lon, real& h,
                    real M[dim2_]) const;
    void MatrixMultiply(real M[dim2_]) const;
  public:

    /**
     * Constructor setting the origin.
     *
     * @param[in] lat0 latitude at origin (degrees).
     * @param[in] lon0 longitude at origin (degrees).
     * @param[in] h0 height above ellipsoid at origin (meters); default 0.
     * @param[in] earth Geocentric object for the transformation; default
     *   Geocentric::WGS84().
     *
     * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    LocalCartesian(real lat0, real lon0, real h0 = 0,
                   const Geocentric& earth = Geocentric::WGS84())
      : _earth(earth)
    { Reset(lat0, lon0, h0); }

    /**
     * Default constructor.
     *
     * @param[in] earth Geocentric object for the transformation; default
     *   Geocentric::WGS84().
     *
     * Sets \e lat0 = 0, \e lon0 = 0, \e h0 = 0.
     **********************************************************************/
    explicit LocalCartesian(const Geocentric& earth = Geocentric::WGS84())
      : _earth(earth)
    { Reset(real(0), real(0), real(0)); }

    /**
     * Reset the origin.
     *
     * @param[in] lat0 latitude at origin (degrees).
     * @param[in] lon0 longitude at origin (degrees).
     * @param[in] h0 height above ellipsoid at origin (meters); default 0.
     *
     * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    void Reset(real lat0, real lon0, real h0 = 0);

    /**
     * Convert from geodetic to local cartesian coordinates.
     *
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[in] h height of point above the ellipsoid (meters).
     * @param[out] x local cartesian coordinate (meters).
     * @param[out] y local cartesian coordinate (meters).
     * @param[out] z local cartesian coordinate (meters).
     *
     * \e lat should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    void Forward(real lat, real lon, real h, real& x, real& y, real& z)
      const {
      IntForward(lat, lon, h, x, y, z, NULL);
    }

    /**
     * Convert from geodetic to local cartesian coordinates and return rotation
     * matrix.
     *
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[in] h height of point above the ellipsoid (meters).
     * @param[out] x local cartesian coordinate (meters).
     * @param[out] y local cartesian coordinate (meters).
     * @param[out] z local cartesian coordinate (meters).
     * @param[out] M if the length of the vector is 9, fill with the rotation
     *   matrix in row-major order.
     *
     * \e lat should be in the range [&minus;90&deg;, 90&deg;].
     *
     * Let \e v be a unit vector located at (\e lat, \e lon, \e h).  We can
     * express \e v as \e column vectors in one of two ways
     * - in east, north, up coordinates (where the components are relative to a
     *   local coordinate system at (\e lat, \e lon, \e h)); call this
     *   representation \e v1.
     * - in \e x, \e y, \e z coordinates (where the components are relative to
     *   the local coordinate system at (\e lat0, \e lon0, \e h0)); call this
     *   representation \e v0.
     * .
     * Then we have \e v0 = \e M &sdot; \e v1.
     **********************************************************************/
    void Forward(real lat, real lon, real h, real& x, real& y, real& z,
                 std::vector<real>& M)
      const  {
      if (M.end() == M.begin() + dim2_) {
        real t[dim2_];
        IntForward(lat, lon, h, x, y, z, t);
        std::copy(t, t + dim2_, M.begin());
      } else
        IntForward(lat, lon, h, x, y, z, NULL);
    }

    /**
     * Convert from local cartesian to geodetic coordinates.
     *
     * @param[in] x local cartesian coordinate (meters).
     * @param[in] y local cartesian coordinate (meters).
     * @param[in] z local cartesian coordinate (meters).
     * @param[out] lat latitude of point (degrees).
     * @param[out] lon longitude of point (degrees).
     * @param[out] h height of point above the ellipsoid (meters).
     *
     * The value of \e lon returned is in the range [&minus;180&deg;,
     * 180&deg;].
     **********************************************************************/
    void Reverse(real x, real y, real z, real& lat, real& lon, real& h)
      const {
      IntReverse(x, y, z, lat, lon, h, NULL);
    }

    /**
     * Convert from local cartesian to geodetic coordinates and return rotation
     * matrix.
     *
     * @param[in] x local cartesian coordinate (meters).
     * @param[in] y local cartesian coordinate (meters).
     * @param[in] z local cartesian coordinate (meters).
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
     * - in \e x, \e y, \e z coordinates (where the components are relative to
     *   the local coordinate system at (\e lat0, \e lon0, \e h0)); call this
     *   representation \e v0.
     * .
     * Then we have \e v1 = <i>M</i><sup>T</sup> &sdot; \e v0, where
     * <i>M</i><sup>T</sup> is the transpose of \e M.
     **********************************************************************/
    void Reverse(real x, real y, real z, real& lat, real& lon, real& h,
                 std::vector<real>& M)
      const {
      if (M.end() == M.begin() + dim2_) {
        real t[dim2_];
        IntReverse(x, y, z, lat, lon, h, t);
        std::copy(t, t + dim2_, M.begin());
      } else
        IntReverse(x, y, z, lat, lon, h, NULL);
    }

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return latitude of the origin (degrees).
     **********************************************************************/
    Math::real LatitudeOrigin() const { return _lat0; }

    /**
     * @return longitude of the origin (degrees).
     **********************************************************************/
    Math::real LongitudeOrigin() const { return _lon0; }

    /**
     * @return height of the origin (meters).
     **********************************************************************/
    Math::real HeightOrigin() const { return _h0; }

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value of \e a inherited from the Geocentric object used in the
     *   constructor.
     **********************************************************************/
    Math::real MajorRadius() const { return _earth.MajorRadius(); }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the Geocentric object used in the constructor.
     **********************************************************************/
    Math::real Flattening() const { return _earth.Flattening(); }
    ///@}

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_LOCALCARTESIAN_HPP
