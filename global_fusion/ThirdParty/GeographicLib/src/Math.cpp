/**
 * \file Math.cpp
 * \brief Implementation for GeographicLib::Math class
 *
 * Copyright (c) Charles Karney (2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include "Math.hpp"

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (disable: 4127)
#endif

namespace GeographicLib {

  using namespace std;

  template<typename T> T Math::eatanhe(T x, T es)  {
    return es > T(0) ? es * atanh(es * x) : -es * atan(es * x);
  }

  template<typename T> T Math::taupf(T tau, T es) {
    T tau1 = hypot(T(1), tau),
      sig = sinh( eatanhe(tau / tau1, es ) );
    return hypot(T(1), sig) * tau - sig * tau1;
  }

  template<typename T> T Math::tauf(T taup, T es) {
    static const int numit = 5;
    static const T tol = sqrt(numeric_limits<T>::epsilon()) / T(10);
    T e2m = T(1) - sq(es),
      // To lowest order in e^2, taup = (1 - e^2) * tau = _e2m * tau; so use
      // tau = taup/_e2m as a starting guess.  (This starting guess is the
      // geocentric latitude which, to first order in the flattening, is equal
      // to the conformal latitude.)  Only 1 iteration is needed for |lat| <
      // 3.35 deg, otherwise 2 iterations are needed.  If, instead, tau = taup
      // is used the mean number of iterations increases to 1.99 (2 iterations
      // are needed except near tau = 0).
      tau = taup/e2m,
      stol = tol * max(T(1), abs(taup));
    // min iterations = 1, max iterations = 2; mean = 1.94
    for (int i = 0; i < numit || GEOGRAPHICLIB_PANIC; ++i) {
      T taupa = taupf(tau, es),
        dtau = (taup - taupa) * (1 + e2m * sq(tau)) /
        ( e2m * hypot(T(1), tau) * hypot(T(1), taupa) );
      tau += dtau;
      if (!(abs(dtau) >= stol))
        break;
    }
    return tau;
  }

  /// \cond SKIP
  // Instantiate
  template Math::real Math::eatanhe<Math::real>(Math::real, Math::real);
  template Math::real Math::taupf<Math::real>(Math::real, Math::real);
  template Math::real Math::tauf<Math::real>(Math::real, Math::real);
  /// \endcond

} // namespace GeographicLib
