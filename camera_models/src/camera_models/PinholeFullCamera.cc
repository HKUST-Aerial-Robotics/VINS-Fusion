#include "camodocal/camera_models/PinholeFullCamera.h"

#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camodocal/gpl/gpl.h"

namespace camodocal
{

PinholeFullCamera::Parameters::Parameters( )
: Camera::Parameters( PINHOLE_FULL )
, m_k1( 0.0 )
, m_k2( 0.0 )
, m_k3( 0.0 )
, m_k4( 0.0 )
, m_k5( 0.0 )
, m_k6( 0.0 )
, m_p1( 0.0 )
, m_p2( 0.0 )
, m_fx( 0.0 )
, m_fy( 0.0 )
, m_cx( 0.0 )
, m_cy( 0.0 )
{
}

PinholeFullCamera::Parameters::Parameters( const std::string& cameraName, //
                                           int w,
                                           int h,
                                           double k1,
                                           double k2,
                                           double k3,
                                           double k4,
                                           double k5,
                                           double k6,
                                           double p1,
                                           double p2,
                                           double fx,
                                           double fy,
                                           double cx,
                                           double cy )
: Camera::Parameters( PINHOLE_FULL, cameraName, w, h )
, m_k1( k1 )
, m_k2( k2 )
, m_k3( k3 )
, m_k4( k4 )
, m_k5( k5 )
, m_k6( k6 )
, m_p1( p1 )
, m_p2( p2 )
, m_fx( fx )
, m_fy( fy )
, m_cx( cx )
, m_cy( cy )
{
}

double&
PinholeFullCamera::Parameters::k1( void )
{
    return m_k1;
}

double&
PinholeFullCamera::Parameters::k2( void )
{
    return m_k2;
}

double&
PinholeFullCamera::Parameters::k3( )
{
    return m_k3;
}

double&
PinholeFullCamera::Parameters::k4( )
{
    return m_k4;
}

double&
PinholeFullCamera::Parameters::k5( )
{
    return m_k5;
}

double&
PinholeFullCamera::Parameters::k6( )
{
    return m_k6;
}

double&
PinholeFullCamera::Parameters::p1( void )
{
    return m_p1;
}

double&
PinholeFullCamera::Parameters::p2( void )
{
    return m_p2;
}

double&
PinholeFullCamera::Parameters::fx( void )
{
    return m_fx;
}

double&
PinholeFullCamera::Parameters::fy( void )
{
    return m_fy;
}

double&
PinholeFullCamera::Parameters::cx( void )
{
    return m_cx;
}

double&
PinholeFullCamera::Parameters::cy( void )
{
    return m_cy;
}

double
PinholeFullCamera::Parameters::k1( void ) const
{
    return m_k1;
}

double
PinholeFullCamera::Parameters::k2( void ) const
{
    return m_k2;
}

double
PinholeFullCamera::Parameters::p1( void ) const
{
    return m_p1;
}

double
PinholeFullCamera::Parameters::p2( void ) const
{
    return m_p2;
}

double
PinholeFullCamera::Parameters::fx( void ) const
{
    return m_fx;
}

double
PinholeFullCamera::Parameters::fy( void ) const
{
    return m_fy;
}

double
PinholeFullCamera::Parameters::cx( void ) const
{
    return m_cx;
}

double
PinholeFullCamera::Parameters::cy( void ) const
{
    return m_cy;
}

double
PinholeFullCamera::Parameters::k3( ) const
{
    return m_k3;
}

double
PinholeFullCamera::Parameters::k4( ) const
{
    return m_k4;
}

double
PinholeFullCamera::Parameters::k5( ) const
{
    return m_k5;
}

double
PinholeFullCamera::Parameters::k6( ) const
{
    return m_k6;
}

bool
PinholeFullCamera::Parameters::readFromYamlFile( const std::string& filename )
{
    cv::FileStorage fs( filename, cv::FileStorage::READ );

    if ( !fs.isOpened( ) )
    {
        return false;
    }

    if ( !fs["model_type"].isNone( ) )
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if ( sModelType.compare( "PINHOLE_FULL" ) != 0 )
        {
            return false;
        }
    }

    m_modelType = PINHOLE_FULL;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth  = static_cast< int >( fs["image_width"] );
    m_imageHeight = static_cast< int >( fs["image_height"] );

    cv::FileNode n = fs["distortion_parameters"];
    m_k1           = static_cast< double >( n["k1"] );
    m_k2           = static_cast< double >( n["k2"] );
    m_k3           = static_cast< double >( n["k3"] );
    m_k4           = static_cast< double >( n["k4"] );
    m_k5           = static_cast< double >( n["k5"] );
    m_k6           = static_cast< double >( n["k6"] );
    m_p1           = static_cast< double >( n["p1"] );
    m_p2           = static_cast< double >( n["p2"] );

    n    = fs["projection_parameters"];
    m_fx = static_cast< double >( n["fx"] );
    m_fy = static_cast< double >( n["fy"] );
    m_cx = static_cast< double >( n["cx"] );
    m_cy = static_cast< double >( n["cy"] );

    return true;
}

void
PinholeFullCamera::Parameters::writeToYamlFile( const std::string& filename ) const
{
    cv::FileStorage fs( filename, cv::FileStorage::WRITE );

    fs << "model_type"
       << "PINHOLE_FULL";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    fs << "distortion_parameters";
    fs << "{"
       << "k1" << m_k1 << "k2" << m_k2 << "k3" << m_k3 << "k4" << m_k4 << "k5" << m_k5
       << "k6" << m_k6 << "p1" << m_p1 << "p2" << m_p2 << "}";

    // projection: fx, fy, cx, cy
    fs << "projection_parameters";
    fs << "{"
       << "fx" << m_fx << "fy" << m_fy << "cx" << m_cx << "cy" << m_cy << "}";

    fs.release( );
}

PinholeFullCamera::Parameters&
PinholeFullCamera::Parameters::operator=( const PinholeFullCamera::Parameters& other )
{
    if ( this != &other )
    {
        m_modelType   = other.m_modelType;
        m_cameraName  = other.m_cameraName;
        m_imageWidth  = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_k1          = other.m_k1;
        m_k2          = other.m_k2;
        m_k3          = other.m_k3;
        m_k4          = other.m_k4;
        m_k5          = other.m_k5;
        m_k6          = other.m_k6;
        m_p1          = other.m_p1;
        m_p2          = other.m_p2;
        m_fx          = other.m_fx;
        m_fy          = other.m_fy;
        m_cx          = other.m_cx;
        m_cy          = other.m_cy;
    }

    return *this;
}

std::ostream&
operator<<( std::ostream& out, const PinholeFullCamera::Parameters& params )
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type "
        << "PINHOLE_FULL" << std::endl;
    out << "   camera_name " << params.m_cameraName << std::endl;
    out << "   image_width " << params.m_imageWidth << std::endl;
    out << "  image_height " << params.m_imageHeight << std::endl;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    out << "Distortion Parameters" << std::endl;
    out << "            k1 " << params.m_k1 << std::endl
        << "            k2 " << params.m_k2 << std::endl
        << "            k3 " << params.m_k3 << std::endl
        << "            k4 " << params.m_k4 << std::endl
        << "            k5 " << params.m_k5 << std::endl
        << "            k6 " << params.m_k6 << std::endl
        << "            p1 " << params.m_p1 << std::endl
        << "            p2 " << params.m_p2 << std::endl;

    // projection: fx, fy, cx, cy
    out << "Projection Parameters" << std::endl;
    out << "            fx " << params.m_fx << std::endl
        << "            fy " << params.m_fy << std::endl
        << "            cx " << params.m_cx << std::endl
        << "            cy " << params.m_cy << std::endl;

    return out;
}

PinholeFullCamera::PinholeFullCamera( )
: m_inv_K11( 1.0 )
, m_inv_K13( 0.0 )
, m_inv_K22( 1.0 )
, m_inv_K23( 0.0 )
, m_noDistortion( true )
{
}

PinholeFullCamera::PinholeFullCamera( const std::string& cameraName,
                                      int imageWidth,
                                      int imageHeight,
                                      double k1,
                                      double k2,
                                      double k3,
                                      double k4,
                                      double k5,
                                      double k6,
                                      double p1,
                                      double p2,
                                      double fx,
                                      double fy,
                                      double cx,
                                      double cy )
: mParameters( cameraName, imageWidth, imageHeight, k1, k2, k3, k4, k5, k6, p1, p2, fx, fy, cx, cy )
{
    if ( ( mParameters.k1( ) == 0.0 ) && ( mParameters.k2( ) == 0.0 )
         && ( mParameters.p1( ) == 0.0 ) && ( mParameters.p2( ) == 0.0 ) )
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx( );
    m_inv_K13 = -mParameters.cx( ) / mParameters.fx( );
    m_inv_K22 = 1.0 / mParameters.fy( );
    m_inv_K23 = -mParameters.cy( ) / mParameters.fy( );
}

PinholeFullCamera::PinholeFullCamera( const PinholeFullCamera::Parameters& params )
: mParameters( params )
{
    if ( ( mParameters.k1( ) == 0.0 ) && ( mParameters.k2( ) == 0.0 )
         && ( mParameters.p1( ) == 0.0 ) && ( mParameters.p2( ) == 0.0 ) )
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx( );
    m_inv_K13 = -mParameters.cx( ) / mParameters.fx( );
    m_inv_K22 = 1.0 / mParameters.fy( );
    m_inv_K23 = -mParameters.cy( ) / mParameters.fy( );
}

Camera::ModelType
PinholeFullCamera::modelType( void ) const
{
    return mParameters.modelType( );
}

const std::string&
PinholeFullCamera::cameraName( void ) const
{
    return mParameters.cameraName( );
}

int
PinholeFullCamera::imageWidth( void ) const
{
    return mParameters.imageWidth( );
}

int
PinholeFullCamera::imageHeight( void ) const
{
    return mParameters.imageHeight( );
}

void
PinholeFullCamera::estimateIntrinsics( const cv::Size& boardSize,
                                       const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                       const std::vector< std::vector< cv::Point2f > >& imagePoints )
{
    // Z. Zhang, A Flexible New Technique for Camera Calibration, PAMI 2000

    Parameters params = getParameters( );

    params.k1( ) = 0.0;
    params.k2( ) = 0.0;
    params.p1( ) = 0.0;
    params.p2( ) = 0.0;

    double cx    = params.imageWidth( ) / 2.0;
    double cy    = params.imageHeight( ) / 2.0;
    params.cx( ) = cx;
    params.cy( ) = cy;

    size_t nImages = imagePoints.size( );

    cv::Mat A( nImages * 2, 2, CV_64F );
    cv::Mat b( nImages * 2, 1, CV_64F );

    for ( size_t i = 0; i < nImages; ++i )
    {
        const std::vector< cv::Point3f >& oPoints = objectPoints.at( i );

        std::vector< cv::Point2f > M( oPoints.size( ) );
        for ( size_t j = 0; j < M.size( ); ++j )
        {
            M.at( j ) = cv::Point2f( oPoints.at( j ).x, oPoints.at( j ).y );
        }

        cv::Mat H = cv::findHomography( M, imagePoints.at( i ) );

        H.at< double >( 0, 0 ) -= H.at< double >( 2, 0 ) * cx;
        H.at< double >( 0, 1 ) -= H.at< double >( 2, 1 ) * cx;
        H.at< double >( 0, 2 ) -= H.at< double >( 2, 2 ) * cx;
        H.at< double >( 1, 0 ) -= H.at< double >( 2, 0 ) * cy;
        H.at< double >( 1, 1 ) -= H.at< double >( 2, 1 ) * cy;
        H.at< double >( 1, 2 ) -= H.at< double >( 2, 2 ) * cy;

        double h[3], v[3], d1[3], d2[3];
        double n[4] = { 0, 0, 0, 0 };

        for ( int j = 0; j < 3; ++j )
        {
            double t0 = H.at< double >( j, 0 );
            double t1 = H.at< double >( j, 1 );
            h[j]      = t0;
            v[j]      = t1;
            d1[j]     = ( t0 + t1 ) * 0.5;
            d2[j]     = ( t0 - t1 ) * 0.5;
            n[0] += t0 * t0;
            n[1] += t1 * t1;
            n[2] += d1[j] * d1[j];
            n[3] += d2[j] * d2[j];
        }

        for ( int j = 0; j < 4; ++j )
        {
            n[j] = 1.0 / sqrt( n[j] );
        }

        for ( int j = 0; j < 3; ++j )
        {
            h[j] *= n[0];
            v[j] *= n[1];
            d1[j] *= n[2];
            d2[j] *= n[3];
        }

        A.at< double >( i * 2, 0 )     = h[0] * v[0];
        A.at< double >( i * 2, 1 )     = h[1] * v[1];
        A.at< double >( i * 2 + 1, 0 ) = d1[0] * d2[0];
        A.at< double >( i * 2 + 1, 1 ) = d1[1] * d2[1];
        b.at< double >( i * 2, 0 )     = -h[2] * v[2];
        b.at< double >( i * 2 + 1, 0 ) = -d1[2] * d2[2];
    }

    cv::Mat f( 2, 1, CV_64F );
    cv::solve( A, b, f, cv::DECOMP_NORMAL | cv::DECOMP_LU );

    params.fx( ) = sqrt( fabs( 1.0 / f.at< double >( 0 ) ) );
    params.fy( ) = sqrt( fabs( 1.0 / f.at< double >( 1 ) ) );

    setParameters( params );
}

/**
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void
PinholeFullCamera::liftSphere( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const
{
    liftProjective( p, P );

    P.normalize( );
}

/**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void
PinholeFullCamera::liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const
{
    double k1 = mParameters.k1( );
    double k2 = mParameters.k2( );
    double k3 = mParameters.k3( );
    double k4 = mParameters.k4( );
    double k5 = mParameters.k5( );
    double k6 = mParameters.k6( );
    double p1 = mParameters.p1( );
    double p2 = mParameters.p2( );

    double fx  = mParameters.fx( );
    double fy  = mParameters.fy( );
    double ifx = 1. / fx;
    double ify = 1. / fy;
    double cx  = mParameters.cx( );
    double cy  = mParameters.cy( );

    // Lift points to normalised plane
    double mx_d = ifx * p( 0 ) + m_inv_K13;
    double my_d = ify * p( 1 ) + m_inv_K23;
    double u    = p( 0 );
    double v    = p( 1 );
    double x    = mx_d;
    double y    = my_d;
    double x0   = x;
    double y0   = y;

    double error = std::numeric_limits< double >::max( );

    int max_cnt   = 8; // 5
    int min_error = 0.01;
    for ( int j = 0;; j++ )
    {
        if ( j > max_cnt )
            break;
        if ( error < min_error )
            break;

        double r2     = x * x + y * y;
        double icdist = ( 1 + ( ( k6 * r2 + k5 ) * r2 + k4 ) * r2 )
                        / ( 1 + ( ( k3 * r2 + k2 ) * r2 + k1 ) * r2 );
        double deltaX = 2 * p1 * x * y + p2 * ( r2 + 2 * x * x );
        double deltaY = p1 * ( r2 + 2 * y * y ) + 2 * p2 * x * y;

        x = ( x0 - deltaX ) * icdist;
        y = ( y0 - deltaY ) * icdist;

        if ( 1 )
        {
            double r4, r6, a1, a2, a3, cdist, icdist2;
            double xd, yd, xd0, yd0;

            r2      = x * x + y * y;
            r4      = r2 * r2;
            r6      = r4 * r2;
            a1      = 2 * x * y;
            a2      = r2 + 2 * x * x;
            a3      = r2 + 2 * y * y;
            cdist   = 1 + k1 * r2 + k2 * r4 + k3 * r6;
            icdist2 = 1. / ( 1 + k4 * r2 + k5 * r4 + k6 * r6 );
            xd0     = x * cdist * icdist2 + p1 * a1 + p2 * a2;
            yd0     = y * cdist * icdist2 + p1 * a3 + p2 * a1;

            double x_proj = xd * fx + cx;
            double y_proj = yd * fy + cy;

            error = sqrt( pow( x_proj - u, 2 ) + pow( y_proj - v, 2 ) );
        }
    }

    P << x, y, 1.0;
}

void
PinholeFullCamera::liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P, float image_scale ) const
{
    Eigen::Vector2d p_tmp = p / image_scale; // p_tmp is without resize, p is with resize
    liftProjective( p_tmp, P );              // p_tmp is without resize
}

/**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
PinholeFullCamera::spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p ) const
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    p_u << P( 0 ) / P( 2 ), P( 1 ) / P( 2 );

    if ( m_noDistortion )
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion( p_u, d_u );
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx( ) * p_d( 0 ) + mParameters.cx( ),
    mParameters.fy( ) * p_d( 1 ) + mParameters.cy( );
}

void
PinholeFullCamera::spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p, float image_scalse ) const
{
    Eigen::Vector2d p_tmp;
    spaceToPlane( P, p_tmp );
    p = p_tmp * image_scalse;
}

/**
 * \brief Project a 3D point to the image plane and calculate Jacobian
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
PinholeFullCamera::spaceToPlane( const Eigen::Vector3d& P,
                                 Eigen::Vector2d& p,
                                 Eigen::Matrix< double, 2, 3 >& J ) const
{
    Eigen::Vector2d p_u, p_d;
    double norm, inv_denom;
    double dxdmx, dydmx, dxdmy, dydmy;

    norm = P.norm( );
    // Project points to the normalised plane
    inv_denom = 1.0 / P( 2 );
    p_u << inv_denom * P( 0 ), inv_denom * P( 1 );

    // Calculate jacobian
    double dudx = inv_denom;
    double dvdx = 0.0;
    double dudy = 0.0;
    double dvdy = inv_denom;
    inv_denom   = -inv_denom * inv_denom;
    double dudz = P( 0 ) * inv_denom;
    double dvdz = P( 1 ) * inv_denom;

    if ( m_noDistortion )
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion( p_u, d_u );
        p_d = p_u + d_u;
    }

    double fx = mParameters.fx( );
    double fy = mParameters.fy( );

    // Make the product of the jacobians
    // and add projection matrix jacobian
    inv_denom = fx * ( dudx * dxdmx + dvdx * dxdmy ); // reuse
    dvdx      = fy * ( dudx * dydmx + dvdx * dydmy );
    dudx      = inv_denom;

    inv_denom = fx * ( dudy * dxdmx + dvdy * dxdmy ); // reuse
    dvdy      = fy * ( dudy * dydmx + dvdy * dydmy );
    dudy      = inv_denom;

    inv_denom = fx * ( dudz * dxdmx + dvdz * dxdmy ); // reuse
    dvdz      = fy * ( dudz * dydmx + dvdz * dydmy );
    dudz      = inv_denom;

    // Apply generalised projection matrix
    p << fx * p_d( 0 ) + mParameters.cx( ), fy * p_d( 1 ) + mParameters.cy( );

    J << dudx, dudy, dudz, dvdx, dvdy, dvdz;
}

/**
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 */
void
PinholeFullCamera::undistToPlane( const Eigen::Vector2d& p_u, Eigen::Vector2d& p ) const
{
    Eigen::Vector2d p_d;

    if ( m_noDistortion )
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion( p_u, d_u );
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx( ) * p_d( 0 ) + mParameters.cx( ),
    mParameters.fy( ) * p_d( 1 ) + mParameters.cy( );
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
PinholeFullCamera::distortion( const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u ) const
{
    // project 3D object point to the image plane
    double k1 = mParameters.k1( );
    double k2 = mParameters.k2( );
    double k3 = mParameters.k3( );
    double k4 = mParameters.k4( );
    double k5 = mParameters.k5( );
    double k6 = mParameters.k6( );
    double p1 = mParameters.p1( );
    double p2 = mParameters.p2( );

    // Transform to model plane
    double x = p_u( 0 );
    double y = p_u( 1 );

    double r2      = x * x + y * y;
    double r4      = r2 * r2;
    double r6      = r4 * r2;
    double a1      = 2 * x * y;
    double a2      = r2 + 2 * x * x;
    double a3      = r2 + 2 * y * y;
    double cdist   = 1 + k1 * r2 + k2 * r4 + k3 * r6;
    double icdist2 = 1. / ( 1 + k4 * r2 + k5 * r4 + k6 * r6 );

    d_u << x * cdist * icdist2 + p1 * a1 + p2 * a2 - x, //
    y * cdist * icdist2 + p1 * a3 + p2 * a1 - y;
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate Jacobian
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
PinholeFullCamera::distortion( const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u, Eigen::Matrix2d& J ) const
{
    // project 3D object point to the image plane
    double k1 = mParameters.k1( );
    double k2 = mParameters.k2( );
    double k3 = mParameters.k3( );
    double k4 = mParameters.k4( );
    double k5 = mParameters.k5( );
    double k6 = mParameters.k6( );
    double p1 = mParameters.p1( );
    double p2 = mParameters.p2( );

    // Transform to model plane
    double x = p_u( 0 );
    double y = p_u( 1 );

    double r2      = x * x + y * y;
    double r4      = r2 * r2;
    double r6      = r4 * r2;
    double a1      = 2 * x * y;
    double a2      = r2 + 2 * x * x;
    double a3      = r2 + 2 * y * y;
    double cdist   = 1 + k1 * r2 + k2 * r4 + k3 * r6;
    double icdist2 = 1. / ( 1 + k4 * r2 + k5 * r4 + k6 * r6 );

    d_u << x * cdist * icdist2 + p1 * a1 + p2 * a2 - x, //
    y * cdist * icdist2 + p1 * a3 + p2 * a1 - y;        //
}

void
PinholeFullCamera::initUndistortMap( cv::Mat& map1, cv::Mat& map2, double fScale ) const
{
    cv::Size imageSize( mParameters.imageWidth( ), mParameters.imageHeight( ) );

    cv::Mat mapX = cv::Mat::zeros( imageSize, CV_32F );
    cv::Mat mapY = cv::Mat::zeros( imageSize, CV_32F );

    for ( int v = 0; v < imageSize.height; ++v )
    {
        for ( int u = 0; u < imageSize.width; ++u )
        {
            double mx_u = m_inv_K11 / fScale * u + m_inv_K13 / fScale;
            double my_u = m_inv_K22 / fScale * v + m_inv_K23 / fScale;

            Eigen::Vector3d P;
            P << mx_u, my_u, 1.0;

            Eigen::Vector2d p;
            spaceToPlane( P, p );

            mapX.at< float >( v, u ) = p( 0 );
            mapY.at< float >( v, u ) = p( 1 );
        }
    }

    cv::convertMaps( mapX, mapY, map1, map2, CV_32FC1, false );
}

cv::Mat
PinholeFullCamera::initUndistortRectifyMap(
cv::Mat& map1, cv::Mat& map2, float fx, float fy, cv::Size imageSize, float cx, float cy, cv::Mat rmat ) const
{
    if ( imageSize == cv::Size( 0, 0 ) )
    {
        imageSize = cv::Size( mParameters.imageWidth( ), mParameters.imageHeight( ) );
    }

    cv::Mat mapX = cv::Mat::zeros( imageSize.height, imageSize.width, CV_32F );
    cv::Mat mapY = cv::Mat::zeros( imageSize.height, imageSize.width, CV_32F );

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen( rmat, R );
    R_inv = R.inverse( );

    // assume no skew
    Eigen::Matrix3f K_rect;

    if ( cx == -1.0f || cy == -1.0f )
    {
        K_rect << fx, 0, imageSize.width / 2, 0, fy, imageSize.height / 2, 0, 0, 1;
    }
    else
    {
        K_rect << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    }

    if ( fx == -1.0f || fy == -1.0f )
    {
        K_rect( 0, 0 ) = mParameters.fx( );
        K_rect( 1, 1 ) = mParameters.fy( );
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse( );

    for ( int v = 0; v < imageSize.height; ++v )
    {
        for ( int u = 0; u < imageSize.width; ++u )
        {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            Eigen::Vector2d p;
            spaceToPlane( uo.cast< double >( ), p );

            mapX.at< float >( v, u ) = p( 0 );
            mapY.at< float >( v, u ) = p( 1 );
        }
    }

    cv::convertMaps( mapX, mapY, map1, map2, CV_32FC1, false );

    cv::Mat K_rect_cv;
    cv::eigen2cv( K_rect, K_rect_cv );
    return K_rect_cv;
}

int
PinholeFullCamera::parameterCount( void ) const
{
    return 12;
}

const PinholeFullCamera::Parameters&
PinholeFullCamera::getParameters( void ) const
{
    return mParameters;
}

void
PinholeFullCamera::setParameters( const PinholeFullCamera::Parameters& parameters )
{
    mParameters = parameters;

    if ( ( mParameters.k1( ) == 0.0 ) && ( mParameters.k2( ) == 0.0 )
         && ( mParameters.p1( ) == 0.0 ) && ( mParameters.p2( ) == 0.0 ) )
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    m_inv_K11 = 1.0 / mParameters.fx( );
    m_inv_K13 = -mParameters.cx( ) / mParameters.fx( );
    m_inv_K22 = 1.0 / mParameters.fy( );
    m_inv_K23 = -mParameters.cy( ) / mParameters.fy( );
}

void
PinholeFullCamera::readParameters( const std::vector< double >& parameterVec )
{
    if ( ( int )parameterVec.size( ) != parameterCount( ) )
    {
        return;
    }

    Parameters params = getParameters( );

    params.k1( ) = parameterVec.at( 0 );
    params.k2( ) = parameterVec.at( 1 );
    params.k3( ) = parameterVec.at( 2 );
    params.k4( ) = parameterVec.at( 3 );
    params.k5( ) = parameterVec.at( 4 );
    params.k6( ) = parameterVec.at( 5 );
    params.p1( ) = parameterVec.at( 6 );
    params.p2( ) = parameterVec.at( 7 );
    params.fx( ) = parameterVec.at( 8 );
    params.fy( ) = parameterVec.at( 9 );
    params.cx( ) = parameterVec.at( 10 );
    params.cy( ) = parameterVec.at( 11 );

    setParameters( params );
}

void
PinholeFullCamera::writeParameters( std::vector< double >& parameterVec ) const
{
    parameterVec.resize( parameterCount( ) );
    parameterVec.at( 0 )  = mParameters.k1( );
    parameterVec.at( 1 )  = mParameters.k2( );
    parameterVec.at( 2 )  = mParameters.k3( );
    parameterVec.at( 3 )  = mParameters.k4( );
    parameterVec.at( 4 )  = mParameters.k5( );
    parameterVec.at( 5 )  = mParameters.k6( );
    parameterVec.at( 6 )  = mParameters.p1( );
    parameterVec.at( 7 )  = mParameters.p2( );
    parameterVec.at( 8 )  = mParameters.fx( );
    parameterVec.at( 9 )  = mParameters.fy( );
    parameterVec.at( 10 ) = mParameters.cx( );
    parameterVec.at( 11 ) = mParameters.cy( );
}

void
PinholeFullCamera::writeParametersToYamlFile( const std::string& filename ) const
{
    mParameters.writeToYamlFile( filename );
}

std::string
PinholeFullCamera::parametersToString( void ) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str( );
}
}
