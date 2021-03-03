#include "wgs84.h"
#include <math.h>
#include <iostream>

#define SCALE_FACTOR 0.58
namespace LIMoSim
{

WGS84::WGS84()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

double WGS84::computeDistance(const Vector3d &_from, const Vector3d &_to)
{
    double n = (_to-_from).norm();
    double distance_m = 0;

    double lat0 = Vector3d::toRad(_from.y);
    double lon0 = Vector3d::toRad(_from.x);
    double lat1 = Vector3d::toRad(_to.y);
    double lon1 = Vector3d::toRad(_to.x);

    double sinTerm = sin(lat0)*sin(lat1);
    double cosTerm = cos(lat0)*cos(lat1)*cos(lon1-lon0);

    distance_m = acos(sinTerm+cosTerm)*40000/360*1000;    // map to earth
    distance_m = Vector3d::toGrad(distance_m);

    return distance_m * SCALE_FACTOR;
}

Vector3d WGS84::computeOffset(const Vector3d &_position, const Vector3d &_origin)
{
    double dX = computeDistance(_origin, Vector3d(_position.x, _origin.y));
    double dY = computeDistance(_origin, Vector3d(_origin.x, _position.y));
    double dZ = 0;

    // adjust the origin alignment if needed
    if(_position.x<_origin.x)
        dX *= -1;
    if(_position.y<_origin.y)
        dY *= -1;
    if(_position.z<_origin.z)
        dZ *= -1;
    // std::cout << "(" << dX << "," << dY << "," << dZ << ")\n";
    // static double max = 0.0;
    // max = std::max(max, dX);

    // static double min = 0.0;
    // min = std::min(min, dX);

    // static double ymax = 0.0;
    // ymax = std::max(ymax, dY);

    // static double ymin = 0.0;
    // ymin = std::min(ymin, dY);
    // std::cout << "x: " << max - min << "\ty:" << ymax-ymin << "\n";
    return Vector3d(dX, dY, dZ);
}

}
