#include "Orientation_Matrix.h"

OrientationMatrix::OrientationMatrix() {
    dlongdx = 0, dlongdy = 0, dlongdz = 0;
    dlatdx  = 0, dlatdy  = 0, dlatdz  = 0;
    dvertdx = 0, dvertdy = 0, dvertdz = 0;
}

OrientationMatrix::~OrientationMatrix() {
    // Nothing
}

float OrientationMatrix::longitudinal(float& x, float& y, float& z) {
    return dlongdx * x + dlongdy * y + dlongdz * z;
}

float OrientationMatrix::lateral     (float& x, float& y, float& z) {
    return dlatdx * x + dlatdy * y + dlatdz * z;
}

float OrientationMatrix::vertical    (float& x, float& y, float& z) {
    return dvertdx * x + dvertdy * y + dvertdz * z;
}

void OrientationMatrix::apply_orientation_adjustment(float& x_in_longitudinal_out, float& y_in_lateral_out, float& z_in_vertical_out) {
    float x = x_in_longitudinal_out; float y = y_in_lateral_out; float z = z_in_vertical_out;
    
    x_in_longitudinal_out = longitudinal(x, y, z);
    y_in_lateral_out      = lateral(x, y, z);
    z_in_vertical_out     = vertical(x, y, z);
}

void OrientationMatrix::update(float x_0, float y_0, float z_0) {
    /*
    dlong/dx  dlong/dy  dlong/dz  =  cos(P)*cos(Y)          -cos(P)*sin(Y)   -sin(P)
    dlat /dx  dlat /dy  dlat /dz  =  sin(Y)*cos(R)           cos(Y)*cos(R)    sin(R)
    dvert/dx  dvert/dy  dvert/dz  =  sin(P)*cos(Y)*cos(R)   -sin(P)*sin(Y)*cos(R)    cos(P)*cos(R)
                                    -sin(P)sin(Y)*sin(R)    -sin(P)*cos(Y)*sin(R)
    cos(P) = z_0 / g_0
    sin(P) = r_0 / g_0
    cos(Y) = x_0 / r_0
    sin(Y) = y_0 / r_0
    r_0 = sqrt(x_0 ^ 2 + y_0 ^ 2)
    g_0 = sqrt(r_0 ^ 2 + z_0 ^ 2)

    dlong/dx  dlong/dy  dlong/dz  =   z_0 * x_0 / (g_0 * r_0)             -z_0 * y_0 / (g_0 * r_0)          -r_0 / g_0
    dlat /dx  dlat /dy  dlat /dz  =   y_0 * cos(R) / r_0                   x_0 * cos(R) / r_0                sin(R)
    dvert/dx  dvert/dy  dvert/dz  =   (x_0*cos(R) - y_0*sin(R)) / g_0    (-y_0*cos(R) -x_0*sin(R)) / g_0     z_0 * cos(R) / g_0

    Assume R is small.
    */
    float r_0 = sqrt(x_0 * x_0 + y_0 * y_0);
    float g_0 = sqrt(r_0 * r_0 + z_0 * z_0);

    dlongdx = z_0 * x_0 / (g_0 * r_0);     dlongdy = -z_0 * y_0 / (g_0 * r_0);     dlongdz = -r_0 / g_0;
    dlatdx  = y_0 / r_0;                   dlatdy  =  x_0 / r_0;                   dlatdz  =  0;
    dvertdx = x_0 / g_0;                   dvertdy = -y_0 / g_0;                   dvertdz =  z_0 / g_0;
}

void OrientationMatrix::update(float initial_orientation[3]) {
    this->update(initial_orientation[0], initial_orientation[1], initial_orientation[2]);
}