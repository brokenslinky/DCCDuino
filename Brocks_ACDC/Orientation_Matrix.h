#include "math.h"

// Structure for translating sensor coordinates into vehicle coordinates
struct OrientationMatrix
{
    // Conversion coefficients
    float dlongdx, dlongdy, dlongdz;
    float dlatdx,  dlatdy,  dlatdz;
    float dvertdx, dvertdy, dvertdz;

    OrientationMatrix();
    ~OrientationMatrix();

    // Update the orientation_matrix based on the initial_orientation vector
    void update(float x_0, float y_0, float z_0);
    void update(float initial_orientation[3]);

    // Longitudinal component from sensor data
    float longitudinal(float& x, float& y, float& z);
    // Lateral component from sensor data
    float lateral     (float& x, float& y, float& z);
    // Vertical component from sensor data
    float vertical    (float& x, float& y, float& z);

    // Adjust the adjustment to shift from sensor coordinates to vehicle coordinates
    void apply_orientation_adjustment(float& x_in_longitudinal_out, float& y_in_lateral_out, float& z_in_vertical_out);
};