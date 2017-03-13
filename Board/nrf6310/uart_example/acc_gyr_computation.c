/* 
 *
 * The information contained herein is property of ESME Sudria.
 * 
 * 
 *
 * Licensees are granted free for ROOl'In, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
#include "acc_gyr_computation.h"
#include <math.h>


double complementary_filter(double acc_angle, double gyr_angle, double previous_angle, float dt)
{
  return (0.98 * (previous_angle + gyr_angle * dt) + 0.02 * acc_angle);
}

double alpha_angle_acc(const int16_t acc_x, const int16_t acc_y, const int16_t acc_z)
{
  return (180.0 * acos(acc_z/(sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2))))/3.14159265);
}

double yaw_angle_gyr(double previous_angle, int16_t gyr_z, double dt)
{
  return (previous_angle  + (gyr_z)) * dt;
}

