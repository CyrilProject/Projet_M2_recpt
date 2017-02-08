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
#ifndef ACC_GYR_COMPUTATION_H
#define ACC_GYR_COMPUTATION_H

#include <stdint.h>

double complementary_filter(double acc_angle, double gyr_angle);

double alpha_angle_acc(const int16_t acc_x, const int16_t acc_y, const int16_t acc_z);

double yaw_angle_gyr(double previous_angle, int16_t gyr_z, double time);

#endif

