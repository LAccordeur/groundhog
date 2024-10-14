//
// Created by yangguo on 11/23/22.
//

#ifndef TRAJ_BLOCK_FORMAT_COMMON_UTIL_H
#define TRAJ_BLOCK_FORMAT_COMMON_UTIL_H

#define size_of_attribute(Struct, Attribute) sizeof(((Struct*)0)->Attribute)
#include <time.h>
#include <sys/time.h>
#include <stdint.h>

double average_values(long *array, int array_size);

double average_values_double(double *array, int array_size);

uint64_t get_posix_clock_time ();

#endif //TRAJ_BLOCK_FORMAT_COMMON_UTIL_H
