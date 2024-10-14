//
// Created by yangguo on 24-4-10.
//

#include "groundhog/common_util.h"

double average_values(long *array, int array_size) {
    double sum = 0;
    for (int i = 0; i < array_size; i++) {
        sum += array[i];
    }
    return sum / array_size;
}

double average_values_double(double *array, int array_size) {
    double sum = 0;
    for (int i = 0; i < array_size; i++) {
        sum += array[i];
    }
    return sum / array_size;
}

uint64_t get_posix_clock_time ()
{
    struct timespec ts;

    if (clock_gettime (CLOCK_MONOTONIC, &ts) == 0)
        return (uint64_t) (ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
    else
        return 0;
}