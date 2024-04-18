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