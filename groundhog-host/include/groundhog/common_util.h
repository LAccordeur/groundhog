//
// Created by yangguo on 11/23/22.
//

#ifndef TRAJ_BLOCK_FORMAT_COMMON_UTIL_H
#define TRAJ_BLOCK_FORMAT_COMMON_UTIL_H

#define size_of_attribute(Struct, Attribute) sizeof(((Struct*)0)->Attribute)

double average_values(long *array, int array_size);

#endif //TRAJ_BLOCK_FORMAT_COMMON_UTIL_H
