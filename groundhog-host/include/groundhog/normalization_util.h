//
// Created by yangguo on 11/10/22.
//

#ifndef TRAJ_BLOCK_FORMAT_NORMALIZATION_UTIL_H
#define TRAJ_BLOCK_FORMAT_NORMALIZATION_UTIL_H

#define PRECISION (24)

int normalize_longitude(double lon);

int normalize_latitude(double lat);

int normalize_datetime(char* datetime);

long generate_zcurve_value(int x, int y, int z);

long generate_zcurve_value_spatial(int x, int y);

#endif //TRAJ_BLOCK_FORMAT_NORMALIZATION_UTIL_H
