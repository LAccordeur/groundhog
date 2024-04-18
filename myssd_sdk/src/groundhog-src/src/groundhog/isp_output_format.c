//
// Created by yangguo on 12/15/22.
//
#include "groundhog/isp_output_format.h"
#include "string.h"

int parse_points_num_from_output_buffer_page(void *data_page) {
    int count = 0;
    memcpy(&count, data_page, 4);
    return count;
}

void deserialize_output_buffer_page(void *data_page, struct traj_point **points, int points_num) {
    int offset_base = 4;
    int point_data_size = get_traj_point_size();
    for (int i = 0; i < points_num; i++) {

        deserialize_traj_point(data_page + offset_base + i * point_data_size, points[i]);
    }
}