//
// Created by yangguo on 12/14/22.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/isp_output_buffer.h"
#include "groundhog/traj_block_format.h"
#include "groundhog/isp_output_format.h"
}

TEST(isp_output_buffer, allocate) {
    struct isp_output_buffer buffer;
    int capacity = 16;
    struct iovec *iov = (struct iovec *) malloc(sizeof(struct iovec) * capacity);

    allocate_isp_output_buffer(&buffer, iov, capacity);

    print_isp_output_buffer_info(&buffer);
    free_isp_output_buffer(&buffer);
    free(iov);
}

TEST(isp_output_buffer, put) {
    struct isp_output_buffer buffer;
    int capacity = 16;
    struct iovec *iov = (struct iovec *) malloc(sizeof(struct iovec) * capacity);

    allocate_isp_output_buffer(&buffer, iov, capacity);

    struct traj_point point_s = {.oid = 11,.timestamp_sec = 11, .normalized_longitude = 11, .normalized_latitude = 11};
    struct traj_point point = {.oid = 12,.timestamp_sec = 12, .normalized_longitude = 32, .normalized_latitude = 54};
    struct traj_point point_e = {.oid = 22,.timestamp_sec = 22, .normalized_longitude = 22, .normalized_latitude = 22};
    void *point_data = malloc(get_traj_point_size());
    serialize_traj_point(&point, point_data);

    printf("trajectory_point size: %d\n", get_traj_point_size());
    for (int i = 0; i < 1000; i++) {
        if (i == 0) {
            serialize_traj_point(&point_s, point_data);
        } else if (i == 999) {
            serialize_traj_point(&point_e, point_data);
        } else {
            serialize_traj_point(&point, point_data);
        }
        put_point_data_to_isp_output_buffer(&buffer, point_data, get_traj_point_size());
    }
    format_the_last_isp_buffer_page(&buffer);

    for (int i = 0; i <= buffer.current_iov_index; i++) {
        void *data = buffer.iov[i].iov_base;

        int count = parse_points_num_from_output_buffer_page(data);
        printf("points count: %d\n", count);
        struct traj_point **points = allocate_points_memory(count);
        deserialize_output_buffer_page(data, points, count);
        print_traj_points(points, count);
    }


    free_isp_output_buffer(&buffer);
    free(iov);
}