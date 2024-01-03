//
// Created by yangguo on 12/14/22.
//

#ifndef TRAJ_BLOCK_FORMAT_ISP_OUTPUT_BUFFER_H
#define TRAJ_BLOCK_FORMAT_ISP_OUTPUT_BUFFER_H

#include <stddef.h>
#include <iov_iter.h>
#include "groundhog/traj_block_format.h"

#define ISP_BUFFER_PAGE_SIZE 0x1000

struct isp_output_buffer {
    struct iovec *iov;
    size_t iov_capacity;    // the number of allocated iovec
    size_t current_iov_index;
    size_t current_iov_offset;
    int current_tuple_count;
    size_t used_bytes_count;
};

void allocate_isp_output_buffer(struct isp_output_buffer *output_buffer, struct iovec *iov, size_t capacity);

void free_isp_output_buffer(struct isp_output_buffer *output_buffer);

size_t put_point_data_to_isp_output_buffer(struct isp_output_buffer *output_buffer, const void *data, size_t data_bytes_count);

void print_isp_output_buffer_info(struct isp_output_buffer *output_buffer);

/*int parse_points_num_from_output_buffer_page(void *data_page);

void deserialize_output_buffer_page(void *data_page, struct traj_point **points, int points_num);*/

void format_the_last_isp_buffer_page(struct isp_output_buffer *output_buffer);

#endif //TRAJ_BLOCK_FORMAT_ISP_OUTPUT_BUFFER_H
