//
// Created by yangguo on 12/14/22.
//

#ifndef TRAJ_BLOCK_FORMAT_ISP_QUERY_ENGINE_H
#define TRAJ_BLOCK_FORMAT_ISP_QUERY_ENGINE_H

#include <stddef.h>
#include <groundhog/isp_output_buffer.h>
#include <groundhog/isp_descriptor.h>

void run_id_temporal_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_range_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

#endif //TRAJ_BLOCK_FORMAT_ISP_QUERY_ENGINE_H
