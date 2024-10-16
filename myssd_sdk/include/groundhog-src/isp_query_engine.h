//
// Created by yangguo on 12/14/22.
//

#ifndef TRAJ_BLOCK_FORMAT_ISP_QUERY_ENGINE_H
#define TRAJ_BLOCK_FORMAT_ISP_QUERY_ENGINE_H

#include <stddef.h>
#include <groundhog/isp_output_buffer.h>
#include <groundhog/isp_descriptor.h>
#include <groundhog/knn_util.h>

void run_id_temporal_query_naive(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_range_query_naive(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_count_query_naive(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_query_naive(struct iovec* iov, size_t nr_segs, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_query_naive_add_mbr_pruning(struct iovec* iov, size_t nr_segs, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_join_query_naive(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_join_query_naive_add_mbr_pruning(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_count_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_id_temporal_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_range_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_query(struct iovec* iov, size_t nr_segs, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_query_heap_wo_pruning(struct iovec* iov, size_t nr_segs, struct knn_max_heap *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_query_heap(struct iovec* iov, size_t nr_segs, struct knn_max_heap *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_query_heap_optimized(struct iovec* iov, size_t nr_segs, struct buffered_knn_max_heap *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_join_query(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_join_query_optimized(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor);

void run_spatio_temporal_knn_join_query_heap_optimized(struct iovec* iov, size_t nr_segs, struct knnjoin_max_heap *output_buffer, struct isp_descriptor *descriptor);


#endif //TRAJ_BLOCK_FORMAT_ISP_QUERY_ENGINE_H
