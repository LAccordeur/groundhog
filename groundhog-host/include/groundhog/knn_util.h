//
// Created by yangguo on 24-4-2.
//
/**
 * The optimization in knn queue. We use the two extra data structure: low_half buffer and high_half buffer to buffer some result items without sorting them
 * This optimization is affected by the parameter k and query types
 * higher k values can benefit more
 * knn query benefit more than knn join. This is because in a knn join query, more time is spent on the probe and find on two datasets.
 */

#ifndef TRAJ_BLOCK_FORMAT_KNN_UTIL_H
#define TRAJ_BLOCK_FORMAT_KNN_UTIL_H

#include <stdbool.h>
#include "groundhog/traj_block_format.h"

struct runtime_statistics {
    int sort_func_call_num;
    int add_item_to_buffer_call_num;
    int combine_and_sort_call_num;
    int buffer_low_half_full_num;
    int buffer_high_hal_full_num;
    int checked_segment_num;
};

struct heap_runtime_statistics {
    int heap_sawp_operation_count;
    int add_to_heap_item_count;
    int add_to_buffer_item_count;
    int non_discard_count;
    int discard_count;
};

struct result_item {
    struct traj_point point;
    long distance;
};

struct knnjoin_result_item {
    struct traj_point point1;
    struct traj_point point2;
    long distance;
};


struct knn_max_heap {
    struct result_item *arr;
    int size;
    int capacity;
    struct heap_runtime_statistics statistics;
};

struct knnjoin_max_heap {
    struct knnjoin_result_item *arr;
    int size;
    int capacity;
    struct heap_runtime_statistics statistics;
};

struct buffered_knn_max_heap {
    struct knn_max_heap *h;
    struct result_item *result_buffer;
    int buffer_factor;
    int buffer_capacity;
    int buffer_size;
    long max_distance_ref;
    bool is_max_dist_ref_initialized;
    struct heap_runtime_statistics statistics;
};

struct knn_max_heap* create_knn_max_heap(int capacity);

void free_knn_max_heap(struct knn_max_heap *h);

struct result_item knn_max_heap_extract_max(struct knn_max_heap *h);

void knn_max_heap_insert(struct knn_max_heap *h, struct result_item *item);

void knn_max_heap_replace(struct knn_max_heap *h, struct result_item *item);

struct result_item knn_max_heap_find_max(struct knn_max_heap *h);

void print_knn_max_heap(struct knn_max_heap *h);



struct knnjoin_max_heap* create_knnjoin_max_heap(int capacity);

void free_knnjoin_max_heap(struct knnjoin_max_heap *h);

struct knnjoin_result_item knnjoin_max_heap_extract_max(struct knnjoin_max_heap *h);

void knnjoin_max_heap_insert(struct knnjoin_max_heap *h, struct knnjoin_result_item *item);

void knnjoin_max_heap_replace(struct knnjoin_max_heap *h, struct knnjoin_result_item *item);

struct knnjoin_result_item knnjoin_max_heap_find_max(struct knnjoin_max_heap *h);

void print_knnjoin_max_heap(struct knnjoin_max_heap *h);



struct buffered_knn_max_heap* create_buffered_knn_max_heap(int capacity, int buffer_factor);

void free_buffered_knn_max_heap(struct buffered_knn_max_heap *bh);

void buffered_knn_max_heap_insert(struct buffered_knn_max_heap *bh, struct result_item *item);

void buffered_knn_max_heap_compact(struct buffered_knn_max_heap *bh);




long cal_points_distance(struct traj_point *point1, struct traj_point *point2);

long cal_min_distance(struct traj_point *point, struct seg_meta *mbr);

long cal_minmax_distance(struct traj_point *point, struct seg_meta *mbr);

void print_runtime_statistics(struct runtime_statistics *statistics);


// not used
struct knn_result_buffer {
    int buffer_capacity;    // equal to the parameter k
    int current_buffer_size; // the number of items in the result_buffer_k
    int current_buffer_low_half_size;
    int current_buffer_high_half_size;
    long max_distance;
    bool is_result_buffer_k_sorted;
    struct result_item *result_buffer_k;  // store the k-nearest result items
    struct result_item *result_buffer_low_half;
    struct result_item *result_buffer_high_half;
    struct runtime_statistics statistics;
};

// not used
struct knnjoin_result_buffer {
    int buffer_capacity;
    int current_buffer_size;
    int current_buffer_low_half_size;
    int current_buffer_high_half_size;
    long max_distance;
    bool is_result_buffer_k_sorted;
    struct knnjoin_result_item *knnjoin_result_buffer_k;
    struct knnjoin_result_item *knnjoin_result_buffer_low_half;
    struct knnjoin_result_item *knnjoin_result_buffer_high_half;
    struct runtime_statistics statistics;
};

// not used (replaced by heap)
void init_knn_result_buffer(int k, struct knn_result_buffer *buffer);

void free_knn_result_buffer(struct knn_result_buffer *buffer);

void sort_buffer(struct result_item *buffer, int buffer_size);

void add_item_to_buffer_baseline(struct knn_result_buffer *buffer, struct result_item *item);

void add_item_to_buffer(struct knn_result_buffer *buffer, struct result_item *item);

void combine_and_sort(struct knn_result_buffer *buffer);

void print_result_buffer(struct knn_result_buffer *buffer);

// not used (replaced by heap)
void init_knnjoin_result_buffer(int k, struct knnjoin_result_buffer *buffer);

void free_knnjoin_result_buffer(struct knnjoin_result_buffer *buffer);

void sort_knnjoin_buffer(struct knnjoin_result_item *buffer, int buffer_size);

void add_item_to_knnjoin_buffer_baseline(struct knnjoin_result_buffer *buffer, struct knnjoin_result_item *item);

void add_item_to_knnjoin_buffer(struct knnjoin_result_buffer *buffer, struct knnjoin_result_item *item);

void combine_and_sort_knnjoin(struct knnjoin_result_buffer *buffer);

void print_knnjoin_result_buffer(struct knnjoin_result_buffer *buffer);

#endif //TRAJ_BLOCK_FORMAT_KNN_UTIL_H
