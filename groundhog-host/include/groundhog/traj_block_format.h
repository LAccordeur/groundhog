//
// Created by yangguo on 11/10/22.
//

#ifndef TRAJ_BLOCK_FORMAT_TRAJ_BLOCK_FORMAT_H
#define TRAJ_BLOCK_FORMAT_TRAJ_BLOCK_FORMAT_H

#define MY_OID_FILTER_SIZE 16

struct traj_point {
    int oid;
    int timestamp_sec;
    int normalized_longitude;
    int normalized_latitude;
};

struct seg_meta {
    int lon_min;
    int lon_max;
    int lat_min;
    int lat_max;
    int time_min;
    int time_max;
    int seg_offset;
    int seg_size;
    char oid_filter[MY_OID_FILTER_SIZE];
    //int oid_filter_size;
};

struct traj_block_header {
    int seg_count;
    int alignment_field;
};

struct seg_meta_pair {
    struct seg_meta meta;
    void* seg_data;
    int seg_length;
};

struct seg_meta_pair_itr {
    struct seg_meta_pair *meta_pair_array_base;
    int array_size;
    int current_index;
};

int calculate_points_num_via_block_size(int block_size, int segment_num);

int calculate_block_size_via_points_num(int points_num, int segment_num);

void* calculate_point_slot_in_traj_block(void *seg_offset, int point_index_in_seg);

void init_seg_meta(struct seg_meta *meta);

void print_seg_meta(struct seg_meta *meta);

void print_seg_meta_pair(struct seg_meta_pair *meta_pair);

void print_seg_meta_pair_itr(struct seg_meta_pair_itr *meta_pair_itr);

void init_seg_meta_pair_array(struct seg_meta_pair_itr *pair_array, struct seg_meta_pair *array_base, int array_size);

//void append_seg_meta_pair(struct seg_meta_pair_itr *pair_array, struct seg_meta_pair *meta_pair);

void extract_spatiotemporal_seg_meta(struct traj_point **points, int array_size, struct seg_meta *meta);

int assemble_traj_block(struct seg_meta_pair_itr *pair_array, void* block, int block_size);

void do_self_contained_traj_block(struct traj_point **points, int points_num, void* block, int block_size);

void extract_seg_meta_section(void* self_contained_block, void* destination);

int get_seg_meta_section_size(void* self_contained_block);

void parse_traj_block_for_header(void* block, struct traj_block_header *header);

void parse_traj_block_for_seg_meta_section(void* block, struct seg_meta *meta_array, int array_size);

void parse_traj_block_for_seg_data(void* block, int offset, struct traj_point **points, int points_num);

void free_tmp_seg_data(struct seg_meta_pair_itr *pair_array);

int get_traj_point_size();

int get_seg_meta_size();

int get_header_size();

void serialize_traj_point(struct traj_point* source, void* destination);

void deserialize_traj_point(void* source, struct traj_point* destination);

void serialize_seg_meta(struct seg_meta* source, void* destination);

void deserialize_seg_meta(void* source, struct seg_meta* destination);

struct traj_point** allocate_points_memory(int array_size);

void free_points_memory(struct traj_point **points, int array_size);

void print_traj_points(struct traj_point **points, int array_size);

void convert_serialized_to_struct_traj(void *serialized_buffer, struct traj_point **points, int array_size);

void convert_struct_to_serialized_traj(struct traj_point **points, void *serialized_buffer, int array_size);
#endif //TRAJ_BLOCK_FORMAT_TRAJ_BLOCK_FORMAT_H
