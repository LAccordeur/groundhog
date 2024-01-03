//
// Created by yangguo on 11/11/22.
//
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "groundhog/traj_block_format.h"
#include "groundhog/traj_processing.h"
#include "groundhog/log.h"
#include "groundhog/common_util.h"
#include "groundhog/config.h"
#include "groundhog/bloom/bloom.h"
//#define size_of_attribute(Struct, Attribute) sizeof(((Struct*)0)->Attribute)

static const int OID_SIZE = size_of_attribute(struct traj_point, oid);
static const int TIMESTAMP_SIZE = size_of_attribute(struct traj_point, timestamp_sec);
static const int LONGITUDE_SIZE = size_of_attribute(struct traj_point, normalized_longitude);
static const int LATITUDE_SIZE = size_of_attribute(struct traj_point, normalized_latitude);
static const int OID_OFFSET = 0;
static const int TIMESTAMP_OFFSET = OID_OFFSET + OID_SIZE;
static const int LONGITUDE_OFFSET = TIMESTAMP_OFFSET + TIMESTAMP_SIZE;
static const int LATITUDE_OFFSET = LONGITUDE_OFFSET + LONGITUDE_SIZE;
static const int TRAJ_POINT_SIZE = OID_SIZE + TIMESTAMP_SIZE + LONGITUDE_SIZE + LATITUDE_SIZE;

static const int LON_MIN_SIZE = size_of_attribute(struct seg_meta, lon_min);
static const int LON_MAX_SIZE = size_of_attribute(struct seg_meta, lon_max);
static const int LAT_MIN_SIZE = size_of_attribute(struct seg_meta, lat_min);
static const int LAT_MAX_SIZE = size_of_attribute(struct seg_meta, lat_max);
static const int TIME_MIN_SIZE = size_of_attribute(struct seg_meta, time_min);
static const int TIME_MAX_SIZE = size_of_attribute(struct seg_meta, time_max);
static const int SEG_OFFSET_SIZE = size_of_attribute(struct seg_meta, seg_offset);
static const int SEG_SIZE_SIZE = size_of_attribute(struct seg_meta, seg_size);
static const int OID_FILTER_SIZE = MY_OID_FILTER_SIZE;  // the unit is byte (8 bits)
static const int LON_MIN_OFFSET = 0;
static const int LON_MAX_OFFSET = LON_MIN_OFFSET + LON_MIN_SIZE;
static const int LAT_MIN_OFFSET = LON_MAX_OFFSET + LON_MAX_SIZE;
static const int LAT_MAX_OFFSET = LAT_MIN_OFFSET + LAT_MIN_SIZE;
static const int TIME_MIN_OFFSET = LAT_MAX_OFFSET + LAT_MAX_SIZE;
static const int TIME_MAX_OFFSET = TIME_MIN_OFFSET + TIME_MIN_SIZE;
static const int SEG_OFFSET_OFFSET = TIME_MAX_OFFSET + TIME_MAX_SIZE;
static const int SEG_SIZE_OFFSET = SEG_OFFSET_OFFSET + SEG_OFFSET_SIZE;
static const int OID_FILTER_OFFSET = SEG_SIZE_OFFSET + SEG_SIZE_SIZE;
static const int SEG_META_SIZE = LON_MIN_SIZE + LON_MAX_SIZE + LAT_MIN_SIZE + LAT_MAX_SIZE + TIME_MIN_SIZE + TIME_MAX_SIZE + SEG_OFFSET_SIZE + SEG_SIZE_SIZE + OID_FILTER_SIZE;

static const int HEADER_SEG_COUNT_SIZE = size_of_attribute(struct traj_block_header, seg_count) + size_of_attribute(struct traj_block_header, alignment_field);
static const int HEADER_SEG_COUNT_OFFSET = 0;
static const int HEADER_SIZE = HEADER_SEG_COUNT_SIZE;



int get_traj_point_size() {
    return TRAJ_POINT_SIZE;
}

int get_seg_meta_size() {
    return SEG_META_SIZE;
}

int get_header_size() {
    return HEADER_SIZE;
}

void serialize_traj_point(struct traj_point* source, void* destination) {
    char *d = destination;
    memcpy(d + OID_OFFSET, &(source->oid), OID_SIZE);
    memcpy(d + TIMESTAMP_OFFSET, &(source->timestamp_sec), TIMESTAMP_SIZE);
    memcpy(d + LONGITUDE_OFFSET, &(source->normalized_longitude), LONGITUDE_SIZE);
    memcpy(d + LATITUDE_OFFSET, &(source->normalized_latitude), LATITUDE_SIZE);
}

void deserialize_traj_point(void* source, struct traj_point* destination) {
    char *s = source;
    //xil_printf("[deserialize_traj_point] source: %p\n", source);
    memcpy(&(destination->oid), s + OID_OFFSET, OID_SIZE);
    //xil_printf("[deserialize_traj_point] oid src: %p, oid dst: %p, oid size: %d\n", s + OID_OFFSET, &(destination->oid), OID_SIZE);
    memcpy(&(destination->timestamp_sec), s + TIMESTAMP_OFFSET, TIMESTAMP_SIZE);
    //xil_printf("[deserialize_traj_point] time src: %p, time dst: %p, time size: %d\n", s + TIMESTAMP_OFFSET, &(destination->timestamp_sec), TIMESTAMP_SIZE);
    memcpy(&(destination->normalized_longitude), s + LONGITUDE_OFFSET, LONGITUDE_SIZE);
    //xil_printf("[deserialize_traj_point] lon src: %p, lon dst: %p, lon size: %d\n", s + LONGITUDE_OFFSET, &(destination->normalized_longitude), LONGITUDE_SIZE);
    memcpy(&(destination->normalized_latitude), s + LATITUDE_OFFSET, LATITUDE_SIZE);
    //xil_printf("[deserialize_traj_point] lat src: %p, lat dst: %p, lon size: %d\n", s + LATITUDE_OFFSET, &(destination->normalized_latitude), LATITUDE_SIZE);
}

void serialize_seg_meta(struct seg_meta* source, void* destination) {
    char *d = destination;
    memcpy(d + LON_MIN_OFFSET, &(source->lon_min), LON_MIN_SIZE);
    memcpy(d + LON_MAX_OFFSET, &(source->lon_max), LON_MAX_SIZE);
    memcpy(d + LAT_MIN_OFFSET, &(source->lat_min), LAT_MIN_SIZE);
    memcpy(d + LAT_MAX_OFFSET, &(source->lat_max), LAT_MAX_SIZE);
    memcpy(d + TIME_MIN_OFFSET, &(source->time_min), TIME_MIN_SIZE);
    memcpy(d + TIME_MAX_OFFSET, &(source->time_max), TIME_MAX_SIZE);
    memcpy(d + SEG_OFFSET_OFFSET, &(source->seg_offset), SEG_OFFSET_SIZE);
    memcpy(d + SEG_SIZE_OFFSET, &(source->seg_size), SEG_SIZE_SIZE);
    memcpy(d + OID_FILTER_OFFSET, source->oid_filter, OID_FILTER_SIZE);
}

void deserialize_seg_meta(void* source, struct seg_meta* destination) {
    char *s = source;
    memcpy(&(destination->lon_min), s + LON_MIN_OFFSET, LON_MIN_SIZE);
    memcpy(&(destination->lon_max), s + LON_MAX_OFFSET, LON_MAX_SIZE);
    memcpy(&(destination->lat_min), s + LAT_MIN_OFFSET, LAT_MIN_SIZE);
    memcpy(&(destination->lat_max), s + LAT_MAX_OFFSET, LAT_MAX_SIZE);
    memcpy(&(destination->time_min), s + TIME_MIN_OFFSET, TIME_MIN_SIZE);
    memcpy(&(destination->time_max), s + TIME_MAX_OFFSET, TIME_MAX_SIZE);
    memcpy(&(destination->seg_offset), s + SEG_OFFSET_OFFSET, SEG_OFFSET_SIZE);
    memcpy(&(destination->seg_size), s + SEG_SIZE_OFFSET, SEG_SIZE_SIZE);
    memcpy(destination->oid_filter, s + OID_FILTER_OFFSET, OID_FILTER_SIZE);
}

struct traj_point** allocate_points_memory(int array_size) {
    struct traj_point** points;
    points = (struct traj_point**) malloc(array_size * sizeof(struct traj_point*));

    for (int i = 0; i < array_size; i++) {
        points[i] = (struct traj_point*) malloc(sizeof(struct traj_point));
    }
    return points;
}

void free_points_memory(struct traj_point **points, int array_size) {
    for (int i = 0; i < array_size; i++) {
        free(points[i]);
        points[i] = NULL;
    }
    free(points);
    points = NULL;
}

void print_traj_points(struct traj_point **points, int array_size) {
    for (int i = 0; i < array_size; i++) {
        struct traj_point *point = points[i];
        printf("oid: %d\n", point->oid);
        printf("timestamp_sec: %d\n", point->timestamp_sec);
        printf("normalized longitude: %d\n", point->normalized_longitude);
        printf("normalized latitude: %d\n\n", point->normalized_latitude);
    }
}

void init_seg_meta_pair_array(struct seg_meta_pair_itr *pair_array, struct seg_meta_pair *array_base, int array_size) {
    pair_array->meta_pair_array_base = array_base;
    pair_array->array_size = array_size;
    pair_array->current_index = 0;
}

/*void append_seg_meta_pair(struct seg_meta_pair_itr *pair_array, struct seg_meta_pair *meta_pair) {
    if (pair_array->current_index >= pair_array->total_size - 1) {
        return;
    }
    (pair_array->meta_pair_array_base)[pair_array->current_index] = *meta_pair;
    pair_array->current_index++;
}*/

void init_seg_meta(struct seg_meta *meta) {
    meta->seg_offset = -1;
    meta->seg_size = -1;
    meta->lon_min = INT_MAX;
    meta->lon_max = 0;
    meta->lat_min = INT_MAX;
    meta->lat_max = 0;
    meta->time_min = INT_MAX;
    meta->time_max = 0;
    //meta->oid_filter_size = OID_FILTER_SIZE;
    memset(meta->oid_filter, 0, OID_FILTER_SIZE);
}

void extract_spatiotemporal_seg_meta(struct traj_point **points, int array_size, struct seg_meta *meta) {
    bloom_filter *filter = bloom_filter_new_default(OID_FILTER_SIZE * 8);

    struct traj_point *point;
    for (int i = 0; i < array_size; i++) {
        point = points[i];
        if (point->normalized_longitude > meta->lon_max) {
            meta->lon_max = point->normalized_longitude;
        }
        if (point->normalized_longitude < meta->lon_min) {
            meta->lon_min = point->normalized_longitude;
        }
        if (point->normalized_latitude > meta->lat_max) {
            meta->lat_max = point->normalized_latitude;
        }
        if (point->normalized_latitude < meta->lat_min) {
            meta->lat_min = point->normalized_latitude;
        }
        if (point->timestamp_sec > meta->time_max) {
            meta->time_max = point->timestamp_sec;
        }
        if (point->timestamp_sec < meta->time_min) {
            meta->time_min = point->timestamp_sec;
        }

        // add oid filter
        bloom_filter_put(filter, &(point->oid), sizeof(point->oid));
    }
    //memcpy(meta->oid_filter, filter->vect->mem, meta->oid_filter_size);
    memcpy(meta->oid_filter, filter->vect->mem, OID_FILTER_SIZE);
    bloom_filter_free(filter);
}

int assemble_traj_block(struct seg_meta_pair_itr *pair_array, void* block, int block_size) {
    // check if the block can hold all data
    int needed_block_size = 0;

    int block_offset = 0;
    int count_in_header = pair_array->current_index + 1;
    int header_offset = 0;
    int header_size = HEADER_SIZE;
    int seg_meta_section_offset = header_offset + header_size;
    int seg_meta_section_size = count_in_header * SEG_META_SIZE;
    int seg_data_section_offset = seg_meta_section_offset + seg_meta_section_size;


    struct seg_meta *meta;
    int previous_seg_data_offset;
    int previous_seg_data_size;
    for (int i = 0; i < count_in_header; i++) {
        meta = &((pair_array->meta_pair_array_base)[i].meta);
        meta->seg_size = (pair_array->meta_pair_array_base)[i].seg_length;

        if (i == 0) {
            meta->seg_offset = seg_data_section_offset;
        } else {
            meta->seg_offset = previous_seg_data_offset + previous_seg_data_size;
        }
        previous_seg_data_offset = meta->seg_offset;
        previous_seg_data_size = meta->seg_size;
    }

    needed_block_size = previous_seg_data_size + previous_seg_data_offset;
    if (needed_block_size > block_size) {
        debug_print("need more space to hold the block data: needed %d, provided %d\n", needed_block_size, block_size);
        return -1;
    }

    // assemble block
    char* b = block;
    block_offset = header_offset;
    // header
    memcpy(b + block_offset, &count_in_header, header_size);
    block_offset =  block_offset + header_size;
    // seg meta and data section
    char* seg_data;
    char seg_meta[SEG_META_SIZE];
    for (int i = 0; i < count_in_header; i++) {
        meta = &((pair_array->meta_pair_array_base)[i].meta);
        serialize_seg_meta(meta, seg_meta);
        seg_data = (pair_array->meta_pair_array_base)[i].seg_data;
        // seg meta
        memcpy(b + block_offset, seg_meta, SEG_META_SIZE);
        block_offset = block_offset + SEG_META_SIZE;
        // seg data
        memcpy(b + meta->seg_offset, seg_data, meta->seg_size);
    }

    return 0;
}

void do_self_contained_traj_block(struct traj_point **points, int points_num, void* block, int block_size) {
    // split trajectories
    //sort_traj_points(points, points_num);

    int meta_pair_array_size = SPLIT_SEGMENT_NUM;
    struct seg_meta_pair_itr meta_pair_array;
    struct seg_meta_pair meta_pairs[meta_pair_array_size];
    init_seg_meta_pair_array(&meta_pair_array, meta_pairs, meta_pair_array_size);

    split_traj_points_via_point_num_and_seg_num(points, points_num, &meta_pair_array, meta_pair_array_size);

    assemble_traj_block(&meta_pair_array, block, block_size);
    free_tmp_seg_data(&meta_pair_array);
}

void extract_seg_meta_section(void* self_contained_block, void* destination) {
    struct traj_block_header header;
    parse_traj_block_for_header(self_contained_block, &header);
    int block_offset = HEADER_SIZE;
    int seg_meta_section_size = SEG_META_SIZE * header.seg_count;
    memcpy(destination, self_contained_block + block_offset, seg_meta_section_size);
}

int get_seg_meta_section_size(void* self_contained_block) {
    struct traj_block_header header;
    parse_traj_block_for_header(self_contained_block, &header);
    return SEG_META_SIZE * header.seg_count;
}

void free_tmp_seg_data(struct seg_meta_pair_itr *pair_array) {
    for (int i = 0; i <= pair_array->current_index; i++) {
        struct seg_meta_pair *meta_pair = &(pair_array->meta_pair_array_base)[i];
        free(meta_pair->seg_data);
    }
}

void parse_traj_block_for_header(void* block, struct traj_block_header *header) {
    char* b = block;
    memcpy(&(header->seg_count), b, sizeof(int));
}

void parse_traj_block_for_seg_meta_section(void* block, struct seg_meta *meta_array, int array_size) {
    char* b = block;
    for (int i = 0; i < array_size; i++) {
        char* offset = b + HEADER_SIZE + i * SEG_META_SIZE;
        deserialize_seg_meta(offset, meta_array + i);
    }

}

void parse_traj_block_for_seg_data(void* block, int offset, struct traj_point **points, int points_num) {
    char* b = block;
    char* seg_data_base = b + offset;
    for (int i = 0; i < points_num; i++) {
        void* source = seg_data_base + TRAJ_POINT_SIZE * i;
        deserialize_traj_point(source, points[i]);
    }
}

static void* row_slot_in_buffer_block(void* buffer_block, size_t row_offset) {
    char* b = buffer_block;
    return b + row_offset * get_traj_point_size();
}


void convert_serialized_to_struct_traj(void *serialized_buffer, struct traj_point **points, int array_size) {

    for (int i = 0; i < array_size; i++) {
        void *buffer_ptr = row_slot_in_buffer_block(serialized_buffer, i);
        deserialize_traj_point(buffer_ptr, points[i]);
    }

}

void convert_struct_to_serialized_traj(struct traj_point **points, void *serialized_buffer, int array_size) {

    for (int i = 0; i < array_size; i++) {
        struct traj_point *point = points[i];
        void *buffer_ptr = row_slot_in_buffer_block(serialized_buffer, i);
        serialize_traj_point(point, buffer_ptr);
    }
}

int calculate_points_num_via_block_size(int block_size, int segment_num) {
    int serialized_point_size = get_traj_point_size();
    int seg_meta_size = SEG_META_SIZE;

    int header_section_size = HEADER_SIZE;
    int seg_meta_section_size = seg_meta_size * segment_num;
    int remaining_space_for_points = block_size - header_section_size - seg_meta_section_size;

    return remaining_space_for_points / serialized_point_size;
}

int calculate_block_size_via_points_num(int points_num, int segment_num) {
    int header_section_size = HEADER_SIZE;
    int seg_meta_section_size = segment_num * SEG_META_SIZE;
    int space_for_points = points_num * get_traj_point_size();

    int needed_space = header_section_size + seg_meta_section_size + space_for_points;
    int remainder = needed_space % 4096;
    if (remainder == 0) {
        return needed_space;
    } else {
        return ((needed_space / 4096) + 1) * 4096;
    }
}

void* calculate_point_slot_in_traj_block(void *seg_offset, int point_index_in_seg) {
    char *s = seg_offset;
    char *slot = seg_offset + point_index_in_seg * TRAJ_POINT_SIZE;
    return slot;
}
