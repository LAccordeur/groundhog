//
// Created by yangguo on 24-4-2.
//
#include "groundhog/knn_util.h"
#include <malloc.h>
#include <limits.h>
#include <stdlib.h>

void init_knn_result_buffer(int k, struct knn_result_buffer *buffer) {
    buffer->buffer_capacity = k;
    buffer->current_buffer_size = 0;
    buffer->current_buffer_low_half_size = 0;
    buffer->current_buffer_high_half_size = 0;
    buffer->max_distance = LONG_MAX;
    buffer->is_result_buffer_k_sorted = false;
    buffer->result_buffer_k = (struct result_item *) malloc((k+1) * sizeof(struct result_item));
    buffer->result_buffer_low_half = (struct result_item *) malloc(k * sizeof(struct result_item));
    buffer->result_buffer_high_half = (struct result_item *) malloc(k * sizeof(struct result_item));

    buffer->statistics.add_item_to_buffer_call_num = 0;
    buffer->statistics.buffer_high_hal_full_num = 0;
    buffer->statistics.buffer_low_half_full_num = 0;
    buffer->statistics.combine_and_sort_call_num = 0;
    buffer->statistics.sort_func_call_num = 0;
    buffer->statistics.checked_segment_num = 0;
}

void free_knn_result_buffer(struct knn_result_buffer *buffer) {
    free(buffer->result_buffer_k);
    buffer->result_buffer_k = NULL;
    free(buffer->result_buffer_low_half);
    buffer->result_buffer_low_half = NULL;
    free(buffer->result_buffer_high_half);
    buffer->result_buffer_high_half = NULL;
}

int cmpfunc (const void * a, const void * b) {
    long diff = ( ((struct result_item*)a)->distance - ((struct result_item*)b)->distance );
    int flag;
    if (diff > 0) {
        flag = 1;
    } else if (diff < 0) {
        flag = -1;
    } else {
        flag = 0;
    }
    return flag;
}

void sort_buffer(struct result_item *buffer, int buffer_size) {
    qsort(buffer, buffer_size, sizeof(struct result_item), cmpfunc);
}

void combine_and_sort(struct knn_result_buffer *buffer) {
    int tmp_buffer_size = buffer->current_buffer_size + buffer->current_buffer_low_half_size + buffer->current_buffer_high_half_size;
    struct result_item tmp_buffer[tmp_buffer_size];
    int tmp_buffer_index = 0;
    for (int i = 0; i < buffer->current_buffer_size; i++) {
        tmp_buffer[tmp_buffer_index] = buffer->result_buffer_k[i];
        tmp_buffer_index++;
    }
    for (int i = 0; i < buffer->current_buffer_low_half_size; i++) {
        tmp_buffer[tmp_buffer_index] = buffer->result_buffer_low_half[i];
        tmp_buffer_index++;
    }
    for (int i = 0; i < buffer->current_buffer_high_half_size; i++) {
        tmp_buffer[tmp_buffer_index] = buffer->result_buffer_high_half[i];
        tmp_buffer_index++;
    }
    sort_buffer(tmp_buffer, tmp_buffer_size);

    for (int i = 0; i < buffer->buffer_capacity; i++) {
        buffer->result_buffer_k[i] = tmp_buffer[i];
    }
    buffer->current_buffer_size = buffer->buffer_capacity;
    buffer->current_buffer_low_half_size = 0;
    buffer->current_buffer_high_half_size = 0;

}

void add_item_to_buffer_baseline(struct knn_result_buffer *buffer, struct result_item *item) {
    buffer->result_buffer_k[buffer->current_buffer_size] = *item;
    buffer->current_buffer_size++;
    sort_buffer(buffer->result_buffer_k, buffer->current_buffer_size);
    if (buffer->current_buffer_size > buffer->buffer_capacity) {
        buffer->current_buffer_size = buffer->buffer_capacity;
    }
    if (buffer->current_buffer_size == buffer->buffer_capacity) {
        buffer->max_distance = buffer->result_buffer_k[buffer->buffer_capacity - 1].distance;
    }
    buffer->statistics.sort_func_call_num++;
    buffer->statistics.add_item_to_buffer_call_num++;
}


void add_item_to_buffer(struct knn_result_buffer *buffer, struct result_item *item) {
    if (buffer->current_buffer_size < buffer->buffer_capacity) {
        // we directly append the item to the buffer without sorting
        buffer->result_buffer_k[buffer->current_buffer_size] = *item;
        buffer->current_buffer_size++;

        buffer->statistics.add_item_to_buffer_call_num++;
    } else {
        // when the buffer_k is full
        if (!buffer->is_result_buffer_k_sorted) {
            sort_buffer(buffer->result_buffer_k, buffer->current_buffer_size);
            buffer->is_result_buffer_k_sorted = true;
            buffer->max_distance = buffer->result_buffer_k[buffer->buffer_capacity-1].distance;

            buffer->statistics.sort_func_call_num++;
        }
        long medium_distance = buffer->result_buffer_k[buffer->current_buffer_size/2].distance;
        long item_distance = item->distance;
        if (item_distance <= medium_distance) {
            buffer->result_buffer_low_half[buffer->current_buffer_low_half_size] = *item;
            buffer->current_buffer_low_half_size++;

            buffer->statistics.add_item_to_buffer_call_num++;

            if (buffer->current_buffer_low_half_size >= buffer->buffer_capacity / 2) {
                // now the items in the low half buffer becomes the k-nearest result, so we discard the items in the other two buffers
                int offset = buffer->buffer_capacity / 2;
                for (int i = 0; i < buffer->buffer_capacity / 2; i++) {
                    buffer->result_buffer_k[i + offset] = buffer->result_buffer_low_half[i];

                    buffer->statistics.add_item_to_buffer_call_num++;
                }
                sort_buffer(buffer->result_buffer_k, buffer->buffer_capacity);

                buffer->current_buffer_low_half_size = 0;
                buffer->current_buffer_high_half_size = 0;

                buffer->max_distance = buffer->result_buffer_k[buffer->buffer_capacity-1].distance;

                buffer->statistics.buffer_low_half_full_num++;
                buffer->statistics.sort_func_call_num++;
            }

        } else {
            buffer->result_buffer_high_half[buffer->current_buffer_high_half_size] = *item;
            buffer->current_buffer_high_half_size++;

            buffer->statistics.add_item_to_buffer_call_num++;

            if (buffer->current_buffer_high_half_size >= buffer->buffer_capacity) {
                combine_and_sort(buffer);
                buffer->max_distance = buffer->result_buffer_k[buffer->buffer_capacity-1].distance;

                buffer->statistics.buffer_high_hal_full_num++;
                buffer->statistics.combine_and_sort_call_num++;
            }
        }

    }
}


void init_knnjoin_result_buffer(int k, struct knnjoin_result_buffer *buffer) {
    buffer->buffer_capacity = k;
    buffer->current_buffer_size = 0;
    buffer->current_buffer_low_half_size = 0;
    buffer->current_buffer_high_half_size = 0;
    buffer->max_distance = LONG_MAX;
    buffer->is_result_buffer_k_sorted = false;
    buffer->knnjoin_result_buffer_k = (struct knnjoin_result_item *) malloc((k+1) * sizeof(struct knnjoin_result_item));
    buffer->knnjoin_result_buffer_low_half = (struct knnjoin_result_item *) malloc(k * sizeof(struct knnjoin_result_item));
    buffer->knnjoin_result_buffer_high_half = (struct knnjoin_result_item *) malloc(k * sizeof(struct knnjoin_result_item));

    buffer->statistics.add_item_to_buffer_call_num = 0;
    buffer->statistics.buffer_high_hal_full_num = 0;
    buffer->statistics.buffer_low_half_full_num = 0;
    buffer->statistics.combine_and_sort_call_num = 0;
    buffer->statistics.sort_func_call_num = 0;
}

void free_knnjoin_result_buffer(struct knnjoin_result_buffer *buffer) {
    free(buffer->knnjoin_result_buffer_k);
    buffer->knnjoin_result_buffer_k = NULL;
    free(buffer->knnjoin_result_buffer_low_half);
    buffer->knnjoin_result_buffer_low_half = NULL;
    free(buffer->knnjoin_result_buffer_high_half);
    buffer->knnjoin_result_buffer_high_half = NULL;
}

int cmpfunc_knnjoin (const void * a, const void * b) {
    long diff = ( ((struct knnjoin_result_item*)a)->distance - ((struct knnjoin_result_item*)b)->distance );
    int flag;
    if (diff > 0) {
        flag = 1;
    } else if (diff < 0) {
        flag = -1;
    } else {
        flag = 0;
    }
    return flag;
}

void sort_knnjoin_buffer(struct knnjoin_result_item *buffer, int buffer_size) {
    qsort(buffer, buffer_size, sizeof(struct knnjoin_result_item), cmpfunc_knnjoin);
}

void combine_and_sort_knnjoin(struct knnjoin_result_buffer *buffer) {
    int tmp_buffer_size = buffer->current_buffer_size + buffer->current_buffer_low_half_size + buffer->current_buffer_high_half_size;
    struct knnjoin_result_item tmp_buffer[tmp_buffer_size];
    int tmp_buffer_index = 0;
    for (int i = 0; i < buffer->current_buffer_size; i++) {
        tmp_buffer[tmp_buffer_index] = buffer->knnjoin_result_buffer_k[i];
        tmp_buffer_index++;
    }
    for (int i = 0; i < buffer->current_buffer_low_half_size; i++) {
        tmp_buffer[tmp_buffer_index] = buffer->knnjoin_result_buffer_low_half[i];
        tmp_buffer_index++;
    }
    for (int i = 0; i < buffer->current_buffer_high_half_size; i++) {
        tmp_buffer[tmp_buffer_index] = buffer->knnjoin_result_buffer_high_half[i];
        tmp_buffer_index++;
    }

    sort_knnjoin_buffer(tmp_buffer, tmp_buffer_size);

    int current_size;
    if (tmp_buffer_size < buffer->buffer_capacity) {
        current_size = tmp_buffer_size;
    } else {
        current_size = buffer->buffer_capacity;
    }

    for (int i = 0; i < current_size; i++) {
        buffer->knnjoin_result_buffer_k[i] = tmp_buffer[i];
    }
    buffer->current_buffer_size = current_size;
    buffer->current_buffer_low_half_size = 0;
    buffer->current_buffer_high_half_size = 0;

}

void add_item_to_knnjoin_buffer_baseline(struct knnjoin_result_buffer *buffer, struct knnjoin_result_item *item) {
    buffer->knnjoin_result_buffer_k[buffer->current_buffer_size] = *item;
    buffer->current_buffer_size++;
    sort_knnjoin_buffer(buffer->knnjoin_result_buffer_k, buffer->current_buffer_size);
    if (buffer->current_buffer_size > buffer->buffer_capacity) {
        buffer->current_buffer_size = buffer->buffer_capacity;
    }
    if (buffer->current_buffer_size == buffer->buffer_capacity) {
        buffer->max_distance = buffer->knnjoin_result_buffer_k[buffer->buffer_capacity - 1].distance;
    }

    buffer->statistics.sort_func_call_num++;
    buffer->statistics.add_item_to_buffer_call_num++;
}

void add_item_to_knnjoin_buffer(struct knnjoin_result_buffer *buffer, struct knnjoin_result_item *item) {
    if (buffer->current_buffer_size < buffer->buffer_capacity) {
        // we directly append the item to the buffer without sorting
        buffer->knnjoin_result_buffer_k[buffer->current_buffer_size] = *item;
        buffer->current_buffer_size++;

        buffer->statistics.add_item_to_buffer_call_num++;
    } else {
        // when the buffer_k is full
        if (!buffer->is_result_buffer_k_sorted) {
            sort_knnjoin_buffer(buffer->knnjoin_result_buffer_k, buffer->current_buffer_size);
            buffer->is_result_buffer_k_sorted = true;
            buffer->max_distance = buffer->knnjoin_result_buffer_k[buffer->buffer_capacity-1].distance;

            buffer->statistics.sort_func_call_num++;
        }
        long medium_distance = buffer->knnjoin_result_buffer_k[buffer->current_buffer_size/2].distance;
        long item_distance = item->distance;
        if (item_distance <= medium_distance) {
            buffer->knnjoin_result_buffer_low_half[buffer->current_buffer_low_half_size] = *item;
            buffer->current_buffer_low_half_size++;

            buffer->statistics.add_item_to_buffer_call_num++;

            if (buffer->current_buffer_low_half_size >= buffer->buffer_capacity / 2) {
                // now the items in the low half buffer becomes the k-nearest result, so we discard the items in the other two buffers
                int offset = buffer->buffer_capacity / 2;
                for (int i = 0; i < buffer->buffer_capacity / 2; i++) {
                    buffer->knnjoin_result_buffer_k[i + offset] = buffer->knnjoin_result_buffer_low_half[i];

                    buffer->statistics.add_item_to_buffer_call_num++;
                }
                sort_knnjoin_buffer(buffer->knnjoin_result_buffer_k, buffer->buffer_capacity);

                buffer->current_buffer_low_half_size = 0;
                buffer->current_buffer_high_half_size = 0;

                buffer->max_distance = buffer->knnjoin_result_buffer_k[buffer->buffer_capacity-1].distance;

                buffer->statistics.buffer_low_half_full_num++;
                buffer->statistics.sort_func_call_num++;
            }

        } else {
            buffer->knnjoin_result_buffer_high_half[buffer->current_buffer_high_half_size] = *item;
            buffer->current_buffer_high_half_size++;

            buffer->statistics.add_item_to_buffer_call_num++;

            if (buffer->current_buffer_high_half_size >= buffer->buffer_capacity) {
                combine_and_sort_knnjoin(buffer);
                buffer->max_distance = buffer->knnjoin_result_buffer_k[buffer->buffer_capacity-1].distance;

                buffer->statistics.buffer_high_hal_full_num++;
                buffer->statistics.combine_and_sort_call_num++;
            }
        }

    }
}



long cal_points_distance(struct traj_point *point1, struct traj_point *point2) {
    return (long)(point1->normalized_longitude - point2->normalized_longitude) * (point1->normalized_longitude - point2->normalized_longitude) + (long)(point1->normalized_latitude - point2->normalized_latitude) * (point1->normalized_latitude - point2->normalized_latitude);
}

long cal_min_distance(struct traj_point *point, struct seg_meta *mbr) {
    if (point->normalized_longitude >= mbr->lon_min && point->normalized_longitude <= mbr->lon_max
    && point->normalized_latitude >= mbr->lat_min && point->normalized_longitude <= mbr->lat_max) {
        return 0;
    }

    int r_lon, r_lat;
    if (point->normalized_longitude < mbr->lon_min) {
        r_lon = mbr->lon_min;
    } else if (point->normalized_longitude > mbr->lon_min){
        r_lon = mbr->lon_max;
    } else {
        r_lon = point->normalized_longitude;
    }

    if (point->normalized_latitude < mbr->lat_min) {
        r_lat = mbr->lat_min;
    } else if (point->normalized_latitude > mbr->lat_max) {
        r_lat = mbr->lat_max;
    } else {
        r_lat = point->normalized_latitude;
    }
    return (long)(point->normalized_longitude - r_lon) * (point->normalized_longitude - r_lon)
    + (long)(point->normalized_latitude - r_lat) * (point->normalized_latitude - r_lat);
}

long cal_minmax_distance(struct traj_point *point, struct seg_meta *mbr) {
    int r_lon_mid = (mbr->lon_min + mbr->lon_max) / 2;
    int r_lat_mid = (mbr->lat_min + mbr->lat_max) / 2;

    int r_lon_M, r_lat_M;
    if (point->normalized_longitude >= r_lon_mid) {
        r_lon_M = mbr->lon_min;
    } else {
        r_lon_M = mbr->lon_max;
    }
    if (point->normalized_latitude >= r_lat_mid) {
        r_lat_M = mbr->lat_min;
    } else {
        r_lat_M = mbr->lat_max;
    }

    long max_dist = (long)(point->normalized_longitude - r_lon_M) * (point->normalized_longitude - r_lon_M) + (long)(point->normalized_latitude - r_lat_M) * (point->normalized_latitude - r_lat_M);

    int r_lon_m, r_lat_m;
    if (point->normalized_longitude <= r_lon_mid) {
        r_lon_m = mbr->lon_min;
    } else {
        r_lon_m = mbr->lon_max;
    }
    if (point->normalized_latitude <= r_lat_mid) {
        r_lat_m = mbr->lat_min;
    } else {
        r_lat_m = mbr->lat_max;
    }

    long value1 = max_dist - (long)(point->normalized_longitude - r_lon_M) * (point->normalized_longitude - r_lon_M) + (long)(point->normalized_longitude - r_lon_m) * (point->normalized_longitude - r_lon_m);
    long value2 = max_dist - (long)(point->normalized_latitude - r_lat_M) * (point->normalized_latitude - r_lat_M) + (long)(point->normalized_latitude - r_lat_m) * (point->normalized_latitude - r_lat_m);

    return value1 > value2 ? value2 : value1;

}

void print_result_buffer(struct knn_result_buffer *buffer) {
    printf("total num: %d\n", buffer->current_buffer_size);
    for (int i = 0; i < buffer->current_buffer_size; i++) {
        struct result_item item = buffer->result_buffer_k[i];
        /*printf("oid: %d, lon: %d, lat: %d, time: %d, dist: %d\n",
               item.point->oid, item.point->normalized_longitude, item.point->normalized_latitude, item.point->timestamp_sec, item.distance);*/
        printf("oid: %d, lon: %d, lat: %d, time: %d, dist: %d\n",
               item.point.oid, item.point.normalized_longitude, item.point.normalized_latitude, item.point.timestamp_sec, item.distance);
    }
}

void print_knnjoin_result_buffer(struct knnjoin_result_buffer *buffer) {
    for (int i = 0; i < buffer->current_buffer_size; i++) {
        struct knnjoin_result_item item = buffer->knnjoin_result_buffer_k[i];
        printf("oid1: %d, lon1: %d, lat1: %d, time1: %d, oid2: %d, lon2: %d, lat2: %d, time2: %d, dist: %d\n",
               item.point1.oid, item.point1.normalized_longitude, item.point1.normalized_latitude, item.point1.timestamp_sec,
               item.point2.oid, item.point2.normalized_longitude, item.point2.normalized_latitude, item.point2.timestamp_sec,
               item.distance);
    }
}

void print_runtime_statistics(struct runtime_statistics *statistics) {
    printf("add item call num: %d, sort call num: %d, low half full num: %d, high half full num: %d, combine and sort num: %d, checked data segment num: %d\n",
           statistics->add_item_to_buffer_call_num, statistics->sort_func_call_num, statistics->buffer_low_half_full_num, statistics->buffer_high_hal_full_num, statistics->combine_and_sort_call_num, statistics->checked_segment_num);
}
