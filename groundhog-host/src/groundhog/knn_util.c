//
// Created by yangguo on 24-4-2.
//
#include "groundhog/knn_util.h"
#include <malloc.h>
#include <limits.h>
#include <stdlib.h>

static void knn_max_heap_insert_helper(struct knn_max_heap *h, int index);

static void knn_max_heap_heapify(struct knn_max_heap *h, int index);

static void knnjoin_max_heap_insert_helper(struct knnjoin_max_heap *h, int index);

static void knnjoin_max_heap_heapify(struct knnjoin_max_heap *h, int index);



struct knn_max_heap* create_knn_max_heap(int capacity) {
// Allocating memory to heap h
    struct knn_max_heap* h = (struct knn_max_heap*)malloc(sizeof(struct knn_max_heap));

    // Checking if memory is allocated to h or not
    if (h == NULL) {
        printf("Memory error");
        return NULL;
    }
    // set the values to size and capacity
    h->size = 0;
    h->capacity = capacity;

    // Allocating memory to array
    h->arr = (struct result_item*)malloc(capacity * sizeof(struct result_item));

    // Checking if memory is allocated to h or not
    if (h->arr == NULL) {
        printf("Memory error");
        return NULL;
    }

    h->statistics.non_discard_count = 0;
    h->statistics.discard_count = 0;
    h->statistics.add_to_buffer_item_count = 0;
    h->statistics.add_to_heap_item_count = 0;
    h->statistics.heap_sawp_operation_count = 0;

    return h;
}

void free_knn_max_heap(struct knn_max_heap *h) {
    free(h->arr);
    free(h);
}

static
void knn_max_heap_insert_helper(struct knn_max_heap *h, int index) {
    // Store parent of element at index
    // in parent variable
    int parent = (index - 1) / 2;

    if (h->arr[parent].distance < h->arr[index].distance) {
        // Swapping when child is smaller
        // than parent element
        struct result_item temp = h->arr[parent];
        h->arr[parent] = h->arr[index];
        h->arr[index] = temp;

        // Recursively calling maxHeapify_bottom_up
        knn_max_heap_insert_helper(h, parent);
    }
}

static
void knn_max_heap_heapify(struct knn_max_heap *h, int index) {
    int left = index * 2 + 1;
    int right = index * 2 + 2;
    int max = index;

    // Checking whether our left or child element
    // is at right index of not to avoid index error
    if (left >= h->size || left < 0)
        left = -1;
    if (right >= h->size || right < 0)
        right = -1;

    // store left or right element in max if
    // any of these is smaller that its parent
    if (left != -1 && h->arr[left].distance > h->arr[max].distance) {
        max = left;
    }
    if (right != -1 && h->arr[right].distance > h->arr[max].distance) {
        max = right;
    }

    // Swapping the nodes
    if (max != index) {
        h->statistics.heap_sawp_operation_count++;

        struct result_item temp = h->arr[max];
        h->arr[max] = h->arr[index];
        h->arr[index] = temp;

        // recursively calling for their child elements
        // to maintain max heap
        knn_max_heap_heapify(h, max);

    }
}

struct result_item knn_max_heap_extract_max(struct knn_max_heap *h) {
    struct result_item deleteItem;

    // Checking if the heap is empty or not
    if (h->size == 0) {
        printf("\nHeap id empty.");
        return deleteItem;
    }

    // Store the node in deleteItem that
    // is to be deleted.
    deleteItem = h->arr[0];

    // Replace the deleted node with the last node
    h->arr[0] = h->arr[h->size - 1];
    // Decrement the size of heap
    h->size--;

    // Call maxheapify_top_down for 0th index
    // to maintain the heap property
    knn_max_heap_heapify(h, 0);
    return deleteItem;
}

void knn_max_heap_insert(struct knn_max_heap *h, struct result_item *item) {
    // Checking if heap is full or not
    if (h->size < h->capacity) {
        // Inserting data into an array
        h->arr[h->size] = *item;
        // Calling maxHeapify_bottom_up function
        knn_max_heap_insert_helper(h, h->size);
        // Incrementing size of array
        h->size++;
    }
}

void knn_max_heap_replace(struct knn_max_heap *h, struct result_item *item) {

    // Checking if the heap is empty or not
    if (h->size == 0) {
        printf("\nHeap is empty.");
        return;
    }


    // Replace the root node with the newly added node
    h->arr[0] = *item;

    // Call maxheapify_top_down for 0th index
    // to maintain the heap property
    knn_max_heap_heapify(h, 0);

}

struct result_item knn_max_heap_find_max(struct knn_max_heap *h) {
    if (h->size == 0) {
        printf("\nHeap is empty.");
        h->arr[0].distance = INT_MAX;
    }
    return h->arr[0];
}

void print_knn_max_heap(struct knn_max_heap *h) {
    printf("total num: %d\n", h->size);
    sort_buffer(h->arr, h->size);
    for (int i = 0; i < h->size; i++) {
        struct result_item item = h->arr[i];
        /*printf("oid: %d, lon: %d, lat: %d, time: %d, dist: %d\n",
               item.point->oid, item.point->normalized_longitude, item.point->normalized_latitude, item.point->timestamp_sec, item.distance);*/
        printf("oid: %d, lon: %d, lat: %d, time: %d, dist: %ld\n",
               item.point.oid, item.point.normalized_longitude, item.point.normalized_latitude, item.point.timestamp_sec, item.distance);
    }
}



struct buffered_knn_max_heap* create_buffered_knn_max_heap(int capacity, int buffer_factor) {
    struct buffered_knn_max_heap* bh = (struct buffered_knn_max_heap*)malloc(sizeof(struct buffered_knn_max_heap));

    if (bh == NULL) {
        printf("Memory error");
        return NULL;
    }

    struct knn_max_heap* h = create_knn_max_heap(capacity);

    bh->h = h;
    bh->buffer_factor = buffer_factor;
    bh->buffer_size = 0;
    bh->buffer_capacity = (int)(capacity * (1 - 1.0/buffer_factor) * (buffer_factor - 1));
    bh->is_max_dist_ref_initialized = false;
    bh->result_buffer = (struct result_item *)malloc(bh->buffer_capacity * sizeof(struct result_item));
    bh->max_distance_ref = LONG_MAX;

    bh->statistics.non_discard_count = 0;
    bh->statistics.discard_count = 0;
    bh->statistics.add_to_buffer_item_count = 0;
    bh->statistics.add_to_heap_item_count = 0;
    bh->statistics.heap_sawp_operation_count = 0;
    return bh;
}

void free_buffered_knn_max_heap(struct buffered_knn_max_heap *bh) {
    free(bh->result_buffer);
    free_knn_max_heap(bh->h);
    free(bh);
}

void buffered_knn_max_heap_insert(struct buffered_knn_max_heap *bh, struct result_item *item) {
    struct knn_max_heap *h = bh->h;
    struct result_item *b = bh->result_buffer;
    if (h->size < h->capacity) {
        knn_max_heap_insert(h, item);
        bh->statistics.add_to_heap_item_count++;
    } else {
        /*if (!bh->is_max_dist_ref_initialized) {
            bh->max_distance_ref = knn_max_heap_find_max(h).distance;
            bh->is_max_dist_ref_initialized = true;
        }*/

        // heap is full, so we add items to the buffer if their distances are larger than the split distance
        long split_dist = bh->max_distance_ref / bh->buffer_factor;
        //long split_dist = h->arr[bh->buffer_factor - 1].distance;
        if (item->distance > split_dist) {
            // put to buffer
            if (bh->buffer_size < bh->buffer_capacity) {
                b[bh->buffer_size] = *item;
                bh->buffer_size++;
                bh->statistics.add_to_buffer_item_count++;
            } else {
                long current_max_dist = h->arr[0].distance;
                if (current_max_dist < split_dist) {
                    // all items in the buffer can be discarded
                    bh->buffer_size = 0;
                    bh->max_distance_ref = current_max_dist;
                    bh->statistics.discard_count++;
                    //printf("i am here buffer clear\n");
                } else {
                    if (item->distance < h->arr[0].distance) {
                        knn_max_heap_replace(h, item);
                        bh->statistics.add_to_heap_item_count++;
                    }

                    for (int i = 0; i < bh->buffer_size; i++) {
                        if (b[i].distance < h->arr[0].distance) {
                            struct result_item *item1 = &b[i];
                            knn_max_heap_replace(h, &b[i]);
                            bh->statistics.add_to_heap_item_count++;
                        }
                    }
                    bh->buffer_size = 0;
                    bh->max_distance_ref = h->arr[0].distance;
                    bh->statistics.non_discard_count++;
                    //printf("i am here\n");
                }
            }
        } else {
            // put to heap
            knn_max_heap_replace(h, item);
            bh->statistics.add_to_heap_item_count++;

            if (h->arr[0].distance < split_dist) {
                bh->buffer_size = 0;
                bh->max_distance_ref = h->arr[0].distance;
                bh->statistics.discard_count++;
            }
        }

    }
}

void buffered_knn_max_heap_compact(struct buffered_knn_max_heap *bh) {
    struct knn_max_heap *h = bh->h;
    struct result_item *b = bh->result_buffer;
    for (int i = 0; i < bh->buffer_size; i++) {
        if (b[i].distance < h->arr[0].distance) {
            knn_max_heap_replace(h, &b[i]);
            bh->statistics.add_to_heap_item_count++;
        }
    }
    bh->buffer_size = 0;
    bh->statistics.non_discard_count++;
}





struct knnjoin_max_heap* create_knnjoin_max_heap(int capacity) {
    // Allocating memory to heap h
    struct knnjoin_max_heap* h = (struct knnjoin_max_heap*)malloc(sizeof(struct knnjoin_max_heap));

    // Checking if memory is allocated to h or not
    if (h == NULL) {
        printf("Memory error");
        return NULL;
    }
    // set the values to size and capacity
    h->size = 0;
    h->capacity = capacity;

    // Allocating memory to array
    h->arr = (struct knnjoin_result_item*)malloc(capacity * sizeof(struct knnjoin_result_item));

    // Checking if memory is allocated to h or not
    if (h->arr == NULL) {
        printf("Memory error");
        return NULL;
    }

    return h;
}

void free_knnjoin_max_heap(struct knnjoin_max_heap *h) {
    free(h->arr);
    free(h);
}


static
void knnjoin_max_heap_insert_helper(struct knnjoin_max_heap *h, int index) {
    // Store parent of element at index
    // in parent variable
    int parent = (index - 1) / 2;

    if (h->arr[parent].distance < h->arr[index].distance) {
        // Swapping when child is smaller
        // than parent element
        struct knnjoin_result_item temp = h->arr[parent];
        h->arr[parent] = h->arr[index];
        h->arr[index] = temp;

        // Recursively calling maxHeapify_bottom_up
        knnjoin_max_heap_insert_helper(h, parent);
    }
}

static
void knnjoin_max_heap_heapify(struct knnjoin_max_heap *h, int index) {
    int left = index * 2 + 1;
    int right = index * 2 + 2;
    int max = index;

    // Checking whether our left or child element
    // is at right index of not to avoid index error
    if (left >= h->size || left < 0)
        left = -1;
    if (right >= h->size || right < 0)
        right = -1;

    // store left or right element in max if
    // any of these is smaller that its parent
    if (left != -1 && h->arr[left].distance > h->arr[max].distance) {
        max = left;
    }
    if (right != -1 && h->arr[right].distance > h->arr[max].distance) {
        max = right;
    }

    // Swapping the nodes
    if (max != index) {
        h->statistics.heap_sawp_operation_count++;

        struct knnjoin_result_item temp = h->arr[max];
        h->arr[max] = h->arr[index];
        h->arr[index] = temp;

        // recursively calling for their child elements
        // to maintain max heap
        knnjoin_max_heap_heapify(h, max);

    }
}


struct knnjoin_result_item knnjoin_max_heap_extract_max(struct knnjoin_max_heap *h) {
    struct knnjoin_result_item deleteItem;

    // Checking if the heap is empty or not
    if (h->size == 0) {
        printf("\nHeap id empty.");
        return deleteItem;
    }

    // Store the node in deleteItem that
    // is to be deleted.
    deleteItem = h->arr[0];

    // Replace the deleted node with the last node
    h->arr[0] = h->arr[h->size - 1];
    // Decrement the size of heap
    h->size--;

    // Call maxheapify_top_down for 0th index
    // to maintain the heap property
    knnjoin_max_heap_heapify(h, 0);
    return deleteItem;
}

void knnjoin_max_heap_insert(struct knnjoin_max_heap *h, struct knnjoin_result_item *item) {
    // Checking if heap is full or not
    if (h->size < h->capacity) {
        // Inserting data into an array
        h->arr[h->size] = *item;
        // Calling maxHeapify_bottom_up function
        knnjoin_max_heap_insert_helper(h, h->size);
        // Incrementing size of array
        h->size++;
    }
}

void knnjoin_max_heap_replace(struct knnjoin_max_heap *h, struct knnjoin_result_item *item) {
    // Checking if the heap is empty or not
    if (h->size == 0) {
        printf("\nHeap is empty.");
        return;
    }


    // Replace the root node with the newly added node
    h->arr[0] = *item;

    // Call maxheapify_top_down for 0th index
    // to maintain the heap property
    knnjoin_max_heap_heapify(h, 0);
}

struct knnjoin_result_item knnjoin_max_heap_find_max(struct knnjoin_max_heap *h) {
    if (h->size == 0) {
        printf("\nHeap is empty.");
        h->arr[0].distance = INT_MAX;
    }
    return h->arr[0];
}

void print_knnjoin_max_heap(struct knnjoin_max_heap *h) {
    printf("total num: %d\n", h->size);
    sort_knnjoin_buffer(h->arr, h->size);
    for (int i = 0; i < h->size; i++) {
        struct knnjoin_result_item item = h->arr[i];
        /*printf("oid: %d, lon: %d, lat: %d, time: %d, dist: %d\n",
               item.point->oid, item.point->normalized_longitude, item.point->normalized_latitude, item.point->timestamp_sec, item.distance);*/
        printf("(oid: %d, lon: %d, lat: %d, time: %d), (oid: %d, lon: %d, lat: %d, time: %d), dist: %ld\n",
               item.point1.oid, item.point1.normalized_longitude, item.point1.normalized_latitude, item.point1.timestamp_sec,
               item.point2.oid, item.point2.normalized_longitude, item.point2.normalized_latitude, item.point2.timestamp_sec,
               item.distance);
    }
}








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
    } else if (point->normalized_longitude > mbr->lon_max){
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
