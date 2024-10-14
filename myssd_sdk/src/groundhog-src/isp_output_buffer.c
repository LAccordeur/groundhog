//
// Created by yangguo on 12/14/22.
//

#include "groundhog/isp_output_buffer.h"
#include "groundhog/traj_block_format.h"
#include "const.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void allocate_isp_output_buffer(struct isp_output_buffer *output_buffer, struct iovec *iov, size_t capacity) {
    output_buffer->iov_capacity = capacity;
    output_buffer->iov = iov;

    for (size_t i = 0; i < capacity; i++) {
        output_buffer->iov[i].iov_base = alloc_pages(1, ZONE_PS_DDR_LOW);
        memset(output_buffer->iov[i].iov_base, 0, ISP_BUFFER_PAGE_SIZE);
        //xil_printf("[ISP] allocation iov_base ptr: %p\n", output_buffer->iov[i].iov_base);
        //output_buffer->iov[i].iov_base = malloc(ISP_BUFFER_PAGE_SIZE);
        output_buffer->iov[i].iov_len = ISP_BUFFER_PAGE_SIZE;
    }
    output_buffer->current_iov_index = 0;
    output_buffer->current_iov_offset = 4;  // the first byte are used to record the count of tuples in the page
    output_buffer->current_tuple_count = 0;
    output_buffer->used_bytes_count = 4;
    //xil_printf("[ISP] allocate %d page buffer for pushdown output\n", capacity);
}

/**
 * allocate continuous memory buffer
 */
void allocate_isp_output_buffer_new_format(struct isp_output_buffer *output_buffer, struct iovec *iov, size_t capacity) {
    output_buffer->iov_capacity = capacity;
    output_buffer->iov = iov;

    void* buffer_base = alloc_pages(capacity, ZONE_PS_DDR_LOW);
    for (size_t i = 0; i < capacity; i++) {
        output_buffer->iov[i].iov_base = buffer_base;
        //memset(output_buffer->iov[i].iov_base, 0, ISP_BUFFER_PAGE_SIZE);

        output_buffer->iov[i].iov_len = ISP_BUFFER_PAGE_SIZE;
        buffer_base = (char *)buffer_base + ISP_BUFFER_PAGE_SIZE;
    }
    output_buffer->used_bytes_count = 4; // the first byte are used to record the count of tuples in the page
    output_buffer->current_tuple_count = 0;
}

void free_isp_output_buffer_new_format(struct isp_output_buffer *output_buffer, size_t capacity) {
	free_mem(output_buffer->iov[0].iov_base, capacity * ISP_BUFFER_PAGE_SIZE);
}

void free_isp_output_buffer(struct isp_output_buffer *output_buffer) {
    for (size_t i = 0; i < output_buffer->iov_capacity; i++) {
        free_mem(output_buffer->iov[i].iov_base, ISP_BUFFER_PAGE_SIZE);
        //free(output_buffer->iov[i].iov_base);
    }
    //xil_printf("[ISP] free %d page buffer for pushdown output\n", output_buffer->iov_capacity);
}

size_t put_point_data_to_isp_output_buffer(struct isp_output_buffer *output_buffer, const void *data, size_t data_bytes_count) {

	if ((output_buffer->used_bytes_count >= output_buffer->iov_capacity * ISP_BUFFER_PAGE_SIZE)
        || (output_buffer->used_bytes_count + data_bytes_count >= output_buffer->iov_capacity * ISP_BUFFER_PAGE_SIZE)) {
        return 0;
    }

    if (output_buffer->current_iov_offset + data_bytes_count > ISP_BUFFER_PAGE_SIZE) {
        // this buffer page cannot hold the full data, we first update the count and then skip to the next page
        memcpy(output_buffer->iov[output_buffer->current_iov_index].iov_base, &output_buffer->current_tuple_count, 4);
        if (output_buffer->current_iov_index + 1 >= output_buffer->iov_capacity) {
            return 0;
        }
        output_buffer->current_iov_index = output_buffer->current_iov_index + 1;
        output_buffer->current_iov_offset = 4;
        output_buffer->current_tuple_count = 0;
        output_buffer->used_bytes_count = ISP_BUFFER_PAGE_SIZE * output_buffer->current_iov_index + output_buffer->current_iov_offset;

    }

    int current_iov_index = output_buffer->current_iov_index;
    void *dst = (char *)((output_buffer->iov[current_iov_index]).iov_base) + output_buffer->current_iov_offset;


    /*for (int i = 0; i < data_bytes_count; i += 4)
    	*(unsigned int*)(dst + i) = *(unsigned int*)(data + i);*/
    memcpy(dst, data, data_bytes_count);


    output_buffer->current_iov_offset = output_buffer->current_iov_offset + data_bytes_count;
    output_buffer->current_tuple_count += (data_bytes_count / get_traj_point_size());
    output_buffer->used_bytes_count += data_bytes_count;

    return data_bytes_count;
}

size_t put_point_data_to_isp_output_buffer_new_format(struct isp_output_buffer *output_buffer, const void *data, size_t data_bytes_count) {

	/*if ((output_buffer->used_bytes_count >= output_buffer->iov_capacity * ISP_BUFFER_PAGE_SIZE)
        || (output_buffer->used_bytes_count + data_bytes_count >= output_buffer->iov_capacity * ISP_BUFFER_PAGE_SIZE)) {
        return 0;
    }*/


    struct traj_point *dst = (struct traj_point*)output_buffer->iov[0].iov_base + 4 ;
    struct traj_point *src = (struct traj_point*)data;
    dst[output_buffer->current_tuple_count] = *src;

    output_buffer->used_bytes_count += data_bytes_count;
    output_buffer->current_tuple_count++;

    return data_bytes_count;
}


void format_the_last_isp_buffer_page(struct isp_output_buffer *output_buffer) {
    memcpy(output_buffer->iov[output_buffer->current_iov_index].iov_base, &output_buffer->current_tuple_count, 4);
}


void inline add_result_count_new_format(struct isp_output_buffer *output_buffer) {
	memcpy(output_buffer->iov[0].iov_base, &output_buffer->current_tuple_count, 4);
}

void print_isp_output_buffer_info(struct isp_output_buffer *output_buffer) {
    printf("iov_capacity: %d\n", output_buffer->iov_capacity);
    printf("current_iov_index: %d\n", output_buffer->current_iov_index);
    printf("current_iov_offset: %d\n", output_buffer->current_iov_offset);
    printf("current_tuple_count: %d\n", output_buffer->current_tuple_count);
    printf("used_bytes_count: %d\n", output_buffer->used_bytes_count);
}

