//
// Created by yangguo on 12/14/22.
//

#include "groundhog/isp_query_engine.h"
#include <stdbool.h>
#include "groundhog/bloom/bloom.h"
#include "groundhog/bloom/bitutil.h"
#include <types.h>
#include "const.h"
#include "barrier.h"
#include "groundhog/knn_util.h"
#include <limits.h>
#include "tls.h"

#define ARMV8_PMEVTYPER_EVTCOUNT_MASK 0x3ff

//#define TLS_BASE_SECTION ".tlsdata"
//#define DEFINE_TLS(type, name) \
//    __attribute__((section(TLS_BASE_SECTION))) __typeof__(type) name

//static DEFINE_TLS(struct result_item, result_item_tmp_buffer[65536]);

static inline void init_perfcounters(void)
{
	/*Enable user-mode access to counters. */
	asm volatile("msr pmuserenr_el0, %0" : : "r"(0xf));

	/*   Performance Monitors Count Enable Set register bit 30:0 disable, 31 enable. Can also enable other event counters here. */
	asm volatile("msr pmcntenset_el0, %0" : : "r" ((u32)(1<<31)));

	/* Enable counters */
	u64 val=0;
	asm volatile("mrs %0, pmcr_el0" : "=r" (val));
	asm volatile("msr pmcr_el0, %0" : : "r" (val|(1 << 0)));

}

static inline void init_perfcounters_instr(void) {
	/* Setup PMU counter to record specific event */
	/* evtCount is the event id */
	uint32_t evtCount = 0x08;	// Instruction architecturally executed
	evtCount &= ARMV8_PMEVTYPER_EVTCOUNT_MASK;
	asm volatile("isb");
	/* Just use counter 0 here */
	asm volatile("msr pmevtyper0_el0, %0" : : "r" (evtCount));
	/*   Performance Monitors Count Enable Set register bit 30:1 disable, 31,1 enable */
	uint32_t r = 0;
	asm volatile("mrs %0, pmcntenset_el0" : "=r" (r));
	asm volatile("msr pmcntenset_el0, %0" : : "r" (r|1));
}

static inline int read_counter() {
	isb();
	u64 cval;
	asm volatile("mrs %0, PMCCNTR_EL0" : "=r"(cval));
	return cval;
}

static inline int read_counter_instr() {
	isb();
	u64 cval;
	asm volatile("mrs %0, pmevcntr0_el0" : "=r" (cval));
	return cval;
}


// not used
static void id_temporal_query_page_s1(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin id temporal query page: data_block ptr: %p\n", data_block);
	init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    //xil_printf("[ISP] header parsing finished: seg_count: %d\n", block_header.seg_count);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    //xil_printf("[ISP] seg_meta section parsing finished\n");
    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);
    /*for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        //xil_printf("[ISP] seg_meta index: %d, seg offset: %d, seg size: %d, time_min: %d, time_max: %d\n", j, meta_item.seg_offset, meta_item.seg_size, meta_item.time_min, meta_item.time_max);
        bloom_filter rebuild_filter;
        bit_vect rebuild_bit_vec;
        void* bit_mem = meta_item.oid_filter;
        bloom_filter_rebuild_default(&rebuild_filter, &rebuild_bit_vec, bit_mem, MY_OID_FILTER_SIZE * 8);
        bool oid_contained = bloom_filter_test(&rebuild_filter, &descriptor->oid, 4);

        if (oid_contained && descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;
                //xil_printf("point src ptr: %p\n", source);
                //deserialize_traj_point(source, &seg_point);
                seg_point = (struct traj_point *) source;

                if (seg_point->oid == descriptor->oid
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                     //xil_printf("[ISP] oid: %d, timestamp_sec: %d, lon: %d, lat: %d\n", seg_point.oid, seg_point.timestamp_sec, seg_point.normalized_longitude, seg_point.normalized_latitude);
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);

                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;
                }
            }
        }
    }*/

    count2 = read_counter_instr();
    xil_printf("full num: %d\n", (count2 - count1));
}

// not used
static void id_temporal_query_page_s1_s2(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin id temporal query page: data_block ptr: %p\n", data_block);
	init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    //xil_printf("[ISP] header parsing finished: seg_count: %d\n", block_header.seg_count);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    //xil_printf("[ISP] seg_meta section parsing finished\n");
    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        //xil_printf("[ISP] seg_meta index: %d, seg offset: %d, seg size: %d, time_min: %d, time_max: %d\n", j, meta_item.seg_offset, meta_item.seg_size, meta_item.time_min, meta_item.time_max);
        bloom_filter rebuild_filter;
        bit_vect rebuild_bit_vec;
        void* bit_mem = meta_item.oid_filter;
        bloom_filter_rebuild_default(&rebuild_filter, &rebuild_bit_vec, bit_mem, MY_OID_FILTER_SIZE * 8);
        bool oid_contained = bloom_filter_test(&rebuild_filter, &descriptor->oid, 4);

        if (oid_contained && descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min) {
            /*int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;
                //xil_printf("point src ptr: %p\n", source);
                //deserialize_traj_point(source, &seg_point);
                seg_point = (struct traj_point *) source;

                if (seg_point->oid == descriptor->oid
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                     //xil_printf("[ISP] oid: %d, timestamp_sec: %d, lon: %d, lat: %d\n", seg_point.oid, seg_point.timestamp_sec, seg_point.normalized_longitude, seg_point.normalized_latitude);
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);

                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;
                }
            }*/
        }
    }

    count2 = read_counter_instr();
    xil_printf("full num: %d\n", (count2 - count1));
}

// not used
static void id_temporal_query_page_s1_s2_s3(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin id temporal query page: data_block ptr: %p\n", data_block);
	init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    //xil_printf("[ISP] header parsing finished: seg_count: %d\n", block_header.seg_count);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    //xil_printf("[ISP] seg_meta section parsing finished\n");
    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        //xil_printf("[ISP] seg_meta index: %d, seg offset: %d, seg size: %d, time_min: %d, time_max: %d\n", j, meta_item.seg_offset, meta_item.seg_size, meta_item.time_min, meta_item.time_max);
        bloom_filter rebuild_filter;
        bit_vect rebuild_bit_vec;
        void* bit_mem = meta_item.oid_filter;
        bloom_filter_rebuild_default(&rebuild_filter, &rebuild_bit_vec, bit_mem, MY_OID_FILTER_SIZE * 8);
        bool oid_contained = bloom_filter_test(&rebuild_filter, &descriptor->oid, 4);

        if (oid_contained && descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;
                //xil_printf("point src ptr: %p\n", source);
                //deserialize_traj_point(source, &seg_point);
                seg_point = (struct traj_point *) source;

                if (seg_point->oid == descriptor->oid
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                     //xil_printf("[ISP] oid: %d, timestamp_sec: %d, lon: %d, lat: %d\n", seg_point.oid, seg_point.timestamp_sec, seg_point.normalized_longitude, seg_point.normalized_latitude);
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);

                	/*buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;*/
                }
            }
        }
    }

    count2 = read_counter_instr();
    xil_printf("full num: %d\n", (count2 - count1));
}

// not used
static void id_temporal_query_page_without_meta_filtering(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin id temporal query page: data_block ptr: %p\n", data_block);
    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    //xil_printf("[ISP] header parsing finished: seg_count: %d\n", block_header.seg_count);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    //xil_printf("[ISP] seg_meta section parsing finished\n");
    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];

            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;
                //xil_printf("point src ptr: %p\n", source);

                seg_point = (struct traj_point *) source;

                if (seg_point->oid == descriptor->oid
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                     //xil_printf("[ISP] oid: %d, timestamp_sec: %d, lon: %d, lat: %d\n", seg_point.oid, seg_point.timestamp_sec, seg_point.normalized_longitude, seg_point.normalized_latitude);
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);

                    buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                    output_buffer->used_bytes_count += traj_point_size;
                    output_buffer->current_tuple_count++;
                }
            }
        //}
    }
}

// not used
static void spatio_temporal_query_page_s1(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin spatio temporal query page: data_block ptr: %p\n", data_block);

	init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    /*for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        if (descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min
            && descriptor->lon_min <= meta_item.lon_max && descriptor->lon_max >= meta_item.lon_min
            && descriptor->lat_min <= meta_item.lat_max && descriptor->lat_max >= meta_item.lat_min) {


            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;

                seg_point = (struct traj_point *) source;

                if (descriptor->lon_min <= seg_point->normalized_longitude
                    && descriptor->lon_max >= seg_point->normalized_longitude
                    && descriptor->lat_min <= seg_point->normalized_latitude
                    && descriptor->lat_max >= seg_point->normalized_latitude
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);


                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;

                }
            }
        }
    }*/
    count2 = read_counter_instr();
    xil_printf("s1 num: %d\n", (count2 - count1));

}

// not used
static void spatio_temporal_query_page_s1_s2(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin spatio temporal query page: data_block ptr: %p\n", data_block);

	init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        if (descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min
            && descriptor->lon_min <= meta_item.lon_max && descriptor->lon_max >= meta_item.lon_min
            && descriptor->lat_min <= meta_item.lat_max && descriptor->lat_max >= meta_item.lat_min) {


            /*int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;

                seg_point = (struct traj_point *) source;

                if (descriptor->lon_min <= seg_point->normalized_longitude
                    && descriptor->lon_max >= seg_point->normalized_longitude
                    && descriptor->lat_min <= seg_point->normalized_latitude
                    && descriptor->lat_max >= seg_point->normalized_latitude
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);


                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;

                }
            }*/
        }
    }
    count2 = read_counter_instr();
    xil_printf("s1 s2 num: %d\n", (count2 - count1));

}

// not used
static void spatio_temporal_query_page_s1_s2_s3(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin spatio temporal query page: data_block ptr: %p\n", data_block);
	int tmp = 0;
	init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        if (descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min
            && descriptor->lon_min <= meta_item.lon_max && descriptor->lon_max >= meta_item.lon_min
            && descriptor->lat_min <= meta_item.lat_max && descriptor->lat_max >= meta_item.lat_min) {


            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;

                seg_point = (struct traj_point *) source;

                if (descriptor->lon_min <= seg_point->normalized_longitude
                    && descriptor->lon_max >= seg_point->normalized_longitude
                    && descriptor->lat_min <= seg_point->normalized_latitude
                    && descriptor->lat_max >= seg_point->normalized_latitude
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);
                	tmp++;

                	/*buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;*/

                }
            }
        }
    }
    count2 = read_counter_instr();
    xil_printf("s1 s2 s3 num: %d\n", (count2 - count1));

}


static void spatio_temporal_query_page_without_meta_filtering(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);


    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);


    struct isp_descriptor descriptor_data = *descriptor;
    for (int j = 0; j < block_header.seg_count; j++) {


        struct seg_meta meta_item = meta_array[j];
        /*if (descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min
            && descriptor->lon_min <= meta_item.lon_max && descriptor->lon_max >= meta_item.lon_min
            && descriptor->lat_min <= meta_item.lat_max && descriptor->lat_max >= meta_item.lat_min) {*/


            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {

                void* source = seg_data_base + traj_point_size * i;

                seg_point = (struct traj_point *) source;

                if (descriptor_data.lon_min <= seg_point->normalized_longitude
                    && descriptor_data.lon_max >= seg_point->normalized_longitude
                    && descriptor_data.lat_min <= seg_point->normalized_latitude
                    && descriptor_data.lat_max >= seg_point->normalized_latitude
                    && descriptor_data.time_min <= seg_point->timestamp_sec
                    && descriptor_data.time_max >= seg_point->timestamp_sec) {
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);


                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;

                }

            }

        //}

    }
}

// not used
static void spatio_temporal_query_page_without_meta_filtering_without_parse(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

	int result_count = 0;

    /*// parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);*/

    int traj_point_size = get_traj_point_size();
    int point_num = (4096 - 488) / traj_point_size;
    char* data_base = (char*)data_block + 488;

    //result_start = timer_get_cycles();
    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);


    for (int j = 0; j < point_num; j++) {

            char *seg_data_base = data_base;
            struct traj_point *seg_point;
            void* source = seg_data_base + traj_point_size * j;
            seg_point = (struct traj_point *) source;

                if (descriptor->lon_min <= seg_point->normalized_longitude
                    && descriptor->lon_max >= seg_point->normalized_longitude
                    && descriptor->lat_min <= seg_point->normalized_latitude
                    && descriptor->lat_max >= seg_point->normalized_latitude
                    && descriptor->time_min <= seg_point->timestamp_sec
					&& descriptor->time_max >= seg_point->timestamp_sec) {
                	//put_point_data_to_isp_output_buffer_new_format(output_buffer, source, traj_point_size);

                	/*if (unlikely((output_buffer->used_bytes_count + traj_point_size >= output_buffer->iov_capacity * ISP_BUFFER_PAGE_SIZE))) {
                	    xil_printf("exceed isp result buffer\n");
                		return 0;
                	}*/

                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;
                	//result_count++;
                }


    }

    //result_stop = timer_get_cycles();
    //xil_printf("result take: %d\n", (result_stop - result_start));
}


static void id_temporal_query_page(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin id temporal query page: data_block ptr: %p\n", data_block);
	/*init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();*/

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);
    //xil_printf("[ISP] header parsing finished: seg_count: %d\n", block_header.seg_count);
    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);
    //xil_printf("[ISP] seg_meta section parsing finished\n");
    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        //xil_printf("[ISP] seg_meta index: %d, seg offset: %d, seg size: %d, time_min: %d, time_max: %d\n", j, meta_item.seg_offset, meta_item.seg_size, meta_item.time_min, meta_item.time_max);
        bloom_filter rebuild_filter;
        bit_vect rebuild_bit_vec;
        void* bit_mem = meta_item.oid_filter;
        bloom_filter_rebuild_default(&rebuild_filter, &rebuild_bit_vec, bit_mem, MY_OID_FILTER_SIZE * 8);
        bool oid_contained = bloom_filter_test(&rebuild_filter, &descriptor->oid, 4);

        if (oid_contained && descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            for (int i = 0; i < data_seg_points_num; i++) {
                void* source = seg_data_base + traj_point_size * i;
                //xil_printf("point src ptr: %p\n", source);
                //deserialize_traj_point(source, &seg_point);
                seg_point = (struct traj_point *) source;

                if (seg_point->oid == descriptor->oid
                    && descriptor->time_min <= seg_point->timestamp_sec
                    && descriptor->time_max >= seg_point->timestamp_sec) {
                     //xil_printf("[ISP] oid: %d, timestamp_sec: %d, lon: %d, lat: %d\n", seg_point.oid, seg_point.timestamp_sec, seg_point.normalized_longitude, seg_point.normalized_latitude);
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);

                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;
                }
            }
        }
    }

    /*count2 = read_counter_instr();
    xil_printf("full num: %d\n", (count2 - count1));*/
}

static int checked_count = 0;
static int total_count = 0;
static int evaluation_count = 0;
static void spatio_temporal_query_page(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);


    /*char* b = (char *) data_block;
    int seg_meta_size = get_seg_meta_size();
    char *meta_ptr;*/

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    int point_num = 0;

    struct isp_descriptor descriptor_data = *descriptor;
    for (int j = 0; j < block_header.seg_count; j++) {
    	total_count++;

    	/*meta_ptr = b + 8 + j * seg_meta_size;	// 8 is header size;
    	struct seg_meta meta_item = *(struct seg_meta *)meta_ptr;*/

        struct seg_meta meta_item = meta_array[j];
        if (descriptor_data.time_min <= meta_item.time_max && descriptor_data.time_max >= meta_item.time_min
            && descriptor_data.lon_min <= meta_item.lon_max && descriptor_data.lon_max >= meta_item.lon_min
            && descriptor_data.lat_min <= meta_item.lat_max && descriptor_data.lat_max >= meta_item.lat_min) {

        	checked_count++;
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            char *seg_data_base = (char*)data_block + meta_item.seg_offset;
            struct traj_point *seg_point;
            point_num += data_seg_points_num;

            for (int i = 0; i < data_seg_points_num; i++) {

                void* source = seg_data_base + traj_point_size * i;

                seg_point = (struct traj_point *) source;
                evaluation_count++;
                if (descriptor_data.lon_min <= seg_point->normalized_longitude
                    && descriptor_data.lon_max >= seg_point->normalized_longitude
                    && descriptor_data.lat_min <= seg_point->normalized_latitude
                    && descriptor_data.lat_max >= seg_point->normalized_latitude
                    && descriptor_data.time_min <= seg_point->timestamp_sec
                    && descriptor_data.time_max >= seg_point->timestamp_sec) {
                    //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);


                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;

                }

            }

        }

    }
    //xil_printf("evaluation count: %d\n", evaluation_count);
    //xil_printf("total point num: %d\n\n\n", point_num);
    //xil_printf("total seg count: %d, checked: %d\n", total_count, checked_count);


}

static int naive_checked_count = 0;
static void spatio_temporal_query_page_naive(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

	int result_count = 0;
	naive_checked_count++;

	int header_size = get_header_size();
	struct traj_block_header block_header;
	//parse_traj_block_for_header(data_block, &block_header);
	//int data_offset = header_size + block_header.seg_count * get_seg_meta_size();
	int data_offset = 584;
	//struct traj_point *point_ptr = (struct traj_point *)((char*) data_block + data_offset);
	char *seg_data_base = ((char*) data_block + data_offset);


	int traj_point_size = get_traj_point_size();
	//int point_num = (4096 - data_offset) / traj_point_size;
	int point_num = 219;


    //result_start = timer_get_cycles();
    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    struct isp_descriptor descriptor_data = *descriptor;
    for (int j = 0; j < point_num; j++) {

    	//struct traj_point *seg_point = point_ptr+j;

    	void* source = seg_data_base + traj_point_size * j;
    	struct traj_point *seg_point = (struct traj_point *) source;


        if (descriptor_data.lon_min <= seg_point->normalized_longitude
            && descriptor_data.lon_max >= seg_point->normalized_longitude
            && descriptor_data.lat_min <= seg_point->normalized_latitude
            && descriptor_data.lat_max >= seg_point->normalized_latitude
            && descriptor_data.time_min <= seg_point->timestamp_sec
            && descriptor_data.time_max >= seg_point->timestamp_sec) {
            //put_point_data_to_isp_output_buffer(output_buffer, source, traj_point_size);


        	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
        	output_buffer->used_bytes_count += traj_point_size;
        	output_buffer->current_tuple_count++;

        }


    }

    //xil_printf("naive checked count: %d\n", naive_checked_count);


}

static void id_temporal_query_page_naive(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

	int result_count = 0;

	int header_size = get_header_size();
	struct traj_block_header block_header;
	parse_traj_block_for_header(data_block, &block_header);
	int data_offset = header_size + block_header.seg_count * get_seg_meta_size();

	struct traj_point *point_ptr = (struct traj_point *)((char*) data_block + data_offset);


	int traj_point_size = get_traj_point_size();
	int point_num = (4096 - data_offset) / traj_point_size;


    //result_start = timer_get_cycles();
    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    for (int j = 0; j < point_num; j++) {

    	struct traj_point *seg_point = point_ptr+j;
    	if (seg_point->oid == descriptor->oid
    	    && descriptor->time_min <= seg_point->timestamp_sec
    	    && descriptor->time_max >= seg_point->timestamp_sec) {

    	    buffer_dst[output_buffer->current_tuple_count] = *seg_point;
    	    output_buffer->used_bytes_count += traj_point_size;
    	    output_buffer->current_tuple_count++;
    	}


    }

    //result_stop = timer_get_cycles();
    //xil_printf("result take: %d\n", (result_stop - result_start));
}




static void spatio_temporal_count_query_page_naive(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	int header_size = get_header_size();
	struct traj_block_header block_header;
	parse_traj_block_for_header(data_block, &block_header);
	int data_offset = header_size + block_header.seg_count * get_seg_meta_size();

	struct traj_point *point_ptr = (struct traj_point *)((char*) data_block + data_offset);

	int result_count_vec[1024] = {0};

	int traj_point_size = get_traj_point_size();
	int point_num = (4096 - data_offset) / traj_point_size;
	int count = 0;
	int *buffer_dst = (int*)((char*)output_buffer->iov[0].iov_base);

	int previous_result_index = -1;
	int tmp_count = 0;
	for (int i = 0; i < point_num; i++) {
		int lon_offset = (point_ptr[i].normalized_longitude - descriptor->lon_min) / descriptor->time_min; // normalized value for cell width 0.01 -> 466
		int lat_offset = (point_ptr[i].normalized_latitude - descriptor->lat_min) / descriptor->time_max;	// normalized value for cell width 0.01 -> 932
		int result_index = lon_offset * 30 + lat_offset;	// 30 x 30 grids

		if (result_index < 0 || result_index > 1023) {
			continue;
		}

		buffer_dst[result_index] += 1;

	}


}

static void spatio_temporal_count_query_page(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	int header_size = get_header_size();
	struct traj_block_header block_header;
	parse_traj_block_for_header(data_block, &block_header);
	int data_offset = header_size + block_header.seg_count * get_seg_meta_size();

	struct traj_point *point_ptr = (struct traj_point *)((char*) data_block + data_offset);

	int result_count_vec[1024] = {0};

	int traj_point_size = get_traj_point_size();
	int point_num = (4096 - data_offset) / traj_point_size;
	int count = 0;
	int *buffer_dst = (int*)((char*)output_buffer->iov[0].iov_base);

	int previous_result_index = -1;
	int tmp_count = 0;
	for (int i = 0; i < point_num; i++) {
		int lon_offset = (point_ptr[i].normalized_longitude - descriptor->lon_min) / descriptor->time_min; // normalized value for cell width 0.01 -> 466
		int lat_offset = (point_ptr[i].normalized_latitude - descriptor->lat_min) / descriptor->time_max;	// normalized value for cell width 0.01 -> 932
		int result_index = lon_offset * 30 + lat_offset;	// 30 x 30 grids
		if (result_index < 0 || result_index > 1023) {
			continue;
		}
		if (previous_result_index == result_index) {

			tmp_count++;
		} else {
			buffer_dst[previous_result_index] = buffer_dst[previous_result_index] + tmp_count;
			previous_result_index = result_index;
			tmp_count = 1;
		}

	}

}


static void spatio_temporal_knn_query_page(void *data_block, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    //struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    struct traj_point query_point = {0, descriptor->time_min, descriptor->lon_min, descriptor->lat_min};
    long current_max_dist = output_buffer->max_distance;

    // pruning based on mbr
    long minmaxdist_min = LONG_MAX;
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long minmaxdist = cal_minmax_distance(&query_point, &meta_item);
        if (minmaxdist < minmaxdist_min) {
            minmaxdist_min = minmaxdist;
        }
    }

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long min_dist = cal_min_distance(&query_point, &meta_item);
        if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {

                struct traj_point *point = &point_ptr[k];
                long distance = cal_points_distance(&query_point, point);
                if (distance < output_buffer->max_distance) {
                	//struct result_item item = {*point, distance};
                	struct result_item item = {point, distance};
                    add_item_to_buffer(output_buffer, &item);


                }
            }
        }


    }

}


static void spatio_temporal_knn_query_page_heap_wo_pruning(void *data_block, struct knn_max_heap *output_buffer, struct isp_descriptor *descriptor) {
    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();


    struct traj_point query_point = {0, descriptor->time_min, descriptor->lon_min, descriptor->lat_min};
    //long current_max_dist = knn_max_heap_find_max(output_buffer).distance;

    // pruning based on mbr
    /*long minmaxdist_min = LONG_MAX;
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long minmaxdist = cal_minmax_distance(&query_point, &meta_item);
        if (minmaxdist < minmaxdist_min) {
            minmaxdist_min = minmaxdist;
        }
    }*/

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        //long min_dist = cal_min_distance(&query_point, &meta_item);
        //if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {

                struct traj_point *point = &point_ptr[k];
                long distance = cal_points_distance(&query_point, point);
                //if (distance < knn_max_heap_find_max(output_buffer).distance || output_buffer->size < output_buffer->capacity) {
                if (distance < output_buffer->arr[0].distance || output_buffer->size < output_buffer->capacity) {
                	//struct result_item item = {*point, distance};
                	struct result_item item = {point, distance};
                    if (output_buffer->size < output_buffer->capacity) {
                    	knn_max_heap_insert(output_buffer, &item);
                    	output_buffer->statistics.add_to_heap_item_count++;
                    } else {
                    	knn_max_heap_replace(output_buffer, &item);
                    	output_buffer->statistics.add_to_heap_item_count++;
                    }


                }
            }
        //}


    }

}


static void spatio_temporal_knn_query_page_heap(void *data_block, struct knn_max_heap *output_buffer, struct isp_descriptor *descriptor) {
    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();


    struct traj_point query_point = {0, descriptor->time_min, descriptor->lon_min, descriptor->lat_min};
    //long current_max_dist = knn_max_heap_find_max(output_buffer).distance;

    // pruning based on mbr
    /*long minmaxdist_min = LONG_MAX;
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long minmaxdist = cal_minmax_distance(&query_point, &meta_item);
        if (minmaxdist < minmaxdist_min) {
            minmaxdist_min = minmaxdist;
        }
    }*/

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long min_dist = cal_min_distance(&query_point, &meta_item);
        //if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
        if (min_dist <= output_buffer->arr[0].distance || output_buffer->size < output_buffer->capacity) {
        	int data_seg_points_num = meta_item.seg_size / traj_point_size;
            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {

                struct traj_point *point = &point_ptr[k];
                long distance = cal_points_distance(&query_point, point);
                //if (distance < knn_max_heap_find_max(output_buffer).distance || output_buffer->size < output_buffer->capacity) {
                if (distance < output_buffer->arr[0].distance || output_buffer->size < output_buffer->capacity) {
                	//struct result_item item = {*point, distance};
                	struct result_item item = {point, distance};
                    if (output_buffer->size < output_buffer->capacity) {
                    	knn_max_heap_insert(output_buffer, &item);
                    	output_buffer->statistics.add_to_heap_item_count++;
                    } else {
                    	knn_max_heap_replace(output_buffer, &item);
                    	output_buffer->statistics.add_to_heap_item_count++;
                    }


                }
            }
        }


    }

}


static void spatio_temporal_knn_query_page_heap_optimized(void *data_block, struct buffered_knn_max_heap *output_buffer, struct isp_descriptor *descriptor) {
    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();


    struct traj_point query_point = {0, descriptor->time_min, descriptor->lon_min, descriptor->lat_min};
    //long current_max_dist = knn_max_heap_find_max(output_buffer->h).distance;

    // pruning based on mbr
    /*long minmaxdist_min = LONG_MAX;
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long minmaxdist = cal_minmax_distance(&query_point, &meta_item);
        if (minmaxdist < minmaxdist_min) {
            minmaxdist_min = minmaxdist;
        }
    }*/



    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long min_dist = cal_min_distance(&query_point, &meta_item);
        //if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
        if (min_dist <= output_buffer->h->arr[0].distance || output_buffer->h->size < output_buffer->h->capacity) {
        	int data_seg_points_num = meta_item.seg_size / traj_point_size;
            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {

                struct traj_point *point = &point_ptr[k];
                long distance = cal_points_distance(&query_point, point);
                //if (distance < knn_max_heap_find_max(output_buffer->h).distance || output_buffer->h->size < output_buffer->h->capacity) {
                if (distance < (output_buffer->h)->arr[0].distance || output_buffer->h->size < output_buffer->h->capacity) {

                	//struct result_item item = {*point, distance};
                	struct result_item item = {point, distance};
                    buffered_knn_max_heap_insert(output_buffer, &item);


                }
            }
        }


    }



}



static void spatio_temporal_knn_query_page_naive(void *data_block, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    //struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    struct traj_point query_point = {0, descriptor->time_min, descriptor->lon_min, descriptor->lat_min};
    long current_max_dist = output_buffer->max_distance;

    // pruning based on mbr
    /*long minmaxdist_min = LONG_MAX;
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long minmaxdist = cal_minmax_distance(&query_point, &meta_item);
        if (minmaxdist < minmaxdist_min) {
            minmaxdist_min = minmaxdist;
        }
    }*/

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        //long min_dist = cal_min_distance(&query_point, &meta_item);
        //if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {

                struct traj_point *point = &point_ptr[k];
                long distance = cal_points_distance(&query_point, point);
                if (distance < output_buffer->max_distance) {
                	//struct result_item item = {*point, distance};
                	struct result_item item = {point, distance};
                    add_item_to_buffer_baseline(output_buffer, &item);


                }
            }
        //}


    }

}

static void spatio_temporal_knn_query_page_naive_add_mbr_pruning(void *data_block, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();

    //struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    struct traj_point query_point = {0, descriptor->time_min, descriptor->lon_min, descriptor->lat_min};
    long current_max_dist = output_buffer->max_distance;

    // pruning based on mbr
    long minmaxdist_min = LONG_MAX;
    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long minmaxdist = cal_minmax_distance(&query_point, &meta_item);
        if (minmaxdist < minmaxdist_min) {
            minmaxdist_min = minmaxdist;
        }
    }

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];
        long min_dist = cal_min_distance(&query_point, &meta_item);
        if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
            int data_seg_points_num = meta_item.seg_size / traj_point_size;
            struct traj_point *point_ptr = (struct traj_point *)((char *) data_block + meta_item.seg_offset);

            for (int k = 0; k < data_seg_points_num; k++) {

                struct traj_point *point = &point_ptr[k];
                long distance = cal_points_distance(&query_point, point);
                if (distance < output_buffer->max_distance) {
                	//struct result_item item = {*point, distance};
                	struct result_item item = {point, distance};
                    add_item_to_buffer_baseline(output_buffer, &item);


                }
            }
        }


    }

}


/**
 * we perform knn(r, data_block2) for every r belongs to data_block1
 * @param data_block1
 * @param data_block2
 * @param result_buffer
 * @return
 */

static void spatio_temporal_knn_join_query_page(void* data_block1, void* data_block2, struct knnjoin_result_buffer *result_buffer) {

    int traj_point_size = get_traj_point_size();
    long current_max_dist = result_buffer->max_distance;

    struct traj_block_header block_header1, block_header2;
    parse_traj_block_for_header(data_block1, &block_header1);
    parse_traj_block_for_header(data_block2, &block_header2);

    struct seg_meta meta_array1[block_header1.seg_count];
    parse_traj_block_for_seg_meta_section(data_block1, meta_array1, block_header1.seg_count);
    struct seg_meta meta_array2[block_header2.seg_count];
    parse_traj_block_for_seg_meta_section(data_block2, meta_array2, block_header2.seg_count);



    int data_offset_in_block1 = get_header_size() + block_header1.seg_count * get_seg_meta_size();
    int points_num = 0;
    for (int i = 0; i < block_header1.seg_count; i++) {
        points_num += meta_array1[i].seg_size / traj_point_size;
    }
    struct traj_point *block1_points_ptr = (struct traj_point *)((char*) data_block1 + data_offset_in_block1);



    for (int i = 0; i < points_num; i++) {
        struct traj_point *query_point = &block1_points_ptr[i];

        // pruning based on mbr
        /*long minmaxdist_min = LONG_MAX;
        for (int j = 0; j < block_header2.seg_count; j++) {
            struct seg_meta metaitem = meta_array2[j];
            long minmaxdist = cal_minmax_distance(query_point, &metaitem);
            if (minmaxdist < minmaxdist_min) {
                minmaxdist_min = minmaxdist;
            }
        }*/

        // for each point (block1_points_ptr[i]) in block1, we do the knn query
        for (int j = 0; j < block_header2.seg_count; j++) {

            struct seg_meta meta_item2 = meta_array2[j];
            long min_dist = cal_min_distance(query_point, &meta_item2);
            //if (min_dist <= current_max_dist && min_dist <= minmaxdist_min) {
            if (min_dist <= current_max_dist) {
            	int data_seg_points_num = meta_item2.seg_size / traj_point_size;
                struct traj_point *point_ptr = (struct traj_point *)((char *) data_block2 + meta_item2.seg_offset);

                for (int k = 0; k < data_seg_points_num; k++) {
                    struct traj_point *point = &point_ptr[k];
                    long distance = cal_points_distance(query_point, point);
                    if (distance < result_buffer->max_distance) {
                        struct knnjoin_result_item item = {*query_point, *point, distance};
                        add_item_to_knnjoin_buffer(result_buffer, &item);

                    }
                }
            }
        }
    }

}


static void spatio_temporal_knn_join_query_page_optimized(void* data_block1, void* data_block2, struct knnjoin_result_buffer *result_buffer) {

    int traj_point_size = get_traj_point_size();
    long current_max_dist = result_buffer->max_distance;

    struct traj_block_header block_header1, block_header2;
    parse_traj_block_for_header(data_block1, &block_header1);
    parse_traj_block_for_header(data_block2, &block_header2);

    struct seg_meta meta_array1[block_header1.seg_count];
    parse_traj_block_for_seg_meta_section(data_block1, meta_array1, block_header1.seg_count);
    struct seg_meta meta_array2[block_header2.seg_count];
    parse_traj_block_for_seg_meta_section(data_block2, meta_array2, block_header2.seg_count);



    /*int data_offset_in_block1 = get_header_size() + block_header1.seg_count * get_seg_meta_size();
    int points_num = 0;
    for (int i = 0; i < block_header1.seg_count; i++) {
        points_num += meta_array1[i].seg_size / traj_point_size;
    }
    struct traj_point *block1_points_ptr = (struct traj_point *)((char*) data_block1 + data_offset_in_block1);*/


    // pruning based on mbr
    for (int i = 0; i < block_header1.seg_count; i++) {
    	struct seg_meta meta_item1 = meta_array1[i];
    	int data_seg1_points_num = meta_item1.seg_size / traj_point_size;
    	struct traj_point *seg1_point_ptr = (struct traj_point *)((char *) data_block1 + meta_item1.seg_offset);



        for (int j = 0; j < block_header2.seg_count; j++) {

            struct seg_meta meta_item2 = meta_array2[j];

            if (is_mbr_overlap(&meta_item1, &meta_item2)) {
            	// for each point in segment, we do the knn query

            	for (int m = 0; m < data_seg1_points_num; m++) {
            		struct traj_point *seg1_point = &seg1_point_ptr[m];
            		long min_dist = cal_min_distance(seg1_point, &meta_item2);

            		if (min_dist <= current_max_dist) {
            			int data_seg2_points_num = meta_item2.seg_size / traj_point_size;
            			struct traj_point *seg2_point_ptr = (struct traj_point *)((char *) data_block2 + meta_item2.seg_offset);
						for (int n = 0; n < data_seg2_points_num; n++) {
							struct traj_point *seg2_point = &seg2_point_ptr[n];
							long distance = cal_points_distance(seg1_point, seg2_point);
							if (distance < result_buffer->max_distance) {
								struct knnjoin_result_item item = {*seg1_point, *seg2_point, distance};
								add_item_to_knnjoin_buffer(result_buffer, &item);
							}

						}
            		}
            	}
            }


        }
    }

}

static void spatio_temporal_knn_join_query_page_heap_optimized(void* data_block1, void* data_block2, struct knnjoin_max_heap *result_buffer) {

    int traj_point_size = get_traj_point_size();


    struct traj_block_header block_header1, block_header2;
    parse_traj_block_for_header(data_block1, &block_header1);
    parse_traj_block_for_header(data_block2, &block_header2);

    struct seg_meta meta_array1[block_header1.seg_count];
    parse_traj_block_for_seg_meta_section(data_block1, meta_array1, block_header1.seg_count);
    struct seg_meta meta_array2[block_header2.seg_count];
    parse_traj_block_for_seg_meta_section(data_block2, meta_array2, block_header2.seg_count);


    // pruning based on mbr
    for (int i = 0; i < block_header1.seg_count; i++) {
    	struct seg_meta meta_item1 = meta_array1[i];
    	int data_seg1_points_num = meta_item1.seg_size / traj_point_size;
    	struct traj_point *seg1_point_ptr = (struct traj_point *)((char *) data_block1 + meta_item1.seg_offset);



        for (int j = 0; j < block_header2.seg_count; j++) {

            struct seg_meta meta_item2 = meta_array2[j];

            if (is_mbr_overlap(&meta_item1, &meta_item2)) {
            	// for each point in segment, we do the knn query

            	for (int m = 0; m < data_seg1_points_num; m++) {
            		struct traj_point *seg1_point = &seg1_point_ptr[m];
            		long min_dist = cal_min_distance(seg1_point, &meta_item2);

            		if (min_dist <= result_buffer->arr[0].distance) {
            			int data_seg2_points_num = meta_item2.seg_size / traj_point_size;
            			struct traj_point *seg2_point_ptr = (struct traj_point *)((char *) data_block2 + meta_item2.seg_offset);
						for (int n = 0; n < data_seg2_points_num; n++) {
							struct traj_point *seg2_point = &seg2_point_ptr[n];
							long distance = cal_points_distance(seg1_point, seg2_point);
							if (distance < result_buffer->arr[0].distance) {
								struct knnjoin_result_item item = {*seg1_point, *seg2_point, distance};
								if (result_buffer->size < result_buffer->capacity) {
									knnjoin_max_heap_insert(result_buffer, &item);
								} else {
									knnjoin_max_heap_replace(result_buffer, &item);
								}
								//add_item_to_knnjoin_buffer(result_buffer, &item);
							}

						}
            		}
            	}
            }


        }
    }

}


/**
 * without mbr pruning and sort optimizations
 */
static void spatio_temporal_knn_join_query_page_naive(void* data_block1, void* data_block2, struct knnjoin_result_buffer *result_buffer) {

    int traj_point_size = get_traj_point_size();
    long current_max_dist = result_buffer->max_distance;

    struct traj_block_header block_header1, block_header2;
    parse_traj_block_for_header(data_block1, &block_header1);
    parse_traj_block_for_header(data_block2, &block_header2);

    struct seg_meta meta_array1[block_header1.seg_count];
    parse_traj_block_for_seg_meta_section(data_block1, meta_array1, block_header1.seg_count);
    struct seg_meta meta_array2[block_header2.seg_count];
    parse_traj_block_for_seg_meta_section(data_block2, meta_array2, block_header2.seg_count);



    int data_offset_in_block1 = get_header_size() + block_header1.seg_count * get_seg_meta_size();
    int points_num = 0;
    for (int i = 0; i < block_header1.seg_count; i++) {
        points_num += meta_array1[i].seg_size / traj_point_size;
    }
    struct traj_point *block1_points_ptr = (struct traj_point *)((char*) data_block1 + data_offset_in_block1);



    for (int i = 0; i < points_num; i++) {
        struct traj_point *query_point = &block1_points_ptr[i];


        // for each point (block1_points_ptr[i]) in block1, we do the knn query
        for (int j = 0; j < block_header2.seg_count; j++) {

            struct seg_meta meta_item2 = meta_array2[j];
            //long min_dist = cal_min_distance(query_point, &meta_item2);
            //if (min_dist <= current_max_dist) {
            	int data_seg_points_num = meta_item2.seg_size / traj_point_size;
                struct traj_point *point_ptr = (struct traj_point *)((char *) data_block2 + meta_item2.seg_offset);

                for (int k = 0; k < data_seg_points_num; k++) {
                    struct traj_point *point = &point_ptr[k];
                    long distance = cal_points_distance(query_point, point);
                    if (distance < result_buffer->max_distance) {
                        struct knnjoin_result_item item = {*query_point, *point, distance};
                        add_item_to_knnjoin_buffer_baseline(result_buffer, &item);

                    }
                }
            //}
        }
    }

}

/**
 * without sort optimization
 */
static void spatio_temporal_knn_join_query_page_naive_add_mbr_pruning(void* data_block1, void* data_block2, struct knnjoin_result_buffer *result_buffer) {

    int traj_point_size = get_traj_point_size();
    long current_max_dist = result_buffer->max_distance;

    struct traj_block_header block_header1, block_header2;
    parse_traj_block_for_header(data_block1, &block_header1);
    parse_traj_block_for_header(data_block2, &block_header2);

    struct seg_meta meta_array1[block_header1.seg_count];
    parse_traj_block_for_seg_meta_section(data_block1, meta_array1, block_header1.seg_count);
    struct seg_meta meta_array2[block_header2.seg_count];
    parse_traj_block_for_seg_meta_section(data_block2, meta_array2, block_header2.seg_count);



    int data_offset_in_block1 = get_header_size() + block_header1.seg_count * get_seg_meta_size();
    int points_num = 0;
    for (int i = 0; i < block_header1.seg_count; i++) {
        points_num += meta_array1[i].seg_size / traj_point_size;
    }
    struct traj_point *block1_points_ptr = (struct traj_point *)((char*) data_block1 + data_offset_in_block1);



    for (int i = 0; i < points_num; i++) {
        struct traj_point *query_point = &block1_points_ptr[i];


        // for each point (block1_points_ptr[i]) in block1, we do the knn query
        for (int j = 0; j < block_header2.seg_count; j++) {

            struct seg_meta meta_item2 = meta_array2[j];
            long min_dist = cal_min_distance(query_point, &meta_item2);
            if (min_dist <= current_max_dist) {
            	int data_seg_points_num = meta_item2.seg_size / traj_point_size;
                struct traj_point *point_ptr = (struct traj_point *)((char *) data_block2 + meta_item2.seg_offset);

                for (int k = 0; k < data_seg_points_num; k++) {
                    struct traj_point *point = &point_ptr[k];
                    long distance = cal_points_distance(query_point, point);
                    if (distance < result_buffer->max_distance) {
                        struct knnjoin_result_item item = {*query_point, *point, distance};
                        add_item_to_knnjoin_buffer_baseline(result_buffer, &item);

                    }
                }
            }
        }
    }

}



void run_spatio_temporal_count_query_naive(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	for (int i = 0; i < nr_segs; i++) {
	    struct iovec iov_item = iov[i];
	    for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
	    	spatio_temporal_count_query_page_naive(iov_item.iov_base + offset, output_buffer, descriptor);
	    }
	}
	output_buffer->used_bytes_count = 4096;
}

void run_id_temporal_query_naive(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	id_temporal_query_page_naive(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
}

void run_spatio_temporal_range_query_naive(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {


    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	//spatio_temporal_query_page_naive(iov_item.iov_base + offset, output_buffer, descriptor);
        	spatio_temporal_query_page_without_meta_filtering(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }


}


void run_spatio_temporal_knn_query_naive(struct iovec* iov, size_t nr_segs, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	spatio_temporal_knn_query_page_naive(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
    combine_and_sort(output_buffer);
}

void run_spatio_temporal_knn_query_naive_add_mbr_pruning(struct iovec* iov, size_t nr_segs, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	spatio_temporal_knn_query_page_naive_add_mbr_pruning(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
    combine_and_sort(output_buffer);
}


void run_spatio_temporal_knn_join_query_naive(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
	int count = 0;
	void* data_page1;
	void* data_page2;
	for (int i = 0; i < nr_segs; i++) {
	    struct iovec iov_item = iov[i];
	    for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
	    	if (count % 2 == 0) {
	    		data_page1 = iov_item.iov_base + offset;
	    	} else {
	    		data_page2 = iov_item.iov_base + offset;
	    		spatio_temporal_knn_join_query_page_naive(data_page1, data_page2, output_buffer);
	    	}
	        count++;
	    }
	}
	combine_and_sort_knnjoin(output_buffer);
}

void run_spatio_temporal_knn_join_query_naive_add_mbr_pruning(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
	int count = 0;
	void* data_page1;
	void* data_page2;
	for (int i = 0; i < nr_segs; i++) {
	    struct iovec iov_item = iov[i];
	    for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
	    	if (count % 2 == 0) {
	    		data_page1 = iov_item.iov_base + offset;
	    	} else {
	    		data_page2 = iov_item.iov_base + offset;
	    		spatio_temporal_knn_join_query_page_naive_add_mbr_pruning(data_page1, data_page2, output_buffer);
	    	}
	        count++;
	    }
	}
	combine_and_sort_knnjoin(output_buffer);
}


void run_spatio_temporal_count_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	for (int i = 0; i < nr_segs; i++) {
	    struct iovec iov_item = iov[i];
	    for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
	    	spatio_temporal_count_query_page(iov_item.iov_base + offset, output_buffer, descriptor);
	    }
	}
	output_buffer->used_bytes_count = 4096;
}

void run_id_temporal_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	bool with_meta_filtering = true;
	//xil_printf("[ISP] begin id temporal query: nr_segs: %d\n", nr_segs);
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
        	if (with_meta_filtering) {
        		id_temporal_query_page(iov_item.iov_base + offset, output_buffer, descriptor);
        	} else {
        		// not used
        		id_temporal_query_page_without_meta_filtering(iov_item.iov_base + offset, output_buffer, descriptor);
        	}
        }
    }
}

void run_spatio_temporal_range_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

	int count1, count2;
	bool with_meta_filtering = true;

	timestamp_t result_start, result_stop;
	//result_start = timer_get_cycles();
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	spatio_temporal_query_page(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
    //result_stop = timer_get_cycles();
    //xil_printf("result take: %d\n", (result_stop - result_start));

}

void run_spatio_temporal_knn_query(struct iovec* iov, size_t nr_segs, struct knn_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	spatio_temporal_knn_query_page(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
    combine_and_sort(output_buffer);
}


void run_spatio_temporal_knn_query_heap_wo_pruning(struct iovec* iov, size_t nr_segs, struct knn_max_heap *output_buffer, struct isp_descriptor *descriptor) {
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	spatio_temporal_knn_query_page_heap_wo_pruning(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
}


void run_spatio_temporal_knn_query_heap(struct iovec* iov, size_t nr_segs, struct knn_max_heap *output_buffer, struct isp_descriptor *descriptor) {
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	spatio_temporal_knn_query_page_heap(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
}

void run_spatio_temporal_knn_query_heap_optimized(struct iovec* iov, size_t nr_segs, struct buffered_knn_max_heap *output_buffer, struct isp_descriptor *descriptor) {
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {

        	spatio_temporal_knn_query_page_heap_optimized(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }
    buffered_knn_max_heap_compact(output_buffer);
    //xil_printf("total seg count: %d, checked: %d\n", total_count, checked_count);
    //xil_printf("naive checked count: %d\n", naive_checked_count);

}


void run_spatio_temporal_knn_join_query(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
	int count = 0;
	void* data_page1;
	void* data_page2;
	for (int i = 0; i < nr_segs; i++) {
	    struct iovec iov_item = iov[i];
	    for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
	    	if (count % 2 == 0) {
	    		data_page1 = iov_item.iov_base + offset;
	    	} else {
	    		data_page2 = iov_item.iov_base + offset;
	    		spatio_temporal_knn_join_query_page(data_page1, data_page2, output_buffer);
	    	}
	        count++;
	    }
	}
	combine_and_sort_knnjoin(output_buffer);
}


void run_spatio_temporal_knn_join_query_optimized(struct iovec* iov, size_t nr_segs, struct knnjoin_result_buffer *output_buffer, struct isp_descriptor *descriptor) {
	int count = 0;
	void* data_page1;
	void* data_page2;
	for (int i = 0; i < nr_segs; i++) {
	    struct iovec iov_item = iov[i];
	    for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
	    	if (count % 2 == 0) {
	    		data_page1 = iov_item.iov_base + offset;
	    	} else {
	    		data_page2 = iov_item.iov_base + offset;
	    		spatio_temporal_knn_join_query_page_optimized(data_page1, data_page2, output_buffer);
	    	}
	        count++;
	    }
	}
	combine_and_sort_knnjoin(output_buffer);
}


void run_spatio_temporal_knn_join_query_heap_optimized(struct iovec* iov, size_t nr_segs, struct knnjoin_max_heap *output_buffer, struct isp_descriptor *descriptor) {
	int count = 0;
	void* data_page1;
	void* data_page2;
	for (int i = 0; i < nr_segs; i++) {
	    struct iovec iov_item = iov[i];
	    for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
	    	if (count % 2 == 0) {
	    		data_page1 = iov_item.iov_base + offset;
	    	} else {
	    		data_page2 = iov_item.iov_base + offset;
	    		spatio_temporal_knn_join_query_page_heap_optimized(data_page1, data_page2, output_buffer);
	    	}
	        count++;
	    }
	}
}



