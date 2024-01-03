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

#define ARMV8_PMEVTYPER_EVTCOUNT_MASK 0x3ff

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





static void spatio_temporal_query_page(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("[ISP] begin spatio temporal query page: data_block ptr: %p\n", data_block);
	/*init_perfcounters();
	init_perfcounters_instr();
	int count1, count2;
	count1 = read_counter_instr();*/

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


                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;

                }

            }

        }

    }
    /*count2 = read_counter_instr();
    xil_printf("full: %d\n", (count2 - count1));*/


}

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

	int result_count = 0;

    // parse data block
    struct traj_block_header block_header;
    parse_traj_block_for_header(data_block, &block_header);

    struct seg_meta meta_array[block_header.seg_count];
    parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

    int traj_point_size = get_traj_point_size();


    //result_start = timer_get_cycles();
    struct traj_point *buffer_dst = (struct traj_point*)((char*)output_buffer->iov[0].iov_base + 4);

    for (int j = 0; j < block_header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];

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

    }

    //result_stop = timer_get_cycles();
    //xil_printf("result take: %d\n", (result_stop - result_start));
}


static void spatio_temporal_query_page_naive(void *data_block, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

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
        if (descriptor->lon_min <= seg_point->normalized_longitude
            && descriptor->lon_max >= seg_point->normalized_longitude
            && descriptor->lat_min <= seg_point->normalized_latitude
            && descriptor->lat_max >= seg_point->normalized_latitude
            && descriptor->time_min <= seg_point->timestamp_sec
			&& descriptor->time_max >= seg_point->timestamp_sec) {


                	buffer_dst[output_buffer->current_tuple_count] = *seg_point;
                	output_buffer->used_bytes_count += traj_point_size;
                	output_buffer->current_tuple_count++;
               	//result_count++;
       }


    }

    //result_stop = timer_get_cycles();
    //xil_printf("result take: %d\n", (result_stop - result_start));
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

        	spatio_temporal_query_page_naive(iov_item.iov_base + offset, output_buffer, descriptor);

        }
    }


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
        		id_temporal_query_page_without_meta_filtering(iov_item.iov_base + offset, output_buffer, descriptor);
        	}
        }
    }
}

void run_spatio_temporal_range_query(struct iovec* iov, size_t nr_segs, struct isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {

	int count1, count2;
	bool with_meta_filtering = true;

	timestamp_t result_start, result_stop;
	result_start = timer_get_cycles();
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += ISP_BUFFER_PAGE_SIZE) {
        	if (with_meta_filtering) {
        		spatio_temporal_query_page(iov_item.iov_base + offset, output_buffer, descriptor);
        	} else {
        		spatio_temporal_query_page_without_meta_filtering(iov_item.iov_base + offset, output_buffer, descriptor);
        	}
        }
    }
    result_stop = timer_get_cycles();
    //xil_printf("result take: %d\n", (result_stop - result_start));

}

