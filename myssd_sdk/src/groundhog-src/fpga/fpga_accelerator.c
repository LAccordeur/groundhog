//
// Created by yangguo on 3/7/23.
//

#include "groundhog/fpga/fpga_accelerator.h"
#include "const.h"
#include "xaxidma.h"
#include "xparameters.h"
#include "xdebug.h"
#include "xdostfiltering.h"
#include "xdoidfiltering.h"
#include "xtime_l.h"
#include "groundhog/traj_block_format.h"
#include "flash_config.h"
#include "groundhog/fpga/fpga_accelerator.h"
#include <groundhog/isp_descriptor.h>
#include <stdbool.h>
#include "groundhog/bloom/bloom.h"
#include "groundhog/bloom/bitutil.h"
#include <types.h>


#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID		// dma for spatio-temporal range query
#define ID_DMA_DEV_ID	XPAR_AXI_DMA_10_DEVICE_ID	// dma for id temporal query


XAxiDma AxiDma;		// dma for spatio-temporal range query
XAxiDma_Config *CfgPtr;		// dma for spatio-temporal range query

XAxiDma IDAxiDma;
XAxiDma_Config *IDCfgPtr;

XDostfiltering doFiltering;
XDostfiltering_Config *doFiltering_cfg;

XDoidfiltering doIDFiltering;
XDoidfiltering_Config *doIDFiltering_cfg;


static int STFiltering_FPGA(void *points_base_addr_ptr, int points_num, void *result_buffer_ptr, int lon_min, int lon_max, int lat_min, int lat_max, int time_min, int time_max);

static u64 my_clock() {
	XTime tCur = 0;
	XTime_GetTime(&tCur);
	return (tCur);
}


void allocate_fpga_isp_output_buffer(struct fpga_isp_output_buffer *output_buffer, struct iovec *iov, size_t capacity) {
    output_buffer->iov_capacity = capacity;
    output_buffer->iov = iov;

    void* buffer_base = alloc_pages(capacity, ZONE_PS_DDR_LOW);	// TODO pre-allocate a fixed memory region for fpga
    for (size_t i = 0; i < capacity; i++) {
        output_buffer->iov[i].iov_base = buffer_base;
        memset(output_buffer->iov[i].iov_base, 0, FPGA_ISP_BUFFER_PAGE_SIZE);

        output_buffer->iov[i].iov_len = FPGA_ISP_BUFFER_PAGE_SIZE;
        buffer_base = (char *)buffer_base + FPGA_ISP_BUFFER_PAGE_SIZE;
    }
    output_buffer->used_bytes_count = 0;

}

void allocate_fpga_isp_input_buffer(struct fpga_isp_input_buffer *input_buffer, size_t page_num) {
	input_buffer->base = alloc_pages(page_num, ZONE_PS_DDR_LOW);
	//input_buffer->base = alloc_vmpages(page_num, ZONE_PL_DDR);
	input_buffer->length = page_num * FPGA_ISP_BUFFER_PAGE_SIZE;
	input_buffer->used_bytes_count = 0;

}

void free_fpga_isp_input_buffer(struct fpga_isp_input_buffer *input_buffer) {
	free_mem(input_buffer->base, input_buffer->length);

}

void put_segment_to_fpga_isp_input_buffer(struct fpga_isp_input_buffer *input_buffer, void* segment, size_t segment_size) {
	if (input_buffer->used_bytes_count + segment_size > input_buffer->length) {
		xil_printf("[fpga accelerator] the data size exceed buffer size. buffer size is %d\n", input_buffer->length);
		return;
	}
	char* buffer_base = (char *)input_buffer->base + input_buffer->used_bytes_count;
	//xil_printf("dst:%p, src: %p, size: %d\n", buffer_base, segment, segment_size);
	memcpy(buffer_base, segment, segment_size);
	//xil_printf("finish\n");
	input_buffer->used_bytes_count += segment_size;
}

/**
 * for spatio-temporal range query
 */
int segment_filter_and_aggregate(void *data_block, struct fpga_isp_input_buffer *input_buffer, struct isp_descriptor *descriptor) {
	int points_count = 0;
	struct traj_block_header block_header;
	parse_traj_block_for_header(data_block, &block_header);

	struct seg_meta meta_array[block_header.seg_count];
	parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);


	int traj_point_size = get_traj_point_size();
	for (int j = 0; j < block_header.seg_count; j++) {
	    struct seg_meta meta_item = meta_array[j];

	    if (descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min
	            && descriptor->lon_min <= meta_item.lon_max && descriptor->lon_max >= meta_item.lon_min
	            && descriptor->lat_min <= meta_item.lat_max && descriptor->lat_max >= meta_item.lat_min) {

	    	int data_seg_points_num = meta_item.seg_size / traj_point_size;
	       points_count += data_seg_points_num;

	        char *seg_data_base = (char*)data_block + meta_item.seg_offset;
	        put_segment_to_fpga_isp_input_buffer(input_buffer, seg_data_base, meta_item.seg_size);

	    }

	}
	return points_count;
}

/**
 * for id temporal query
 */
int segment_filter_and_aggregate_for_id(void *data_block, struct fpga_isp_input_buffer *input_buffer, struct isp_descriptor *descriptor) {
	int points_count = 0;
	struct traj_block_header block_header;
	parse_traj_block_for_header(data_block, &block_header);

	struct seg_meta meta_array[block_header.seg_count];
	parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

	int traj_point_size = get_traj_point_size();
	for (int j = 0; j < block_header.seg_count; j++) {
	    struct seg_meta meta_item = meta_array[j];

	    bloom_filter rebuild_filter;
	    bit_vect rebuild_bit_vec;
	    void* bit_mem = meta_item.oid_filter;
	    bloom_filter_rebuild_default(&rebuild_filter, &rebuild_bit_vec, bit_mem, MY_OID_FILTER_SIZE * 8);
	    bool oid_contained = bloom_filter_test(&rebuild_filter, &descriptor->oid, 4);

	    if (oid_contained && descriptor->time_min <= meta_item.time_max && descriptor->time_max >= meta_item.time_min) {

	    	int data_seg_points_num = meta_item.seg_size / traj_point_size;
	       points_count += data_seg_points_num;

	        char *seg_data_base = (char*)data_block + meta_item.seg_offset;
	        put_segment_to_fpga_isp_input_buffer(input_buffer, seg_data_base, meta_item.seg_size);

	    }
	}
	return points_count;
}

int only_segment_aggregate(void *data_block, struct fpga_isp_input_buffer *input_buffer, struct isp_descriptor *descriptor) {
	int points_count = 0;
	struct traj_block_header block_header;
	parse_traj_block_for_header(data_block, &block_header);

	struct seg_meta meta_array[block_header.seg_count];
	parse_traj_block_for_seg_meta_section(data_block, meta_array, block_header.seg_count);

	int traj_point_size = get_traj_point_size();
	for (int j = 0; j < block_header.seg_count; j++) {
	    struct seg_meta meta_item = meta_array[j];


	    	int data_seg_points_num = meta_item.seg_size / traj_point_size;
	       points_count += data_seg_points_num;

	        char *seg_data_base = (char*)data_block + meta_item.seg_offset;
	        put_segment_to_fpga_isp_input_buffer(input_buffer, seg_data_base, meta_item.seg_size);


	}
	return points_count;
}


static int send_to_fpga_st_filtering(void* input_buffer, size_t points_num, void* output_buffer, int lon_min, int lon_max, int lat_min, int lat_max, int time_min, int time_max) {
	if (points_num <= 0) {
		return 0;
	}
	struct traj_point *points_base_addr = (struct traj_point *)input_buffer;
		struct traj_point *result_buffer = (struct traj_point *)output_buffer;
		u64 start = my_clock();
		int Status;
		init_dma_and_filtering();

		XDostfiltering_Set_lon_min(&doFiltering, lon_min);
		XDostfiltering_Set_lon_max(&doFiltering, lon_max);
		XDostfiltering_Set_lat_min(&doFiltering, lat_min);
		XDostfiltering_Set_lat_max(&doFiltering, lat_max);
		XDostfiltering_Set_time_min(&doFiltering, time_min);
		XDostfiltering_Set_time_max(&doFiltering, time_max);
		XDostfiltering_Start(&doFiltering);

		// Flush the cache of the buffers
		Xil_DCacheFlushRange((u32)points_base_addr, points_num*sizeof(struct traj_point));
		Xil_DCacheFlushRange((u32)result_buffer, points_num*sizeof(struct traj_point));

		//xil_printf("Sending data to IP core slave\n");
		u64 start1 = my_clock();

		Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) points_base_addr,
				points_num*sizeof(struct traj_point), XAXIDMA_DMA_TO_DEVICE);

		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}


		//xil_printf("Get data\n");
		Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) result_buffer,
				points_num*sizeof(struct traj_point), XAXIDMA_DEVICE_TO_DMA);

		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		while ((XAxiDma_Busy(&AxiDma,XAXIDMA_DEVICE_TO_DMA)) ||
				(XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE))) {
								/* Wait */
		}
		u64 start3 = my_clock();

				/* Invalidate the DestBuffer before receiving the data, in case the
					 * Data Cache is enabled
					 */
		Xil_DCacheInvalidateRange((u32)result_buffer, points_num*sizeof(struct traj_point));

		while (!XDostfiltering_IsDone(&doFiltering)) {
			xil_printf("Calculation complete\n");
		}

		int result_count = XDostfiltering_Get_return(&doFiltering);

		/*struct traj_point *check_data_base;
		check_data_base = (struct traj_point *) result_buffer;
		for (int i = 0; i < result_count; i++) {
			xil_printf("data point oid %d\n", check_data_base[i].oid);
		}*/


		u64 end = my_clock();

		/*xil_printf("set configuration take %d\n", (start1 - start));
		xil_printf("data transfer take %d\n", (start3 - start1));
		xil_printf("fpga takes time: %d\n", (end - start));*/
		return result_count;
}

static int send_to_fpga_id_filtering(void* input_buffer, size_t points_num, void* output_buffer, int oid, int time_min, int time_max) {
	if (points_num <= 0) {
		return 0;
	}
	struct traj_point *points_base_addr = (struct traj_point *)input_buffer;
		struct traj_point *result_buffer = (struct traj_point *)output_buffer;
		u64 start = my_clock();
		int Status;
		init_dma_and_filtering_for_id();

		XDoidfiltering_Set_oid(&doIDFiltering, oid);
		XDoidfiltering_Set_time_min(&doIDFiltering, time_min);
		XDoidfiltering_Set_time_max(&doIDFiltering, time_max);
		XDoidfiltering_Start(&doIDFiltering);

		// Flush the cache of the buffers
		Xil_DCacheFlushRange((u32)points_base_addr, points_num*sizeof(struct traj_point));
		Xil_DCacheFlushRange((u32)result_buffer, points_num*sizeof(struct traj_point));

		//xil_printf("Sending data to IP core slave\n");
		u64 start1 = my_clock();

		Status = XAxiDma_SimpleTransfer(&IDAxiDma,(u32) points_base_addr,
				points_num*sizeof(struct traj_point), XAXIDMA_DMA_TO_DEVICE);

		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}


		//xil_printf("Get data\n");
		Status = XAxiDma_SimpleTransfer(&IDAxiDma,(u32) result_buffer,
				points_num*sizeof(struct traj_point), XAXIDMA_DEVICE_TO_DMA);

		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		while ((XAxiDma_Busy(&IDAxiDma,XAXIDMA_DEVICE_TO_DMA)) ||
				(XAxiDma_Busy(&IDAxiDma,XAXIDMA_DMA_TO_DEVICE))) {
								/* Wait */
		}
		u64 start3 = my_clock();

				/* Invalidate the DestBuffer before receiving the data, in case the
					 * Data Cache is enabled
					 */
		Xil_DCacheInvalidateRange((u32)result_buffer, points_num*sizeof(struct traj_point));

		while (!XDoidfiltering_IsDone(&doIDFiltering)) {
			xil_printf("Calculation complete\n");
		}

		int result_count = XDoidfiltering_Get_return(&doIDFiltering);

		/*struct traj_point *check_data_base;
		check_data_base = (struct traj_point *) result_buffer;
		for (int i = 0; i < result_count; i++) {
			xil_printf("data point oid %d\n", check_data_base[i].oid);
		}*/


		u64 end = my_clock();

		/*xil_printf("set configuration take %d\n", (start1 - start));
		xil_printf("data transfer take %d\n", (start3 - start1));
		xil_printf("fpga takes time: %d\n", (end - start));*/
		return result_count;
}

void run_fpga_st_filtering(struct iovec* iov, size_t nr_segs, struct fpga_isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("fpga spatio-temporal filtering\n");
	bool with_meta_filtering = false;

	timestamp_t result_start, result_stop;
	result_start = timer_get_cycles();

	int total_points_count = 0;
	struct fpga_isp_input_buffer input_buffer;
	int page_num_factor = 4;	// nr_segs is the number of ssd page which is 16 kb by default
	allocate_fpga_isp_input_buffer(&input_buffer, nr_segs * page_num_factor);
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += FPGA_ISP_BUFFER_PAGE_SIZE) {
            int points_count = 0;
            if (with_meta_filtering) {
            	points_count = segment_filter_and_aggregate(iov_item.iov_base + offset, &input_buffer, descriptor);
            } else {
            	points_count = only_segment_aggregate(iov_item.iov_base + offset, &input_buffer, descriptor);
            }
            total_points_count += points_count;
        }
    }
    //xil_printf("input addr: %p, total points count: %d, output addr: %p\n", input_buffer.base, total_points_count, output_buffer->iov[0].iov_base);
    char *output_addr = (char *)output_buffer->iov[0].iov_base + 16;	// the first 16 bytes is uesd to record total count and we use 16 because of 16-bytes alignment
    int result_count = send_to_fpga_st_filtering(input_buffer.base, total_points_count, output_addr, descriptor->lon_min, descriptor->lon_max, descriptor->lat_min, descriptor->lat_max, descriptor->time_min, descriptor->time_max);
    output_buffer->used_bytes_count = result_count * sizeof(struct traj_point) + 16;
    memcpy(output_buffer->iov[0].iov_base, &result_count, 4);
    free_fpga_isp_input_buffer(&input_buffer);
    result_stop = timer_get_cycles();
    //xil_printf("result take: %d\n", (result_stop - result_start));
    //xil_printf("result count: %d\n", result_count);
}


void run_fpga_id_filtering(struct iovec* iov, size_t nr_segs, struct fpga_isp_output_buffer *output_buffer, struct isp_descriptor *descriptor) {
	//xil_printf("fpga id temporal filtering\n");
	bool with_meta_filtering = false;

	int total_points_count = 0;
	struct fpga_isp_input_buffer input_buffer;
	int page_num_factor = 4;	// nr_segs is the number of ssd page which is 16 kb by default
	allocate_fpga_isp_input_buffer(&input_buffer, nr_segs * page_num_factor);
    for (int i = 0; i < nr_segs; i++) {
        struct iovec iov_item = iov[i];
        for (int offset = 0; offset < iov_item.iov_len; offset += FPGA_ISP_BUFFER_PAGE_SIZE) {
            int points_count = 0;
            if (with_meta_filtering) {
            	points_count = segment_filter_and_aggregate_for_id(iov_item.iov_base + offset, &input_buffer, descriptor);
            } else {
            	points_count = only_segment_aggregate(iov_item.iov_base + offset, &input_buffer, descriptor);
            }
            total_points_count += points_count;
        }
    }
    //xil_printf("input addr: %p, total points count: %d, output addr: %p\n", input_buffer.base, total_points_count, output_buffer->iov[0].iov_base);
    char *output_addr = (char *)output_buffer->iov[0].iov_base + 16;	// the first 16 bytes is uesd to record total count and we use 16 because of 16-bytes alignment
    int result_count = send_to_fpga_id_filtering(input_buffer.base, total_points_count, output_addr, descriptor->oid, descriptor->time_min, descriptor->time_max);
    output_buffer->used_bytes_count = result_count * sizeof(struct traj_point) + 16;
    memcpy(output_buffer->iov[0].iov_base, &result_count, 4);
    free_fpga_isp_input_buffer(&input_buffer);
    //xil_printf("result count: %d\n", result_count);
}


int init_dma_and_filtering() {
	int Status;
	u16 DeviceId = DMA_DEV_ID;
	// Initialize dogain core
	//xil_printf("Initializing doAdd\n");
	doFiltering_cfg = XDostfiltering_LookupConfig(XPAR_DOSTFILTERING_0_DEVICE_ID);
	if (doFiltering_cfg) {
		int status = XDostfiltering_CfgInitialize(&doFiltering, doFiltering_cfg);
		if (status != XST_SUCCESS) {
			xil_printf("Error initializing doGain core\n");
		}
	}

	/* Initialize the XAxiDma device.
		 */
	CfgPtr = XAxiDma_LookupConfig(DeviceId);
	if (!CfgPtr) {
		xil_printf("No config found for %d\r\n", DeviceId);
		return XST_FAILURE;
	}

	Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}

	if(XAxiDma_HasSg(&AxiDma)){
		xil_printf("Device configured as SG mode \r\n");
		return XST_FAILURE;
	}

	/* Disable interrupts, we use polling mode
	 */
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DMA_TO_DEVICE);
}

int init_dma_and_filtering_for_id() {
	int Status;
	u16 DeviceId = ID_DMA_DEV_ID;
	// Initialize dogain core
	//xil_printf("Initializing doAdd\n");
	doIDFiltering_cfg = XDoidfiltering_LookupConfig(XPAR_DOIDFILTERING_0_DEVICE_ID);
	if (doIDFiltering_cfg) {
		int status = XDoidfiltering_CfgInitialize(&doIDFiltering, doIDFiltering_cfg);
		if (status != XST_SUCCESS) {
			xil_printf("Error initializing doGain core\n");
		}
	}

	/* Initialize the XAxiDma device.
		 */
	IDCfgPtr = XAxiDma_LookupConfig(DeviceId);
	if (!IDCfgPtr) {
		xil_printf("No config found for %d\r\n", DeviceId);
		return XST_FAILURE;
	}

	Status = XAxiDma_CfgInitialize(&IDAxiDma, IDCfgPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}

	if(XAxiDma_HasSg(&IDAxiDma)){
		xil_printf("Device configured as SG mode \r\n");
		return XST_FAILURE;
	}

	/* Disable interrupts, we use polling mode
	 */
	XAxiDma_IntrDisable(&IDAxiDma, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&IDAxiDma, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DMA_TO_DEVICE);
}

static void prepare_input_data(void* instream_ptr, int points_num) {

	struct traj_point *inStream = (struct traj_point *)instream_ptr;

	for (int idx = 0; idx < points_num; idx++) {
		inStream[idx].oid = idx;
		inStream[idx].normalized_longitude = idx;
		inStream[idx].normalized_latitude = idx;
		inStream[idx].timestamp_sec = idx;

	}
}


void fpga_hello_world() {

	int lon_min = 0;
	int lon_max = 128000;
	int lat_min = 0;
	int lat_max = 128000;
	int time_min = 0;
	int time_max = 128000;

	int points_num = 4096;
	void* input_buffer = alloc_vmpages(16,
            ZONE_PS_DDR_LOW);
	void* output_buffer = alloc_vmpages(16,
            ZONE_PS_DDR_LOW);
	prepare_input_data(input_buffer, points_num);
	STFiltering_FPGA(input_buffer, points_num, output_buffer, lon_min, lon_max, lat_min, lat_max, time_min, time_max);


}

static int STFiltering_FPGA(void *points_base_addr_ptr, int points_num, void *result_buffer_ptr, int lon_min, int lon_max, int lat_min, int lat_max, int time_min, int time_max) {

	struct traj_point *points_base_addr = (struct traj_point *)points_base_addr_ptr;
	struct traj_point *result_buffer = (struct traj_point *)result_buffer_ptr;
	u64 start = my_clock();
	int Status;
	init_dma_and_filtering();

	XDostfiltering_Set_lon_min(&doFiltering, lon_min);
	XDostfiltering_Set_lon_max(&doFiltering, lon_max);
	XDostfiltering_Set_lat_min(&doFiltering, lat_min);
	XDostfiltering_Set_lat_max(&doFiltering, lat_max);
	XDostfiltering_Set_time_min(&doFiltering, time_min);
	XDostfiltering_Set_time_max(&doFiltering, time_max);
	XDostfiltering_Start(&doFiltering);

	// Flush the cache of the buffers
	Xil_DCacheFlushRange((u32)points_base_addr, points_num*sizeof(struct traj_point));
	Xil_DCacheFlushRange((u32)result_buffer, points_num*sizeof(struct traj_point));

	//xil_printf("Sending data to IP core slave\n");
	u64 start1 = my_clock();

	Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) points_base_addr,
			points_num*sizeof(struct traj_point), XAXIDMA_DMA_TO_DEVICE);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	//xil_printf("Get data\n");
	Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) result_buffer,
			points_num*sizeof(struct traj_point), XAXIDMA_DEVICE_TO_DMA);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	while ((XAxiDma_Busy(&AxiDma,XAXIDMA_DEVICE_TO_DMA)) ||
			(XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE))) {
							/* Wait */
	}
	u64 start3 = my_clock();

			/* Invalidate the DestBuffer before receiving the data, in case the
				 * Data Cache is enabled
				 */
	Xil_DCacheInvalidateRange((u32)result_buffer, points_num*sizeof(struct traj_point));

	while (!XDostfiltering_IsDone(&doFiltering)) {
		xil_printf("Calculation complete\n");
	}

	int result_count = XDostfiltering_Get_return(&doFiltering);

	/*struct traj_point *check_data_base;
	check_data_base = (struct traj_point *) result_buffer;
	for (int i = 0; i < result_count; i++) {
		xil_printf("data point oid %d\n", check_data_base[i].oid);
	}*/


	u64 end = my_clock();

	XTime tEnd, tBegin;
	u32 tused;
	XTime_GetTime(&tBegin);
	sleep(5);
	XTime_GetTime(&tEnd);

	xil_printf("5 second counts: %d\n", (tEnd-tBegin));
	xil_printf("set configuration take %d\n", (start1 - start));
	xil_printf("data transfer take %d\n", (start3 - start1));
	xil_printf("fpga takes time: %d\n", (end - start));
	return result_count;

}

