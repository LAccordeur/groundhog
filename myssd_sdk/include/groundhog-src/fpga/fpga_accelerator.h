//
// Created by yangguo on 3/7/23.
//

#ifndef TRAJ_BLOCK_FORMAT_FPGA_ACCELERATOR_H
#define TRAJ_BLOCK_FORMAT_FPGA_ACCELERATOR_H
#include <stddef.h>
#include <iov_iter.h>

#define FPGA_ISP_BUFFER_PAGE_SIZE 0x1000

void fpga_hello_world();

struct fpga_isp_output_buffer {
    struct iovec *iov;
    size_t iov_capacity;    // the number of allocated iovec
    size_t used_bytes_count;
};

struct fpga_isp_input_buffer {
	void* base;
	size_t length;
	size_t used_bytes_count;
};


#endif //TRAJ_BLOCK_FORMAT_FPGA_ACCELERATOR_H
