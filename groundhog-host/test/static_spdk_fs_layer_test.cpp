//
// Created by yangguo on 23-1-3.
//
#include "gtest/gtest.h"
#include "groundhog/config.h"

extern "C" {
#include "groundhog/static_spdk_fs_layer.h"
#include "stdio.h"
#include "groundhog/traj_block_format.h"
#include "groundhog/porto_dataset_reader.h"
}

TEST(common_fs, test) {
    FILE* fp = fopen("test.txt", "w");
    fwrite("helloworld", 1, 10, fp);
    //fclose(fp);
    char buffer[10];
    //fseek(fp, 0, SEEK_SET);
    FILE* fp1 = fopen("test.txt", "r");
    fread(buffer, 1, 1, fp1);
    printf("%s\n", buffer);
}

TEST(spdk_fs, test) {
    struct spdk_nvme_driver_desc driver_desc;
    init_spdk_nvme_driver(&driver_desc);
    cleanup_spdk_nvme_driver(&driver_desc);
    printf("finish\n");
}

TEST(spdk_fs, io) {
    struct spdk_nvme_driver_desc driver_desc;
    init_spdk_nvme_driver(&driver_desc);

    struct spdk_static_fs_desc fs_desc;
    int file_desc_vec_num = 1;
    struct spdk_static_file_desc file_desc_vec[file_desc_vec_num];
    file_desc_vec[0].current_read_offset = 0;
    file_desc_vec[0].current_write_offset = 0;
    file_desc_vec[0].start_lba = 0;
    file_desc_vec[0].sector_count = 100;
    char filename[] = "test.file";
    memcpy(file_desc_vec[0].filename, filename, sizeof(filename));
    spdk_mk_static_fs(&fs_desc, file_desc_vec, file_desc_vec_num, &driver_desc, false);

    struct spdk_static_file_desc *my_file = spdk_static_fs_fopen(filename, &fs_desc);
    void* buffer = malloc(0x1000);
    snprintf((char*)buffer, 0x1000, "%s", "Hello World!\n");
    spdk_static_fs_fwrite(buffer, 0x1000, my_file);


    void* result_buffer = malloc(0x1000);
    spdk_static_fs_fread(result_buffer, 0x1000, my_file);
    printf("%s", (char*)result_buffer);

    cleanup_spdk_nvme_driver(&driver_desc);
    printf("finish\n");
}

static int
iterate_raw_traj_block(void* block) {
    int count = 0;
    struct traj_block_header header;
    parse_traj_block_for_header(block, &header);
    struct seg_meta meta_array[header.seg_count];
    parse_traj_block_for_seg_meta_section(block, meta_array, header.seg_count);
    for (int j = 0; j < header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];

        int data_seg_points_num = meta_item.seg_size / get_traj_point_size();
        count += data_seg_points_num;
        struct traj_point **points = allocate_points_memory(data_seg_points_num);
        parse_traj_block_for_seg_data(block, meta_item.seg_offset, points, data_seg_points_num);
        print_traj_points(points, data_seg_points_num);
        free_points_memory(points, data_seg_points_num);

    }
    return count;
}

TEST(spdk_fs, complex_io) {
    struct spdk_nvme_driver_desc driver_desc;
    init_spdk_nvme_driver(&driver_desc);

    struct spdk_static_fs_desc fs_desc;
    int file_desc_vec_num = 1;
    struct spdk_static_file_desc file_desc_vec[file_desc_vec_num];
    file_desc_vec[0].current_read_offset = 0;
    file_desc_vec[0].current_write_offset = 0;
    file_desc_vec[0].start_lba = 0;
    file_desc_vec[0].sector_count = 100;
    char filename[] = "test.file";
    memcpy(file_desc_vec[0].filename, filename, sizeof(filename));
    spdk_mk_static_fs(&fs_desc, file_desc_vec, file_desc_vec_num, &driver_desc, false);

    struct spdk_static_file_desc *my_file = spdk_static_fs_fopen(filename, &fs_desc);


    int block_num = 99;
    int traj_block_size = 4096;
    int split_segment_num = 10;
    int points_num = calculate_points_num_via_block_size(traj_block_size, split_segment_num);
    printf("point num per block: %d\n", points_num);
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    char my_buffer[traj_block_size];
    struct traj_point **points = allocate_points_memory(points_num);

    for (int i = 0; i < block_num; i++) {
        read_points_from_csv(fp, points, i*points_num, points_num);
        //print_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, my_buffer, traj_block_size);
        spdk_static_fs_fwrite(my_buffer, traj_block_size, my_file);
    }

    for (int i = 0; i < block_num; i++) {
        spdk_static_fs_fread(my_buffer, traj_block_size, my_file);
        iterate_raw_traj_block(my_buffer);
    }
}

TEST(spdk_fs, io_boundary) {
    struct spdk_nvme_driver_desc driver_desc;
    init_spdk_nvme_driver(&driver_desc);

    struct spdk_static_fs_desc fs_desc;
    int file_desc_vec_num = 1;
    struct spdk_static_file_desc file_desc_vec[file_desc_vec_num];
    file_desc_vec[0].current_read_offset = 0;
    file_desc_vec[0].current_write_offset = 0;
    file_desc_vec[0].start_lba = 0;
    file_desc_vec[0].sector_count = 100;
    char filename[] = "test.file";
    memcpy(file_desc_vec[0].filename, filename, sizeof(filename));
    spdk_mk_static_fs(&fs_desc, file_desc_vec, file_desc_vec_num, &driver_desc, false);

    struct spdk_static_file_desc *my_file = spdk_static_fs_fopen(filename, &fs_desc);


    int block_num = 5;
    int traj_block_size = 4096;
    int split_segment_num = 10;
    int points_num = calculate_points_num_via_block_size(traj_block_size, split_segment_num);
    printf("point num per block: %d\n", points_num);
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    char my_buffer[traj_block_size];
    struct traj_point **points = allocate_points_memory(points_num);

    for (int i = 0; i < block_num; i++) {
        read_points_from_csv(fp, points, i*points_num, points_num);
        //print_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, my_buffer, traj_block_size);
        int write_size = spdk_static_fs_fwrite(my_buffer, traj_block_size, my_file);
        printf("write id: %d; write size: %d\n", i, write_size);
    }

    for (int i = 0; i < 6; i++) {
        int read_size = spdk_static_fs_fread(my_buffer, traj_block_size, my_file);
        printf("read id: %d; read size: %d\n", i, read_size);
        //iterate_raw_traj_block(my_buffer);
    }
}

TEST(spdk_fs, superblock_flush) {
    struct spdk_nvme_driver_desc driver_desc;
    init_spdk_nvme_driver(&driver_desc);

    struct spdk_static_fs_desc fs_desc;
    int file_desc_vec_num = 1;
    struct spdk_static_file_desc file_desc_vec[file_desc_vec_num];
    file_desc_vec[0].current_read_offset = 0;
    file_desc_vec[0].current_write_offset = 0;
    file_desc_vec[0].start_lba = 1;
    file_desc_vec[0].sector_count = 100;
    char filename[] = "test.file";
    memcpy(file_desc_vec[0].filename, filename, sizeof(filename));
    spdk_mk_static_fs(&fs_desc, file_desc_vec, file_desc_vec_num, &driver_desc, false);

    struct spdk_static_file_desc *my_file = spdk_static_fs_fopen(filename, &fs_desc);


    int block_num = 5;
    int traj_block_size = 4096;
    int split_segment_num = 10;
    int points_num = calculate_points_num_via_block_size(traj_block_size, split_segment_num);
    printf("point num per block: %d\n", points_num);
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    char my_buffer[traj_block_size];
    struct traj_point **points = allocate_points_memory(points_num);

    for (int i = 0; i < block_num; i++) {
        read_points_from_csv(fp, points, i*points_num, points_num);
        //print_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, my_buffer, traj_block_size);
        int write_size = spdk_static_fs_fwrite(my_buffer, traj_block_size, my_file);
        printf("write id: %d; write size: %d\n", i, write_size);
    }
    spdk_flush_static_fs_meta(&fs_desc);
    print_spdk_static_fs_meta(&fs_desc);
}

TEST(spdk_fs, superblock_rebuild) {

    struct spdk_nvme_driver_desc driver_desc;
    init_spdk_nvme_driver(&driver_desc);

    struct spdk_static_fs_desc fs_desc;
    int file_desc_vec_num = 1;
    struct spdk_static_file_desc file_desc_vec[file_desc_vec_num];

    char filename[] = "test.file";
    spdk_mk_static_fs(&fs_desc, file_desc_vec, file_desc_vec_num, &driver_desc, true);
    print_spdk_static_fs_meta(&fs_desc);

    struct spdk_static_file_desc *my_file = spdk_static_fs_fopen(filename, &fs_desc);


    int block_num = 5;
    int traj_block_size = 4096;
    char my_buffer[traj_block_size];

    for (int i = 0; i < block_num; i++) {
        int read_size = spdk_static_fs_fread(my_buffer, traj_block_size, my_file);
        printf("read id: %d; read size: %d\n", i, read_size);
    }
    print_spdk_static_fs_meta(&fs_desc);
}


TEST(spdk_fs, exe) {
    struct spdk_nvme_driver_desc driver_desc;
    init_spdk_nvme_driver(&driver_desc);

    struct spdk_static_fs_desc fs_desc;
    int file_desc_vec_num = 1;
    struct spdk_static_file_desc file_desc_vec[file_desc_vec_num];
    file_desc_vec[0].current_read_offset = 0;
    file_desc_vec[0].current_write_offset = 0;
    file_desc_vec[0].start_lba = 0;
    file_desc_vec[0].sector_count = 100;
    char filename[] = "test.file";
    memcpy(file_desc_vec[0].filename, filename, sizeof(filename));
    spdk_mk_static_fs(&fs_desc, file_desc_vec, file_desc_vec_num, &driver_desc, false);

    struct spdk_static_file_desc *my_file = spdk_static_fs_fopen(filename, &fs_desc);
    void* buffer = malloc(0x1000);

    int estimated_result_block_num = 1;
    int lba_vec_size = 3;
    struct lba lba_vec[lba_vec_size];
    lba_vec[0].start_lba = 0;
    lba_vec[0].sector_count = 100;
    lba_vec[1].start_lba = 100;
    lba_vec[1].sector_count = 100;
    lba_vec[2].start_lba = 200;
    lba_vec[2].sector_count = 56;
    struct isp_descriptor desc = {.isp_type = 1, .oid=3, .time_min = 16, .time_max = 17, .lon_min = 12, .lon_max = 13, .lat_min = 14, .lat_max = 15, .estimated_result_page_num = estimated_result_block_num, .lba_count = lba_vec_size, .lba_array=lba_vec};



    spdk_static_fs_fread_isp(buffer, 0x1000, my_file, &desc);

    cleanup_spdk_nvme_driver(&driver_desc);
    printf("finish\n");
}

extern struct spdk_static_fs_desc spdk_static_fs_layer_for_traj;
TEST(spdk_fs, traj_fs) {
    init_and_mk_fs_for_traj(false);
    struct spdk_static_file_desc *data_file = spdk_static_fs_fopen(DATA_FILENAME, &spdk_static_fs_layer_for_traj);

    int block_num = 5;
    int traj_block_size = 4096;
    int split_segment_num = 10;
    int points_num = calculate_points_num_via_block_size(traj_block_size, split_segment_num);
    printf("point num per block: %d\n", points_num);
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");
    char my_buffer[traj_block_size];
    struct traj_point **points = allocate_points_memory(points_num);

    for (int i = 0; i < block_num; i++) {
        read_points_from_csv(fp, points, i*points_num, points_num);
        //print_traj_points(points, points_num);
        do_self_contained_traj_block(points, points_num, my_buffer, traj_block_size);
        int write_size = spdk_static_fs_fwrite(my_buffer, traj_block_size, data_file);
        printf("write id: %d; write size: %d\n", i, write_size);
    }

    spdk_flush_static_fs_meta(&spdk_static_fs_layer_for_traj);
}

TEST(spdk_fs, traj_fs_rebuild) {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta(&spdk_static_fs_layer_for_traj);

    struct spdk_static_file_desc *data_file = spdk_static_fs_fopen(DATA_FILENAME, &spdk_static_fs_layer_for_traj);


    int block_num = 256;
    int traj_block_size = 4096;
    char my_buffer[traj_block_size];

    for (int i = 0; i < block_num; i++) {
        int offset = TRAJ_BLOCK_SIZE * 256 + i * TRAJ_BLOCK_SIZE;
        spdk_static_fs_fseek(data_file, offset);

        int read_size = spdk_static_fs_fread(my_buffer, traj_block_size, data_file);
        int *int_ptr = (int *) my_buffer;
        printf("read id: %d; read size: %d; first int: %d\n", i, read_size, int_ptr[0]);
    }
    print_spdk_static_fs_meta(&spdk_static_fs_layer_for_traj);

}
