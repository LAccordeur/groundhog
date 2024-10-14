//
// Created by yangguo on 24-10-9.
//
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "groundhog/static_spdk_fs_layer.h"
#include "groundhog/persistence_manager.h"
#include "groundhog/common_util.h"

static int getRandom(int min, int max) {
    srand(time(NULL));
    int rd_num = rand() % (max - min + 1) + min;
    return rd_num;
}

static void write_random_data_to_file() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();


    struct my_file *test_file = my_fopen(TEST_FILENAME, "r", SPDK_FS_MODE);
    void* data_block = malloc(4096);

    uint64_t prev_time_value, time_value;

    prev_time_value = get_posix_clock_time();
    for (int i = 0; i < 100000; i++) {
        int write_offset = getRandom(0, 28279);
        test_file->spdk_file->current_write_offset = write_offset;
        int write_size = my_fwrite(data_block, 1, 4096, test_file, test_file->fs_mode);
        printf("i: %d, write size: %d\n", i, write_size);
    }
    time_value = get_posix_clock_time();
    printf("total write time: %d\n", (time_value - prev_time_value));

    free(data_block);

}

int main() {

    write_random_data_to_file();
    return 0;
}