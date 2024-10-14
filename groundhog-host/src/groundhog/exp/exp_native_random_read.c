//
// Created by yangguo on 24-10-9.
//


#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "groundhog/static_spdk_fs_layer.h"
#include "groundhog/persistence_manager.h"
#include "groundhog/common_util.h"

// Generates and prints 'count' random
// numbers in range [min, max].
void printRandoms(int min, int max, int count) {
    printf("Random numbers between %d and %d: ", min, max);
    srand(time(NULL));
    // Loop that will print the count random numbers
    for (int i = 0; i < count; i++) {

        // Find the random number in the range [min, max]
        int rd_num = rand() % (max - min + 1) + min;

        printf("%d ", rd_num);
    }
}

static int getRandom(int min, int max) {
    srand(time(NULL));
    int rd_num = rand() % (max - min + 1) + min;
    return rd_num;
}

static void read_random_data_from_file() {
    init_and_mk_fs_for_traj(true);
    print_spdk_static_fs_meta_for_traj();


    struct my_file *test_file = my_fopen(SEG_META_FILENAME, "r", SPDK_FS_MODE);
    void* data_block = malloc(4096);

    uint64_t prev_time_value, time_value;

    prev_time_value = get_posix_clock_time();
    for (int i = 0; i < 1000000; i++) {
        int read_offset = getRandom(0, 28279);
        my_fseek(test_file, read_offset, SPDK_FS_MODE);
        int read_size = my_fread(data_block, 1, 4096, test_file, test_file->fs_mode);
        printf("i: %d, read size: %d\n", i, read_size);
    }
    time_value = get_posix_clock_time();
    printf("total read time: %d\n", (time_value - prev_time_value));

    free(data_block);

}

int main() {

    read_random_data_from_file();
    return 0;
}

