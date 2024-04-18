//
// Created by yangguo on 11/30/22.
//

#ifndef TRAJ_BLOCK_FORMAT_PERSISTENCE_MANAGER_H
#define TRAJ_BLOCK_FORMAT_PERSISTENCE_MANAGER_H

#define COMMON_FS_MODE 0
#define SPDK_FS_MODE 1

#include <stdio.h>

struct my_file {
    FILE *common_file;
    char* filename;
    char* file_operation_mode;
    int fs_mode;
};

struct my_file *my_fopen(const char *filename, const char *file_operation_mode, int fs_mode);

size_t my_fwrite(const void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode);

size_t my_fread(void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode);

size_t my_fseek(struct my_file *stream, long int offset, int fs_mode);

#endif //TRAJ_BLOCK_FORMAT_PERSISTENCE_MANAGER_H
