//
// Created by yangguo on 11/30/22.
//

#ifndef TRAJ_BLOCK_FORMAT_COMMON_FS_LAYER_H
#define TRAJ_BLOCK_FORMAT_COMMON_FS_LAYER_H

#include <stdio.h>

FILE *common_fs_open(const char *filename, const char *mode);

size_t common_fs_write(const void *ptr, size_t size, size_t nmemb, FILE *stream);

size_t common_fs_read(void *ptr, size_t size, size_t nmemb, FILE *stream);

int common_fs_seek(FILE *stream, long int offset, int whence);

#endif //TRAJ_BLOCK_FORMAT_COMMON_FS_LAYER_H
