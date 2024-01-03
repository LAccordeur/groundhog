//
// Created by yangguo on 11/30/22.
//
#include "groundhog/common_fs_layer.h"

FILE *common_fs_open(const char *filename, const char *mode) {
    return fopen(filename, mode);
}

size_t common_fs_write(const void *ptr, size_t size, size_t nmemb, FILE *stream) {
    return fwrite(ptr, size, nmemb, stream);
}

size_t common_fs_read(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    return fread(ptr, size, nmemb, stream);
}

int common_fs_seek(FILE *stream, long int offset, int whence) {
    return fseek(stream, offset, whence);
}
