//
// Created by yangguo on 11/30/22.
//

#include <malloc.h>
#include "groundhog/persistence_manager.h"
#include "groundhog/common_fs_layer.h"

struct my_file *my_fopen(const char *filename, const char *file_operation_mode, int fs_mode) {
    struct my_file *my_fp = malloc(sizeof(struct my_file));
    if (fs_mode == COMMON_FS_MODE) {
        FILE *fp = common_fs_open(filename, file_operation_mode);
        my_fp->common_file = fp;
        my_fp->filename = filename;
        my_fp->file_operation_mode = file_operation_mode;
        my_fp->fs_mode = fs_mode;
    }
    return my_fp;
}

size_t my_fwrite(const void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode) {
    if (fs_mode == COMMON_FS_MODE) {
        return common_fs_write(ptr, size, nmemb, stream->common_file);
    }
}

size_t my_fread(void *ptr, size_t size, size_t nmemb, struct my_file *stream, int fs_mode) {
    if (fs_mode == COMMON_FS_MODE) {
        return common_fs_read(ptr, size, nmemb, stream->common_file);
    }
}

size_t my_fseek(struct my_file *stream, long int offset, int fs_mode) {
    if (fs_mode == COMMON_FS_MODE) {
        return common_fs_seek(stream->common_file, offset, SEEK_SET);
    }
}
