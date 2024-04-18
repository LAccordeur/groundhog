//
// Created by yangguo on 11/16/22.
//

#include "groundhog/simple_index.h"
#include <stdlib.h>
#include <limits.h>
#include "groundhog/hashset_int.h"
#include "groundhog/log.h"
#include "groundhog/common_util.h"
#include "groundhog/config.h"
#include "groundhog/persistence_manager.h"

static const int OID_ARRAY_SIZE_SIZE = size_of_attribute(struct index_entry, oid_array_size);
static const int OID_ARRAY_ITEM_SIZE = sizeof(int);
static const int LON_MIN_SIZE = size_of_attribute(struct index_entry, lon_min);
static const int LON_MAX_SIZE = size_of_attribute(struct index_entry, lon_max);
static const int LAT_MIN_SIZE = size_of_attribute(struct index_entry, lat_min);
static const int LAT_MAX_SIZE = size_of_attribute(struct index_entry, lat_max);
static const int TIME_MIN_SIZE = size_of_attribute(struct index_entry, time_min);
static const int TIME_MAX_SIZE = size_of_attribute(struct index_entry, time_max);
static const int BLOCK_LOGICAL_ADR_SIZE = size_of_attribute(struct index_entry, block_logical_adr);


/*
 * index entry storage layout: | lon_min | lon_max | lat_min | lat_max | time_min | time_max | block_logical_adr | oid_array_size | oid_array (variable length) |
 * */
static const int LON_MIN_OFFSET = 0;
static const int LON_MAX_OFFSET = LON_MIN_OFFSET + LON_MIN_SIZE;
static const int LAT_MIN_OFFSET = LON_MAX_OFFSET + LON_MAX_SIZE;
static const int LAT_MAX_OFFSET = LAT_MIN_OFFSET + LAT_MIN_SIZE;
static const int TIME_MIN_OFFSET = LAT_MAX_OFFSET + LAT_MAX_SIZE;
static const int TIME_MAX_OFFSET = TIME_MIN_OFFSET + TIME_MIN_SIZE;
static const int BLOCK_LOGICAL_ADR_OFFSET = TIME_MAX_OFFSET + TIME_MAX_SIZE;
static const int OID_ARRAY_SIZE_OFFSET = BLOCK_LOGICAL_ADR_OFFSET + BLOCK_LOGICAL_ADR_SIZE;
static const int OID_ARRAY_OFFSET = OID_ARRAY_SIZE_OFFSET + OID_ARRAY_SIZE_SIZE;

static int calculate_oid_array_space(int oid_array_size) {
    return oid_array_size * OID_ARRAY_ITEM_SIZE;
}

static int calculate_index_entry_space(int oid_array_size) {
    return LON_MIN_SIZE + LON_MAX_SIZE + LAT_MIN_SIZE + LAT_MAX_SIZE + TIME_MIN_SIZE + TIME_MAX_SIZE + BLOCK_LOGICAL_ADR_SIZE + OID_ARRAY_SIZE_SIZE + oid_array_size * OID_ARRAY_ITEM_SIZE;
}

void serialize_index_entry(struct index_entry *source, void *destination) {
    char *d = destination;
    memcpy(d + LON_MIN_OFFSET, &(source->lon_min), LON_MIN_SIZE);
    memcpy(d + LON_MAX_OFFSET, &(source->lon_max), LON_MAX_SIZE);
    memcpy(d + LAT_MIN_OFFSET, &(source->lat_min), LAT_MIN_SIZE);
    memcpy(d + LAT_MAX_OFFSET, &(source->lat_max), LAT_MAX_SIZE);
    memcpy(d + TIME_MIN_OFFSET, &(source->time_min), TIME_MIN_SIZE);
    memcpy(d + TIME_MAX_OFFSET, &(source->time_max), TIME_MAX_SIZE);
    memcpy(d + BLOCK_LOGICAL_ADR_OFFSET, &(source->block_logical_adr), BLOCK_LOGICAL_ADR_SIZE);
    memcpy(d + OID_ARRAY_SIZE_OFFSET, &(source->oid_array_size), OID_ARRAY_SIZE_OFFSET);
    memcpy(d + OID_ARRAY_OFFSET, source->oid_array, calculate_oid_array_space(source->oid_array_size));
}

void deserialize_index_entry(void* source, struct index_entry *destination) {
    char *s = source;
    memcpy(&(destination->lon_min), s + LON_MIN_OFFSET, LON_MIN_SIZE);
    memcpy(&(destination->lon_max), s + LON_MAX_OFFSET, LON_MAX_SIZE);
    memcpy(&(destination->lat_min), s + LAT_MIN_OFFSET, LAT_MIN_SIZE);
    memcpy(&(destination->lat_max), s + LAT_MAX_OFFSET, LAT_MAX_SIZE);
    memcpy(&(destination->time_min), s + TIME_MIN_OFFSET, TIME_MIN_SIZE);
    memcpy(&(destination->time_max), s + TIME_MAX_OFFSET, TIME_MAX_SIZE);
    memcpy(&(destination->block_logical_adr), s + BLOCK_LOGICAL_ADR_OFFSET, BLOCK_LOGICAL_ADR_SIZE);
    memcpy(&(destination->oid_array_size), s + OID_ARRAY_SIZE_OFFSET, OID_ARRAY_SIZE_SIZE);
    int oid_array_space = calculate_oid_array_space(destination->oid_array_size);
    int *oid_array = malloc(oid_array_space);
    memcpy(oid_array, s + OID_ARRAY_OFFSET, oid_array_space);
    destination->oid_array = oid_array;
    destination->block_physical_ptr = NULL;
}

void init_serialized_index_storage(struct serialized_index_storage *storage) {
    storage->current_index = -1;
    storage->total_size = 1000;
    storage->index_block_base = (void**) malloc(1000 * sizeof(void*));
}

void free_serialized_index_storage(struct serialized_index_storage *storage) {
    for (int i = 0; i <= storage->current_index; i++) {
        free(storage->index_block_base[i]);
        storage->index_block_base[i] = NULL;
    }
    free(storage->index_block_base);
    storage->index_block_base = NULL;
}

void append_serialized_index_block_to_storage(struct serialized_index_storage *storage, void *block) {
    storage->current_index++;

    if (storage->current_index < storage->total_size) {
        (storage->index_block_base)[storage->current_index] = block;
    } else {
        // we need to extend array
        int new_total_size = storage->total_size * 2;
        debug_print("[append_serialized_index_block_to_storage] extend serialized index storage from size %d to size %d\n", storage->total_size, new_total_size);
        void **tmp_base = malloc(new_total_size * sizeof(void*));
        for (int i = 0; i < storage->total_size; i++) {
            tmp_base[i] = (storage->index_block_base)[i];
        }
        free(storage->index_block_base);
        storage->index_block_base = tmp_base;

        (storage->index_block_base)[storage->current_index] = block;
        storage->total_size = new_total_size;
    }
}

void serialize_index_entry_storage(struct index_entry_storage *entry_storage, struct serialized_index_storage *serialized_storage) {

    int current_offset_in_block = 4;    // the first 4 byte are used to record the total num of entries
    void *index_block = malloc(INDEX_BLOCK_SIZE);
    memset((char*)index_block, 0, INDEX_BLOCK_SIZE);

    int total_serialized_count = 0;
    int count = 0;
    for (int i = 0; i <= entry_storage->current_index; i++) {
        struct index_entry *entry = entry_storage->index_entry_base[i];
        int serialized_space = calculate_index_entry_space(entry->oid_array_size);
        if (current_offset_in_block + serialized_space <= INDEX_BLOCK_SIZE) {
            serialize_index_entry(entry, (char*)index_block + current_offset_in_block);
            current_offset_in_block += serialized_space;
            count++;

        } else {
            // this serialized block is full, so we add it to the storage and create a new block
            append_serialized_index_block_to_storage(serialized_storage, index_block);
            memcpy(index_block, &count, 4);
            total_serialized_count += count;
            index_block = malloc(INDEX_BLOCK_SIZE);
            current_offset_in_block = 4;
            count = 0;
            memset((char*)index_block, 0, INDEX_BLOCK_SIZE);
            serialize_index_entry(entry, (char*)index_block + current_offset_in_block);
            current_offset_in_block += serialized_space;
            count++;
        }
    }

    // handle the last block that not full
    append_serialized_index_block_to_storage(serialized_storage, index_block);
    memcpy(index_block, &count, 4);
    total_serialized_count += count;
    debug_print("[serialize_index_entry_storage] expected serialized index entry count: %d\n", entry_storage->current_index + 1);
    debug_print("[serialize_index_entry_storage] total serialized index entry count: %d\n", total_serialized_count);
}

void deserialize_index_entry_storage(struct serialized_index_storage *serialized_storage, struct index_entry_storage *entry_storage) {

    int total_deserialized_count = 0;

    int serialized_space = 0;
    for (int i = 0; i <= serialized_storage->current_index; i++) {
        char *block_base = serialized_storage->index_block_base[i];
        int count = 0;
        int current_offset_in_block = 4;
        memcpy(&count, block_base, 4);
        total_deserialized_count += count;
        for (int j = 0; j < count; j++) {
            //printf("i: %d, j: %d\n", i, j);
            struct index_entry *entry = malloc(sizeof(struct index_entry));
            deserialize_index_entry(block_base + current_offset_in_block, entry);
            append_index_entry_to_storage(entry_storage, entry);
            serialized_space = calculate_index_entry_space(entry->oid_array_size);
            current_offset_in_block += serialized_space;
        }
    }
    debug_print("[deserialize_index_entry_storage] total deserialized index entry count: %d\n", total_deserialized_count);
    //free_serialized_index_storage(serialized_storage);
}



void init_index_entry(struct index_entry *entry) {
    entry->oid_array_size = -1;
    entry->oid_array = NULL;
    entry->lon_min = INT_MAX;
    entry->lon_max = 0;
    entry->lat_min = INT_MAX;
    entry->lat_max = 0;
    entry->time_min = INT_MAX;
    entry->time_max = 0;
    entry->block_logical_adr = -1;
    entry->block_physical_ptr = NULL;
}

void free_index_entry(struct index_entry *entry) {
    if (entry->block_physical_ptr != NULL) {
        // free(entry->block_physical_ptr);  // freed by storage, not here
        entry->block_physical_ptr = NULL;
        free(entry->oid_array);
        entry->oid_array = NULL;
    }
    entry->block_logical_adr = -1;
}

void fill_index_entry(struct index_entry *entry, struct traj_point **points, int points_num, void* block_physical_ptr, int block_logical_adr) {

    struct hashset_int *oids = NULL;    // calculate the unique num of oid

    struct traj_point *point;
    for (int i = 0; i < points_num; i++) {
        point = points[i];
        if (point->normalized_longitude > entry->lon_max) {
            entry->lon_max = point->normalized_longitude;
        }
        if (point->normalized_longitude < entry->lon_min) {
            entry->lon_min = point->normalized_longitude;
        }
        if (point->normalized_latitude > entry->lat_max) {
            entry->lat_max = point->normalized_latitude;
        }
        if (point->normalized_latitude < entry->lat_min) {
            entry->lat_min = point->normalized_latitude;
        }
        if (point->timestamp_sec > entry->time_max) {
            entry->time_max = point->timestamp_sec;
        }
        if (point->timestamp_sec < entry->time_min) {
            entry->time_min = point->timestamp_sec;
        }
        add_key(&oids, point->oid, 0);
    }
    entry->block_physical_ptr = block_physical_ptr;
    entry->block_logical_adr = block_logical_adr;

    int unique_oid_num = hash_count(&oids);
    entry->oid_array = (int *) malloc(unique_oid_num * sizeof(int));
    entry->oid_array_size = unique_oid_num;
    get_key_set(&oids, entry->oid_array);
}

void init_index_entry_storage(struct index_entry_storage *storage) {
    storage->current_index = -1;
    storage->total_size = 1000;
    storage->index_entry_base = (struct index_entry **) malloc(1000 * sizeof(struct index_entry*));
}

void init_index_entry_storage_with_persistence(struct index_entry_storage *storage, char* filename, char* file_operation_mode, int fs_mode) {
    storage->current_index = -1;
    storage->total_size = 1000;
    storage->index_entry_base = (struct index_entry **) malloc(1000 * sizeof(struct index_entry*));
    storage->my_fp = my_fopen(filename, file_operation_mode, fs_mode);
}

void free_index_entry_storage(struct index_entry_storage *storage) {
    for (int i = 0 ; i <= storage->current_index; i++) {
        free_index_entry(storage->index_entry_base[i]);
        free(storage->index_entry_base[i]);
        storage->index_entry_base[i] = NULL;
    }
    free(storage->index_entry_base);
}

void append_index_entry_to_storage(struct index_entry_storage *storage, struct index_entry *entry) {
    storage->current_index++;

    if (storage->current_index < storage->total_size) {
        (storage->index_entry_base)[storage->current_index] = entry;
        //storage->current_index++;
    } else {
        // we need to extend array
        int new_total_size = storage->total_size * 2;
        debug_print("[append_index_entry_to_storage] extend index storage from size %d to size %d\n", storage->total_size, new_total_size);
        struct index_entry **tmp_base = malloc(new_total_size * sizeof(void*));
        for (int i = 0; i < storage->total_size; i++) {
            tmp_base[i] = (storage->index_entry_base)[i];
        }
        free(storage->index_entry_base);
        storage->index_entry_base = tmp_base;

        (storage->index_entry_base)[storage->current_index] = entry;
        //storage->current_index++;
        storage->total_size = new_total_size;
    }
}


void flush_serialized_index_storage(struct serialized_index_storage *storage, char* filename, int fs_mode) {
    struct my_file *fp = my_fopen(filename, "w", fs_mode);

    int block_count = 0;
    for (int i = 0; i <= storage->current_index; i++) {
        void *block_ptr = storage->index_block_base[i];
        int seek_result = my_fseek(fp, i * INDEX_BLOCK_SIZE, fs_mode);
        int write_size = my_fwrite(block_ptr, 1, INDEX_BLOCK_SIZE, fp, fs_mode);
        if (write_size > 0) {
            block_count++;
        }
    }
    debug_print("[flush_serialized_index_storage] total flushed index block: %d\n", block_count);

}


void rebuild_index_storage(char* filename, int fs_mode, struct index_entry_storage *storage) {
    struct my_file *fp = my_fopen(filename, "r", fs_mode);

    struct serialized_index_storage serialized_storage;
    init_serialized_index_storage(&serialized_storage);

    int read_size = -1;
    int block_count = 0;
    do {
        void* index_block = malloc(INDEX_BLOCK_SIZE);
        read_size = my_fread(index_block, 1, INDEX_BLOCK_SIZE, fp, fs_mode);

        if (read_size > 0) {
            append_serialized_index_block_to_storage(&serialized_storage, index_block);
            block_count++;
        }
    } while (read_size > 0);

    deserialize_index_entry_storage(&serialized_storage, storage);
    free_serialized_index_storage(&serialized_storage);
    debug_print("[rebuild_index_storage] total rebuilt index block: %d\n", block_count);

}

