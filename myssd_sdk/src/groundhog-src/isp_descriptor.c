//
// Created by yangguo on 11/28/22.
//

#include <string.h>
#include <malloc.h>
#include "groundhog/isp_descriptor.h"
#include "groundhog/common_util.h"

static const int ISP_TYPE_SIZE = size_of_attribute(struct isp_descriptor, isp_type);
static const int OID_SIZE = size_of_attribute(struct isp_descriptor, oid);
static const int TIME_MIN_SIZE = size_of_attribute(struct isp_descriptor, time_min);
static const int TIME_MAX_SIZE = size_of_attribute(struct isp_descriptor, time_max);
static const int LON_MIN_SIZE = size_of_attribute(struct isp_descriptor, lon_min);
static const int LON_MAX_SIZE = size_of_attribute(struct isp_descriptor, lon_max);
static const int LAT_MIN_SIZE = size_of_attribute(struct isp_descriptor, lat_min);
static const int LAT_MAX_SIZE = size_of_attribute(struct isp_descriptor, lat_max);
static const int ESTIMATED_RESULT_PAGE_NUM_SIZE = size_of_attribute(struct isp_descriptor, estimated_result_page_num);
static const int LBA_COUNT_SIZE = size_of_attribute(struct isp_descriptor, lba_count);

static const int ISP_TYPE_OFFSET = 0;
static const int OID_OFFSET = ISP_TYPE_OFFSET + ISP_TYPE_SIZE;
static const int TIME_MIN_OFFSET = OID_OFFSET + OID_SIZE;
static const int TIME_MAX_OFFSET = TIME_MIN_OFFSET + TIME_MIN_SIZE;
static const int LON_MIN_OFFSET = TIME_MAX_OFFSET + TIME_MAX_SIZE;
static const int LON_MAX_OFFSET = LON_MIN_OFFSET + LON_MIN_SIZE;
static const int LAT_MIN_OFFSET = LON_MAX_OFFSET + LON_MAX_SIZE;
static const int LAT_MAX_OFFSET = LAT_MIN_OFFSET + LAT_MIN_SIZE;
static const int ESTIMATED_RESULT_PAGE_NUM_OFFSET = LAT_MAX_OFFSET + LAT_MAX_SIZE;
static const int LBA_COUNT_OFFSET = ESTIMATED_RESULT_PAGE_NUM_OFFSET + ESTIMATED_RESULT_PAGE_NUM_SIZE;
static const int LBA_ARRAY_OFFSET = LBA_COUNT_OFFSET + LBA_COUNT_SIZE;

static int calculate_lba_array_space(int lba_count) {
    return lba_count * sizeof(struct lba);
}

void serialize_isp_descriptor(struct isp_descriptor *source, void *destination) {
    char *d = destination;
    memcpy(d + ISP_TYPE_OFFSET, &(source->isp_type), ISP_TYPE_SIZE);
    memcpy(d + OID_OFFSET, &(source->oid), OID_SIZE);
    memcpy(d + TIME_MIN_OFFSET, &(source->time_min), TIME_MIN_SIZE);
    memcpy(d + TIME_MAX_OFFSET, &(source->time_max), TIME_MAX_SIZE);
    memcpy(d + LON_MIN_OFFSET, &(source->lon_min), LON_MIN_SIZE);
    memcpy(d + LON_MAX_OFFSET, &(source->lon_max), LON_MAX_SIZE);
    memcpy(d + LAT_MIN_OFFSET, &(source->lat_min), LAT_MIN_SIZE);
    memcpy(d + LAT_MAX_OFFSET, &(source->lat_max), LAT_MAX_SIZE);
    memcpy(d + LBA_COUNT_OFFSET, &(source->lba_count), LBA_COUNT_SIZE);
    memcpy(d + ESTIMATED_RESULT_PAGE_NUM_OFFSET, &(source->estimated_result_page_num), ESTIMATED_RESULT_PAGE_NUM_SIZE);
    int lba_array_space = calculate_lba_array_space(source->lba_count);
    memcpy(d + LBA_ARRAY_OFFSET, source->lba_array, lba_array_space);
}

void deserialize_isp_descriptor(void *source, struct isp_descriptor *destination) {
    char *s = source;
    memcpy(&(destination->isp_type), s + ISP_TYPE_OFFSET, ISP_TYPE_SIZE);
    memcpy(&(destination->oid), s + OID_OFFSET, OID_SIZE);
    memcpy(&(destination->time_min), s + TIME_MIN_OFFSET, TIME_MIN_SIZE);
    memcpy(&(destination->time_max), s + TIME_MAX_OFFSET, TIME_MAX_SIZE);
    memcpy(&(destination->lon_min), s + LON_MIN_OFFSET, LON_MIN_SIZE);
    memcpy(&(destination->lon_max), s + LON_MAX_OFFSET, LON_MAX_SIZE);
    memcpy(&(destination->lat_min), s + LAT_MIN_OFFSET, LAT_MIN_SIZE);
    memcpy(&(destination->lat_max), s + LAT_MAX_OFFSET, LAT_MAX_SIZE);
    memcpy(&(destination->lba_count), s + LBA_COUNT_OFFSET, LBA_COUNT_SIZE);
    memcpy(&(destination->estimated_result_page_num), s + ESTIMATED_RESULT_PAGE_NUM_OFFSET, ESTIMATED_RESULT_PAGE_NUM_SIZE);
    int lba_array_space = calculate_lba_array_space(destination->lba_count);
    void *lba_array_ptr = malloc(lba_array_space);
    memcpy(lba_array_ptr, s + LBA_ARRAY_OFFSET, lba_array_space);
    destination->lba_array = lba_array_ptr;
}

void free_isp_descriptor(struct isp_descriptor *descriptor) {
    if (descriptor->lba_count > 0 && descriptor->lba_array != NULL) {
        free(descriptor->lba_array);
        descriptor->lba_array = NULL;
    }
}

int calculate_isp_descriptor_space(struct isp_descriptor *descriptor) {
    int size = ISP_TYPE_SIZE + OID_SIZE + TIME_MIN_SIZE + TIME_MAX_SIZE + LON_MIN_SIZE + LON_MAX_SIZE + LAT_MIN_SIZE + LAT_MAX_SIZE  + LBA_COUNT_SIZE + ESTIMATED_RESULT_PAGE_NUM_SIZE +
            calculate_lba_array_space(descriptor->lba_count);
}

void print_isp_descriptor(struct isp_descriptor *descriptor) {
    printf("isp_type: %d\n", descriptor->isp_type);
    printf("oid: %d\n", descriptor->oid);
    printf("time_min: %d\n", descriptor->time_min);
    printf("time_max: %d\n", descriptor->time_max);
    printf("lon_min: %d\n", descriptor->lon_min);
    printf("lon_max: %d\n", descriptor->lon_max);
    printf("lat_min: %d\n", descriptor->lat_min);
    printf("lat_max: %d\n", descriptor->lat_max);
    printf("lba_count: %d\n", descriptor->lba_count);
    printf("estimated_result_page_num: %d\n", descriptor->estimated_result_page_num);
    printf("lba_array: %p\n", descriptor->lba_array);
    for (int i = 0; i < descriptor->lba_count; i++) {
        printf("item [%d]: start_lba: %d, sector_count:%d\n", i, descriptor->lba_array[i].start_lba, descriptor->lba_array[i].sector_count);
    }
    printf("\n");
}

