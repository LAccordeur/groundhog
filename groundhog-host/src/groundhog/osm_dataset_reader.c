//
// Created by yangguo on 24-4-8.
//
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "groundhog/osm_dataset_reader.h"
#include "groundhog/normalization_util.h"

void parse_osm_row(char* line, struct traj_point *destination_row) {
    char *token = strtok(line, ",");

    // point_id
    if (token) {
        destination_row->oid = atoi(token);
    }



    // longitude
    token = strtok(NULL, ",");
    //double longitude = atof(token);
    if (token) {
        double longitude = atof(token);
        int normalized_value = normalize_longitude(longitude);
        destination_row->normalized_longitude = normalized_value;
    }

    // latitude
    token = strtok(NULL, ",");
    //double latitude = atof(token);
    if (token) {
        double latitude = atof(token);
        int normalized_value = normalize_latitude(latitude);
        destination_row->normalized_latitude = normalized_value;
    }

    // timestamp_sec
    token = strtok(NULL, "\n");
    if (token) {

        //int normalized_time = normalize_datetime(token);
        destination_row->timestamp_sec = atoi(token);
    }
}

int read_points_from_csv_osm(FILE *fp, struct traj_point **points, int row_offset, int row_count) {
    long int file_offset = (long int) row_offset * OSM_CSV_ROW_SIZE;
    fseek(fp, file_offset, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {
        struct traj_point *row = points[line_count];
        parse_osm_row(line, row);
        line_count++;

        if (line_count >= row_count) {
            break;
        }

    }
    return line_count;
}
