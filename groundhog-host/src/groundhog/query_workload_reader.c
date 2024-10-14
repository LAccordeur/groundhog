//
// Created by yangguo on 11/28/22.
//

#include <malloc.h>
#include <string.h>
#include <stdlib.h>
#include "groundhog/simple_query_engine.h"
#include "groundhog/normalization_util.h"

static void parse_id_temporal_row(char* line, struct id_temporal_predicate *destination_row) {
    char *token = strtok(line, ",");

    if (token) {
        destination_row->oid = atoi(token);
    }

    token = strtok(NULL, ",");
    if (token) {
        destination_row->time_min = atoi(token);
    }

    token = strtok(NULL, ",");
    if (token) {
        destination_row->time_max = atoi(token);
    }

}

static void parse_id_temporal_row_nyc(char* line, struct id_temporal_predicate *destination_row) {
    char *token = strtok(line, ",");

    if (token) {
        destination_row->oid = atoi(token);
    }

    token = strtok(NULL, ",");
    if (token) {
        int normalized_time = normalize_datetime(token);
        destination_row->time_min = normalized_time;
    }

    token = strtok(NULL, ",");
    if (token) {
        int normalized_time = normalize_datetime(token);
        destination_row->time_max = normalized_time;
    }
}

static void parse_spatio_temporal_row(char* line, struct spatio_temporal_range_predicate *destination_row) {

    char *token = strtok(line, ",");

    if (token) {
        destination_row->time_min = atoi(token);
    }

    token = strtok(NULL, ",");
    if (token) {
        destination_row->time_max = atoi(token);
    }

    token = strtok(NULL, ",");
    if (token) {
        double lon_min = atof(token);
        int normalized_lon_min = normalize_longitude(lon_min);
        destination_row->lon_min = normalized_lon_min;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lon_max = atof(token);
        int normalized_lon_max = normalize_longitude(lon_max);
        destination_row->lon_max = normalized_lon_max;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lat_min = atof(token);
        int normalized_lat_min = normalize_latitude(lat_min);
        destination_row->lat_min = normalized_lat_min;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lat_max = atof(token);
        int normalized_lat_max = normalize_latitude(lat_max);
        destination_row->lat_max = normalized_lat_max;
    }

}

static void parse_spatio_temporal_row_nyc(char* line, struct spatio_temporal_range_predicate *destination_row) {

    char *token = strtok(line, ",");

    if (token) {
        int normalized_time = normalize_datetime(token);
        destination_row->time_min = normalized_time;
    }

    token = strtok(NULL, ",");
    if (token) {
        int normalized_time = normalize_datetime(token);
        destination_row->time_max = normalized_time;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lon_min = atof(token);
        int normalized_lon_min = normalize_longitude(lon_min);
        destination_row->lon_min = normalized_lon_min;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lon_max = atof(token);
        int normalized_lon_max = normalize_longitude(lon_max);
        destination_row->lon_max = normalized_lon_max;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lat_min = atof(token);
        int normalized_lat_min = normalize_latitude(lat_min);
        destination_row->lat_min = normalized_lat_min;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lat_max = atof(token);
        int normalized_lat_max = normalize_latitude(lat_max);
        destination_row->lat_max = normalized_lat_max;
    }

}


static void parse_spatio_temporal_knn_row(char* line, struct spatio_temporal_knn_predicate *destination_row) {

    char *token = strtok(line, ",");

    if (token) {
        double lon = atof(token);
        int normalized_lon = normalize_longitude(lon);
        destination_row->query_point.normalized_longitude = normalized_lon;
    }

    token = strtok(NULL, ",");
    if (token) {
        double lat = atof(token);
        int normalized_lat = normalize_latitude(lat);
        destination_row->query_point.normalized_latitude = normalized_lat;
    }

    token = strtok(NULL, ",");
    if (token) {
        int k = atoi(token);
        destination_row->k = k;
    }

}


struct id_temporal_predicate** allocate_id_temporal_predicate_mem(int array_size) {
    struct id_temporal_predicate **predicates;
    predicates = (struct id_temporal_predicate**) malloc(array_size * sizeof(struct id_temporal_predicate));

    for (int i = 0; i < array_size; i++) {
        predicates[i] = (struct id_temporal_predicate*) malloc(sizeof(struct id_temporal_predicate));
    }

    return predicates;
}

void free_id_temporal_predicate_mem(struct id_temporal_predicate **predicates, int array_size) {
    for (int i = 0; i < array_size; i++) {
        free(predicates[i]);
        predicates[i] = NULL;
    }
    free(predicates);
    predicates = NULL;
}

int read_id_temporal_queries_from_csv(FILE *fp, struct id_temporal_predicate **predicate, int row_count) {
    fseek(fp, 0, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {
        struct id_temporal_predicate *row = predicate[line_count];
        parse_id_temporal_row(line, row);
        line_count++;

        if (line_count >= row_count) {
            break;
        }
    }

    return line_count;
}

int read_id_temporal_queries_from_csv_nyc(FILE *fp, struct id_temporal_predicate **predicate, int row_count) {
    fseek(fp, 0, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {
        struct id_temporal_predicate *row = predicate[line_count];
        parse_id_temporal_row_nyc(line, row);
        line_count++;

        if (line_count >= row_count) {
            break;
        }
    }

    return line_count;
}

struct spatio_temporal_range_predicate** allocate_spatio_temporal_predicate_mem(int array_size) {
    struct spatio_temporal_range_predicate **predicates;
    predicates = (struct spatio_temporal_range_predicate**) malloc(array_size * sizeof(struct spatio_temporal_range_predicate));

    for (int i = 0; i < array_size; i++) {
        predicates[i] = (struct spatio_temporal_range_predicate*) malloc(sizeof(struct spatio_temporal_range_predicate));
    }

    return predicates;
}

void free_spatio_temporal_predicate_mem(struct spatio_temporal_range_predicate **predicates, int array_size) {
    for (int i = 0; i < array_size; i++) {
        free(predicates[i]);
        predicates[i] = NULL;
    }
    free(predicates);
    predicates = NULL;
}


int read_spatio_temporal_queries_from_csv(FILE *fp, struct spatio_temporal_range_predicate **predicate, int row_count) {
    fseek(fp, 0, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {
        struct spatio_temporal_range_predicate *row = predicate[line_count];
        parse_spatio_temporal_row(line, row);
        line_count++;

        if (line_count >= row_count) {
            break;
        }
    }

    return line_count;
}

int read_spatio_temporal_queries_from_csv_nyc(FILE *fp, struct spatio_temporal_range_predicate **predicate, int row_count) {
    fseek(fp, 0, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {
        struct spatio_temporal_range_predicate *row = predicate[line_count];
        parse_spatio_temporal_row_nyc(line, row);
        line_count++;

        if (line_count >= row_count) {
            break;
        }
    }

    return line_count;
}

int read_spatio_temporal_queries_from_csv_geolife(FILE *fp, struct spatio_temporal_range_predicate **predicate, int row_count) {
    fseek(fp, 0, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {
        struct spatio_temporal_range_predicate *row = predicate[line_count];
        parse_spatio_temporal_row(line, row);
        line_count++;

        if (line_count >= row_count) {
            break;
        }
    }

    return line_count;
}

struct spatio_temporal_knn_predicate** allocate_spatio_temporal_knn_predicate_mem(int array_size) {
    struct spatio_temporal_knn_predicate **predicates;
    predicates = (struct spatio_temporal_knn_predicate**) malloc(array_size * sizeof(struct spatio_temporal_knn_predicate));

    for (int i = 0; i < array_size; i++) {
        predicates[i] = (struct spatio_temporal_knn_predicate*) malloc(sizeof(struct spatio_temporal_knn_predicate));
    }

    return predicates;
}

void free_spatio_temporal_knn_predicate_mem(struct spatio_temporal_knn_predicate **predicates, int array_size) {
    for (int i = 0; i < array_size; i++) {
        free(predicates[i]);
        predicates[i] = NULL;
    }
    free(predicates);
    predicates = NULL;
}

int read_spatio_temporal_knn_queries_from_csv(FILE *fp, struct spatio_temporal_knn_predicate **predicate, int row_count) {
    fseek(fp, 0, SEEK_SET);

    char line[128];
    int line_count = 0;
    while (fgets(line, 128, fp)) {
        struct spatio_temporal_knn_predicate *row = predicate[line_count];
        parse_spatio_temporal_knn_row(line, row);
        line_count++;

        if (line_count >= row_count) {
            break;
        }
    }

    return line_count;
}
