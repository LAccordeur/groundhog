//
// Created by yangguo on 24-4-3.
//
#include "gtest/gtest.h"

extern "C" {
#include "groundhog/knn_util.h"
#include "groundhog/porto_dataset_reader.h"
}

int point_num = 100000;
int k_value = 1000;
int time_dist_pred = 60 * 60;

TEST(knn_util_test, knn_query_baseline) {
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");

    // load data points
    int point_num_in_buffer = point_num;
    struct traj_point **points = allocate_points_memory(point_num_in_buffer);
    read_points_from_csv(fp,points, 0, point_num_in_buffer);

    // query
    struct traj_point *query_point = points[0];
    int k = k_value;

    struct knn_result_buffer result_buffer;
    init_knn_result_buffer(k, &result_buffer);

    clock_t start, end, total, t1, t2;
    total = 0;
    start = clock();
    for (int i = 0; i < point_num_in_buffer; i++) {
        long distance = cal_points_distance(query_point, points[i]);
        if (distance < result_buffer.max_distance) {
            struct result_item item = {*points[i], distance};
            t1 = clock();
            add_item_to_buffer_baseline(&result_buffer, &item);
            t2 = clock();
            total += (t2 - t1);
        }
    }

    end = clock();
    printf("time: %f, add item time: %f\n", (double)(end - start), (double)total);

    print_result_buffer(&result_buffer);
    print_runtime_statistics(&result_buffer.statistics);
    free_points_memory(points, point_num_in_buffer);
    free_knn_result_buffer(&result_buffer);

}


TEST(knn_util_test, knn_query) {
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");

    // load data points
    int point_num_in_buffer = point_num;
    struct traj_point **points = allocate_points_memory(point_num_in_buffer);
    read_points_from_csv(fp,points, 0, point_num_in_buffer);

    // query
    struct traj_point *query_point = points[0];
    int k = k_value;

    struct knn_result_buffer result_buffer;
    init_knn_result_buffer(k, &result_buffer);

    clock_t start, end, total, t1, t2;
    total = 0;
    start = clock();
    for (int i = 0; i < point_num_in_buffer; i++) {
        long distance = cal_points_distance(query_point, points[i]);
        if (distance < result_buffer.max_distance) {
            struct result_item item = {*points[i], distance};
            t1 = clock();
            add_item_to_buffer(&result_buffer, &item);
            t2 = clock();
            total += (t2 - t1);
        }
    }
    combine_and_sort(&result_buffer);
    end = clock();
    printf("time: %f, add item time: %f\n", (double)(end - start), (double)total);

    print_result_buffer(&result_buffer);
    print_runtime_statistics(&result_buffer.statistics);
    free_points_memory(points, point_num_in_buffer);
    free_knn_result_buffer(&result_buffer);

}


TEST(knn_util_test, knnjoin_query_baseline) {
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");

    // load data points
    int point_num_in_buffer = point_num;
    struct traj_point **points = allocate_points_memory(point_num_in_buffer);
    struct traj_point **points2 = allocate_points_memory(point_num_in_buffer);
    read_points_from_csv(fp,points, 0, point_num_in_buffer);
    read_points_from_csv(fp, points2, 0, 2);

    // query

    int k = k_value;

    struct knnjoin_result_buffer result_buffer;
    init_knnjoin_result_buffer(k, &result_buffer);

    clock_t start, end, total, t1, t2;
    total = 0;
    start = clock();
    for (int i = 0; i < point_num_in_buffer; i++) {
        for (int j = 0; j < 2; j++) {
            long distance = cal_points_distance(points[i], points2[j]);
            long time_dist = points[i]->timestamp_sec > points2[j]->timestamp_sec ? (points[i]->timestamp_sec - points2[j]->timestamp_sec) : (points2[j]->timestamp_sec - points[i]->timestamp_sec);
            if (time_dist > time_dist_pred && distance < result_buffer.max_distance) {
                struct knnjoin_result_item item = {*points[i], *points2[j], distance};
                t1 = clock();
                add_item_to_knnjoin_buffer_baseline(&result_buffer, &item);
                t2 = clock();
                total += (t2 - t1);
            }
        }
    }

    end = clock();
    printf("time: %f, add item time: %f\n", (double)(end - start), (double)total);

    print_knnjoin_result_buffer(&result_buffer);
    print_runtime_statistics(&result_buffer.statistics);
    free_points_memory(points, point_num_in_buffer);
    free_knnjoin_result_buffer(&result_buffer);

}

TEST(knn_util_test, knnjoin_query) {
    FILE *fp = fopen("/home/yangguo/Dataset/trajectory/porto_data_v2.csv", "r");

    // load data points
    int point_num_in_buffer = point_num;
    struct traj_point **points = allocate_points_memory(point_num_in_buffer);
    struct traj_point **points2 = allocate_points_memory(point_num_in_buffer);
    read_points_from_csv(fp,points, 0, point_num_in_buffer);
    read_points_from_csv(fp, points2, 0, 2);

    // query

    int k = k_value;

    struct knnjoin_result_buffer result_buffer;
    init_knnjoin_result_buffer(k, &result_buffer);

    clock_t start, end, total, t1, t2;
    total = 0;
    start = clock();
    for (int i = 0; i < point_num_in_buffer; i++) {
        for (int j = 0; j < 2; j++) {
            long distance = cal_points_distance(points[i], points2[j]);
            long time_dist = points[i]->timestamp_sec > points2[j]->timestamp_sec ? (points[i]->timestamp_sec - points2[j]->timestamp_sec) : (points2[j]->timestamp_sec - points[i]->timestamp_sec);
            if (time_dist > time_dist_pred && distance < result_buffer.max_distance) {
                struct knnjoin_result_item item = {*points[i], *points2[j], distance};
                t1 = clock();
                add_item_to_knnjoin_buffer(&result_buffer, &item);
                t2 = clock();
                total += (t2 - t1);
            }
        }
    }
    combine_and_sort_knnjoin(&result_buffer);
    end = clock();
    printf("time: %f, add item time: %f\n", (double)(end - start), (double)total);

    print_knnjoin_result_buffer(&result_buffer);
    print_runtime_statistics(&result_buffer.statistics);
    free_points_memory(points, point_num_in_buffer);
    free_knnjoin_result_buffer(&result_buffer);

}

TEST(knn_util_test, sortbuffer) {
    struct knn_result_buffer result_buffer;
    init_knn_result_buffer(10, &result_buffer);
    struct traj_point point = {11, 1, 2, 1};
    for (int i = 0; i < 10; i++) {
        int distance = 10 - i;

        struct result_item item = {point, distance};
        result_buffer.result_buffer_k[i] = item;
        result_buffer.current_buffer_size++;

    }
    print_result_buffer(&result_buffer);
    sort_buffer(result_buffer.result_buffer_k, result_buffer.current_buffer_size);
    printf("after sorting\n");
    print_result_buffer(&result_buffer);

    free_knn_result_buffer(&result_buffer);
}

TEST(knn_util_test, pointdist) {
    struct traj_point point = {11, 1, 2, 1};
    struct traj_point point2 = {11, 1, 1, 4};
    int dist = cal_points_distance(&point, &point2);
    printf("dist value: %d\n", dist);
}

TEST(knn_util_test, mindist) {
    struct traj_point point = {11, 1, 1, 1};
    struct seg_meta meta = {3, 5, 3, 4, 1, 1, 1, 1, NULL};
    int dist = cal_min_distance(&point, &meta);
    // the expected value is 8
    printf("dist value: %d\n", dist);

    struct traj_point point1 = {11, 1, 2, 1};
    struct seg_meta meta1 = {3, 6, 2, 4, 1, 1, 1, 1, NULL};
    int dist1 = cal_min_distance(&point1, &meta1);
    // the expected value is 2
    printf("dist value: %d\n", dist1);

    struct traj_point point2 = {11, 1, 2, 1};
    struct seg_meta meta2 = {0, 6, 0, 7, 1, 1, 1, 1, NULL};
    int dist2 = cal_min_distance(&point2, &meta2);
    // the expected value is 0
    printf("dist value: %d\n", dist2);
}

TEST(knn_util_test, minmaxdistance) {
    struct traj_point point = {11, 1, 1, 1};
    struct seg_meta meta = {3, 5, 3, 4, 1, 1, 1, 1, NULL};
    int dist = cal_minmax_distance(&point, &meta);
    // the expected value is 13
    printf("dist value: %d\n", dist);

    struct traj_point point1 = {11, 1, 2, 1};
    struct seg_meta meta1 = {3, 6, 2, 4, 1, 1, 1, 1, NULL};
    int dist1 = cal_minmax_distance(&point1, &meta1);
    // the expected value is 10
    printf("dist value: %d\n", dist1);
}
