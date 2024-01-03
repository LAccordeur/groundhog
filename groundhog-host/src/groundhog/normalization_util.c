//
// Created by yangguo on 11/10/22.
//

#define _XOPEN_SOURCE 700
#include "groundhog/normalization_util.h"
#include <time.h>
#include <string.h>
#include <stdio.h>

static const int bins = 1 << PRECISION;
static const double lon_normalizer = bins / (180.0 - (-180));
static const double lat_normalizer = bins / (90.0 - (-90));
static const int max_index = (bins - 1);

int normalize_longitude(double lon) {
    if (lon >= 180) {
        return max_index;
    } else if (lon <= -180) {
        return 0;
    } else {
        return (int) ((lon - (-180)) * lon_normalizer);
    }
}

int normalize_latitude(double lat) {
    if (lat >= 90) {
        return max_index;
    } else if (lat <= -90) {
        return 0;
    } else {
        return (int)((lat - (-90)) * lat_normalizer);
    }
}

int normalize_datetime(char* timestamp_str) {
    struct tm tm;
    time_t seconds;
    int r;

    if (timestamp_str == NULL) {
        printf("null argument\n");

    }
    r = sscanf(timestamp_str, "%d-%d-%d %d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
    if (r != 6) {
        printf("expected %d numbers scanned in %s\n", r, timestamp_str);

    }

    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    tm.tm_isdst = 0;
    seconds = mktime(&tm);
    if (seconds == (time_t)-1) {
        printf("reading time from %s failed\n", timestamp_str);
    }
    return seconds;
}

static long MaxMask = 0x1fffffL;
static long split_z3(long value) {
    long x = value & MaxMask;
    x = (x | x << 32) & 0x1f00000000ffffL;
    //System.out.println(CurveUtil.bytesToBit(CurveUtil.toBytes(x)));
    x = (x | x << 16) & 0x1f0000ff0000ffL;
    //System.out.println(CurveUtil.bytesToBit(CurveUtil.toBytes(x)));
    x = (x | x << 8)  & 0x100f00f00f00f00fL;
    //System.out.println(CurveUtil.bytesToBit(CurveUtil.toBytes(x)));
    x = (x | x << 4)  & 0x10c30c30c30c30c3L;
    //System.out.println(CurveUtil.bytesToBit(CurveUtil.toBytes(x)));
    return (x | x << 2)      & 0x1249249249249249L;
}

/** insert 0 between every bit in value. Only first 31 bits can be considered. */
static long split_z2(long value) {
    long x = value & 0x7fffffffL;
    x = (x ^ (x << 32)) & 0x00000000ffffffffL;
    x = (x ^ (x << 16)) & 0x0000ffff0000ffffL;
    x = (x ^ (x <<  8)) & 0x00ff00ff00ff00ffL; // 11111111000000001111111100000000..
    x = (x ^ (x <<  4)) & 0x0f0f0f0f0f0f0f0fL; // 1111000011110000
    x = (x ^ (x <<  2)) & 0x3333333333333333L; // 11001100..
    x = (x ^ (x <<  1)) & 0x5555555555555555L; // 1010...
    return x;
}

long generate_zcurve_value(int x, int y, int z) {
    long value = split_z3(z) | split_z3(y) << 1 | split_z3(x) << 2;
    return value;
}

long generate_zcurve_value_spatial(int x, int y) {
    return split_z2(x) | split_z2(y) << 1;
}