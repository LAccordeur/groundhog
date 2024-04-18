//
// Created by yangguo on 11/10/22.
//

#include "groundhog/normalization_util.h"

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