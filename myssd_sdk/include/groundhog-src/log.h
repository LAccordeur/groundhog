//
// Created by yangguo on 11/15/22.
//

#ifndef TRAJ_BLOCK_FORMAT_LOG_H
#define TRAJ_BLOCK_FORMAT_LOG_H

#include <stdio.h>
#define DEBUG 1
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

#endif //TRAJ_BLOCK_FORMAT_LOG_H
