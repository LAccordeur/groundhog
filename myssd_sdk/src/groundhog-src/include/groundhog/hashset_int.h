//
// Created by yangguo on 11/21/22.
//

#ifndef TRAJ_BLOCK_FORMAT_HASHSET_INT_H
#define TRAJ_BLOCK_FORMAT_HASHSET_INT_H

#include "uthash/uthash.h"

struct hashset_int {
    int key;    /* key */
    int value;
    UT_hash_handle hh;  /* makes this structure hashable */
};

void add_key(struct hashset_int **set, int key, int value);

struct hashset_int *find_key(struct hashset_int **set, int key);

void delete_key(struct hashset_int **set, struct hashset_int *key);

void delete_all(struct hashset_int **set);

int hash_count(struct hashset_int **set);

void get_key_set(struct hashset_int **set, int *keys);

void print_hashset_int(struct hashset_int **set);

#endif //TRAJ_BLOCK_FORMAT_HASHSET_INT_H

