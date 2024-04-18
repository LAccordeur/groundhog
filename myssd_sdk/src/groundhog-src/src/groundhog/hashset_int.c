//
// Created by yangguo on 11/21/22.
//
#include "groundhog/hashset_int.h"
#include <stdio.h>

void add_key(struct hashset_int **set, int key, int value) {
    struct hashset_int *s;

    HASH_FIND_INT(*set, &key, s);
    if (s == NULL) {
        s = (struct hashset_int *) malloc(sizeof *s);
        s->key = key;
        s->value = value;
        HASH_ADD_INT(*set, key, s);
    }
}

struct hashset_int *find_key(struct hashset_int **set, int key) {
    struct hashset_int *s;
    HASH_FIND_INT(*set, &key, s);
    return s;
}

void delete_key(struct hashset_int **set, struct hashset_int *key) {
    HASH_DEL(*set, key);
    free(key);
}

void delete_all(struct hashset_int **set) {
    struct hashset_int *current_key, *tmp;

    HASH_ITER(hh, *set, current_key, tmp) {
        HASH_DEL(*set, current_key);
        free(current_key);
    }
};

int hash_count(struct hashset_int **set) {
    return HASH_COUNT(*set);
}

void get_key_set(struct hashset_int **set, int *keys) {
    struct hashset_int *s;
    int i = 0;
    for (s = *set; s != NULL; s = s->hh.next) {
        keys[i] = s->key;
        i++;
    }
}

void print_hashset_int(struct hashset_int **set) {
    struct hashset_int *s;
    for (s = *set; s != NULL; s = s->hh.next) {
        printf("key: %d; value: %d\n", s->key, s->value);
    }
}
