//
// Created by yangguo on 11/22/22.
//

#include "gtest/gtest.h"

extern "C" {
#include "groundhog/hashset_int.h"
}

TEST(hashsettest, add) {
    struct hashset_int *hashset = NULL;
    add_key(&hashset, 1, 32);
    print_hashset_int(&hashset);
    delete_all(&hashset);
}


TEST(hashsettest, find) {
    struct hashset_int *hashset = NULL;
    add_key(&hashset, 1, 32);
    add_key(&hashset, 4, 12);
    add_key(&hashset, 4, 22);
    print_hashset_int(&hashset);
    struct hashset_int *result = find_key(&hashset, 3);
    printf("count: %d\n", hash_count(&hashset));
    delete_all(&hashset);
    printf("count: %d\n", hash_count(&hashset));
}

TEST(hashsettest, key_set) {
    struct hashset_int *hashset = NULL;
    add_key(&hashset, 1, 32);
    add_key(&hashset, 4, 12);

    print_hashset_int(&hashset);
    int array[2];
    get_key_set(&hashset, array);
    printf("count: %d\n", hash_count(&hashset));
    delete_all(&hashset);

}
