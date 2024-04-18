//
// Created by yangguo on 11/10/22.
//
#include "gtest/gtest.h"

extern "C" {
#include "groundhog/library.h"
#include "groundhog/simple_index.h"
}

TEST(mytestmain, ok) {
    printf("size1: %d\n", sizeof(struct index_entry));
    printf("size2: %d\n", sizeof(struct index_entry*));
    struct index_entry* ptr;
    printf("size3: %d\n", sizeof *ptr);
}