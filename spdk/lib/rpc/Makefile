#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (c) Intel Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 4
SO_MINOR := 0

C_SRCS = rpc.c
LIBNAME = rpc

SPDK_MAP_FILE = $(abspath $(CURDIR)/spdk_rpc.map)

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
