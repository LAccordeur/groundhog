/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) Intel Corporation.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"

#include "spdk/json.h"
#include "spdk/thread.h"
#include "spdk/likely.h"
#include "spdk/log.h"

#include "spdk/vmd.h"

#include "spdk_internal/init.h"
#include "event_vmd.h"

static struct spdk_poller *g_hotplug_poller;
static bool g_enabled;

static int
vmd_hotplug_monitor(void *ctx)
{
	return spdk_vmd_hotplug_monitor();
}

int
vmd_subsystem_init(void)
{
	int rc;

	if (g_enabled) {
		SPDK_ERRLOG("The initialization has already been performed\n");
		return -EBUSY;
	}

	rc = spdk_vmd_init();
	if (spdk_likely(rc != 0)) {
		SPDK_ERRLOG("Failed to initialize the VMD library\n");
		return rc;
	}

	assert(g_hotplug_poller == NULL);

	g_hotplug_poller = SPDK_POLLER_REGISTER(vmd_hotplug_monitor, NULL, 1000000ULL);
	if (g_hotplug_poller == NULL) {
		SPDK_ERRLOG("Failed to register hotplug monitor poller\n");
		return -ENOMEM;
	}

	g_enabled = true;

	return 0;
}

static void
vmd_subsystem_fini(void)
{
	spdk_poller_unregister(&g_hotplug_poller);

	spdk_vmd_fini();

	spdk_subsystem_fini_next();
}

static void
vmd_write_config_json(struct spdk_json_write_ctx *w)
{
	spdk_json_write_array_begin(w);

	if (g_enabled) {
		spdk_json_write_object_begin(w);
		spdk_json_write_named_string(w, "method", "enable_vmd");
		spdk_json_write_named_object_begin(w, "params");
		spdk_json_write_object_end(w);
		spdk_json_write_object_end(w);
	}

	spdk_json_write_array_end(w);
}

static struct spdk_subsystem g_spdk_subsystem_vmd = {
	.name = "vmd",
	.fini = vmd_subsystem_fini,
	.write_config_json = vmd_write_config_json,
};

SPDK_SUBSYSTEM_REGISTER(g_spdk_subsystem_vmd);
