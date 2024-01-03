//
// Created by yangguo on 23-3-29.
//
//
// Created by yangguo on 22-12-29.
//
/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) Intel Corporation.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"

#include "spdk/nvme.h"
#include "spdk/vmd.h"
#include "spdk/nvme_zns.h"
#include "spdk/env.h"
#include "spdk/string.h"
#include "spdk/log.h"
#include "groundhog/traj_block_format.h"
#include "groundhog/isp_descriptor.h"
#include "groundhog/simple_query_engine.h"
#include "groundhog/isp_output_format.h"
#include "groundhog/query_workload_reader.h"

/*struct ctrlr_entry {
    struct spdk_nvme_ctrlr		*ctrlr;
    TAILQ_ENTRY(ctrlr_entry)	link;
    char				name[1024];
};

struct ns_entry {
    struct spdk_nvme_ctrlr	*ctrlr;
    struct spdk_nvme_ns	*ns;
    TAILQ_ENTRY(ns_entry)	link;
    struct spdk_nvme_qpair	*qpair;
};*/

static TAILQ_HEAD(, ctrlr_entry) g_controllers = TAILQ_HEAD_INITIALIZER(g_controllers);
static TAILQ_HEAD(, ns_entry) g_namespaces = TAILQ_HEAD_INITIALIZER(g_namespaces);
static struct spdk_nvme_transport_id g_trid = {};

static bool g_vmd = false;

static void
register_ns(struct spdk_nvme_ctrlr *ctrlr, struct spdk_nvme_ns *ns)
{
    struct ns_entry *entry;

    if (!spdk_nvme_ns_is_active(ns)) {
        return;
    }

    entry = malloc(sizeof(struct ns_entry));
    if (entry == NULL) {
        perror("ns_entry malloc");
        exit(1);
    }

    entry->ctrlr = ctrlr;
    entry->ns = ns;
    TAILQ_INSERT_TAIL(&g_namespaces, entry, link);

    printf("  Namespace ID: %d size: %juGB\n", spdk_nvme_ns_get_id(ns),
           spdk_nvme_ns_get_size(ns) / 1000000000);
}

struct hello_world_sequence {
    struct ns_entry	*ns_entry;
    char		*buf;
    unsigned        using_cmb_io;
    int         block_index;
    int		is_completed;
    int points_num;
    bool is_id_temporal_query;
    int estimated_result_block_num;
    struct id_temporal_predicate id_temporal;
    struct spatio_temporal_range_predicate spatio_temporal;
};

static int
id_temporal_isp_traj_block(struct hello_world_sequence *sequence) {
    printf("oid: %d\n", sequence->id_temporal.oid);

    struct id_temporal_predicate *predicate = &sequence->id_temporal;
    char* block = sequence->buf;
    int result_count = 0;

    memcpy(&result_count, block, 4);
    printf("result count: %d\n", result_count);
    struct traj_point *points_base = (struct traj_point *)((char *)block + 16);
    for (int i = 0; i < result_count; i++) {
        struct traj_point tmp = points_base[i];
        //printf("oid: %d, lon: %d, lat: %d, time: %d\n", tmp.oid, tmp.normalized_longitude, tmp.normalized_latitude, tmp.timestamp_sec);
    }


    /*for (int block_count = 0; block_count < sequence->estimated_result_block_num; block_count++, block+=4096) {
        int points_num = parse_points_num_from_output_buffer_page(block);
        printf("block points num: %d\n", points_num);
        if (points_num > 0) {
            struct traj_point **points = allocate_points_memory(points_num);
            deserialize_output_buffer_page(block, points, points_num);
            for (int k = 0; k < points_num; k++) {
                struct traj_point *point = points[k];
                //printf("oid: %d, time: %d, lon: %d, lat: %d\n", point->oid, point->timestamp_sec, point->normalized_longitude, point->normalized_latitude);
                if (point->oid == predicate->oid
                    && predicate->time_min <= point->timestamp_sec
                    && predicate->time_max >= point->timestamp_sec) {
                    result_count++;
                }
            }
            free_points_memory(points, points_num);
        }
    }*/

    return result_count;
}

static int
spatio_temporal_isp_traj_block(struct hello_world_sequence *sequence) {
    printf("spatio-temporal query\n");

    struct spatio_temporal_range_predicate *predicate = &sequence->spatio_temporal;
    void* block = sequence->buf;
    int result_count = 0;
    memcpy(&result_count, block, 4);
    printf("result count: %d\n", result_count);
    struct traj_point *points_base = (struct traj_point *)((char *)block + 16);
    for (int i = 0; i < result_count; i++) {
        struct traj_point tmp = points_base[i];
        //printf("oid: %d, lon: %d, lat: %d, time: %d\n", tmp.oid, tmp.normalized_longitude, tmp.normalized_latitude, tmp.timestamp_sec);
    }

    return result_count;
}

static void
exe_complete(void *arg, const struct spdk_nvme_cpl *completion)
{
    struct hello_world_sequence *sequence = arg;

    /* Assume the I/O was successful */
    sequence->is_completed = 1;
    /* See if an error occurred. If so, display information
     * about it, and set completion value so that I/O
     * caller is aware that an error occurred.
     */
    if (spdk_nvme_cpl_is_error(completion)) {
        spdk_nvme_qpair_print_completion(sequence->ns_entry->qpair, (struct spdk_nvme_cpl *)completion);
        fprintf(stderr, "I/O error status: %s\n", spdk_nvme_cpl_get_status_string(&completion->status));
        fprintf(stderr, "Read I/O failed, aborting run\n");
        sequence->is_completed = 2;
        exit(1);
    }

    /*
     * The read I/O has completed.  Print the contents of the
     *  buffer, free the buffer, then mark the sequence as
     *  completed.  This will trigger the hello_world() function
     *  to exit its polling loop.
     */
    int count = 0;
    if (sequence->is_id_temporal_query) {
        count = id_temporal_isp_traj_block(sequence);
    } else {
        count = spatio_temporal_isp_traj_block(sequence);
    }
    spdk_free(sequence->buf);
    sequence->points_num = count;
    printf("total result count: %d\n", count);
}


static void
reset_zone_complete(void *arg, const struct spdk_nvme_cpl *completion)
{
    struct hello_world_sequence *sequence = arg;

    /* Assume the I/O was successful */
    sequence->is_completed = 1;
    /* See if an error occurred. If so, display information
     * about it, and set completion value so that I/O
     * caller is aware that an error occurred.
     */
    if (spdk_nvme_cpl_is_error(completion)) {
        spdk_nvme_qpair_print_completion(sequence->ns_entry->qpair, (struct spdk_nvme_cpl *)completion);
        fprintf(stderr, "I/O error status: %s\n", spdk_nvme_cpl_get_status_string(&completion->status));
        fprintf(stderr, "Reset zone I/O failed, aborting run\n");
        sequence->is_completed = 2;
        exit(1);
    }
}

static void
reset_zone_and_wait_for_completion(struct hello_world_sequence *sequence)
{
    if (spdk_nvme_zns_reset_zone(sequence->ns_entry->ns, sequence->ns_entry->qpair,
                                 0, /* starting LBA of the zone to reset */
                                 false, /* don't reset all zones */
                                 reset_zone_complete,
                                 sequence)) {
        fprintf(stderr, "starting reset zone I/O failed\n");
        exit(1);
    }
    while (!sequence->is_completed) {
        spdk_nvme_qpair_process_completions(sequence->ns_entry->qpair, 0);
    }
    sequence->is_completed = 0;
}

static void
generate_id_temporal_computation_descriptor(void *buf, struct id_temporal_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    struct isp_descriptor desc = {.isp_type = 0, .oid = predicate->oid, .time_min = predicate->time_min, .time_max = predicate->time_max, .lba_count = lba_vec_size, .estimated_result_page_num = estimated_result_block_num};

    desc.lba_array = lba_vec;
    int desc_size = calculate_isp_descriptor_space(&desc);
    if (desc_size >= 4096) {
        printf("the descriptor size is too big\n");
    }
    serialize_isp_descriptor(&desc, buf);
}

static void
generate_spatio_temporal_computation_descriptor(void *buf, struct spatio_temporal_range_predicate *predicate, int estimated_result_block_num, struct lba *lba_vec, int lba_vec_size) {
    struct isp_descriptor desc = {.isp_type = 1, .lon_min = predicate->lon_min, .lon_max = predicate->lon_max, .lat_min = predicate->lat_min, .lat_max = predicate->lat_max, .time_min = predicate->time_min, .time_max = predicate->time_max, .lba_count = lba_vec_size, .estimated_result_page_num = estimated_result_block_num};

    desc.lba_array = lba_vec;
    int desc_size = calculate_isp_descriptor_space(&desc);
    if (desc_size >= 4096) {
        printf("the descriptor size is too big\n");
    }
    serialize_isp_descriptor(&desc, buf);
}

static void
init_sequence(struct hello_world_sequence *sequence, size_t block_size, int estimated_result_block_num, int block_index, struct ns_entry	*ns_entry, size_t *sz, struct id_temporal_predicate *id_predicate, struct spatio_temporal_range_predicate *range_predicate, bool is_id_temporal_query, struct lba *lba_vec, int lba_vec_size) {
    /*
         * Use spdk_dma_zmalloc to allocate a 4KB zeroed buffer.  This memory
         * will be pinned, which is required for data buffers used for SPDK NVMe
         * I/O operations.
         */
    sequence->using_cmb_io = 1;
    sequence->buf = spdk_nvme_ctrlr_map_cmb(ns_entry->ctrlr, sz);
    if (sequence->buf == NULL || *sz < block_size) {
        sequence->using_cmb_io = 0;
        sequence->buf = spdk_zmalloc(block_size * estimated_result_block_num, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
        //memset(sequence->buf, 0, block_size * estimated_result_block_num);
    }
    if (sequence->buf == NULL) {
        printf("ERROR: write buffer allocation failed\n");
        return;
    }

    printf("buf pointer: %p\n", sequence->buf);
    //struct id_temporal_predicate predicate = {.oid = 20000380, .time_min = 1372636853, .time_max = 1372637853};
    sequence->id_temporal = *id_predicate;
    sequence->spatio_temporal = *range_predicate;
    if (is_id_temporal_query) {
        generate_id_temporal_computation_descriptor(sequence->buf, id_predicate, estimated_result_block_num, lba_vec, lba_vec_size);
    } else {
        generate_spatio_temporal_computation_descriptor(sequence->buf, range_predicate, estimated_result_block_num, lba_vec, lba_vec_size);
    }
    /*struct isp_descriptor deserialized_desc;
    deserialize_isp_descriptor(sequence->buf, &deserialized_desc);
    print_isp_descriptor(&deserialized_desc);
    free_isp_descriptor(&deserialized_desc);*/

    if (sequence->using_cmb_io) {
        printf("INFO: using controller memory buffer for IO\n");
    } else {
        printf("INFO: using host memory buffer for IO\n");
    }
    sequence->is_completed = 0;
    sequence->ns_entry = ns_entry;

    /*
     * If the namespace is a Zoned Namespace, rather than a regular
     * NVM namespace, we need to reset the first zone, before we
     * write to it. This not needed for regular NVM namespaces.
     */
    if (spdk_nvme_ns_get_csi(ns_entry->ns) == SPDK_NVME_CSI_ZNS) {
        reset_zone_and_wait_for_completion(sequence);
    }

    /*
     * Print "Hello world!" to sequence.buf.  We will write this data to LBA
     *  0 on the namespace, and then later read it back into a separate buffer
     *  to demonstrate the full I/O path.
     */
    sequence->block_index = block_index;
    sequence->estimated_result_block_num = estimated_result_block_num;
    sequence->points_num = 0;
    sequence->is_id_temporal_query = is_id_temporal_query;

}

static void
hello_world(void)
{
    struct ns_entry			*ns_entry;
    int				rc;


    TAILQ_FOREACH(ns_entry, &g_namespaces, link) {

        ns_entry->qpair = spdk_nvme_ctrlr_alloc_io_qpair(ns_entry->ctrlr, NULL, 0);
        if (ns_entry->qpair == NULL) {
            printf("ERROR: spdk_nvme_ctrlr_alloc_io_qpair() failed\n");
            return;
        }


        FILE *id_query_fp = fopen("/home/yangguo/Downloads/porto_10w_id_24h.query", "r");
        struct id_temporal_predicate **id_predicates = allocate_id_temporal_predicate_mem(50);
        read_id_temporal_queries_from_csv(id_query_fp, id_predicates, 50);

        FILE *st_query_fp = fopen("/home/yangguo/Downloads/porto_10w_1h_01.query", "r");
        struct spatio_temporal_range_predicate **st_predicates = allocate_spatio_temporal_predicate_mem(50);
        read_spatio_temporal_queries_from_csv(st_query_fp, st_predicates, 50);


        int read_block_size = 4096;
        bool is_id_temporal_query = true;
        int estimated_result_block_num = 224;
        int lba_vec_size = 3;
        struct lba lba_vec[lba_vec_size];
        lba_vec[0].start_lba = 0;
        lba_vec[0].sector_count = 100;
        lba_vec[1].start_lba = 100;
        lba_vec[1].sector_count = 100;
        lba_vec[2].start_lba = 200;
        lba_vec[2].sector_count = 3896;
        /*lba_vec[0].start_lba = 0;
        lba_vec[0].sector_count = 256;
        lba_vec[1].start_lba = 256;
        lba_vec[1].sector_count = 256;
        lba_vec[2].start_lba = 512;
        lba_vec[2].sector_count = 256;
*/
        int total_count = 0;

        bool use_synthetic = true;
        int lon_min = 961559;
        int lon_max = 962559;
        int lat_min = 961559;
        int lat_max = 962559;
        int time_min = 34;
        int time_max = 1223;
        struct spatio_temporal_range_predicate synthetic_range_predicate = {.lon_min = lon_min, .lon_max = lon_max, .lat_min = lat_min, .lat_max = lat_max, .time_min = time_min, .time_max = time_max};

        int oid = 0;
        struct id_temporal_predicate synthetic_id_predicate = {.oid = oid, .time_min = time_min, .time_max = time_max};

        for (int i = 0; i < 50; i++) {
            if (i == 5) {
                struct hello_world_sequence sequence;
                size_t sz;
                if (use_synthetic) {
                    init_sequence(&sequence, read_block_size, estimated_result_block_num, 1, ns_entry, &sz,
                                  &synthetic_id_predicate, &synthetic_range_predicate, is_id_temporal_query, lba_vec, lba_vec_size);
                } else {
                    init_sequence(&sequence, read_block_size, estimated_result_block_num, 1, ns_entry, &sz,
                                  id_predicates[i], st_predicates[i], is_id_temporal_query, lba_vec, lba_vec_size);
                }
                clock_t begin = clock();
                rc = spdk_nvme_ns_cmd_exe_multi_fpga(ns_entry->ns, ns_entry->qpair, sequence.buf,
                                                     0, /* LBA start */
                                                     estimated_result_block_num, /* number of LBAs */
                                                     exe_complete, &sequence, 0);

                if (rc != 0) {
                    fprintf(stderr, "starting write I/O failed\n");
                    exit(1);
                }


                while (!sequence.is_completed) {
                    spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
                }
                clock_t end = clock();
                double time_spent = (double) (end - begin) / CLOCKS_PER_SEC;
                printf("read operation time: %f\n", time_spent);
                total_count += sequence.points_num;
            }
        }

        /*
         * Free the I/O qpair.  This typically is done when an application exits.
         *  But SPDK does support freeing and then reallocating qpairs during
         *  operation.  It is the responsibility of the caller to ensure all
         *  pending I/O are completed before trying to free the qpair.
         */
        spdk_nvme_ctrlr_free_io_qpair(ns_entry->qpair);
        printf("total point count: %d\n", total_count);
    }
}



static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
         struct spdk_nvme_ctrlr_opts *opts)
{
    printf("Attaching to %s\n", trid->traddr);

    return true;
}

static void
attach_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
          struct spdk_nvme_ctrlr *ctrlr, const struct spdk_nvme_ctrlr_opts *opts)
{
    int nsid;
    struct ctrlr_entry *entry;
    struct spdk_nvme_ns *ns;
    const struct spdk_nvme_ctrlr_data *cdata;

    entry = malloc(sizeof(struct ctrlr_entry));
    if (entry == NULL) {
        perror("ctrlr_entry malloc");
        exit(1);
    }

    printf("Attached to %s\n", trid->traddr);

    /*
     * spdk_nvme_ctrlr is the logical abstraction in SPDK for an NVMe
     *  controller.  During initialization, the IDENTIFY data for the
     *  controller is read using an NVMe admin command, and that data
     *  can be retrieved using spdk_nvme_ctrlr_get_data() to get
     *  detailed information on the controller.  Refer to the NVMe
     *  specification for more details on IDENTIFY for NVMe controllers.
     */
    cdata = spdk_nvme_ctrlr_get_data(ctrlr);

    snprintf(entry->name, sizeof(entry->name), "%-20.20s (%-20.20s)", cdata->mn, cdata->sn);

    entry->ctrlr = ctrlr;
    TAILQ_INSERT_TAIL(&g_controllers, entry, link);

    /*
     * Each controller has one or more namespaces.  An NVMe namespace is basically
     *  equivalent to a SCSI LUN.  The controller's IDENTIFY data tells us how
     *  many namespaces exist on the controller.  For Intel(R) P3X00 controllers,
     *  it will just be one namespace.
     *
     * Note that in NVMe, namespace IDs start at 1, not 0.
     */
    for (nsid = spdk_nvme_ctrlr_get_first_active_ns(ctrlr); nsid != 0;
         nsid = spdk_nvme_ctrlr_get_next_active_ns(ctrlr, nsid)) {
        ns = spdk_nvme_ctrlr_get_ns(ctrlr, nsid);
        if (ns == NULL) {
            continue;
        }
        register_ns(ctrlr, ns);
    }
}

static void
cleanup(void)
{
    struct ns_entry *ns_entry, *tmp_ns_entry;
    struct ctrlr_entry *ctrlr_entry, *tmp_ctrlr_entry;
    struct spdk_nvme_detach_ctx *detach_ctx = NULL;

    TAILQ_FOREACH_SAFE(ns_entry, &g_namespaces, link, tmp_ns_entry) {
        TAILQ_REMOVE(&g_namespaces, ns_entry, link);
        free(ns_entry);
    }

    TAILQ_FOREACH_SAFE(ctrlr_entry, &g_controllers, link, tmp_ctrlr_entry) {
        TAILQ_REMOVE(&g_controllers, ctrlr_entry, link);
        spdk_nvme_detach_async(ctrlr_entry->ctrlr, &detach_ctx);
        free(ctrlr_entry);
    }

    if (detach_ctx) {
        spdk_nvme_detach_poll(detach_ctx);
    }
}

static void
usage(const char *program_name)
{
    printf("%s [options]", program_name);
    printf("\t\n");
    printf("options:\n");
    printf("\t[-d DPDK huge memory size in MB]\n");
    printf("\t[-g use single file descriptor for DPDK memory segments]\n");
    printf("\t[-i shared memory group ID]\n");
    printf("\t[-r remote NVMe over Fabrics target address]\n");
    printf("\t[-V enumerate VMD]\n");
#ifdef DEBUG
    printf("\t[-L enable debug logging]\n");
#else
    printf("\t[-L enable debug logging (flag disabled, must reconfigure with --enable-debug)\n");
#endif
}

static int
parse_args(int argc, char **argv, struct spdk_env_opts *env_opts)
{
    int op, rc;

    spdk_nvme_trid_populate_transport(&g_trid, SPDK_NVME_TRANSPORT_PCIE);
    snprintf(g_trid.subnqn, sizeof(g_trid.subnqn), "%s", SPDK_NVMF_DISCOVERY_NQN);

    while ((op = getopt(argc, argv, "d:gi:r:L:V")) != -1) {
        switch (op) {
            case 'V':
                g_vmd = true;
                break;
            case 'i':
                env_opts->shm_id = spdk_strtol(optarg, 10);
                if (env_opts->shm_id < 0) {
                    fprintf(stderr, "Invalid shared memory ID\n");
                    return env_opts->shm_id;
                }
                break;
            case 'g':
                env_opts->hugepage_single_segments = true;
                break;
            case 'r':
                if (spdk_nvme_transport_id_parse(&g_trid, optarg) != 0) {
                    fprintf(stderr, "Error parsing transport address\n");
                    return 1;
                }
                break;
            case 'd':
                env_opts->mem_size = spdk_strtol(optarg, 10);
                if (env_opts->mem_size < 0) {
                    fprintf(stderr, "Invalid DPDK memory size\n");
                    return env_opts->mem_size;
                }
                break;
            case 'L':
                rc = spdk_log_set_flag(optarg);
                if (rc < 0) {
                    fprintf(stderr, "unknown flag\n");
                    usage(argv[0]);
                    exit(EXIT_FAILURE);
                }
#ifdef DEBUG
                spdk_log_set_print_level(SPDK_LOG_DEBUG);
#endif
                break;
            default:
                usage(argv[0]);
                return 1;
        }
    }

    return 0;
}

int
main(int argc, char **argv)
{
    int rc;
    struct spdk_env_opts opts;

    /*
     * SPDK relies on an abstraction around the local environment
     * named env that handles memory allocation and PCI device operations.
     * This library must be initialized first.
     *
     */
    spdk_env_opts_init(&opts);
    rc = parse_args(argc, argv, &opts);
    if (rc != 0) {
        return rc;
    }

    opts.name = "hello_world";
    if (spdk_env_init(&opts) < 0) {
        fprintf(stderr, "Unable to initialize SPDK env\n");
        return 1;
    }

    printf("Initializing NVMe Controllers\n");

    if (g_vmd && spdk_vmd_init()) {
        fprintf(stderr, "Failed to initialize VMD."
                        " Some NVMe devices can be unavailable.\n");
    }

    /*
     * Start the SPDK NVMe enumeration process.  probe_cb will be called
     *  for each NVMe controller found, giving our application a choice on
     *  whether to attach to each controller.  attach_cb will then be
     *  called for each controller after the SPDK NVMe driver has completed
     *  initializing the controller we chose to attach.
     */
    rc = spdk_nvme_probe(&g_trid, NULL, probe_cb, attach_cb, NULL);
    if (rc != 0) {
        fprintf(stderr, "spdk_nvme_probe() failed\n");
        rc = 1;
        goto exit;
    }

    if (TAILQ_EMPTY(&g_controllers)) {
        fprintf(stderr, "no NVMe controllers found\n");
        rc = 1;
        goto exit;
    }

    printf("Initialization complete.\n");
    clock_t begin = clock();
    hello_world();
    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("hello world total time: %f\n", time_spent);
    cleanup();
    if (g_vmd) {
        spdk_vmd_fini();
    }

    exit:
    cleanup();
    spdk_env_fini();
    return rc;
}