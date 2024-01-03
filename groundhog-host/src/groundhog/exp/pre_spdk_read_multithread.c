//
// Created by yangguo on 23-4-6.
//
#include "spdk/stdinc.h"

#include "spdk/nvme.h"
#include "spdk/vmd.h"
#include "spdk/nvme_zns.h"
#include "spdk/env.h"
#include "spdk/string.h"
#include "spdk/log.h"
#include "groundhog/traj_block_format.h"
#include <pthread.h>

struct ctrlr_entry {
    struct spdk_nvme_ctrlr		*ctrlr;
    TAILQ_ENTRY(ctrlr_entry)	link;
    char				name[1024];
};

struct ns_entry {
    struct spdk_nvme_ctrlr	*ctrlr;
    struct spdk_nvme_ns	*ns;
    TAILQ_ENTRY(ns_entry)	link;
    struct spdk_nvme_qpair	*qpair;
};

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
};

static int
iterate_raw_traj_block(void* block) {
    int count = 0;
    struct traj_block_header header;
    parse_traj_block_for_header(block, &header);

    struct seg_meta meta_array[header.seg_count];
    //if (header.seg_count != 10) {
        printf("seg count: %d\n", header.seg_count);
    //}
    parse_traj_block_for_seg_meta_section(block, meta_array, header.seg_count);
    for (int j = 0; j < header.seg_count; j++) {
        struct seg_meta meta_item = meta_array[j];

        int data_seg_points_num = meta_item.seg_size / get_traj_point_size();
        count += data_seg_points_num;
        struct traj_point **points = allocate_points_memory(data_seg_points_num);
        parse_traj_block_for_seg_data(block, meta_item.seg_offset, points, data_seg_points_num);
        //print_traj_points(points, data_seg_points_num);
        free_points_memory(points, data_seg_points_num);

    }
    return count;
}

static int iterate_block(void* block) {
    char *c;

    for (int i = 0; i < 256; i++) {
        c = (char *) block + i * 4096;
        int *int_ptr = (int *)c;
        printf("%d\n", int_ptr[0]);
    }
    //printf("%d\n", int_ptr[0]);
    return 4096;
}

static void
read_complete(void *arg, const struct spdk_nvme_cpl *completion)
{
    struct hello_world_sequence *sequence = arg;

    /* Assume the I/O was successful */
    sequence->is_completed = 1;
    /* See if an error occurred. If so, display information
     * about it, and set completion value so that I/O
     * caller is aware that an error occurred.
     */
    //printf("completion result: %d\n", spdk_nvme_cpl_is_error(completion));
    //printf("sc: %d, sct: %d, crd: %d, dnr: %d\n", completion->status.sc, completion->status.sct, completion->status.crd, completion->status.dnr);
    if (spdk_nvme_cpl_is_error(completion)) {
        spdk_nvme_qpair_print_completion(sequence->ns_entry->qpair, (struct spdk_nvme_cpl *)completion);
        printf("sc: %d, sct: %d, crd: %d, dnr: %d\n", completion->status.sc, completion->status.sct, completion->status.crd, completion->status.dnr);
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
    //int count = iterate_raw_traj_block(sequence->buf);
    //int count = iterate_block(sequence->buf);
    //spdk_free(sequence->buf);
    //sequence->points_num = count;
}

static void
write_complete(void *arg, const struct spdk_nvme_cpl *completion)
{
    struct hello_world_sequence	*sequence = arg;
    struct ns_entry			*ns_entry = sequence->ns_entry;
    int				rc;

    /* See if an error occurred. If so, display information
     * about it, and set completion value so that I/O
     * caller is aware that an error occurred.
     */
    if (spdk_nvme_cpl_is_error(completion)) {
        spdk_nvme_qpair_print_completion(sequence->ns_entry->qpair, (struct spdk_nvme_cpl *)completion);
        fprintf(stderr, "I/O error status: %s\n", spdk_nvme_cpl_get_status_string(&completion->status));
        fprintf(stderr, "Write I/O failed, aborting run\n");
        sequence->is_completed = 2;
        exit(1);
    }
    /*
     * The write I/O has completed.  Free the buffer associated with
     *  the write I/O and allocate a new zeroed buffer for reading
     *  the data back from the NVMe namespace.
     */
    if (sequence->using_cmb_io) {
        spdk_nvme_ctrlr_unmap_cmb(ns_entry->ctrlr);
    } else {
        spdk_free(sequence->buf);
    }
    sequence->buf = spdk_zmalloc(0x1000, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);

    rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence->buf,
                               0, /* LBA start */
                               1, /* number of LBAs */
                               read_complete, (void *)sequence, 0);
    if (rc != 0) {
        fprintf(stderr, "starting read I/O failed\n");
        exit(1);
    }
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
init_sequence(struct hello_world_sequence *sequence, size_t block_size, int block_index, struct ns_entry	*ns_entry, size_t *sz) {
    /*
         * Use spdk_dma_zmalloc to allocate a 4KB zeroed buffer.  This memory
         * will be pinned, which is required for data buffers used for SPDK NVMe
         * I/O operations.
         */
    sequence->using_cmb_io = 1;
    sequence->buf = spdk_nvme_ctrlr_map_cmb(ns_entry->ctrlr, sz);
    if (sequence->buf == NULL || *sz < block_size) {
        sequence->using_cmb_io = 0;
        sequence->buf = spdk_zmalloc(block_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
    }
    if (sequence->buf == NULL) {
        printf("ERROR: write buffer allocation failed\n");
        return;
    }
    if (sequence->using_cmb_io) {
        printf("INFO: using controller memory buffer for IO\n");
    } else {
        //printf("INFO: using host memory buffer for IO\n");
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

}

static void*
hello_world(void* ptr)
{
    struct ns_entry			*ns_entry;
    int				rc;


    TAILQ_FOREACH(ns_entry, &g_namespaces, link) {
        /*
         * Allocate an I/O qpair that we can use to submit read/write requests
         *  to namespaces on the controller.  NVMe controllers typically support
         *  many qpairs per controller.  Any I/O qpair allocated for a controller
         *  can submit I/O to any namespace on that controller.
         *
         * The SPDK NVMe driver provides no synchronization for qpair accesses -
         *  the application must ensure only a single thread submits I/O to a
         *  qpair, and that same thread must also check for completions on that
         *  qpair.  This enables extremely efficient I/O processing by making all
         *  I/O operations completely lockless.
         */
        ns_entry->qpair = spdk_nvme_ctrlr_alloc_io_qpair(ns_entry->ctrlr, NULL, 0);
        if (ns_entry->qpair == NULL) {
            printf("ERROR: spdk_nvme_ctrlr_alloc_io_qpair() failed\n");
            return;
        }


        int read_block_size = 4096;
        int read_block_num = 1;

        int total_count = 0;
        clock_t start, end;
        start = clock();
        for (int i = 0; i < read_block_num; i++) {
            struct hello_world_sequence	sequence;
            size_t				sz;
            init_sequence(&sequence,  read_block_size * 256, i+1, ns_entry, &sz);
            //printf("i: %d\n", i);
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence.buf,
                                        0, /* LBA start */
                                       256, /* number of LBAs */
                                       read_complete, &sequence, 0);


            if (rc != 0) {
                fprintf(stderr, "starting write I/O failed\n");
                exit(1);
            }


            struct hello_world_sequence	sequence1;
            size_t				sz1;
            init_sequence(&sequence1,  read_block_size * 256, i+1, ns_entry, &sz1);
            //printf("i: %d\n", i);
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence1.buf,
                                       256, /* LBA start */
                                       256, /* number of LBAs */
                                       read_complete, &sequence1, 0);


            struct hello_world_sequence	sequence2;
            size_t				sz2;
            init_sequence(&sequence2,  read_block_size * 256, i+1, ns_entry, &sz2);
            //printf("i: %d\n", i);
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence2.buf,
                                       512, /* LBA start */
                                       256, /* number of LBAs */
                                       read_complete, &sequence2, 0);

            struct hello_world_sequence	sequence3;
            size_t				sz3;
            init_sequence(&sequence3,  read_block_size * 256, i+1, ns_entry, &sz3);
            //printf("i: %d\n", i);
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence3.buf,
                                       512, /* LBA start */
                                       256, /* number of LBAs */
                                       read_complete, &sequence3, 0);

            struct hello_world_sequence	sequence4;
            size_t				sz4;
            init_sequence(&sequence4,  read_block_size * 256, i+1, ns_entry, &sz4);
            //printf("i: %d\n", i);
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence4.buf,
                                       768, /* LBA start */
                                       256, /* number of LBAs */
                                       read_complete, &sequence4, 0);

            struct hello_world_sequence	sequence5;
            size_t				sz5;
            init_sequence(&sequence5,  read_block_size * 256, i+1, ns_entry, &sz5);
            //printf("i: %d\n", i);
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence5.buf,
                                       1024, /* LBA start */
                                       256, /* number of LBAs */
                                       read_complete, &sequence5, 0);



            while (!sequence.is_completed || !sequence1.is_completed || !sequence2.is_completed || !sequence3.is_completed || !sequence4.is_completed || !sequence5.is_completed) {
                spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
            }
            total_count += sequence.points_num;

        }
        end = clock();
        spdk_nvme_ctrlr_free_io_qpair(ns_entry->qpair);

        printf("total time: %f second\n",(double)(end-start) / CLOCKS_PER_SEC);
        printf("total point count: %d\n", total_count);
    }
}

static bool has_unfinished_requests(struct hello_world_sequence *sequence_arr, int arr_size) {
    bool result = false;
    for (int i = 0; i < arr_size; i++) {
        result = result || !sequence_arr[i].is_completed;
    }
    return result;
}

static void*
hello_world_multi(void* ptr)
{
    struct ns_entry			*ns_entry;
    int				rc;


    TAILQ_FOREACH(ns_entry, &g_namespaces, link) {
        /*
         * Allocate an I/O qpair that we can use to submit read/write requests
         *  to namespaces on the controller.  NVMe controllers typically support
         *  many qpairs per controller.  Any I/O qpair allocated for a controller
         *  can submit I/O to any namespace on that controller.
         *
         * The SPDK NVMe driver provides no synchronization for qpair accesses -
         *  the application must ensure only a single thread submits I/O to a
         *  qpair, and that same thread must also check for completions on that
         *  qpair.  This enables extremely efficient I/O processing by making all
         *  I/O operations completely lockless.
         */
        ns_entry->qpair = spdk_nvme_ctrlr_alloc_io_qpair(ns_entry->ctrlr, NULL, 0);
        if (ns_entry->qpair == NULL) {
            printf("ERROR: spdk_nvme_ctrlr_alloc_io_qpair() failed\n");
            return;
        }


        int read_block_size = 4096;
        int request_num = 16;

        int total_count = 0;

        struct hello_world_sequence sequence_arr[request_num];
        size_t sz_arr[request_num];
        clock_t start, end;
        start = clock();
        for (int i = 0; i < request_num; i++) {

            init_sequence(&sequence_arr[i],  read_block_size * 256, i+1, ns_entry, &sz_arr[i]);
            //printf("i: %d\n", i);
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, (sequence_arr[i]).buf,
                                       i*256, /* LBA start */
                                       256, /* number of LBAs */
                                       read_complete, &(sequence_arr[i]), 0);


            if (rc != 0) {
                fprintf(stderr, "starting write I/O failed\n");
                exit(1);
            }


        }

        while (has_unfinished_requests(sequence_arr, request_num)) {
            spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
        }

        end = clock();
        spdk_nvme_ctrlr_free_io_qpair(ns_entry->qpair);

        printf("total time: %f second\n",(double)(end-start) / CLOCKS_PER_SEC);
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

    pthread_t thread1, thread2;
    int iret1, iret2;
    iret1 = pthread_create(&thread1, NULL, hello_world_multi, NULL);
    //iret2 = pthread_create(&thread2, NULL, hello_world, NULL);

    pthread_join(thread1, NULL);
    //pthread_join(thread2, NULL);
    cleanup();
    if (g_vmd) {
        spdk_vmd_fini();
    }

    exit:
    cleanup();
    spdk_env_fini();
    return rc;
}