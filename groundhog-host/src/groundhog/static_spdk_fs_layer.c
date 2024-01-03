
//
// Created by yangguo on 22-12-30.
//

#include "groundhog/static_spdk_fs_layer.h"
#include "spdk/nvme_zns.h"
#include <time.h>
#include "groundhog/config.h"
#define MAX_TRANSFER_SECTOR_COUNT 256

struct callback_sequence {
    struct ns_entry *ns_entry;
    char *buf;  // buf used by spdk and use its spdk_zmalloc
    char *application_read_buf;
    int application_read_buf_size;
    int is_completed;
};


struct spdk_static_fs_desc spdk_static_fs_layer_for_traj;
static struct spdk_nvme_driver_desc spdk_driver_desc;
static struct spdk_static_file_desc file_desc_vec[3];

static bool has_unfinished_requests(struct callback_sequence *sequence_arr, int arr_size);

void microsleep(uint32_t microseconds) {
struct timespec ts = {
        microseconds / 1000000,
        (microseconds % 1000000) * 1000    };

while ((-1 == nanosleep(&ts, &ts)) && (EINTR == errno));
}

void init_and_mk_fs_for_traj(bool is_flushed) {
    init_spdk_nvme_driver(&spdk_driver_desc);

    file_desc_vec[0].current_read_offset = 0;
    file_desc_vec[0].current_write_offset = 0;
    memcpy(file_desc_vec[0].filename, DATA_FILENAME, sizeof(DATA_FILENAME));
    file_desc_vec[0].start_lba = DATA_FILE_OFFSET;
    file_desc_vec[0].sector_count = DATA_FILE_LENGTH;

    file_desc_vec[1].current_read_offset = 0;
    file_desc_vec[1].current_write_offset = 0;
    memcpy(file_desc_vec[1].filename, INDEX_FILENAME, sizeof(INDEX_FILENAME));
    file_desc_vec[1].start_lba = INDEX_FILE_OFFSET;
    file_desc_vec[1].sector_count = INDEX_FILE_LENGTH;

    file_desc_vec[2].current_read_offset = 0;
    file_desc_vec[2].current_write_offset = 0;
    memcpy(file_desc_vec[2].filename, SEG_META_FILENAME, sizeof(SEG_META_FILENAME));
    file_desc_vec[2].start_lba = SEG_META_FILE_OFFSET;
    file_desc_vec[2].sector_count = SEG_META_FILE_LENGTH;

    spdk_mk_static_fs(&spdk_static_fs_layer_for_traj, file_desc_vec, 3, &spdk_driver_desc, is_flushed);

}

void spdk_flush_static_fs_meta_for_traj() {
    spdk_flush_static_fs_meta(&spdk_static_fs_layer_for_traj);
    cleanup_spdk_nvme_driver(&spdk_driver_desc);
}

void print_spdk_static_fs_meta_for_traj() {
    print_spdk_static_fs_meta(&spdk_static_fs_layer_for_traj);
}

static struct spdk_nvme_transport_id g_trid = {};
static bool g_vmd = false;
static TAILQ_HEAD(, ctrlr_entry) g_controllers = TAILQ_HEAD_INITIALIZER(g_controllers);
static TAILQ_HEAD(, ns_entry) g_namespaces = TAILQ_HEAD_INITIALIZER(g_namespaces);



static void serialize_spdk_fs_desc(void* dst, struct spdk_static_fs_desc *desc) {

    if (sizeof(desc->file_desc_vec_num) + desc->file_desc_vec_num * sizeof(struct spdk_static_file_desc) > SECTOR_SIZE) {
        printf("Too large super block size\n");
        return;
    }

    char *d = dst;
    memcpy(d, &(desc->file_desc_vec_num), sizeof(desc->file_desc_vec_num));
    memcpy(d + sizeof(desc->file_desc_vec_num), desc->file_desc_vec, desc->file_desc_vec_num * sizeof(struct spdk_static_file_desc));
}

static void deserialize_spdk_fs_desc(void* source, struct spdk_static_fs_desc *dst, struct spdk_nvme_driver_desc *driver_desc) {
    char *s = source;
    memcpy(&(dst->file_desc_vec_num), s, sizeof(dst->file_desc_vec_num));
    int file_desc_vec_space = dst->file_desc_vec_num * sizeof(struct spdk_static_file_desc);
    void *file_desc_vec_ptr = malloc(file_desc_vec_space);
    memcpy(file_desc_vec_ptr, s + sizeof(dst->file_desc_vec_num), file_desc_vec_space);
    dst->file_desc_vec = file_desc_vec_ptr;
    dst->driver_desc = driver_desc;

    for (int i = 0; i < dst->file_desc_vec_num; i++) {
        dst->file_desc_vec[i].fs_desc = dst;
    }

}

static void write_complete(void *arg, const struct spdk_nvme_cpl *completion) {
    struct callback_sequence *sequence = arg;

    /* Assume the I/O was successful*/
    sequence->is_completed = 1;
    /* See if an error occurred. If so,display information about it*/
    if (spdk_nvme_cpl_is_error(completion)) {
        spdk_nvme_qpair_print_completion(sequence->ns_entry->qpair, (struct spdk_nvme_cpl *)completion);
        fprintf(stderr, "I/O error status: %s\n", spdk_nvme_cpl_get_status_string(&completion->status));
        fprintf(stderr, "Write I/O failed, aborting run\n");
        sequence->is_completed = 2;
        exit(1);
    }

    spdk_free(sequence->buf);
}

static void read_complete(void *arg, const struct spdk_nvme_cpl *completion) {
    struct callback_sequence *sequence = arg;

    /* Assume the I/O was successful*/
    sequence->is_completed = 1;
    /* See if an error occurred. If so,display information about it*/
    if (spdk_nvme_cpl_is_error(completion)) {
        spdk_nvme_qpair_print_completion(sequence->ns_entry->qpair, (struct spdk_nvme_cpl *)completion);
        fprintf(stderr, "I/O error status: %s\n", spdk_nvme_cpl_get_status_string(&completion->status));
        fprintf(stderr, "Read I/O failed, aborting run\n");
        sequence->is_completed = 2;
        exit(1);
    }

    /*for (int i = 0; i < 3; i++) {
        void *tmp_buf = malloc(sequence->application_read_buf_size);
        memcpy(tmp_buf, sequence->buf, sequence->application_read_buf_size);
        memcpy(sequence->application_read_buf, tmp_buf, sequence->application_read_buf_size);
        free(tmp_buf);
    }*/

    memcpy(sequence->application_read_buf, sequence->buf, sequence->application_read_buf_size);


    spdk_free(sequence->buf);


}

static void read_complete_for_mkfs(void *arg, const struct spdk_nvme_cpl *completion) {
    struct callback_sequence *sequence = arg;

    /* Assume the I/O was successful*/
    sequence->is_completed = 1;
    /* See if an error occurred. If so,display information about it*/
    if (spdk_nvme_cpl_is_error(completion)) {
        spdk_nvme_qpair_print_completion(sequence->ns_entry->qpair, (struct spdk_nvme_cpl *)completion);
        fprintf(stderr, "I/O error status: %s\n", spdk_nvme_cpl_get_status_string(&completion->status));
        fprintf(stderr, "Read I/O failed, aborting run\n");
        sequence->is_completed = 2;
        exit(1);
    }

    //memcpy(sequence->application_read_buf, sequence->buf, sequence->application_read_buf_size);
    //spdk_free(sequence->buf);
}


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


int init_spdk_nvme_driver(struct spdk_nvme_driver_desc *driver_desc) {
    int rc;
    struct spdk_env_opts opts;

    spdk_env_opts_init(&opts);
    spdk_nvme_trid_populate_transport(&g_trid, SPDK_NVME_TRANSPORT_PCIE);

    opts.name = "spdk_static_fs_driver";
    if (spdk_env_init(&opts) < 0) {
        fprintf(stderr, "Unable to initialize SPDK env\n");
        return -1;
    }

    printf("Initializing NVMe Controllers\n");

    if (g_vmd && spdk_vmd_init()) {
        fprintf(stderr, "Failed to initialize VMD. Some NVMe devices can be unavailable");
    }

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

    struct ns_entry *ns_entry;

    TAILQ_FOREACH(ns_entry, &g_namespaces, link) {
        ns_entry->qpair = spdk_nvme_ctrlr_alloc_io_qpair(ns_entry->ctrlr, NULL, 0);
        if (ns_entry->qpair == NULL) {
            printf("ERROR: spdk_nvme_ctrlr_alloc_io_pair() failed\n");
        }
        driver_desc->ns_entry = ns_entry;
    }

    return 0;

    exit:
    cleanup();
    spdk_env_fini();

}

void spdk_mk_static_fs(struct spdk_static_fs_desc *fs_desc, struct spdk_static_file_desc *file_desc, int file_desc_num, struct spdk_nvme_driver_desc *driver_desc, bool is_flushed) {
    if (!is_flushed) {
        fs_desc->file_desc_vec = file_desc;
        fs_desc->file_desc_vec_num = file_desc_num;
        fs_desc->driver_desc = driver_desc;
        for (int i = 0; i < file_desc_num; i++) {
            file_desc[i].fs_desc = fs_desc;
        }
    } else {
        // read superblock from disk and deserialize
        char *spdk_buffer_ptr = spdk_zmalloc(0x1000, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
        struct ns_entry *ns_entry = driver_desc->ns_entry;
        struct callback_sequence sequence;
        sequence.is_completed = 0;
        sequence.ns_entry = ns_entry;
        sequence.buf = spdk_buffer_ptr;

        int rc;
        rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence.buf,
                                   SUPER_BLOCK_OFFSET,
                                   1,
                                   read_complete_for_mkfs, (void*)&sequence, 0);

        if (rc != 0) {
            fprintf(stderr, "starting read I/O failed\n");
            exit(1);
        }

        /* poll for completions */
        while (!sequence.is_completed) {
            spdk_nvme_qpair_process_completions(ns_entry->qpair, 1);
        }

        deserialize_spdk_fs_desc(spdk_buffer_ptr, fs_desc, driver_desc);
        spdk_free(sequence.buf);
    }
}

void spdk_flush_static_fs_meta(struct spdk_static_fs_desc *fs_desc) {
    char *spdk_buffer_ptr = spdk_zmalloc(0x1000, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
    serialize_spdk_fs_desc(spdk_buffer_ptr, fs_desc);
    struct ns_entry *ns_entry = fs_desc->driver_desc->ns_entry;
    struct callback_sequence sequence;
    sequence.is_completed = 0;
    sequence.ns_entry = ns_entry;
    sequence.buf = spdk_buffer_ptr;

    /*struct spdk_static_fs_desc rebuild;
    deserialize_spdk_fs_desc(sequence.buf, &rebuild, fs_desc->driver_desc);
    print_spdk_static_fs_meta(&rebuild);*/

    int rc;
    rc = spdk_nvme_ns_cmd_write(ns_entry->ns, ns_entry->qpair, sequence.buf,
                                SUPER_BLOCK_OFFSET,
                                1,
                                write_complete, &sequence, 0);
    if (rc != 0) {
        fprintf(stderr, "starting write I/O failed\n");
        exit(1);
    }

    /* poll for completions */
    while (!sequence.is_completed) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 1);
    }

    //spdk_free(spdk_buffer_ptr);
    printf("finish flush superblock\n");
}

void print_spdk_static_fs_meta(struct spdk_static_fs_desc *fs_desc) {
    printf("file num: %d\n", fs_desc->file_desc_vec_num);
    for (int i = 0; i < fs_desc->file_desc_vec_num; i++) {
        struct spdk_static_file_desc file_desc = fs_desc->file_desc_vec[i];
        printf("filename: %s, start_lba: %d, sector_count: %d, current_write_offset: %d, current_read_offset: %d\n",
               file_desc.filename, file_desc.start_lba, file_desc.sector_count, file_desc.current_write_offset, file_desc.current_read_offset);

    }
}

struct spdk_static_file_desc *spdk_static_fs_fopen(const char *filename, struct spdk_static_fs_desc *fs_desc) {
    for (int i = 0; i < fs_desc->file_desc_vec_num; i++) {
        struct spdk_static_file_desc *file_desc = &(fs_desc->file_desc_vec[i]);
        if (strcmp(file_desc->filename, filename) == 0) {
            file_desc->current_read_offset = 0;
            return file_desc;
        }
    }
    return NULL;
}

size_t spdk_static_fs_fwrite(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc) {

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;
    struct callback_sequence sequence;
    sequence.is_completed = 0;
    sequence.ns_entry = ns_entry;

    size_t spdk_buffer_size = size;
    if (size % 0x1000 != 0) {
        spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
    }
    char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
    if (spdk_buffer_ptr == NULL) {
        printf("ERROR: spdk write buffer allocation failed\n");
        return -1;
    }
    memset(spdk_buffer_ptr, 0, size);
    memcpy(spdk_buffer_ptr, data_ptr, size);
    sequence.buf = spdk_buffer_ptr;

    int rc;
    int lba_start = file_desc->start_lba + file_desc->current_write_offset;
    int sector_count;
    if (size % SECTOR_SIZE == 0) {
        sector_count = size / SECTOR_SIZE;
    } else {
        sector_count = size / SECTOR_SIZE + 1;
    }

    if (file_desc->current_write_offset + sector_count > file_desc->sector_count) {
        fprintf(stderr, "spdk file write: out of boundary.\n");
        exit(1);
    }
    rc = spdk_nvme_ns_cmd_write(ns_entry->ns, ns_entry->qpair, sequence.buf,
                                lba_start,
                                sector_count,
                                write_complete, &sequence, 0);
    if (rc != 0) {
        fprintf(stderr, "starting write I/O failed\n");
        exit(1);
    }

    /* poll for completions */
    while (!sequence.is_completed) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 1);
    }

    // update offset
    file_desc->current_write_offset = file_desc->current_write_offset + sector_count;
    return sector_count * SECTOR_SIZE;
}

size_t spdk_static_fs_fread(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc) {

    if (file_desc->current_read_offset >= file_desc->current_write_offset) {
        // we read all data already
        printf("all data are read.\n");
        return 0;
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;
    struct callback_sequence sequence;
    sequence.is_completed = 0;
    sequence.ns_entry = ns_entry;

    size_t spdk_buffer_size = size;
    if (size % 0x1000 != 0) {
        spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
    }
    char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
    if (spdk_buffer_ptr == NULL) {
        printf("ERROR: spdk write buffer allocation failed\n");
        return -1;
    }
    memset(spdk_buffer_ptr, 0, size);
    sequence.buf = spdk_buffer_ptr;
    sequence.application_read_buf = (char*)data_ptr;
    sequence.application_read_buf_size = size;

    int rc;
    int lba_start = file_desc->start_lba + file_desc->current_read_offset;
    int sector_count;
    if (size % SECTOR_SIZE == 0) {
        sector_count = size / SECTOR_SIZE;
    } else {
        sector_count = size / SECTOR_SIZE + 1;
    }

    if (file_desc->current_read_offset + sector_count > file_desc->sector_count) {
        sector_count = file_desc->sector_count - file_desc->current_read_offset;
    }

    rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequence.buf,
                               lba_start,
                               sector_count,
                               read_complete, (void*)&sequence, 0);

    if (rc != 0) {
        fprintf(stderr, "starting read I/O failed\n");
    }

    /* poll for completions */
    while (!sequence.is_completed) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 1);
    }

    // update offset
    file_desc->current_read_offset = file_desc->current_read_offset + sector_count;
    return sector_count * SECTOR_SIZE;

}

size_t spdk_static_fs_fread_batch(int batch_size, const void **data_ptr_vec, const int *logical_sector_start, const size_t *size_vec, struct spdk_static_file_desc *file_desc) {

    if (file_desc->current_read_offset >= file_desc->current_write_offset) {
        // we read all data already
        printf("all data are read.\n");
        return 0;
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;

    int total_sector_count = 0;
    struct callback_sequence sequences[batch_size];

    for (int i = 0; i < batch_size; i++) {

        sequences[i].is_completed = 0;
        sequences[i].ns_entry = ns_entry;

        size_t spdk_buffer_size = size_vec[i];
        if (size_vec[i] % 0x1000 != 0) {
            spdk_buffer_size = ((size_vec[i] / 0x1000) + 1) * 0x1000;
        }
        char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
        if (spdk_buffer_ptr == NULL) {
            printf("ERROR: spdk write buffer allocation failed\n");
            return -1;
        }
        memset(spdk_buffer_ptr, 0, size_vec[i]);
        sequences[i].buf = spdk_buffer_ptr;
        sequences[i].application_read_buf = (char *) data_ptr_vec[i];
        sequences[i].application_read_buf_size = size_vec[i];


        int rc;
        int lba_start = file_desc->start_lba + logical_sector_start[i];
        int sector_count;
        if (size_vec[i] % SECTOR_SIZE == 0) {
            sector_count = size_vec[i] / SECTOR_SIZE;
        } else {
            sector_count = size_vec[i] / SECTOR_SIZE + 1;
        }

        if (file_desc->current_read_offset + sector_count > file_desc->sector_count) {
            sector_count = file_desc->sector_count - file_desc->current_read_offset;
        }

        total_sector_count += sector_count;

        if (WITH_RESULT) {
            rc = spdk_nvme_ns_cmd_read(ns_entry->ns, ns_entry->qpair, sequences[i].buf,
                                       lba_start,
                                       sector_count,
                                       read_complete, (void *) &sequences[i], 0);
        } else {
            rc = spdk_nvme_ns_cmd_read_no_result(ns_entry->ns, ns_entry->qpair, sequences[i].buf,
                                       lba_start,
                                       sector_count,
                                       read_complete, (void *) &sequences[i], 0);
        }

        if (rc != 0) {
            fprintf(stderr, "starting read I/O failed\n");
        }
        // update offset
        file_desc->current_read_offset = file_desc->start_lba + logical_sector_start[i] + sector_count;

    }

    /* poll for completions */
    while (has_unfinished_requests(sequences, batch_size)) {

        spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
    }
    return total_sector_count * SECTOR_SIZE;

}

size_t spdk_static_fs_fread_multi_addr(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc, struct isp_descriptor *isp_desc) {

    // check the number of sector (should be less than or equal to 256)
    int total_sector_count = 0;
    for (int i = 0; i < isp_desc->lba_count; i++) {
        total_sector_count += isp_desc->lba_array[i].sector_count;
    }
    if (total_sector_count > MAX_TRANSFER_SECTOR_COUNT) {
        fprintf(stderr, "[spdk_fs_isp] too much sector read in one operation\n");
        exit(-1);
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;
    struct callback_sequence sequence;
    sequence.is_completed = 0;
    sequence.ns_entry = ns_entry;

    size_t spdk_buffer_size = size;
    if (size % 0x1000 != 0) {
        spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
    }
    char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
    if (spdk_buffer_ptr == NULL) {
        printf("ERROR: spdk write buffer allocation failed\n");
        return -1;
    }
    memset(spdk_buffer_ptr, 0, size);
    sequence.buf = spdk_buffer_ptr;
    sequence.application_read_buf = (char*)data_ptr;
    sequence.application_read_buf_size = size;

    int rc;
    int lba_start = file_desc->start_lba + file_desc->current_read_offset;
    int sector_count;
    if (size % SECTOR_SIZE == 0) {
        sector_count = size / SECTOR_SIZE;
    } else {
        sector_count = size / SECTOR_SIZE + 1;
    }


    int desc_size = calculate_isp_descriptor_space(isp_desc);
    if (desc_size >= 4096) {
        printf("the descriptor size is too big\n");
    }
    serialize_isp_descriptor(isp_desc, sequence.buf);

    // lba_start and sector_count are not actually used in SSD
    rc = spdk_nvme_ns_cmd_exe_multi(ns_entry->ns, ns_entry->qpair, sequence.buf,
                                         lba_start,
                                         sector_count,
                                         read_complete, (void*)&sequence, 0);

    if (rc != 0) {
        fprintf(stderr, "starting read I/O failed\n");
    }

    /* poll for completions */
    while (!sequence.is_completed) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 1);
    }


    return sector_count * SECTOR_SIZE;
}


size_t spdk_static_fs_fread_isp(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc, struct isp_descriptor *isp_desc) {

    // check the number of sector (should be less than or equal to 256)
    int total_sector_count = 0;
    for (int i = 0; i < isp_desc->lba_count; i++) {
        total_sector_count += isp_desc->lba_array[i].sector_count;
    }
    if (total_sector_count > MAX_TRANSFER_SECTOR_COUNT) {
        fprintf(stderr, "[spdk_fs_isp] too much sector read in one operation\n");
        exit(-1);
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;
    struct callback_sequence sequence;
    sequence.is_completed = 0;
    sequence.ns_entry = ns_entry;

    size_t spdk_buffer_size = size;
    if (size % 0x1000 != 0) {
        spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
    }
    char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
    if (spdk_buffer_ptr == NULL) {
        printf("ERROR: spdk write buffer allocation failed\n");
        return -1;
    }
    memset(spdk_buffer_ptr, 0, size);
    sequence.buf = spdk_buffer_ptr;
    sequence.application_read_buf = (char*)data_ptr;
    sequence.application_read_buf_size = size;

    int rc;
    int lba_start = file_desc->start_lba + file_desc->current_read_offset;
    int sector_count;
    if (size % SECTOR_SIZE == 0) {
        sector_count = size / SECTOR_SIZE;
    } else {
        sector_count = size / SECTOR_SIZE + 1;
    }


    int desc_size = calculate_isp_descriptor_space(isp_desc);
    if (desc_size >= 4096) {
        printf("the descriptor size is too big\n");
    }
    serialize_isp_descriptor(isp_desc, sequence.buf);

    // lba_start and sector_count are not actually used in SSD
    rc = spdk_nvme_ns_cmd_exe_multi(ns_entry->ns, ns_entry->qpair, sequence.buf,
                               lba_start,
                               sector_count,
                               read_complete, (void*)&sequence, 0);

    if (rc != 0) {
        fprintf(stderr, "starting read I/O failed\n");
    }

    /* poll for completions */
    while (!sequence.is_completed) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 1);
    }


    return sector_count * SECTOR_SIZE;
}

size_t spdk_static_fs_fread_isp_fpga(const void *data_ptr, size_t size, struct spdk_static_file_desc *file_desc, struct isp_descriptor *isp_desc) {

    // check the number of sector (should be less than or equal to 256)
    int total_sector_count = 0;
    for (int i = 0; i < isp_desc->lba_count; i++) {
        total_sector_count += isp_desc->lba_array[i].sector_count;
    }
    if (total_sector_count > MAX_TRANSFER_SECTOR_COUNT) {
        fprintf(stderr, "[spdk_fs_isp] too much sector read in one operation\n");
        exit(-1);
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;
    struct callback_sequence sequence;
    sequence.is_completed = 0;
    sequence.ns_entry = ns_entry;

    size_t spdk_buffer_size = size;
    if (size % 0x1000 != 0) {
        spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
    }
    char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
    if (spdk_buffer_ptr == NULL) {
        printf("ERROR: spdk write buffer allocation failed\n");
        return -1;
    }
    memset(spdk_buffer_ptr, 0, size);
    sequence.buf = spdk_buffer_ptr;
    sequence.application_read_buf = (char*)data_ptr;
    sequence.application_read_buf_size = size;

    int rc;
    int lba_start = file_desc->start_lba + file_desc->current_read_offset;
    int sector_count;
    if (size % SECTOR_SIZE == 0) {
        sector_count = size / SECTOR_SIZE;
    } else {
        sector_count = size / SECTOR_SIZE + 1;
    }


    int desc_size = calculate_isp_descriptor_space(isp_desc);
    if (desc_size >= 4096) {
        printf("the descriptor size is too big\n");
    }
    serialize_isp_descriptor(isp_desc, sequence.buf);

    // lba_start and sector_count are not actually used in SSD
    rc = spdk_nvme_ns_cmd_exe_multi_fpga(ns_entry->ns, ns_entry->qpair, sequence.buf,
                                    lba_start,
                                    sector_count,
                                    read_complete, (void*)&sequence, 0);

    if (rc != 0) {
        fprintf(stderr, "starting read I/O failed\n");
    }

    /* poll for completions */
    while (!sequence.is_completed) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 1);
    }


    return sector_count * SECTOR_SIZE;
}


static bool has_unfinished_requests(struct callback_sequence *sequence_arr, int arr_size) {
    bool result = false;
    for (int i = 0; i < arr_size; i++) {
        result = result || !sequence_arr[i].is_completed;
    }
    return result;
}


size_t spdk_static_fs_fread_multi_addr_batch(int batch_size, const void **data_ptr_vec, const size_t *size_vec, struct spdk_static_file_desc *file_desc, struct isp_descriptor **isp_desc_vec) {

    // check the number of sector (should be less than or equal to 256)

    for (int i = 0; i < batch_size; i++) {
        struct isp_descriptor *isp_desc = isp_desc_vec[i];
        unsigned int total_sector_count = 0;
        for (int j = 0; j < isp_desc->lba_count; j++) {
            total_sector_count += isp_desc->lba_array[j].sector_count;
        }
        if (total_sector_count > MAX_TRANSFER_SECTOR_COUNT) {
            fprintf(stderr, "[spdk_fs_isp] too much sector read in one operation\n");
            exit(-1);
        }
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;

    int total_sector_count = 0;
    struct callback_sequence sequences[batch_size];
    for (int i = 0; i < batch_size; i++) {
        sequences[i].is_completed = 0;
        sequences[i].ns_entry = ns_entry;

        int size = size_vec[i];
        size_t spdk_buffer_size = size;
        if (size % 0x1000 != 0) {
            spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
        }
        char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
        if (spdk_buffer_ptr == NULL) {
            printf("ERROR: spdk write buffer allocation failed\n");
            return -1;
        }
        memset(spdk_buffer_ptr, 0, size);
        sequences[i].buf = spdk_buffer_ptr;
        sequences[i].application_read_buf = (char*)data_ptr_vec[i];
        sequences[i].application_read_buf_size = size;

        int rc;
        int lba_start = file_desc->start_lba + file_desc->current_read_offset;
        int sector_count;
        if (size % SECTOR_SIZE == 0) {
            sector_count = size / SECTOR_SIZE;
        } else {
            sector_count = size / SECTOR_SIZE + 1;
        }
        total_sector_count += sector_count;

        int desc_size = calculate_isp_descriptor_space(isp_desc_vec[i]);
        if (desc_size >= 4096) {
            printf("the descriptor size is too big\n");
        }
        serialize_isp_descriptor(isp_desc_vec[i], sequences[i].buf);

        // lba_start and sector_count are not actually used in SSD
        rc = spdk_nvme_ns_cmd_exe_multi(ns_entry->ns, ns_entry->qpair, sequences[i].buf,
                                        lba_start,
                                        sector_count,
                                        read_complete, (void*)&sequences[i], 0);

        if (rc != 0) {
            fprintf(stderr, "starting read I/O failed\n");
        }

    }



    /* poll for completions */
    while (has_unfinished_requests(sequences, batch_size)) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
    }


    return total_sector_count * SECTOR_SIZE;
}

size_t spdk_static_fs_fread_hybrid_comp_batch(struct spdk_static_file_desc *file_desc, int host_batch_size, const void **host_data_ptr_vec, const size_t *host_size_vec, struct isp_descriptor **host_isp_desc_vec,
                                              int pushdown_batch_size, const void **pushdown_data_ptr_vec, const size_t *pushdown_size_vec, struct isp_descriptor **pushdown_isp_desc_vec) {

    // check the number of sector (should be less than or equal to 256)

    for (int i = 0; i < host_batch_size; i++) {
        struct isp_descriptor *isp_desc = host_isp_desc_vec[i];
        unsigned int total_sector_count = 0;
        for (int j = 0; j < isp_desc->lba_count; j++) {
            total_sector_count += isp_desc->lba_array[j].sector_count;
        }
        if (total_sector_count > MAX_TRANSFER_SECTOR_COUNT) {
            fprintf(stderr, "[spdk_fs_isp] too much sector read in one operation\n");
            exit(-1);
        }
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;

    int total_sector_count = 0;
    struct callback_sequence sequences[host_batch_size];
    for (int i = 0; i < host_batch_size; i++) {
        sequences[i].is_completed = 0;
        sequences[i].ns_entry = ns_entry;

        int size = host_size_vec[i];
        size_t spdk_buffer_size = size;
        if (size % 0x1000 != 0) {
            spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
        }
        char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
        if (spdk_buffer_ptr == NULL) {
            printf("ERROR: spdk write buffer allocation failed\n");
            return -1;
        }
        memset(spdk_buffer_ptr, 0, size);
        sequences[i].buf = spdk_buffer_ptr;
        sequences[i].application_read_buf = (char*)host_data_ptr_vec[i];
        sequences[i].application_read_buf_size = size;

        int rc;
        int lba_start = file_desc->start_lba + file_desc->current_read_offset;
        int sector_count;
        if (size % SECTOR_SIZE == 0) {
            sector_count = size / SECTOR_SIZE;
        } else {
            sector_count = size / SECTOR_SIZE + 1;
        }
        total_sector_count += sector_count;

        int desc_size = calculate_isp_descriptor_space(host_isp_desc_vec[i]);
        if (desc_size >= 4096) {
            printf("the descriptor size is too big\n");
        }
        serialize_isp_descriptor(host_isp_desc_vec[i], sequences[i].buf);

        // lba_start and sector_count are not actually used in SSD
        rc = spdk_nvme_ns_cmd_exe_multi(ns_entry->ns, ns_entry->qpair, sequences[i].buf,
                                        lba_start,
                                        sector_count,
                                        read_complete, (void*)&sequences[i], 0);

        if (rc != 0) {
            fprintf(stderr, "starting read I/O failed\n");
        }

    }



    /* poll for completions */
    while (has_unfinished_requests(sequences, host_batch_size)) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
    }


    return total_sector_count * SECTOR_SIZE;
}



size_t spdk_static_fs_fread_isp_batch(int batch_size, const void **data_ptr_vec, const size_t *size_vec, struct spdk_static_file_desc *file_desc, struct isp_descriptor **isp_desc_vec) {

    // check the number of sector (should be less than or equal to 256)

    for (int i = 0; i < batch_size; i++) {
        struct isp_descriptor *isp_desc = isp_desc_vec[i];
        unsigned int total_sector_count = 0;
        for (int j = 0; j < isp_desc->lba_count; j++) {
            total_sector_count += isp_desc->lba_array[j].sector_count;
        }
        if (total_sector_count > MAX_TRANSFER_SECTOR_COUNT) {
            fprintf(stderr, "[spdk_fs_isp] too much sector read in one operation\n");
            exit(-1);
        }
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;

    int total_sector_count = 0;
    struct callback_sequence sequences[batch_size];
    for (int i = 0; i < batch_size; i++) {
        sequences[i].is_completed = 0;
        sequences[i].ns_entry = ns_entry;

        int size = size_vec[i];
        size_t spdk_buffer_size = size;
        if (size % 0x1000 != 0) {
            spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
        }
        char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
        if (spdk_buffer_ptr == NULL) {
            printf("ERROR: spdk write buffer allocation failed\n");
            return -1;
        }
        memset(spdk_buffer_ptr, 0, size);
        sequences[i].buf = spdk_buffer_ptr;
        sequences[i].application_read_buf = (char*)data_ptr_vec[i];
        sequences[i].application_read_buf_size = size;

        int rc;
        int lba_start = file_desc->start_lba + file_desc->current_read_offset;
        int sector_count;
        if (size % SECTOR_SIZE == 0) {
            sector_count = size / SECTOR_SIZE;
        } else {
            sector_count = size / SECTOR_SIZE + 1;
        }
        total_sector_count += sector_count;

        int desc_size = calculate_isp_descriptor_space(isp_desc_vec[i]);
        if (desc_size >= 4096) {
            printf("the descriptor size is too big\n");
        }
        serialize_isp_descriptor(isp_desc_vec[i], sequences[i].buf);

        // lba_start and sector_count are not actually used in SSD
        rc = spdk_nvme_ns_cmd_exe_multi(ns_entry->ns, ns_entry->qpair, sequences[i].buf,
                                             lba_start,
                                             sector_count,
                                             read_complete, (void*)&sequences[i], 0);

        if (rc != 0) {
            fprintf(stderr, "starting read I/O failed\n");
        }

    }



    /* poll for completions */

    while (has_unfinished_requests(sequences, batch_size)) {
        //nanosleep(&ts, &ts);
        //microsleep(10);
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
    }


    return total_sector_count * SECTOR_SIZE;
}



size_t spdk_static_fs_fread_isp_fpga_batch(int batch_size, const void **data_ptr_vec, const size_t *size_vec, struct spdk_static_file_desc *file_desc, struct isp_descriptor **isp_desc_vec) {

    // check the number of sector (should be less than or equal to 256)

    for (int i = 0; i < batch_size; i++) {
        struct isp_descriptor *isp_desc = isp_desc_vec[i];
        unsigned int total_sector_count = 0;
        for (int j = 0; j < isp_desc->lba_count; j++) {
            total_sector_count += isp_desc->lba_array[j].sector_count;
        }
        if (total_sector_count > MAX_TRANSFER_SECTOR_COUNT) {
            fprintf(stderr, "[spdk_fs_isp] too much sector read in one operation\n");
            exit(-1);
        }
    }

    struct ns_entry *ns_entry = file_desc->fs_desc->driver_desc->ns_entry;

    int total_sector_count = 0;
    struct callback_sequence sequences[batch_size];
    for (int i = 0; i < batch_size; i++) {
        sequences[i].is_completed = 0;
        sequences[i].ns_entry = ns_entry;

        int size = size_vec[i];
        size_t spdk_buffer_size = size;
        if (size % 0x1000 != 0) {
            spdk_buffer_size = ((size / 0x1000) + 1) * 0x1000;
        }
        char *spdk_buffer_ptr = spdk_zmalloc(spdk_buffer_size, 0x1000, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_DMA);
        if (spdk_buffer_ptr == NULL) {
            printf("ERROR: spdk write buffer allocation failed\n");
            return -1;
        }
        memset(spdk_buffer_ptr, 0, size);
        sequences[i].buf = spdk_buffer_ptr;
        sequences[i].application_read_buf = (char*)data_ptr_vec[i];
        sequences[i].application_read_buf_size = size;

        int rc;
        int lba_start = file_desc->start_lba + file_desc->current_read_offset;
        int sector_count;
        if (size % SECTOR_SIZE == 0) {
            sector_count = size / SECTOR_SIZE;
        } else {
            sector_count = size / SECTOR_SIZE + 1;
        }
        total_sector_count += sector_count;

        int desc_size = calculate_isp_descriptor_space(isp_desc_vec[i]);
        if (desc_size >= 4096) {
            printf("the descriptor size is too big\n");
        }
        serialize_isp_descriptor(isp_desc_vec[i], sequences[i].buf);

        // lba_start and sector_count are not actually used in SSD
        rc = spdk_nvme_ns_cmd_exe_multi_fpga(ns_entry->ns, ns_entry->qpair, sequences[i].buf,
                                             lba_start,
                                             sector_count,
                                             read_complete, (void*)&sequences[i], 0);

        if (rc != 0) {
            fprintf(stderr, "starting read I/O failed\n");
        }

    }



    /* poll for completions */
    while (has_unfinished_requests(sequences, batch_size)) {
        spdk_nvme_qpair_process_completions(ns_entry->qpair, 0);
    }


    return total_sector_count * SECTOR_SIZE;
}


size_t spdk_static_fs_fseek(struct spdk_static_file_desc *file_desc, long long offset) {
    if (offset / SECTOR_SIZE > INT32_MAX) {
        printf("too large offset in spdk fseek\n");
    }
    int sector_offset = offset / SECTOR_SIZE;
    file_desc->current_read_offset = sector_offset;
    return sector_offset * SECTOR_SIZE;
}

int cleanup_spdk_nvme_driver(struct spdk_nvme_driver_desc *driver_desc) {
    struct ns_entry *ns_entry = driver_desc->ns_entry;
    TAILQ_FOREACH(ns_entry, &g_namespaces, link) {
        spdk_nvme_ctrlr_free_io_qpair(ns_entry->qpair);
    }
    cleanup();
    if (g_vmd) {
        spdk_vmd_fini();
    }
    return 0;
}
