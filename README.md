# Groundhog
There are three parts in Groundhog project.
- **groundhog-host**: this directory contains the code on the host side including the self-contained and segment-based library, a simple file system, and codes used in the experiments.
- **myssd_sdk**: this directory contains the FTL code running inside the CSD. We extend the FTL to support the offloading procedure and the computation of spatio-temporal queries.
- **spdk**: this directory is forked from spdk library. We extend the NVMe to support the ISC command.

The whole codebase relies on a specific hardware platform to run. Here, we introduce several major contents in each parts.
## groundhog-host
- The code of the format library: [`groundhog-host/src/groundhog/traj_block_format.c`](https://github.com/LAccordeur/groundhog/blob/main/groundhog-host/src/groundhog/traj_block_format.c).
- The experiment codes: [`groundhog-host/src/groundhog/exp/exp_native_host_and_device.c`](https://github.com/LAccordeur/groundhog/tree/main/groundhog-host/src/groundhog/exp/exp_native_host_and_device.c).

## myssd_sdk
The code of ISC query engine is in the directory [`myssd_sdk/src/groundhog-src`](https://github.com/LAccordeur/groundhog/tree/main/myssd_sdk/src/groundhog-src).

We extend the FTL to support the offloading:
  - Fetch the ISC descriptors from the host memory: [`process_io_request`](https://github.com/LAccordeur/groundhog/blob/main/myssd_sdk/src/ftl/ftl.c#L232)
  - Supporting the ISC cmds: [`handle_cached_exe_multi`](https://github.com/LAccordeur/groundhog/blob/main/myssd_sdk/src/ftl/data_cache.c#L1125).
  - Return the reduced result size: [`transfer_prp_data`](https://github.com/LAccordeur/groundhog/blob/main/myssd_sdk/src/hostif/nvme.c#L106).

## spdk
We extend to following code files to support ISC commands:
- `include/spdk/nvme.h`
- `include/spdk/nvme_spec.h`
- `lib/nvme/nvme_ns_cmd.c`

An added function example can be found [`spdk_nvme_ns_cmd_exe_multi`](https://github.com/LAccordeur/groundhog/blob/main/spdk/lib/nvme/nvme_ns_cmd.c#L847).
