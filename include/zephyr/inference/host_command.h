// SPDX-License-Identifier: Apache-2.0
/* Copyright (c) 2019 Facebook */

#pragma once

//
// Data structures defining commands submitted by the host and responses from
// the device.
//
#include "constants.h"
#include "types.h"

#include <zephyr/sys/__assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COMPLETION_ENTRY_ALIGNMENT 8
#define SUBMISSION_ENTRY_ALIGNMENT 64

// Responses from the device to the host are all 64 bits in size.
struct fba_completion {
  // Identifier for the completed command. Copied from the command's header.
  uint16_t command_id;

  // New head pointer for the squeue containing the command. This is used
  // to indicate to the host how many submission queue entries have been
  // consumed by the device so far.
  cb_index_t sq_head;

  // Command completion status. See enum completion_status in constants.h.
  uint16_t status;

  // Flags used to interpret status.  If the high bit is set the command
  // failed.
  uint8_t flags;

  // Phase tag. This field is written as 1 on the first pass through the
  // circular buffer, and incremented each time the buffer wraps around.
  // It is used by the host to identify the last entry written by the
  // device.
  //
  // This field only requires 1 bit to function correctly. We use 8 bits
  // as a debugging aid and because the bits are available. We may take
  // some of these bits in the future if we need to add other flags.
  uint8_t phase_tag;
};
assert(sizeof(struct fba_completion) == 8);
assert(sizeof(struct fba_completion) % COMPLETION_ENTRY_ALIGNMENT == 0);

// All submission queue entries start with the same 16-byte header.
struct submission_header {
  // Fields provided by USER
  uint16_t opcode; // Identifies command and args format.
  uint16_t reserved_usr; // 16b reserved value.
  uint32_t flags; // 31:24 CMD Flags, 23:0 ARG Flags.

  // Fields filled in by DRIVER
  uint16_t command_id; // Opaque ID mirrored on completion.
  uint16_t packet_tail;
  uint8_t context_id;
  uint8_t reserved_drv[3];
};
assert(sizeof(struct submission_header) == 16);

// Payload for ADMIN_{CREATE,DELETE}_CONTEXT
struct admin_ctx {
  // Context ID to create/delete.
  uint8_t id;
};

// Payload for ADMIN_GET_INFO_WORDS.
//
// The requested info words are copied to host memory.
struct admin_info_words {
  // First 64-bit info word to read. See enum info_word.
  uint32_t first_word;

  // Number of consecutive 64-bit info words to copy into host memory.
  uint32_t num_words;

  // Destination address in host memory where info words are written.
  pci_addr_t host_addr;
};

// Payload for ADMIN_ASYNC commands.
struct admin_async {
  pci_addr_t data_addr; // Host address of the buffer.
  uint32_t data_size; // Size of the block.
};
// Payload for EXE_DMA_{TO,FROM}_DEVICE_{CONTIG,PRP}.
struct exe_dma {
  pci_addr_t host_addr; // PCI address of CONTIG or PRP List
  uint32_t host_bytes; // Number of expected host bytes must match next.
  uint32_t bytes; // Number of expected transfer bytes must match prev.
  device_addr_t device_addr; // Address on the device.
  uint64_t reserved[3];
};

// Payload for EXE_DMA_HOST
struct exe_dma_host {
  pci_addr_t host_addr; // PCI address of CONTIG or PRP List
  uint64_t enc_size; // Encoded size and flags
  device_addr_t device_addr; // Address on the device.
};

struct exe_noop {
  uint32_t sleep_msec;
  uint16_t status; // Status value to return.
  uint8_t flags; // Flag value to return.
};

struct exe_perf {
  pci_addr_t host_addr;
  uint32_t size;
};

// TODO(T92479820): merge exe_perf and exe_mon to a single
//    struct containing host address and size.
struct va_dma_op {
  pci_addr_t host_addr;
  // flags_size is used differently for prp vs. contiguous dma transfers:
  // - requests for prp dma transfers set both size and flags.
  // - requests for contiguous dma transfers set only the size.
  uint64_t flags_size;
};

struct exe_buff_crc_arg {
  // SAIL buffer address
  uint64_t buffer;
  uint32_t size;
  uint32_t result_buffer_size;
  uint64_t result_buffer;
};

struct try_catch_op {
  uint64_t c_idx; // Catch Index.
  uint64_t f_idx; // Finally Index.
};

struct stream_op {
  uint64_t stream;
  uint64_t order;
};

// analogous to struct timespec in time.h without with 32-bit / 64-bit ambiguity removed
struct time_spec_64 {
  int64_t tv_sec;
  int64_t tv_nsec;
};

struct exe_log_test {
  /* The level of logged messages */
  uint8_t level;
  /* The number of simultaneous threads */
  uint8_t num_threads;
  /* The number of seconds to run the test */
  uint16_t duration_s;
  /* The minimum number of logs written before yielding */
  uint16_t min_burst_size;
  /* The maximum number of logs written before yielding */
  uint16_t max_burst_size;
  /* The minimum number of ms to sleep between bursts */
  uint16_t min_burst_delay_ms;
  /* The maximum number of ms to sleep between bursts */
  uint16_t max_burst_delay_ms;
};

struct exe_log_level {
  uint8_t id;
  uint8_t level;
};

#define EXE_LOG_LEVEL_SET_MAX                                                  \
  ((SUBMISSION_ENTRY_ARGS * sizeof(uint64_t) - sizeof(uint8_t)) /        \
   sizeof(struct exe_log_level))

struct exe_log_level_set {
  /*
   * to set all levels, consider map as a char* and write
   * ['a', 'l', 'l', <level>]
   */
  uint8_t map_size;
  struct exe_log_level map[EXE_LOG_LEVEL_SET_MAX];
};

// 64-byte submission queue entry.
struct fba_submission {
  // Common 16-byte header.
  struct submission_header header;

  // 48-byte payload depends on header.opcode as documented in the structs.
  union {
    struct admin_info_words info_words;
    struct admin_ctx ctx;
    struct admin_async async;
    struct exe_dma dma;
    struct exe_dma_host dma_host;
    struct exe_noop noop;
    struct exe_perf perf;
    struct va_dma_op va_dma;
    struct try_catch_op try_catch;
    struct stream_op stream;
    struct time_spec_64 time_spec;
    struct exe_log_test log_test;
    struct exe_log_level_set log_level_set;
    uint64_t args[SUBMISSION_ENTRY_ARGS];
  };
};
assert(sizeof(struct fba_submission) == 64);
assert(sizeof(struct fba_submission) % SUBMISSION_ENTRY_ALIGNMENT == 0);

/*
 * FBA Result Object
 *
 * A result object represents the result for a packet returned
 * from the driver to the application, including the original ID.
 */
struct fba_job_result {
  struct fba_completion cmp;
  uint64_t user_id;
};
assert(sizeof(struct fba_job_result) == 16);

#ifdef __cplusplus
}
#endif
