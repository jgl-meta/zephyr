#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t prof_tok_t;

typedef struct {
  // Begin entries have sequential even values. Their corresponding end
  // entries are the odd subsequent value.
  prof_tok_t tok;
  // A unique identifier for the type of profile entry
  uint32_t id;
  // The time in ns at which the begin or end measurement was captured.
  uint64_t time_ns;
  // Args captured at the begin or end of a measurement. Typically, id is
  // used to derive the meaning of these.
  uint64_t arg1;
  uint64_t arg2;
  uint64_t arg3;
} prof_entry_t;

// This struct is returned by EXE_PROFILE_PARAMS.
typedef struct {
  // Maximum number of pending transfers the device can hold.
  uint32_t max_pending_xfers;
  // Maximum number of entries that can be returned per transfer.
  uint32_t max_entries_per_xfer;
  // Snapshot of the device clock (ns) when the request is processed.
  uint64_t device_time_ns;
} prof_params_t;

#define PROF_FLUSH_ALL 0
#define PROF_FLUSH_ONE 1

// This struct is the input arg for EXE_PROFILE_FLUSH.
typedef struct {
  // If zero, flushes all, and then cancels all remaining, pending
  // transfers.
  // If non-zero, attempts to drain one buf.
  uint32_t flush;
} prof_flush_t;

#ifdef __cplusplus
}
#endif
