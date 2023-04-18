// SPDX-License-Identifier: Akache-2.0
/* Copyright (c) 2019 Facebook */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "profile_type.h"

#include <stdint.h>
#include <zephyr/sys/__assert.h>

// A command chain can have multiple concurrent tasks running at the same time,
// each executing a different command in the chain. Each task is identified by
// a task ID which must be used to signal completion of the task.
//
// The task ID should be treated as an opaque handle. Don't change the fields.
typedef union {
  struct {
    // Which SQueue does this belong to.
    uint8_t sqid;
    // Which Entry in the Packet.
    uint8_t index;
    // Which Packet from the SQueue.
    uint16_t packet;
    // Tracks the duration between start/complete of the task.
    prof_tok_t profile_tok;
  };
  uint64_t value;
} task_id_t;

assert(sizeof(task_id_t) == sizeof(uint64_t));

#ifdef __cplusplus
}
#endif
