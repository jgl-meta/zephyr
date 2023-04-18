/*
 * Copyright (c) 2023 Meta Platforms, Inc. and affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INFERENCE_META_PE_H
#define ZEPHYR_INCLUDE_DRIVERS_INFERENCE_META_PE_H

#include "host_command.h"
#include "task_id.h"

struct pe_control_api {
    /**
        Put a command into the admin queue to be processed
        then invoke the completion function to signal completion
     */
    void (*submit_admin_command)(task_id_t tid, struct fba_submission *sub);

    /**
        Put a command to request work be formed/started/stopped/updated
        then invoke the completion function to signal completion
     */
    void (*submit_work_command)(task_id_t tid, struct fba_submission *sub);

    /**
        Starts statistic gathering
     */
    void (*sail_activate)(void);

    /**
        Stops statistic gathering
     */
    void (*sail_deactivate)(void);
};

#endif
