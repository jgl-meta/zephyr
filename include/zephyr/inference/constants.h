// SPDX-License-Identifier: Apache-2.0
/* Copyright (c) 2019 Facebook */

#pragma once

/*
 * FBA Common Constants
 *
 * Constants shared between application, driver and firmware.
 *
 */

#include "infra_asic_fpga/firmware/lib/fba_core/constants_mon.h"

#ifdef __cplusplus
extern "C" {
#endif

// Platform types
#define PLATFORM_SILICON 0
#define PLATFORM_NON_SILICON 1

#define AT_DMA_FLAG_SHIFT 56
#define AT_DMA_SIZE_MASK ((1ull << AT_DMA_FLAG_SHIFT) - 1ull) // Size mask.
#define AT_DMA_TO_DEVICE (0x01ull << AT_DMA_FLAG_SHIFT) // To device op.
#define AT_DMA_FROM_DEVICE (0x02ull << AT_DMA_FLAG_SHIFT) // From device op.
#define AT_DMA_PRP (0x40ull << AT_DMA_FLAG_SHIFT) // Addr is PRP not Contig.
#define AT_DMA_OPSZ (0x80ull << AT_DMA_FLAG_SHIFT) // Size as OP Flags.
#define AT_DMA_GET_SZ(x) ((x)&AT_DMA_SIZE_MASK)
#define AT_DMA_GET_OP(x) ((x) & ~AT_DMA_SIZE_MASK)
#define AT_DMA_SET_SZ(arg, sz)                                                 \
  arg = AT_DMA_GET_OP(arg) | ((sz)&AT_DMA_SIZE_MASK)
#define AT_DMA_GET_DIR(x) ((x) & (AT_DMA_TO_DEVICE | AT_DMA_FROM_DEVICE))

#define AT_FIELD_BIT_WIDTH 4
#define AT_FIELD_BIT_MASK ((1 << AT_FIELD_BIT_WIDTH) - 1)
// idx below is the uint64_t arg index (legal values [0..5]) within the
// submission args array for which to get or make arg-type (AT) fields.
#define AT_FIELD_GET(idx, flags)                                               \
  (((flags) >> ((idx)*AT_FIELD_BIT_WIDTH)) & AT_FIELD_BIT_MASK)
#define AT_FIELD_MAKE(idx, at) ((at) << ((idx)*AT_FIELD_BIT_WIDTH))

enum argument_type {
  AT_RAW = 0, // Raw data.
  AT_RID, // 64b resource ID.
  AT_RID_ARG32, // 64b resource ID followed by 32b arg.
  AT_ARG32_RID, // 32b arg followed by 64b resource ID.
  AT_ARG64_RID, // 64b arg followed by 64b resource ID.
  AT_VA_DMA64, // 64b host VA followed by 64b DMA op desc.
  AT_P2P_DMA64, // 64b global RID followed by 64b DMA op desc.
};

// TODO(T84637279): Remove these constants, they should be negotiated.
// This change will be required before M7<->CPP support.
#define SAIL_SCRATCH_PAGE_CNT 24
#define SAIL_STATUS_PAGE_CNT 1
#define SAIL_DEBUG_PAGE_CNT 16
#define SAIL_SCRATCH_SIZE (4096 * SAIL_SCRATCH_PAGE_CNT)
#define SAIL_STATUS_SIZE (4096 * SAIL_STATUS_PAGE_CNT)
#define SAIL_DEBUG_SIZE (4096 * SAIL_DEBUG_PAGE_CNT)
#define SAIL_AVAILABLE_SCRATCH_SIZE ((SAIL_SCRATCH_SIZE) - (SAIL_DEBUG_SIZE))

// RID Bit fields
#define RID_ID_BITS 16
#define RID_ID_SHIFT (64 - RID_ID_BITS)
#define RID_OFFS_BITS (64 - RID_ID_BITS)
#define RID_OFFS_MASK ((1ULL << RID_OFFS_BITS) - 1)

// Submission Entry Bit fields
#define SUBMISSION_ENTRY_ARGS 6
#define SUBMISSION_ENTRY_ARG_SIZE sizeof(uint64_t)
#define SQENTRY_ARG_BITS (AT_FIELD_BIT_WIDTH * SUBMISSION_ENTRY_ARGS)
#define SQENTRY_ARG_MASK ((1 << SQENTRY_ARG_BITS) - 1)
#define SQENTRY_CMD_SHIFT SQENTRY_ARG_BITS
#define SQENTRY_FLAG_BARRIER (0x01 << SQENTRY_CMD_SHIFT)

// Completion Entry Bit fields
#define CQENTRY_FLAG_FAIL 0x80

/*
 * SAIL Negotiation Registers
 *
 * Registers are used to negotiate setup of the channel(s) prior to standard
 * inband Client/Server.  The Server defines the main channel properties,
 * which the Client must confirm it can use prior to activation.
 *
 * Client Rd registers must be provided by Server prior to setting SAIL_RDY.
 * The Client will read the values, confirm compatibility, then program any
 * Client Wr registers. Once programmed, client will request activation by
 * setting ACTIVE bit of SAIL_CTRL.  The Server will detect the change and
 * read Client Wr registers to fetch remaing config data.  Server will finish
 * any setup and when done it will clear SAIL_RDY and set ACTIVE bit in
 * SAIL_STAT to indicate setup is finished.
 *
 * While ACTIVE bit is set, Client is free to use the channels.
 */
typedef enum {
  SAIL_CTRL = 0x00, // Control Register (Client Wr)
  SAIL_STAT = 0x01, // Status Register (Client Rd)
  SAIL_BDES = 0x02, // Buffer Descriptor (Client Rd)
  SAIL_CDES = 0x03, // Channel Descriptor (Client Rd)
  SAIL_HDES = 0x04, // Host Descriptor (Client Wr)
  SAIL_RDY = 0x05, // Ready Marker (Client Rd)
  SAIL_SQAL = 0x06, // SQ Addr Low (BDES determines ownership)
  SAIL_SQAH = 0x07, // SQ Addr High (BDES determines ownership)
  SAIL_CQAL = 0x08, // CQ Addr Low (BDES determines ownership)
  SAIL_CQAH = 0x09, // CQ Addr High (BDES determines ownership)
  SAIL_SCAL = 0x0A, // Scratch Area Low (Client Wr)
  SAIL_SCAH = 0x0B, // Scratch Area Low (Client Wr)
  SAIL_STAL = 0x0C, // Status Addr Low (Client Wr)
  SAIL_STAH = 0x0D, // Status Addr High (Client Wr)
  SAIL_VERS = 0x0E, // Version Info (Client Rd)
  SAIL_STRM = 0x10, // Streaming channel configuration
  SAIL_ADMIN_CMD_TIMEOUT = 0x11, // Admin Command Timeout time
  SAIL_PKT_SIZE = 0x14, // Packet size configuration
  SAIL_DCMD = 0x18, // Debug Command Registers (Client Wr)
  SAIL_DA0L = 0x1A, // Debug ARG0 Low Registers (Client Wr)
  SAIL_DA0H = 0x1B, // Debug ARG0 High Registers (Client Wr)
  SAIL_DA1L = 0x1C, // Debug ARG0 Low Registers (Client Wr)
  SAIL_DA1H = 0x1D, // Debug ARG0 High Registers (Client Wr)
  SAIL_DA2L = 0x1E, // Debug ARG0 Low Registers (Client Wr)
  SAIL_DA2H = 0x1F, // Debug ARG0 High Registers (Client Wr)
  SAIL_MAILBOX_END,
} sail_regs_t;

// Assert that SAIL mailboxes do not collide with doorbell mailboxes.
STATIC_ASSERT(SAIL_MAILBOX_END <= 32);

/*
 *
 * SAIL Constants
 */
// Admin command timeout
#define SAIL_ADMIN_CMD_MAX_TIMEOUT_MS 50000

#define SAIL_ADMIN_CMD_MIN_TIMEOUT_MS 6000
// The magic value the Server writes to SAIL_RDY after populating Client Rd
// registers.
#define SAIL_RDY_MAGIC 0xFACEB00C

#define IATU_CONFIGURED 0xFACEB00C

// Fields within SAIL_SDESC
#define SAIL_SQ_ENTRY_ORD 6
#define SAIL_CQ_ENTRY_ORD 3

/*
 * SAIL Channel Descriptor
 * [00:07] CNUM - Number of channels.
 * [11:08] CQES - CQ Entry Size (2^N)
 * [15:12] SQES - SQ Entry Size (2^N)
 * [31:16] ENTR - Number of entries per channel.
 */
#define SAIL_CDES_CNUM(cdes) ((cdes)&0xFF)
#define SAIL_CDES_CQES(cdes) (1 << (((cdes) >> 8) & 0xF))
#define SAIL_CDES_SQES(cdes) (1 << (((cdes) >> 12) & 0xF))
#define SAIL_CDES_PQEC(cdes) (((cdes) >> 16) & 0xFFFF)
#define SAIL_CDES_DEFINE(ent, chan)                                            \
  (((ent) << 16) | (SAIL_SQ_ENTRY_ORD << 12) |                           \
   (SAIL_CQ_ENTRY_ORD << 8) | (chan))

/*
 * SAIL Buffer Descriptor
 * Describes which channel buffers are provided by Host and which are provived
 * by FW.  Includes number of requested pages for the Status area, which will
 * be backed by Host memory, but sub-allocated by Firmware.
 *
 * [07:00] STPC - Status Page Count (number of 4K pages).
 * [30:30] HCQ - Host provided CQ Buffer.
 * [31:31] HSQ - Host provided SQ Buffer.
 */
#define SAIL_BDES_STPC(bdes) ((bdes)&0xFF) // Size of Status in pages.
#define SAIL_BDES_HCQ (1u << 30) // Host provided CQ
#define SAIL_BDES_HSQ (1u << 31) // Host provided SQ
#define SAIL_BDES_DEFINE(flags, sspc) ((flags) | (sspc))
/*
 * SAIL Host Descriptor
 * Describes host provided buffers and flags.
 *
 * [07:00] FLGS - Host provided flags.
 * [31:24] SCPC - Scratch Page Count (number of 4K pages).
 */
#define SAIL_HDES_SCPC(hdes) ((hdes >> 24) & 0xFF) // Size of Status in pages.
#define SAIL_HDES_DEFINE(scratch_page_cnt, flags)                              \
  ((scratch_page_cnt << 24) | (flags))

/*
 * SAIL Version
 * Version HI/LO can be uses to determine if FW and Host share a
 * compatible SAIL communication layer. Application version can be
 * passed to host app to determine feature compatibility.
 *
 * [15:00] VAPP - Application Version ID.
 * [23:16] VLO - Minor version ID (may be used if different).
 * [31:24] VHI - Major version ID (must match to be compatible).
 */
#define SAIL_VERS_HIGH_GET(ver) (((ver) >> 24) & 0xFF)
#define SAIL_VERS_LOW_GET(ver) (((ver) >> 16) & 0xFF)
#define SAIL_VERS_APP_GET(ver) ((ver)&0xFFFF)
#define SAIL_VERS_HI 0x02u
#define SAIL_VERS_LO 0x00u
#define SAIL_VERSION(app) ((SAIL_VERS_HI << 24) | (SAIL_VERS_LO << 16) | (app))

/*
 * Streaming channel parameters
 * Describes parameters used to set up streaming channels
 *
 * [15:00] CNUM - Number of channels
 * [31:16] PQEC - Number of descriptor entries in each channel
 */
#define SAIL_STRM_CNUM(strm) ((strm)&0xFFFF)
#define SAIL_STRM_PQEC(strm) (((strm) >> 16) & 0xFFFF)
#define SAIL_STRM_DEFINE(pqec, cnum) (((pqec) << 16) | (cnum))

/*
 * Packet size configuration
 * Describes packet constraints
 *
 * [15:00] PACKET_SIZE_MAX - Max submission entries in a legacy packet
 * [31:16] STREAMING_PACKET_SIZE_MAX - Max submission entries in a streaming packet
 */
#define SAIL_PKT_SIZE_MAX(pkt) ((pkt)&0xFFFF)
#define SAIL_PKT_STREAMING_SIZE_MAX(pkt) (((pkt) >> 16) & 0xFFFF)
#define SAIL_PKT_SIZE_DEFINE(legacy, streaming)                                \
  ((((streaming)&0xFFFF) << 16) | ((legacy)&0xFFFF))

/*
 * Timeout configuration
 * Describes timeout parameters
 *
 * [15:00] ADMIN_COMMAND_TIMEOUT - Timeout time for admin commands
 */
#define SAIL_TIMEOUT_DEFINE(to) ((to)&0xFFFF)
#define SAIL_ADMIN_CMD_TIMEOUT(to) ((to)&0xFFFF)

/*
 * SAIL BAR Index Register
 * SAIL_SQAL/SQAH can be used to determine where SQ is allocated on BAR.
 *
 * [2:0] - Bar Number.
 * [63:3] - Offset.
 */
#define BIR_TO_BAR(bir) (bir & 0x7)
#define BIR_TO_OFFSET(bir) (bir & ~0x7)
#define MAKE_BIR(bar, offset) (bar | offset)

/*
 * SAIL Control and Status
 * Host requests a status change by settinig control bit(s).
 * Device acknowledges state change by setting status bit(s).
 *
 * [00:00] ACTIVE - Active mode req/status.
 */
#define SAIL_ACTIVE_BIT (1 << 0)

// State of the FBA Driver.
typedef enum {
  // Device is not initialized or failed
  FBA_DOWN,
  //Device out of reset, ROM is running and PCIe detected by host
  FBA_PCIE_UP,
  //Load, Boot Boot 1 image on device
  FBA_BOOT1,
  //Boot1 is running on device and DDR interface is up
  FBA_DRAM_UP,
  //Boot1 is running on device and MBIST test is running
  FBA_MBIST,
  //Load application FW CCP and/or SCP
  FBA_LOAD_APP,
  //Boot Application FW on CCP and/or SCP
  FBA_BOOT_APP,
  //Application FW is running on CCP and/or SCP
  FBA_RUNNING,
  //Sail connection is up
  FBA_ACTIVE,
  // fba device failed state
  FBA_FAILED,
  // Invalid maximum state
  FBA_STATE_COUNT,
} fba_state_t;

// Information word identifiers. These device information words can be read with
// the ADMIN_GET_INFO_WORDS command.
enum info_word {
  INFO_WORD_FIRST = 0,
  INFO_WORD_MAX_COUNT = INFO_WORD_FIRST,

  // Max concurrent contexts that are supported at once.
  INFO_CONTEXT_MAX_COUNT,

  // Count of resources that have been exposed by the target device for use
  // by the host.
  INFO_RESOURCE_MAX_COUNT,

  // Maximum DMA segment size
  INFO_DMA_MAX_SEGMENT_SIZE,

  // uint16_t count of monitoring items.
  INFO_MON_ITEM_COUNT,

  // uint16_t count of monitoring groups.
  INFO_MON_GROUP_COUNT,

  // Device platform identifier to differentiate between silicon and non-silicon
  // devices.
  INFO_PLATFORM_TYPE,

  // Number of fixed words, start of application specific words.
  INFO_WORD_APP_START,
};

#define ADMIN_OPCODE_FIRST 0x100

// Possible values for the opcode field on the admin submission queue (sqid=0).
enum admin_opcode {
  // Do nothing / ping.
  ADMIN_NOOP = ADMIN_OPCODE_FIRST,

  // Create device-side context.
  ADMIN_CREATE_CONTEXT,

  // Delete device-side context begin.
  // store the context-id in a delete_inprogress_ctxids list
  // and don't execute callbacks for contexts that are in this list
  ADMIN_DELETE_CONTEXT_BEGIN,

  // Delete device-side context end.
  // remove the context-id from the delete_inprogress_ctxids
  ADMIN_DELETE_CONTEXT_END,

  // Get one or more device info-words.
  ADMIN_GET_INFO_WORDS,

  // Return the requested resource information.
  ADMIN_GET_RESOURCES,

  // Add an ASYNC message to the alert service.
  ADMIN_ASYNC_ALERT,

  // Add an ASYNC message to the log service.
  ADMIN_ASYNC_LOG,

  // Read monitoring item definitions.
  ADMIN_MON_READ_ITEMS,

  // Read monitoring group definitions.
  ADMIN_MON_READ_GROUPS,

  // Read monitoring data.
  ADMIN_MON_READ_DATA,

  // Write monitoring data.
  ADMIN_MON_WRITE_DATA,

  // Set device "wall clock"
  ADMIN_TIME_SET,

  // Not a real opcode - one past the end
  ADMIN_OPCODE_END,
};

#define EXE_OPCODE_FIRST 0x200

STATIC_ASSERT(ADMIN_OPCODE_END <= EXE_OPCODE_FIRST);

// Possible values for the opcode field in in execution submission queues.
enum exe_opcode {
  // Do nothing / ping and return success or failure.
  EXE_NOOP = EXE_OPCODE_FIRST,

  // Read device log statistics
  EXE_LOG_STATS,

  // Trigger logs to be generated
  EXE_LOG_TEST,

  // Read device logs
  EXE_LOG_READ,

  // Get constant log module data
  EXE_LOG_INFO,

  // Get log level for module(s)
  EXE_LOG_LEVEL_GET,

  // Set log level for module(s)
  EXE_LOG_LEVEL_SET,

  // Configure the perf counter(s).
  EXE_PERF_CONFIG,

  // Read sample buffer
  EXE_PERF_READ_BUF,

  // Start the counter(s).
  EXE_PERF_START,

  // Read the counter(s).
  EXE_PERF_READ,

  // Stop the counter(s).
  EXE_PERF_STOP,

  // Free configured data.
  EXE_PERF_DELETE,

  // Collect profiling stats
  EXE_PROFILE_COLLECT,

  // Flush profiling stats
  EXE_PROFILE_FLUSH,

  // Get parameters required for streaming.
  EXE_PROFILE_PARAMS,

  // Get CRC value for a buff
  EXE_GET_BUFF_CRC,

  // Crash CCP core.
  EXE_CRASH_CCP,

  // DMA transfer to/from host (xfer dir and PRP determined by param)
  EXE_DMA_HOST,

  // Add a try/catch/finally block.
  EXE_TRY,

  // Return mask of started entries.
  EXE_STARTED,

  // Associates stream_id with a streaming device
  EXE_STRM_INIT,

  // Facilitate streaming event-record by sending timestamp back to host
  EXE_STRM_EVENT_RECORD,

  // Blocks current task until the specified event is done.
  EXE_STRM_EVENT_WAIT,

  // Unblocks tasks waiting on external events with status specified by
  // the update
  EXE_STRM_EVENT_UPDATE,

  // Simulate a hung command that never completres.
  EXE_INJECT_HANG,

  // Not a real opcode -- one past the end
  EXE_OPCODE_END,
};

// Start of application specific opcodes
#define EXE_APP_OPCODE 0x300

STATIC_ASSERT(EXE_OPCODE_END <= EXE_APP_OPCODE);
// Status codes in completion entries.
enum completion_status {
  CS_OK = 0,
  CS_BAD_OPCODE, // Unknown opcode.
  CS_INVAL_QID, // Invalid queue ID.
  CS_INVAL_ALIGN, // Invalid address alignment.
  CS_INVAL_QSIZE, // Invalid queue size.
  CS_INVAL_FUNC, // Invalid callback function.
  CS_FAILED, // Command failed for an unspecified reason.
  CS_TIMEOUT, // Operation timed out.
  CS_QUEUE_EXISTS, // Queue ID already in use.
  CS_QUEUE_MISSING, // No such queue exists.
  CS_STRM_FAILED, // Stream is in failed state.
  CS_STRM_INIT, // Stream is not initialized.
  CS_STRM_PKT_ID, // Host and device pkt_id out of sync.
  CS_NOMEM, // Out of memory.
  CS_BUSY, // Resource is busy.
  CS_NO_ADMINQ, // Operation not allowed for admin queue.
  CS_INVAL_IRQV, // Invalid interrupt vector.
  CS_DMA_ACCESS, // DMA access permission denied.
  CS_BAD_INDEX, // Starting index is out of range.
  CS_BAD_RANGE, // Ending index is out of range.
  CS_BAD_ARG, // Bad argument.
  CS_EIO, // I/O error.
  CS_EMPTY, // No entries available to read.
  CS_INUSE, // ID already in use.
  CS_ENOSPC, // No space left.
  CS_BAD_INFO_WORD,
  CS_CANCEL, // Command was cancelled.
  CS_BAD_FLAGS, // Command had incorrect flags
  CS_DMA_BAD_OPCODE, // DMA Unknown opcode
  CS_QUEUE_BUSY, // Queue is busy
  CS_DMA_BAD_DIRECTION_ARG, // Bad direction arg in DMA
  CS_CTX_DELETE, // Context is being deleted
  CS_INALID_CTX, // context-id is out of range
  CS_ABORT, // command aborted due to hard (non-recoverable) failure
  // Start of application specific status codes
  CS_APP_STATUS = 0x100
};

enum resource_type {
  RT_UNKNOWN = 0,
  // Memory resources (could be DRAM, SRAM etc.)
  RT_MEMORY,
  // Compute core resources
  RT_COMPUTE,
  // Synchronization primitive resources
  RT_SYNC,
};

enum allocator_type {
  AT_UNKNOWN = 0,
  AT_BLOCK, // Standard malloc/free
  AT_BITMAP, // Bitmap pattern.
};

enum mem_type {
  MT_UNKNOWN = 0,
  MT_DRAM = 1,
  MT_SRAM = 2,
  MT_SCRATCH = 3,
  MT_P2P_DRAM = 4,
  MT_ECC_DRAM = 5,
  MT_P2P_ECC_DRAM = 6,
};

enum sync_resource_type {
  SRT_UNKNOWN = 0,
  // Contexts created for management and tracking of every unique
  // SAIL device on the host.
  SRT_CONTEXT,
  // Local synchronization resources for Processing Elements
  // e.g. column, row, generic etc.
  SRT_LOCAL_BARRIER,
  // Peer to Peer sychronization resources for Processing
  // Elements.
  SRT_P2P_BARRIER,
  // Start of application defined sync resource.
  SRT_APP_DEFINED,
};

enum compute_type {
  CT_UNKNOWN = 0,
  CT_CORE,
};
typedef uint16_t rtype_t;
typedef uint8_t rsub_t;
typedef uint8_t ralloc_t;

struct resource_def {
  // Type of resources such as as memory, computer etc.
  uint16_t type;
  // Subtype of resources as SRAM, DRAM in memory etc.
  uint16_t subtype;
  // Mainly to ensure 8 byte alignment, can be used for misc. purposes.
  uint16_t alloc;
  // Mainly to ensure 8 byte alignment, can be used for misc. purposes.
  uint16_t misc;
  // Start address of the resource.
  uint64_t start;
  // Size of the resource available for use.
  uint64_t count;
  // Granularity of resources e.g. for memory this could imply alignment.
  uint64_t gran;
};
// Ensure struct is 8-byte aligned.
STATIC_ASSERT((sizeof(struct resource_def) % sizeof(uint64_t)) == 0);

#define DEF_RESOURCE(ty, st, at, s, c, g)                                      \
  {                                                                      \
    .type = ty, .subtype = st, .alloc = at, .misc = 0, .start = s, \
    .count = c, .gran = g                                          \
  }

#define DEF_RAM_BLK_RESOURCE(sub, dat, siz)                                    \
  DEF_RESOURCE(RT_MEMORY, sub, AT_BLOCK, (uint64_t)dat, siz, 512)

#define DEF_SYNC_BLK_RESOURCE(sub, start, count)                               \
  DEF_RESOURCE(RT_SYNC, sub, AT_BLOCK, start, count, 1)

#define DEF_COMP_BLK_RESOURCE(sub, start, count)                               \
  DEF_RESOURCE(RT_COMPUTE, sub, AT_BLOCK, start, count, 1)

#define DEF_COMP_BIT_RESOURCE(sub, start, count)                               \
  DEF_RESOURCE(RT_COMPUTE, sub, AT_BITMAP, start, count, 1)

static inline const char *fba_admin_opcode2str(enum admin_opcode opcode)
{
#define CASE(x)                                                                \
  case x:                                                                \
    return #x
  switch (opcode) {
    CASE(ADMIN_NOOP);
    CASE(ADMIN_CREATE_CONTEXT);
    CASE(ADMIN_DELETE_CONTEXT_BEGIN);
    CASE(ADMIN_DELETE_CONTEXT_END);
    CASE(ADMIN_GET_INFO_WORDS);
    CASE(ADMIN_GET_RESOURCES);
    CASE(ADMIN_ASYNC_ALERT);
    CASE(ADMIN_ASYNC_LOG);
    CASE(ADMIN_MON_READ_ITEMS);
    CASE(ADMIN_MON_READ_GROUPS);
    CASE(ADMIN_MON_READ_DATA);
    CASE(ADMIN_MON_WRITE_DATA);
    CASE(ADMIN_TIME_SET);
  // Not a real opcode
  case ADMIN_OPCODE_END:
    return NULL;
  }
#undef CASE
  return NULL;
}

static inline const char *fba_exe_opcode2str(enum exe_opcode opcode)
{
#define CASE(x)                                                                \
  case x:                                                                \
    return #x
  switch (opcode) {
    CASE(EXE_NOOP);
    CASE(EXE_PERF_CONFIG);
    CASE(EXE_PERF_READ_BUF);
    CASE(EXE_PERF_START);
    CASE(EXE_PERF_READ);
    CASE(EXE_PERF_STOP);
    CASE(EXE_PERF_DELETE);
    CASE(EXE_PROFILE_COLLECT);
    CASE(EXE_PROFILE_FLUSH);
    CASE(EXE_PROFILE_PARAMS);
    CASE(EXE_GET_BUFF_CRC);
    CASE(EXE_CRASH_CCP);
    CASE(EXE_LOG_STATS);
    CASE(EXE_LOG_TEST);
    CASE(EXE_LOG_READ);
    CASE(EXE_LOG_INFO);
    CASE(EXE_LOG_LEVEL_GET);
    CASE(EXE_LOG_LEVEL_SET);
    CASE(EXE_DMA_HOST);
    CASE(EXE_TRY);
    CASE(EXE_STARTED);
    CASE(EXE_STRM_INIT);
    CASE(EXE_STRM_EVENT_RECORD);
    CASE(EXE_STRM_EVENT_WAIT);
    CASE(EXE_STRM_EVENT_UPDATE);
    CASE(EXE_INJECT_HANG);
  // These aren't real opcodes
  case EXE_OPCODE_END:
    return NULL;
  }
#undef CASE
  return NULL;
}
static inline const char *fba_opcode2str(uint32_t opcode)
{
  if (opcode >= EXE_OPCODE_FIRST && opcode < EXE_OPCODE_END)
    return fba_exe_opcode2str((enum exe_opcode)opcode);
  if (opcode >= ADMIN_OPCODE_FIRST && opcode < ADMIN_OPCODE_END)
    return fba_admin_opcode2str((enum admin_opcode)opcode);
  return NULL;
}

static inline const char *completion_status2str(uint16_t cs_status)
{
  switch (cs_status) {
  case CS_OK:
    return "Completion Status OK.";
  case CS_BAD_OPCODE:
    return "Unknown opcode.";
  case CS_INVAL_QID:
    return "Invalid queue ID.";
  case CS_INVAL_ALIGN:
    return "Invalid address alignment.";
  case CS_INVAL_QSIZE:
    return "Invalid queue size.";
  case CS_INVAL_FUNC:
    return "Invalid callback function.";
  case CS_FAILED:
    return "Command failed for an unspecified reason.";
  case CS_TIMEOUT:
    return "Operation timed out.";
  case CS_QUEUE_EXISTS:
    return "Queue ID already in use.";
  case CS_QUEUE_MISSING:
    return "No such queue exists.";
  case CS_STRM_FAILED:
    return "Queue is in an invalid state.";
  case CS_STRM_INIT:
    return "Stream is not initialized";
  case CS_STRM_PKT_ID:
    return "Host and device pkt_id out of sync.";
  case CS_NOMEM:
    return "Out of memory.";
  case CS_BUSY:
    return "Resource is busy.";
  case CS_NO_ADMINQ:
    return "Operation not allowed for admin queue.";
  case CS_INVAL_IRQV:
    return "Invalid interrupt vector.";
  case CS_DMA_ACCESS:
    return "DMA access permission denied.";
  case CS_BAD_INDEX:
    return "Starting index is out of range.";
  case CS_BAD_RANGE:
    return "Ending index is out of range.";
  case CS_BAD_ARG:
    return "Bad argument.";
  case CS_EIO:
    return "I/O error.";
  case CS_EMPTY:
    return "No entries available to read.";
  case CS_INUSE:
    return "ID already in use.";
  case CS_ENOSPC:
    return "No space left.";
  case CS_BAD_INFO_WORD:
    return "Info word is not correct";
  case CS_CANCEL:
    return "Command was cancelled.";
  case CS_BAD_FLAGS:
    return "Command had incorrect flags";
  case CS_DMA_BAD_OPCODE:
    return "DMA Unknown opcode";
  case CS_QUEUE_BUSY:
    return "Queue is busy";
  case CS_DMA_BAD_DIRECTION_ARG:
    return "Bad direction arg in DMA";
  case CS_CTX_DELETE:
    return "Context is being deleted";
  case CS_INALID_CTX:
    return "The context-id passed is out of range";
  case CS_ABORT:
    return "Command aborted due to hard (non-recoverable) failure";
  }
  return NULL;
}

#ifdef __cplusplus
}
#endif
