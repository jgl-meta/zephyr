#pragma once

//
// Common types used by fba_core code.
//

#ifdef __cplusplus
extern "C" {
#endif

// An address in the device address space.
typedef uint64_t device_addr_t;

// An address on the PCIe bus.
typedef uint64_t pci_addr_t;

// Index into a circular host submission buffer.
// These are limited to 64K entries.
typedef uint16_t cb_index_t;

// Index into the SQ and CQ.
typedef uint8_t queueid_t;

// Check if circular buffer indices are in order: a <= b <= c, accounting for
// wraparound.
static inline bool cb_ordered(cb_index_t a, cb_index_t b, cb_index_t c)
{
  if (a <= c) {
    // [ a--b--c ]
    return a <= b && b <= c;
  } else {
    // [-c  a--b-] or [-b--c  a-]
    return a <= b || b <= c;
  }
}

#ifdef __cplusplus
}
#endif
