// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "demo_system.h"
#include "dev_access.h"
#include "timer.h"

// Note: we've hijacked the initial port of USBDEV and introduced the I2C device instead
#define I2C0_BASE USBDEV_BASE

// A few of the I2C registers that conveniently work as 32-bit R/W storage
#define TIMING0 0x30u
#define TIMING1 0x34u
#define TIMING2 0x38u
#define TIMING3 0x3cu
#define TIMING4 0x40u

static void dump_regs() {
  puts("=== I2C registers ===\n");
  for (unsigned offset = 0u; offset < 0x58; offset += 4u) {
    puthex(offset);
    puts(" : ");
    puthex(DEV_READ(I2C0_BASE + offset));
    puts("\n");
  }
  puts("=====\n");
}

int main(void) {
  puts("i2c_test started\n");

  timer_init();

  while (1) {
    // Dump out the contents of the I2C registers
    dump_regs(I2C0_BASE);

    DEV_WRITE(I2C0_BASE + TIMING0, 0x12345678u);
    DEV_WRITE(I2C0_BASE + TIMING1, 0x87654321u);
    DEV_WRITE(I2C0_BASE + TIMING2, 0x01010101u);
    DEV_WRITE(I2C0_BASE + TIMING3, 0xA5A5A5A5u);
    DEV_WRITE(I2C0_BASE + TIMING4, 0x10101010u);

    // Dump out the contents of the I2C registers
    dump_regs(I2C0_BASE);

    DEV_WRITE(I2C0_BASE + TIMING0, 0xffffffffu);
    DEV_WRITE(I2C0_BASE + TIMING1, 0xeeeeeeeeu);
    DEV_WRITE(I2C0_BASE + TIMING2, 0xddddddddu);
    DEV_WRITE(I2C0_BASE + TIMING3, 0xccccccccu);
    DEV_WRITE(I2C0_BASE + TIMING4, 0xbbbbbbbbu);

    // Dump out the contents of the I2C registers
    dump_regs(I2C0_BASE);

    // Hang around for a couple of seconds
    uint64_t end_time = timer_read() + 2 * 50 * 1000 * 1000;
    while (timer_read() < end_time);
  }

  return 0;
}
