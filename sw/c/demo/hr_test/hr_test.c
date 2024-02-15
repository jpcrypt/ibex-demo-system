// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "demo_system.h"
#include "dev_access.h"
#include "timer.h"

// Note: we've hijacked the initial port of USBDEV and introduced the I2C device instead
#define HR_BASE USBDEV_BASE

// A few of the HR registers that conveniently work as 32-bit R/W storage
#define REG0 0x00u
#define REG1 0x04u
#define REG2 0x08u

static void dump_regs() {
  puts("=== HR registers ===\n");
  //for (unsigned offset = 0u; offset < 0x58; offset += 4u) {
  for (unsigned offset = REG0; offset <= REG2; offset += 4u) {
    puthex(offset);
    puts(" : ");
    puthex(DEV_READ(HR_BASE + offset));
    puts("\n");
  }
  puts("=====\n");
}

int main(void) {
  puts("hr_test started\n");

  timer_init();
  DEV_WRITE(0x80005040, 0x12345678u);

  while (1) {
    // Dump out the contents of the HR registers
    dump_regs(HR_BASE);

    DEV_WRITE(HR_BASE + REG0, 0x12345678u);
    DEV_WRITE(HR_BASE + REG1, 0x87654321u);
    DEV_WRITE(HR_BASE + REG2, 0x01010101u);

    // Dump out the contents of the HR registers
    dump_regs(HR_BASE);

    DEV_WRITE(HR_BASE + REG0, 0xffffffffu);
    DEV_WRITE(HR_BASE + REG1, 0xeeeeeeeeu);
    DEV_WRITE(HR_BASE + REG2, 0xddddddddu);

    // Dump out the contents of the HR registers
    dump_regs(HR_BASE);

    // Hang around for a couple of seconds
    uint64_t end_time = timer_read() + 5 * 50 * 1000 * 1000;
    while (timer_read() < end_time);
  }
  //

  return 0;
}
