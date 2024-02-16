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

#define REG_LB_MANUAL         0x10u
#define REG_HYPER_STATUS      0x14u
#define REG_LB_ERRORS         0x18u
#define REG_LB_ERROR_ADDR     0x1cu
#define REG_LB_ITERATIONS     0x20u
#define REG_LB_CURRENT_ADDR   0x24u
#define REG_HBMC_SINGLE_DATA  0x28u
#define REG_HBMC_ACTION       0x2cu
#define REG_LB_STOP_ADDR      0x30u
#define REG_LB_START_ADDR     0x34u

int iteration;
int prev_iteration;
int status;

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
  puts("\n\n\nhr_test started\n");

  timer_init();
  DEV_WRITE(0x80005040, 0x12345678u);

  //while (1) {
    // Dump out the contents of the HR registers
  /*
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


    puts("\n=== HR STATUS ===\n");
    puthex(DEV_READ(HR_BASE + REG_HYPER_STATUS));
    puts(", ");
    puthex(DEV_READ(HR_BASE + REG_HBMC_ACTION));
    puts("\n");

    puts("\n=== HR WRITES ===\n");
    DEV_WRITE(HR_BASE + REG_LB_MANUAL, 0x00000003u);
    for (uint32_t hraddr = 0; hraddr < 16; hraddr += 4u) {
    //for (uint32_t hraddr = 0; hraddr < 4; hraddr += 4u) {
        puts("writing to: ");
        puthex(hraddr);
        puts("\n");
        DEV_WRITE(HR_BASE + REG_LB_START_ADDR, hraddr);
        //DEV_WRITE(HR_BASE + REG_HBMC_SINGLE_DATA, hraddr);
        DEV_WRITE(HR_BASE + REG_HBMC_SINGLE_DATA, 0xffffffffu ^ hraddr);
        DEV_WRITE(HR_BASE + REG_HBMC_ACTION, 0x00000001u);
        DEV_WRITE(HR_BASE + REG_HBMC_ACTION, 0x00000000u);
        // kill some time:
        //dump_regs(HR_BASE);
        //uint64_t end_time = timer_read() + 10 * 1000 * 1000;
        //while (timer_read() < end_time);
    }


    puts("\n=== HR READS ===\n");
    for (uint32_t hraddr = 0; hraddr < 16; hraddr += 4u) {
    //for (uint32_t hraddr = 0; hraddr < 4; hraddr += 4u) {
        DEV_WRITE(HR_BASE + REG_LB_START_ADDR, hraddr);
        DEV_WRITE(HR_BASE + REG_HBMC_ACTION, 0x00000002u);
        DEV_WRITE(HR_BASE + REG_HBMC_ACTION, 0x00000000u);
        puts("reading from: ");
        puthex(hraddr);
        puts(" :");
        uint64_t end_time = timer_read() + 10 * 1000 * 1000;
        while (timer_read() < end_time);
        puthex(DEV_READ(HR_BASE + REG_HBMC_SINGLE_DATA));
        puts("\n");
    }

    puts("\n=== HR STATUS ===\n");
    puthex(DEV_READ(HR_BASE + REG_HYPER_STATUS));
    puts(", ");
    puthex(DEV_READ(HR_BASE + REG_HBMC_ACTION));
    puts("\n");


    // Hang around for a couple of seconds
    uint64_t end_time = timer_read() + 5 * 50 * 1000 * 1000;
    while (timer_read() < end_time);

    */

    // turn on auto test
    //
    DEV_WRITE(HR_BASE + REG_LB_MANUAL, 0x00000003u); // clear fail flag
    DEV_WRITE(HR_BASE + REG_LB_START_ADDR, 0x00000000u);
    //DEV_WRITE(HR_BASE + REG_LB_STOP_ADDR, 0x00080000u);
    DEV_WRITE(HR_BASE + REG_LB_STOP_ADDR, 8*1024*1024-4);
    DEV_WRITE(HR_BASE + REG_LB_MANUAL, 0x00000004u);
    //

    prev_iteration = 0;
    while(1) {
        status = (DEV_READ(HR_BASE + REG_HYPER_STATUS));
        if (status & 4) {
            puts("ERROR: R/W test failed!\n");
            puthex(status);
            break;
        }
        iteration = (DEV_READ(HR_BASE + REG_LB_ITERATIONS));
        if (iteration > prev_iteration) {
            prev_iteration = iteration;
            puts("Iteration ");
            puthex(iteration);
            puts(" complete.\n");
            status = (DEV_READ(HR_BASE + REG_HYPER_STATUS));
            if ((status & 8) == 0) {
                puts("ERROR: Pass flag not set!\n");
                puthex(status);
                break;
            }
        }
        uint64_t end_time = timer_read() + 1 * 50 * 1000 * 1000;
        while (timer_read() < end_time);
    }

  //}

  return 0;
}
