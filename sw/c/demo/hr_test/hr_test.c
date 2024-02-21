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

//#define STOP_ADDRESS 10000
#define STOP_ADDRESS 64
#define REPEATS 10000

uint32_t i, j, errors, rdata;


int main(void) {
  puts("\n\n\nhr_test started\n");

  timer_init();

  /*
  puts("W\n");
  DEV_WRITE(HR_BASE, 0x12345678u);
  DEV_WRITE(HR_BASE+4, 0x5a5a5a5au);
  DEV_WRITE(HR_BASE+8, 0x12345678u);

  puts("R\n");
  puthex(DEV_READ(HR_BASE));
  puts("\n");
  puthex(DEV_READ(HR_BASE+4));
  puts("\n");
  puthex(DEV_READ(HR_BASE+8));
  */

  errors = 0;

  for (i = 0; i < REPEATS; i = i + 1) {
      puts(".");
      for (j = 0; j < STOP_ADDRESS; j = j + 4) {
          DEV_WRITE(HR_BASE+j, (i^j));
      }
      puts(",");
      for (j = 0; j < STOP_ADDRESS; j = j + 4) {
          rdata = DEV_READ(HR_BASE+j);
          if (rdata != (i^j)) {
              errors = errors + 1;
              puts("Expected ");
              puthex(i^j);
              puts(", got ");
              puthex(rdata);
              puts("\n");
          }
      }
  }
  puts("\nErrors: ");
  puthex(errors);
  puts("\n");

  uint64_t end_time = timer_read() + 10 * 50 * 1000 * 1000;
  while (timer_read() < end_time);

  return 0;
}
