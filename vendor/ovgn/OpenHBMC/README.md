HyperRAM controller from [https://github.com/OVGN/OpenHBMC](https://github.com/OVGN/OpenHBMC).

In its native form it is packed as an IP core. Here we use the [generated
source files](hdl) from [this](data/OpenHBMC.xci) core configuration, with some
modifications as noted below.

## Modifications:
[hdl/hbmc\_ufifo.v](hdl/hbmc\_ufifo.v) and
[hdl/hbmc\_dfifo.v](hdl/hbmc\_dfifo.v) have been modified to use
`prim_fifo_async.sv` (when `PRIM_DEFAULT_IMPL=prim_pkg::ImplGeneric`) or
`xpm_fifo_async` (when `PRIM_DEFAULT_IMPL=prim_pkg::ImplXilinx`).

Note that when implementing for the Sonata board FPGA, `prim_fifo_async.sv`,
although functional, has several timing violations; `xpm_fifo_async`, on the
other hand, has a clean implementation without any timing violations.
