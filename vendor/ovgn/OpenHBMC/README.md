HyperRAM controller from [https://github.com/OVGN/OpenHBMC](https://github.com/OVGN/OpenHBMC).

In its native form it is packed as an IP core. Here we use the [generated
source files](hdl) from [this](data/OpenHBMC.xci) core configuration.

## Modifications:
The following source files have been modified:
- [hdl/hbmc\_ufifo.v](hdl/hbmc\_ufifo.v): use `prim_fifo_async.sv` instead of Xilinx FIFO
- [hdl/hbmc\_dfifo.v](hdl/hbmc\_dfifo.v): use `prim_fifo_async.sv` instead of Xilinx FIFO

