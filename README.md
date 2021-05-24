# LiteX for ECP5 evaluation board + SDRAM and SGMII PHY addons

Files to support litex on Lattice LEF5UM5G-85G board with [SDRAM addon](https://github.com/kazkojima/ecp5evn-sdram-addon) and [SGMII PHY](https://github.com/kazkojima/dp83867s-sgmii-board).

##  litex-boards patch

This repo is structured as below.

```
.
|
├── linux-on-litex-vexriscv
│   └── ecp5_evn-sdram-sgmii.patch
├── liteeth
│   └── ecp5sgmii.py
└── litex-boards
    └── lattice_ecp5_evn-sdram-sgmii.patch
```

## litex-board patch

litex-boards/lattice_ecp5_evn-sdram-sgmii.patch is a patch which adds the supports for SDRAM addon and SGMII PHY to the lattice_ecp5_evn platform.

## addend to liteeth

liteeth/ecp5sgmii.py gives a support for SGMII PHY. Tested on my home-brewed board with TI DP83867S PHY chip. TI offers their evaluation board DP83867ERGZ-S-EVM for that chip.

ecp5sgmii.py is simply
(liteeth pcs_1000basex.py + liteiclink serdes_ecp5.py)
adjusted for "GBE" settings of DCU instead of "10BSER" ones.

## linux-on-litex-vexriscv

linux-on-litex-vexriscv/ecp5_evn-sdram-sgmii.patch is a patch to add this kit as the [linux-on-litex-vexriscv](https://github.com/litex-hub/linux-on-litex-vexriscv) target. With this patch,

```
./make.py --board ecp5_evn --cpu-count 4 --local-ip 10.253.253.95 --remote-ip 10.253.253.8 --build --load
```

will gives a linux capable RV32 SoC with 4 CPUs.
--

## Images

![hardwares](https://github.com/kazkojima/litex-lattice-ecp5-evn/blob/main/images/ecp5evn-addons.jpg)

![Screenshot of linux-on-litex-vexriscv console](https://github.com/kazkojima/litex-lattice-ecp5-evn/blob/main/images/console-smp.png)

