#
# This file is part of LiteEth.
#
# Copyright (c) 2019-2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# SGMII PHY for ECP5 Lattice FPGA

from math import ceil

from migen import *
from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.cores.clock.lattice_ecp5  import ECP5PLL

from liteeth.common import *
from liteeth.phy.common import *

def K(x, y):
    return (y << 5) | x

def D(x, y):
    return (y << 5) | x

class SGMIITX(Module):

    def __init__(self):
        self.config_stb = Signal()
        self.config_reg = Signal(16)

        self.sink   = sink   = stream.Endpoint(eth_phy_description(8))
        self.source = source = stream.Endpoint([("data", 8),
                                                ("ctrl", 1),
                                                ("disp", 1)])
        #self.comb += sink.ready.eq(1)
        self.comb += source.valid.eq(1)

        parity = Signal()
        c_type = Signal()
        self.sync += parity.eq(~parity)

        config_reg_buffer = Signal(16)
        load_config_reg_buffer = Signal()
        self.sync += If(load_config_reg_buffer, config_reg_buffer.eq(self.config_reg))

        self.submodules.fsm = fsm = FSM(reset_state="START")
        fsm.act("START",
                sink.ready.eq(1),
                If(self.config_stb,
                   load_config_reg_buffer.eq(1),
                   source.ctrl.eq(1),
                   source.data.eq(K(28,5)),
                   NextState("CONFIG_D")
                ).Else(
                    If(sink.valid,
                       # /S/ replaces the 1st octet of preamble.
                       source.data.eq(K(27,7)),
                       source.ctrl.eq(1),
                       NextState("DATA")
                    ).Else(
                        source.data.eq(K(28,5)), # discard data
                        source.ctrl.eq(1),
                        NextState("IDLE")
                    )
                )
        )
        fsm.act("CONFIG_D",
                If(c_type,
                   source.data.eq(D(2,2))
                ).Else(
                   source.data.eq(D(21,5))
                ),
                NextValue(c_type, ~c_type),
                NextState("CONFIG_REG_LSB")
        )
        fsm.act("CONFIG_REG_LSB",
                source.data.eq(config_reg_buffer[:8]),
                NextState("CONFIG_REG_MSB")
        )
        fsm.act("CONFIG_REG_MSB",
                source.data.eq(config_reg_buffer[8:]),
                NextState("START")
        )
        fsm.act("IDLE",
                # PCS may convert D16.2 to a D5.6 for I2 to flip disparity
                # Fix it with tx_correct_disp_ch.
                source.disp.eq(1),
                self.source.data.eq(D(16,2)),
                NextState("START")
        )
        self.comb += source.data.eq(sink.data)
        fsm.act("DATA",
                sink.connect(source, omit={"data", "last_be", "error"}),
                source.last.eq(0), #?
                If(sink.valid & sink.last,
                   NextState("CEOP")
                )
        )
        fsm.act("CEOP",
                source.data.eq(K(29,7)), # /T/
                source.ctrl.eq(1),
                NextState("CEXT1")
        )
        fsm.act("CEXT1",
                source.data.eq(K(23,7)), # /R/
                source.ctrl.eq(1),
                If(parity,
                   NextState("START"),
                ).Else(
                    NextState("CEXT2"),
                )
        )
        fsm.act("CEXT2",
                source.data.eq(K(23,7)), # /R/
                source.ctrl.eq(1),
                NextState("START")
        )


class SGMIIRX(Module):

    def __init__(self):
        self.seen_valid_ci = Signal()
        self.seen_config_reg = Signal()
        self.config_reg = Signal(16)

        self.sink   = sink   = stream.Endpoint([("data", 8), ("ctrl", 1)])
        self.source = source = stream.Endpoint(eth_phy_description(8))
        self.comb += sink.ready.eq(1)

        self.data_reg = data_reg = Signal(8)
        self.data_valid = data_valid = Signal()
        self.sop = sop = Signal()
        self.soe = soe = Signal()

        config_reg_lsb = Signal(8)
        load_config_reg_lsb = Signal()
        load_config_reg_msb = Signal()
        self.sync += [
            self.seen_config_reg.eq(0),
            If(load_config_reg_lsb,
                config_reg_lsb.eq(sink.data)
            ),
            If(load_config_reg_msb,
                self.config_reg.eq(Cat(config_reg_lsb, sink.data)),
                self.seen_config_reg.eq(1)
            )
        ]

        self.submodules.fsm = fsm = FSM(reset_state="START")
        fsm.act("START",
                If(sink.ctrl,
                   If(sink.data == K(28,5),
                      NextState("K28_5")
                   ),
                   If(sink.data == K(27,7), # /S/
                      data_reg.eq(0x55),
                      data_valid.eq(1),
                      sop.eq(1),
                      NextState("DATA")
                   )
                )
        )
        fsm.act("K28_5",
            NextState("START"),
            If(~sink.ctrl,
               If((sink.data == D(21,5)) | (sink.data == D(2,2)),
                  self.seen_valid_ci.eq(1),
                  NextState("CONFIG_REG_LSB")
               ),
               If((sink.data == D(5,6)) | (sink.data == D(16,2)),
                  # idle
                  self.seen_valid_ci.eq(1),
                  NextState("START")
               ),
            )
        )
        fsm.act("CONFIG_REG_LSB",
                If(sink.ctrl,
                   If(sink.data == K(27,7),
                      data_reg.eq(0x55),
                      data_valid.eq(1),
                      sop.eq(1),
                      NextState("DATA")
                   ).Else(
                       NextState("START")  # error
                   )
                ).Else(
                    load_config_reg_lsb.eq(1),
                    NextState("CONFIG_REG_MSB")
                )
        )
        fsm.act("CONFIG_REG_MSB",
                If(~sink.ctrl,
                   load_config_reg_msb.eq(1)
                ),
                NextState("START")
        )
        fsm.act("DATA",
                If(sink.ctrl,
                   If(sink.data == K(29,7), # /T/
                      soe.eq(1),
                      NextState("START")
                   ),
                   If(sink.data == K(14,7), # coding violation converted to 0xee
                      NextState("START")
                   )
                ).Else(
                    data_reg.eq(sink.data),
                    data_valid.eq(1)
                )
        )

        self.sync += [
            source.valid.eq(data_valid),
            source.data.eq(data_reg)
        ]
        self.comb += source.last.eq(soe & source.valid)


# SerdesInit ---------------------------------------------------------------------------------------
# From liteiclink.serdes.serdes_ecp5
class SGMIIInit(Module):
    def __init__(self, tx_lol, rx_lol, rx_los):
        self.tx_rst  = Signal()
        self.rx_rst  = Signal()
        self.pcs_rst = Signal()
        self.ready   = Signal()

        # # #

        self.tx_lol = _tx_lol = Signal()
        self.rx_lol = _rx_lol = Signal()
        self.rx_los = _rx_los = Signal()
        self.specials += MultiReg(tx_lol, _tx_lol)
        self.specials += MultiReg(rx_lol, _rx_lol)

        timer = WaitTimer(1024)
        self.submodules += timer
        timer.wait.reset = 1

        self.submodules.fsm = fsm = FSM(reset_state="RESET-ALL")
        fsm.act("RESET-ALL",
            # Reset TX Serdes, RX Serdes and PCS.
            self.tx_rst.eq(1),
            self.rx_rst.eq(1),
            self.pcs_rst.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("RESET-RX-PCS-WAIT-TX-PLL-LOCK")
            )
        )
        fsm.act("RESET-RX-PCS-WAIT-TX-PLL-LOCK",
            # Reset RX Serdes and PCS, wait for TX PLL lock.
            self.rx_rst.eq(1),
            self.pcs_rst.eq(1),
            If(timer.done & ~_tx_lol,
                timer.wait.eq(0),
                NextState("RESET-PCS-WAIT-RX-CDR-LOCK")
            )
        )
        fsm.act("RESET-PCS-WAIT-RX-CDR-LOCK",
            # Reset PCS, wait for RX CDR lock.
            self.pcs_rst.eq(1),
            If(timer.done & ~_rx_lol,
                timer.wait.eq(0),
                NextState("READY")
            )
        )
        fsm.act("READY",
            # Ready
            self.ready.eq(1),
            If(_tx_lol | _rx_lol,
                NextState("RESET-ALL")
            ),
            If(_rx_los,
                NextState("RESET-RX-PCS-WAIT-TX-PLL-LOCK")
            )
        )

# SGMII ECP5 ---------------------------------------------------------------------------------------

class SGMIIECP5(Module, AutoCSR):
    def __init__(self, tx_pads, rx_pads, ref_clk,
        dual        = 0,
        channel     = 0):
        assert dual       in [0, 1]
        assert channel    in [0, 1]
        self.dual    = dual
        self.channel = channel

        # TX controls
        self.tx_enable              = Signal(reset=1)
        self.tx_ready               = Signal()
        self.tx_idle                = Signal()

        # RX controls
        self.rx_enable              = Signal(reset=1)
        self.rx_ready               = Signal()
        self.rx_align               = Signal(reset=1)
        self.rx_idle                = Signal()
        self.rx_restart             = Signal()

        # DCU clock outputs
        self.txoutclk = Signal()
        self.rxoutclk = Signal()

        # Reference clock
        self.ref_clk = ref_clk

        # Internal signals -------------------------------------------------------------------------
        rx_los     = Signal()
        rx_lol     = Signal()
        rx_lsm     = Signal()
        rx_data    = Signal(8)
        rx_k       = Signal()
        rx_align   = Signal()
        rx_bus     = Signal(24)

        tx_lol     = Signal()
        tx_data    = Signal(8)
        tx_k       = Signal()
        tx_disp    = Signal()
        tx_bus     = Signal(24)

        # Control/Status CDC -----------------------------------------------------------------------
        self.specials += [
            MultiReg(self.rx_align, rx_align, "eth_rx"),
            MultiReg(rx_los, self.rx_idle, "sys"),
        ]

        # DCU init ---------------------------------------------------------------------------------
        self.submodules.init = init = SGMIIInit(tx_lol, rx_lol, rx_los)

        # Clocking ---------------------------------------------------------------------------------
        # txoutclk(tx_pclk) is connected to txi_clk.
        # rxoutclk(tx_pclk) is connected to rxi_clk and ebrd_clk.

        # TX ClockDomain
        self.clock_domains.cd_eth_tx = ClockDomain()
        self.comb += self.cd_eth_tx.clk.eq(self.txoutclk)
        self.specials += AsyncResetSynchronizer(self.cd_eth_tx, ~init.ready)
        self.comb += self.tx_ready.eq(init.ready)

        # RX ClockDomain
        self.clock_domains.cd_eth_rx = ClockDomain()
        self.comb += self.cd_eth_rx.clk.eq(self.rxoutclk)
        self.specials += AsyncResetSynchronizer(self.cd_eth_rx, ~init.ready)
        self.comb += self.rx_ready.eq(init.ready)

        # DCU instance -----------------------------------------------------------------------------
        self.serdes_params = dict(
            # ECP5's DCU parameters/signals/instance have been documented by whitequark as part of
            #             Yumewatari project: https://github.com/whitequark/Yumewatari
            #                  Copyright (C) 2018 whitequark@whitequark.org
            # DCU ----------------------------------------------------------------------------------
            # Input list
            # Power control
            i_D_FFC_MACROPDB        = 1,
            i_CHX_FFC_RXPWDNB       = 1,
            i_CHX_FFC_TXPWDNB       = 1,
            # Reset signals
            i_D_FFC_DUAL_RST        = ResetSignal("sys"),
            i_D_FFC_MACRO_RST       = ResetSignal("sys"),
            i_CHX_FFC_RRST          = ~self.rx_enable | init.rx_rst,
            i_CHX_FFC_LANE_RX_RST   = ~self.rx_enable | init.pcs_rst | self.rx_restart,
            i_D_FFC_TRST            = ~self.tx_enable | init.tx_rst,
            i_CHX_FFC_LANE_TX_RST   = ~self.tx_enable | init.pcs_rst,
            # Clocks
            i_D_REFCLKI             = self.ref_clk,
            i_CHX_RX_REFCLK         = self.ref_clk,
            i_CHX_FF_RXI_CLK        = ClockSignal("eth_rx"),
            i_CHX_FF_EBRD_CLK       = ClockSignal("eth_rx"),
            i_CHX_FF_TXI_CLK        = ClockSignal("eth_tx"),
            # HD input
            i_CHX_HDINP             = rx_pads.p,
            i_CHX_HDINN             = rx_pads.n,
            # Align
            i_CHX_FFC_SIGNAL_DETECT = rx_align,
            # TX data
            **{"i_CHX_FF_TX_D_%d" % n: tx_bus[n] for n in range(tx_bus.nbits)},

            # Output list
            # Loss of pll lock
            o_D_FFS_PLOL            = tx_lol,
            o_CHX_FFS_RLOL          = rx_lol,
            # Loss of RX signal
            o_CHX_FFS_RLOS          = rx_los,
            # LSM status
            o_CHX_FFS_LS_SYNC_STATUS= rx_lsm,
            # Clock outputs
            o_CHX_FF_RX_PCLK        = self.rxoutclk,
            o_CHX_FF_TX_PCLK        = self.txoutclk,
            #o_CHX_FF_TX_F_CLK       = tx_full_clk,
            # HD output
            o_CHX_HDOUTP            = tx_pads.p,
            o_CHX_HDOUTN            = tx_pads.n,
            # RX data
            **{"o_CHX_FF_RX_D_%d" % n: rx_bus[n] for n in range(rx_bus.nbits)},

	    # Parameter list
            p_D_MACROPDB            = "0b1",    # PCS power control
            p_D_IB_PWDNB            = "0b1",    # undocumented (required for RX)
            p_D_XGE_MODE            = "0b0",    # 10Gb ether
            p_D_LOW_MARK            = "0d4",    # Clock compensation FIFO low  water mark (mean=8)
            p_D_HIGH_MARK           = "0d12",   # Clock compensation FIFO high water mark (mean=8)
            p_D_BUS8BIT_SEL         = "0b0",
            p_D_CDR_LOL_SET         = "0b00",
            p_D_TXPLL_PWDNB         = "0b1",    # TX PLL power control
            p_D_BITCLK_LOCAL_EN     = "0b1",    # Use clock from local PLL
            p_D_BITCLK_ND_EN        = "0b0",
            p_D_BITCLK_FROM_ND_EN   = "0b0",
            p_D_SYNC_LOCAL_EN       = "0b1",
            p_D_SYNC_ND_EN          = "0b0",
            p_CHX_UC_MODE           = "0b0",
            p_CHX_PCIE_MODE         = "0b0",
            p_CHX_RIO_MODE          = "0b0",
            p_CHX_WA_MODE           = "0b0",
            p_CHX_PRBS_SELECTION    = "0b0",
            p_CHX_GE_AN_ENABLE      = "0b0",
            p_CHX_PRBS_LOCK         = "0b0",
            p_CHX_PRBS_ENABLE       = "0b0",
            p_CHX_ENABLE_CG_ALIGN   = "0b1",
            p_CHX_TX_GEAR_MODE      = "0b0",
            p_CHX_RX_GEAR_MODE      = "0b0",
            p_CHX_PCS_DET_TIME_SEL  = "0b00",
            p_CHX_PCIE_EI_EN        = "0b0",
            p_CHX_TX_GEAR_BYPASS    = "0b0",
            p_CHX_ENC_BYPASS        = "0b0",
            p_CHX_SB_BYPASS         = "0b0",
            p_CHX_RX_SB_BYPASS      = "0b0",
            p_CHX_WA_BYPASS         = "0b0",
            p_CHX_DEC_BYPASS        = "0b0",
            p_CHX_CTC_BYPASS        = "0b0",
            p_CHX_RX_GEAR_BYPASS    = "0b0",
            p_CHX_LSM_DISABLE       = "0b0",
            p_CHX_MATCH_2_ENABLE    = "0b1",
            p_CHX_MATCH_4_ENABLE    = "0b0",
            p_CHX_MIN_IPG_CNT       = "0b11",   # minimum interpacket gap of 4
            p_CHX_CC_MATCH_1        = "0x000",
            p_CHX_CC_MATCH_2        = "0x000",
            p_CHX_CC_MATCH_3        = "0x1BC",  # K28.5
            p_CHX_CC_MATCH_4        = "0x050",
            p_CHX_UDF_COMMA_MASK    = "0x3ff",  # compare all 10 bits
            p_CHX_UDF_COMMA_A       = "0x283",  # K28.5 inverted
            p_CHX_UDF_COMMA_B       = "0x17C",  # K28.3
            p_CHX_RX_DCO_CK_DIV     = "0b010",  # 10x
            p_CHX_RCV_DCC_EN        = "0b0",
            p_CHX_TPWDNB            = "0b1",    # Ch TX power control
            p_CHX_RATE_MODE_TX      = "0b0",
            p_CHX_RTERM_TX          = "0d19",   # TX terminate 50-ohms
            p_CHX_TX_CM_SEL         = "0b00",
            p_CHX_TDRV_PRE_EN       = "0b0",
            p_CHX_TDRV_SLICE0_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE1_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE2_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE3_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE4_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE5_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE0_CUR   = "0b000",  # 100 uA
            p_CHX_TDRV_SLICE1_CUR   = "0b000",  # 100 uA
            p_CHX_TDRV_SLICE2_CUR   = "0b11",   # 3200 uA
            p_CHX_TDRV_SLICE3_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE4_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE5_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_DAT_SEL      = "0b00",
            p_CHX_TX_DIV11_SEL      = "0b0",
            p_CHX_RPWDNB            = "0b1",    # Ch RX power control
            p_CHX_RATE_MODE_RX      = "0b0",
            p_CHX_RX_DIV11_SEL      = "0b0",
            p_CHX_SEL_SD_RX_CLK     = "0b0",    # FIFO driven by ebrd clock
            p_CHX_FF_RX_H_CLK_EN    = "0b0",
            p_CHX_FF_RX_F_CLK_DIS   = "0b0",
            p_CHX_FF_TX_H_CLK_EN    = "0b0",
            p_CHX_FF_TX_F_CLK_DIS   = "0b0",
            p_CHX_TDRV_POST_EN      = "0b0",
            p_CHX_TX_POST_SIGN      = "0b0",
            p_CHX_TX_PRE_SIGN       = "0b0",
            p_CHX_REQ_LVL_SET       = "0b00",
            p_CHX_REQ_EN            = "0b1",    # Enable equalizer
            p_CHX_RX_RATE_SEL       = "0d8",    # Equalizer pole position
            p_CHX_RTERM_RX          = "0d22",   # Undocumented value
            p_CHX_RXTERM_CM         = "0b11",   # RX Input (wizard value used)
            p_CHX_PDEN_SEL          = "0b1",    # phase detector disabled on LOS
            p_CHX_RXIN_CM           = "0b11",   # CMFB (wizard value used)
            p_CHX_LEQ_OFFSET_SEL    = "0b0",
            p_CHX_LEQ_OFFSET_TRIM   = "0b000",
            p_CHX_RLOS_SEL          = "0b1",
            p_CHX_RX_LOS_LVL        = "0b100",  # Lattice "TBD" (wizard value used)
            p_CHX_RX_LOS_CEQ        = "0b11",   # Lattice "TBD" (wizard value used)
            p_CHX_RX_LOS_HYST_EN    = "0b0",
            p_CHX_RX_LOS_EN         = "0b1",
            p_CHX_LDR_RX2CORE_SEL   = "0b0",
            p_CHX_LDR_CORE2TX_SEL   = "0b0",
            p_D_TX_MAX_RATE         = "1.25",   # 1.25Gbps
            p_CHX_CDR_MAX_RATE      = "1.25",   # 1.25Gbps
            p_CHX_TXAMPLITUDE       = "0d400",  # 400 mV
            p_CHX_TXDEPRE           = "DISABLED",
            p_CHX_TXDEPOST          = "DISABLED",
            p_CHX_PROTOCOL          = "GBE",
            p_D_ISETLOS             = "0d0",
            p_D_SETIRPOLY_AUX       = "0b00",
            p_D_SETICONST_AUX       = "0b00",
            p_D_SETIRPOLY_CH        = "0b00",
            p_D_SETICONST_CH        = "0b00",
            p_D_REQ_ISET            = "0b000",
            p_D_PD_ISET             = "0b00",
            p_D_DCO_CALIB_TIME_SEL  = "0b00",
            p_CHX_CDR_CNT4SEL       = "0b00",
            p_CHX_CDR_CNT8SEL       = "0b00",
            p_CHX_DCOATDCFG         = "0b00",
            p_CHX_DCOATDDLY         = "0b00",
            p_CHX_DCOBYPSATD        = "0b1",
            p_CHX_DCOCALDIV         = "0b001",
            p_CHX_DCOCTLGI          = "0b010",
            p_CHX_DCODISBDAVOID     = "0b0",
            p_CHX_DCOFLTDAC         = "0b01",
            p_CHX_DCOFTNRG          = "0b110",
            p_CHX_DCOIOSTUNE        = "0b000",
            p_CHX_DCOITUNE          = "0b00",
            p_CHX_DCOITUNE4LSB      = "0b111",
            p_CHX_DCOIUPDNX2        = "0b1",
            p_CHX_DCONUOFLSB        = "0b101",
            p_CHX_DCOSCALEI         = "0b00",
            p_CHX_DCOSTARTVAL       = "0b000",
            p_CHX_DCOSTEP           = "0b00",
            p_CHX_BAND_THRESHOLD    = "0d0",
            p_CHX_AUTO_FACQ_EN      = "0b1",
            p_CHX_AUTO_CALIB_EN     = "0b1",
            p_CHX_CALIB_CK_MODE     = "0b0",
            p_CHX_REG_BAND_OFFSET   = "0d0",
            p_CHX_REG_BAND_SEL      = "0d0",
            p_CHX_REG_IDAC_SEL      = "0d0",
            p_CHX_REG_IDAC_EN       = "0b0",
            p_D_CMUSETISCL4VCO      = "0b000",
            p_D_CMUSETI4VCO         = "0b00",
            p_D_CMUSETINITVCT       = "0b00",
            p_D_CMUSETZGM           = "0b000",
            p_D_CMUSETP2AGM         = "0b000",
            p_D_CMUSETP1GM          = "0b000",
            p_D_CMUSETI4CPZ         = "0d3",
            p_D_CMUSETI4CPP         = "0d3",
            p_D_CMUSETICP4Z         = "0b101",
            p_D_CMUSETICP4P         = "0b01",
            p_D_CMUSETBIASI         = "0b00",
            p_D_SETPLLRC            = "0d1",
            p_D_REFCK_MODE          = "0b001",  # 10x
            p_D_TX_VCO_CK_DIV       = "0b010",  # Divide by 2
            p_D_PLL_LOL_SET         = "0b00",
            p_D_RG_EN               = "0b0",
            p_D_RG_SET              = "0b00",
        )

        # DCU TX sink endpoint
        self.sink = sink = stream.Endpoint([("data", 8),
                                            ("ctrl", 1),
                                            ("disp", 1)])
        self.comb += [
            sink.ready.eq(1),
            tx_data[0:8].eq(sink.data[0:8]),
            tx_k.eq(sink.ctrl),
            tx_disp.eq(sink.disp),
            tx_bus[0:8].eq(tx_data[0:8]),
            tx_bus[8].eq(tx_k),
            tx_bus[9:11].eq(0),
            tx_bus[11].eq(tx_disp),
            tx_bus[12:23].eq(0),
        ]

        # DCU RX source endpoint
        self.source = source = stream.Endpoint([("data", 8), ("ctrl", 1)])
        self.comb += [
            source.valid.eq(1),
            rx_data[0:8].eq(rx_bus[0:8]),
            rx_k.eq(rx_bus[8]),
        ]
        self.sync.eth_rx += [
            source.data[0:8].eq(rx_data[0:8]),
            source.ctrl.eq(rx_k),
        ]

    def do_finalize(self):
        serdes_params = dict()
        for k, v in self.serdes_params.items():
            k = k.replace("CHX", "CH{}".format(self.channel))
            serdes_params[k] = v
        self.specials.dcu0 = Instance("DCUA", **serdes_params)
        self.dcu0.attr.add(("LOC", "DCU{}".format(self.dual)))
        self.dcu0.attr.add(("CHAN", "CH{}".format(self.channel)))
        self.dcu0.attr.add(("BEL", "X42/Y71/DCU"))

class LiteEthPHYSGMII(Module, AutoCSR):
    dw            = 8
    tx_clk_freq   = 125e6
    rx_clk_freq   = 125e6
    def __init__(self, platform, refclk):
        tx_pads     = platform.request("serdes_tx", 0)
        rx_pads     = platform.request("serdes_rx", 0)
        pads        = platform.request("eth_md", 0)

        self.submodules.pcs  = pcs = SGMIIECP5(tx_pads, rx_pads, refclk)
        self.cd_eth_rx = pcs.cd_eth_rx
        self.cd_eth_tx = pcs.cd_eth_tx
        self.submodules.tx = tx = ClockDomainsRenamer("eth_tx")(SGMIITX())
        self.submodules.rx = rx = ClockDomainsRenamer("eth_rx")(SGMIIRX())
        self.submodules.tx_pipeline = stream.Pipeline(tx, pcs)
        self.submodules.rx_pipeline = stream.Pipeline(pcs, rx)
        self.sink, self.source = self.tx_pipeline.sink, self.rx_pipeline.source

        if hasattr(pads, "mdc"):
            self.submodules.mdio = LiteEthPHYMDIO(pads)

        #self.button = button = platform.request("button_1", 0)

        self.link_up = Signal()
        self.restart = Signal()

        # SGMII Speed Adaptation
        is_sgmii = Signal()
        self.link_partner_adv_ability = Signal(16)

        # main module
        self.ci = seen_valid_ci = PulseSynchronizer("eth_rx", "eth_tx")
        self.submodules += seen_valid_ci
        self.comb += seen_valid_ci.i.eq(self.rx.seen_valid_ci)

        check_period  = 6e-3
        more_ack_time = 10e-3
        checker_max_val = ceil(check_period*125e6)
        checker_counter = Signal(max=checker_max_val+1)
        checker_tick = Signal()
        checker_ok = Signal()
        self.sync.eth_tx += [
            checker_tick.eq(0),
            If(checker_counter == 0,
                checker_tick.eq(1),
                checker_counter.eq(checker_max_val)
            ).Else(
                checker_counter.eq(checker_counter-1)
            ),
            If(seen_valid_ci.o, checker_ok.eq(1)),
            If(checker_tick, checker_ok.eq(0))
        ]

        autoneg_ack = Signal()
        self.comb += self.tx.config_reg.eq(
            (is_sgmii) |             # SGMII-specific
            ((~is_sgmii & 1) << 5) | # Full-duplex
            (autoneg_ack << 14)      # ACK
        )

        self.abi = rx_config_reg_abi = PulseSynchronizer("eth_rx", "eth_tx")
        self.ack = rx_config_reg_ack = PulseSynchronizer("eth_rx", "eth_tx")
        self.submodules += [
            rx_config_reg_abi, rx_config_reg_ack
        ]

        more_ack_timer = ClockDomainsRenamer("eth_tx")(
            WaitTimer(ceil(more_ack_time*125e6)))
        self.submodules += more_ack_timer

        fsm_inited = Signal()
        config_reg_empty = Signal()

        self.submodules.fsm = fsm = ClockDomainsRenamer("eth_tx")(FSM())
        fsm.act("AUTONEG_WAIT_ABI",
            self.tx.config_stb.eq(fsm_inited),
            If(rx_config_reg_abi.o,
                NextValue(fsm_inited, 1),
                NextState("AUTONEG_WAIT_ACK")
            ),
            If(checker_tick & ~checker_ok,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )
        fsm.act("AUTONEG_WAIT_ACK",
            self.tx.config_stb.eq(1),
            autoneg_ack.eq(1),
            If(rx_config_reg_ack.o,
                NextState("AUTONEG_SEND_MORE_ACK")
            ),
            If(checker_tick & ~checker_ok,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )
        # COMPLETE_ACKNOWLEDGE
        fsm.act("AUTONEG_SEND_MORE_ACK",
            self.tx.config_stb.eq(1),
            autoneg_ack.eq(1),
            more_ack_timer.wait.eq(1),
            If(more_ack_timer.done,
                NextState("RUNNING")
            ),
            If(checker_tick & ~checker_ok,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )
        # LINK_OK
        fsm.act("RUNNING",
            self.link_up.eq(1),
            If((checker_tick & ~checker_ok) | config_reg_empty,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )

        c_counter = Signal(max=5)
        previous_config_reg = Signal(16)
        preack_config_reg = Signal(16)
        self.sync.eth_rx += [
            # Restart consistency counter
            If(self.rx.seen_config_reg,
                c_counter.eq(4)
            ).Elif(c_counter != 0,
                c_counter.eq(c_counter - 1)
            ),

            rx_config_reg_abi.i.eq(0),
            rx_config_reg_ack.i.eq(0),
            If(self.rx.seen_config_reg,
                previous_config_reg.eq(self.rx.config_reg),
                If((c_counter == 1) &
                   ((previous_config_reg|0x4000) == (self.rx.config_reg|0x4000)) &
                   ((self.rx.config_reg | 1) != 1),
                   # Ability match
                   rx_config_reg_abi.i.eq(1),
                   preack_config_reg.eq(previous_config_reg),
                   # Acknowledgement/Consistency match
                   If((previous_config_reg[14] & self.rx.config_reg[14]) &
                      ((preack_config_reg|0x4000) == (self.rx.config_reg|0x4000)),
                      rx_config_reg_ack.i.eq(1)
                   )
                ),
                # Record advertised ability of link partner
                self.link_partner_adv_ability.eq(self.rx.config_reg)
            )
        ]

        self.comb += [
            is_sgmii.eq(self.link_partner_adv_ability[0]),
            # Detect that config_reg is empty
            config_reg_empty.eq((self.link_partner_adv_ability | 1) == 1),
            pcs.rx_restart.eq(self.restart),
        ]
