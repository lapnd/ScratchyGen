#!/usr/bin/env python3

# Copyright (c) 2023 Joseph Wagane FAYE <joseph-wagane.faye@insa-rennes.fr
# SPDX-License-Identifier: BSD-2-Clause
import time
import argparse
import litex.soc.doc as lxsocdoc
import time
from migen import *
from litex.soc.cores.cpu import CPUS
from litex.soc.cores.uart import *
from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.interconnect import csr_bus
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import *
from litex.soc.integration.doc import AutoDoc
from litex_boards.platforms import terasic_de10lite
from litex.soc.integration.soc import SoCBusHandler, SoCRegion, SoCCSRRegion, SoCError
from litex.build.generic_platform import *
import json
from litex.build.altera.programmer import USBBlaster
# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_sys_ps = ClockDomain()
        self.clock_domains.cd_vga    = ClockDomain()

        # # #

        # Clk / Rst
        clk50 = platform.request("clk50")

        # PLL
        self.submodules.pll = pll = Max10PLL(speedgrade="-7")
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_vga,    40e6)

# BaseSoC ------------------------------------------------------------------------------------------

class Scratchy(SoCMini, AutoDoc):
    mem_map = {
        "csr": 0x10000000,
        "rom": 0x00000000,
        "sram": 0x01000000,
        "main_ram": 0x01010000,
    }
    def __init__(self, platform, platform_name, mux, config_file, build_dir='', sys_clk_freq=int(50e6)):

        SoCMini.__init__(self, platform, sys_clk_freq, ident=f"FractalArch {platform_name}")

        config = self.get_config(config_file)

        platform.add_extension([("serial", 1,
                                 Subsignal("tx", Pins("V9"), IOStandard("3.3-V LVTTL")),  # JP1 GPIO[2]
                                 Subsignal("rx", Pins("W9"), IOStandard("3.3-V LVTTL"))  # JP1 GPIO[3]
                                 ), ])
        platform.add_extension([("serial", 2,
                                 Subsignal("tx", Pins("V8"), IOStandard("3.3-V LVTTL")),  # JP1 GPIO[2]
                                 Subsignal("rx", Pins("W8"), IOStandard("3.3-V LVTTL"))  # JP1 GPIO[3]
                                 ), ])
        platform.add_extension([("serial", 3,
                                 Subsignal("tx", Pins("V7"), IOStandard("3.3-V LVTTL")),  # JP1 GPIO[2]
                                 Subsignal("rx", Pins("W7"), IOStandard("3.3-V LVTTL"))  # JP1 GPIO[3]
                                 ), ])

        contents = [i for i in range(16)]
        n_mux = int(config["n_cpus"])
        if mux :
            uart_pads = platform.request("serial", 0)
            uart_sel  = platform.request("user_sw", 0)
            uart_mux_pads = [UARTPads() for _ in range(n_mux)]
            uart_mux      = UARTMultiplexer(uart_mux_pads, uart_pads)
            self.comb += uart_mux.sel.eq(uart_sel)
            self.submodules += uart_mux

        else :
            uart_mux_pads = []
            for i in range(n_mux) :
                uart_mux_pads.append(platform.request("serial", i)) 

        # Super SoC Shared RAM
        self.add_ram("shared_ram", 0x0000_0000, int(config["shared_ram"], 0))
        r = []
        # Add cluster
        for i in range(int(config["n_cluster"], 0)):
            cluster = config[f'cluster_{i}']

            with open(f"cluster_{i}.json", "w") as outfile:
                json.dump(cluster, outfile)

            #print(f"Cluster : {cluster}")
            #time.sleep(5)
            root_dir = os.path.join(build_dir, f"cluster_{i}") if build_dir else os.path.join(os.getcwd(), 'build', f"cluster_{i}")
            # Uart pads index
            index=0
            soc_name = f"cluster_{i}"
            core_name = cluster[f"core_{i}"].get("name")
            core_soc_name = f"{core_name}_soc_{index + i}"
            # Cluster Generation
            os.system(f"./cluster_gen.py --name {soc_name} "
                      f"--cluster cluster_{i}.json "
                      f"--build_dir {root_dir} "
                      f"--build")
            # Add Cluster Sources
            platform.add_source(f"{os.path.join(root_dir, 'gateware', f'{soc_name}.v')}")
            platform.add_source(f"{os.path.join(root_dir, 'gateware', f'{soc_name}_mem.init')}")

            platform.add_source(f"{os.path.join(root_dir, f'{core_soc_name}', 'gateware', f'{core_soc_name}.v')}")
            platform.add_source(f"{os.path.join(root_dir, f'{core_soc_name}', 'gateware', f'{core_soc_name}_rom.init')}", copy=True)

            CPUS[core_name].add_sources(platform, "standard")

            # Wishbone Interface Modification.

            mmap_wb_1 = wishbone.Interface()
            mmap_wb_0 = wishbone.Interface()
            #print("UART MUX PADS : ",uart_mux_pads)
            #time.sleep(10)
            self.specials += Instance(f"{soc_name}",
                                      i_clk=ClockSignal("sys"),
                                      i_rst=ResetSignal("sys"),
                                      # UARTs
                                      o_uart0_tx=uart_mux_pads[2*i].tx,
                                      i_uart0_rx=uart_mux_pads[2*i].rx,
                                      o_uart1_tx=uart_mux_pads[2*i+1].tx,
                                      i_uart1_rx=uart_mux_pads[2*i+1].rx,
                                      # MMAP 0
                                      o_mmap_m_0_adr=mmap_wb_0.adr[:24],  # CHECKME/FIXME: Base address
                                      o_mmap_m_0_dat_w=mmap_wb_0.dat_w,
                                      i_mmap_m_0_dat_r=mmap_wb_0.dat_r,
                                      o_mmap_m_0_sel=mmap_wb_0.sel,
                                      o_mmap_m_0_cyc=mmap_wb_0.cyc,
                                      o_mmap_m_0_stb=mmap_wb_0.stb,
                                      i_mmap_m_0_ack=mmap_wb_0.ack,
                                      o_mmap_m_0_we=mmap_wb_0.we,
                                      o_mmap_m_0_cti=mmap_wb_0.cti,
                                      o_mmap_m_0_bte=mmap_wb_0.bte,
                                      i_mmap_m_0_err=mmap_wb_0.err,
                                      # MMAP 1
                                      o_mmap_m_1_adr=mmap_wb_1.adr[:24],  # CHECKME/FIXME: Base address
                                      o_mmap_m_1_dat_w=mmap_wb_1.dat_w,
                                      i_mmap_m_1_dat_r=mmap_wb_1.dat_r,
                                      o_mmap_m_1_sel=mmap_wb_1.sel,
                                      o_mmap_m_1_cyc=mmap_wb_1.cyc,
                                      o_mmap_m_1_stb=mmap_wb_1.stb,
                                      i_mmap_m_1_ack=mmap_wb_1.ack,
                                      o_mmap_m_1_we=mmap_wb_1.we,
                                      o_mmap_m_1_cti=mmap_wb_1.cti,
                                      o_mmap_m_1_bte=mmap_wb_1.bte,
                                      i_mmap_m_1_err=mmap_wb_1.err,
                                      )
            self.bus.add_master(master=mmap_wb_0)
            self.bus.add_master(master=mmap_wb_1)
    def get_config(self, config_file):
        print(f"Config file : {config_file}")
        assert os.path.exists(config_file)
        with open(config_file, 'r') as f:
            data = json.load(f)
        return data        

def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser

    parser = LiteXSoCArgumentParser(description="LiteX AMP Cluster Generator")
    target_group = parser.add_argument_group(title="Target options")

    target_group.add_argument("--platform",      default=terasic_de10lite.Platform())
    target_group.add_argument("--sys-clk-freq",  default=50e6,                help="System clock frequency."),
    target_group.add_argument('--config_file',   help='Configuration file',   required=True)
    target_group.add_argument("--build_dir",     default='build',             help="Base output directory.")
    target_group.add_argument("--build",         action="store_true",         help="Build bitstream.")
    target_group.add_argument("--load",          action="store_true",         help="Load bitstream.")
    target_group.add_argument("--mux",           action="store_true",         help="use uart mux.")

    builder_args(parser)  # Builder Arguments.
    soc_core_args(parser) # SoC Arguments.

    args = parser.parse_args()
    args.cpu_type = None  # Build the architecture with no cpu.
    soc = Scratchy(
        platform_name  = 'Scratchy',
        platform       = args.platform,
        config_file    = args.config_file,
        sys_clk_freq   = int(float(args.sys_clk_freq)),
        mux            = args.mux,
        build_dir      = args.build_dir,
    )

    args.output_dir = os.path.join(args.build_dir, soc.platform.name) if args.build_dir else ''
    builder = Builder(soc, **builder_argdict(args))
    build_kwargs = {}
    builder.build(**build_kwargs, run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    lxsocdoc.generate_docs(soc, f"{os.path.join(args.build_dir, 'documentation') if args.build_dir else 'build/documentation'}", project_name="Assymetric Multi-Processing SoC", author="Joseph W. FAYE")

if __name__ == "__main__":
    main()
