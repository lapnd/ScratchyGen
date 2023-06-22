#!/usr/bin/env python3

# Copyright (c) 2022 Joseph Wagane FAYE <joseph-wagane.faye@insa-rennes.fr
# SPDX-License-Identifier: BSD-2-Clause
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

class _CRG(Module, AutoDoc):
    def __init__(self, platform, sys_clk_freq, with_rst=True):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()
        # # #
        # Clk/Rst.
        clk50 = platform.request("clk50")
        # PLL.
        self.submodules.pll = pll = Max10PLL(speedgrade='-7')
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini, AutoDoc):
    mem_map = {
        "csr": 0x10000000,
        "rom": 0x00000000,
        "sram": 0x01000000,
        "main_ram": 0x01010000,
    }
    def __init__(self, platform, platform_name, mux, toolchain="vivado", build_dir='',
                 name_1='firev', name_2='firev', sram_1_size=0x1000, ram_1_size=0x1000,
                 ram_2_size=0x1000, sram_2_size=0x1000, rom_1_size=0x1000, rom_2_size=0x1000,
                 sp_1_size=0x1000, sp_2_size=0x1000, shared_ram_size=0x1000, bus_data_width= 16,
                 sys_clk_freq=int(50e6), with_led_chaser=False):

        if name_1 == name_2:
            soc_name_1 = f"{name_1}_soc_0"
            soc_name_2 = f"{name_2}_soc_1"
        else:
            soc_name_1 = f"{name_1}_soc"
            soc_name_2 = f"{name_2}_soc"

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        platform.add_extension([("arduino_serial", 0,
                                    Subsignal("tx", Pins("AA19"), IOStandard("3.3-V LVTTL")), # Arduino IO11
                                    Subsignal("rx", Pins("Y19"), IOStandard("3.3-V LVTTL"))  # Arduino IO12
                                ),])

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, bus_data_width=bus_data_width,
                         ident="LiteX standalone SoC generator on {}".format(platform_name))


        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.submodules.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

        self.buses = [self.bus]
        # Standalone SoC Generation/Re-Integration -------------------------------------------------
        contents = [i for i in range(16)]

        # Shared or individual UART.

        if mux:
            #uart_pads = platform.request("arduino_serial")
            uart_pads = platform.request("serial", 0)
            uart_sel  = platform.request("user_sw", 0)
            uart_mux_pads = [UARTPads() for _ in range(2)]
            uart_mux      = UARTMultiplexer(uart_mux_pads, uart_pads)
            self.comb += uart_mux.sel.eq(uart_sel)
            self.submodules += uart_mux

        else:
            uart_mux_pads =[platform.request("serial", 0), platform.request("serial", 1)]

        # Shared RAM.
        self.add_ram("shared_ram", 0x0000_0000, shared_ram_size, contents=contents)

        # Buses
        self.submodules.bus1 = SoCBusHandler(data_width=bus_data_width)
        self.buses.append(self.bus1)
        self.submodules.bus2 = SoCBusHandler(data_width=bus_data_width)
        self.buses.append(self.bus2)

        # Interfaces to processors
        mmap_sp1 = wishbone.Interface()
        mmap_sp2 = wishbone.Interface()

        # Scratchpad Memories

        r = []

        if sp_1_size > 0:
            # Scratchpads Memories Interfaces
            interface_1 = wishbone.Interface(data_width=self.bus1.data_width, bursting=self.bus1.bursting)
            self.submodules.scratch1 = wishbone.SRAM(sp_1_size, bus=interface_1, init=contents,
                                                 read_only=False, name='scratchpad1')
            # Interfaces to processors
            for name, width, direction in mmap_sp1.layout:
                sig1 = getattr(self.scratch1.bus, name)
                pad1 = getattr(mmap_sp1, name)
                if direction == DIR_S_TO_M:
                    r.append(pad1.eq(sig1))
                else:
                    r.append(sig1.eq(pad1))

        if sp_2_size > 0:
            # Scratchpads Memories Interfaces
            interface_2 = wishbone.Interface(data_width=self.bus2.data_width, bursting=self.bus2.bursting)
            self.submodules.scratch2 = wishbone.SRAM(sp_2_size, bus=interface_2, init=contents,
                                                 read_only=False, name='scratchpad2')

            for name, width, direction in mmap_sp2.layout:
                sig1 = getattr(self.scratch2.bus, name)
                pad1 = getattr(mmap_sp2, name)
                if direction == DIR_S_TO_M:
                    r.append(pad1.eq(sig1))
                else:
                    r.append(sig1.eq(pad1))
        # Connection to pads
        self.comb += r

        # SoC.
        # ------------
        # Generate standalone SoC.

        os.system(f"litex_soc_gen --cpu-type={name_1} --n_master_i=2 --bus-standard=wishbone "
                  f"--sys-clk-freq=50e6 --name={soc_name_1} "
                  f"--integrated-rom-size={rom_1_size} "
                  f"--integrated-main-ram-size={ram_1_size} "
                  f"--integrated-sram-size={sram_1_size} "
                  f"--output-dir={os.path.join(build_dir, soc_name_1) if build_dir else ''} "
                  f"--build")
        # Add standalone SoC sources.
        platform.add_source(
            f"{os.path.join(build_dir, f'{soc_name_1}', 'gateware', f'{soc_name_1}.v') if build_dir else f'build/{soc_name_1}/gateware/{soc_name_1}.v'}")
        platform.add_source(
            f"{os.path.join(build_dir, f'{soc_name_1}',  'gateware', f'{soc_name_1}_rom.init') if build_dir else f'build/{soc_name_1}/gateware/{soc_name_1}_rom.init'}",
            copy=True)

        # Add CPU sources.
        CPUS[name_1].add_sources(platform, "standard")

        # Do standalone SoC instance.
        mmap_wb = wishbone.Interface()
        self.specials += Instance(f"{soc_name_1}",
            # Clk/Rst.
            i_clk     = ClockSignal("sys"),
            i_rst     = ResetSignal("sys"),

            # UART.
            o_uart_tx = uart_mux_pads[0].tx,
            i_uart_rx = uart_mux_pads[0].rx,

            # MMAP.
            o_mmap_m_0_adr   = mmap_wb.adr[:24], # CHECKME/FIXME: Base address
            o_mmap_m_0_dat_w = mmap_wb.dat_w,
            i_mmap_m_0_dat_r = mmap_wb.dat_r,
            o_mmap_m_0_sel   = mmap_wb.sel,
            o_mmap_m_0_cyc   = mmap_wb.cyc,
            o_mmap_m_0_stb   = mmap_wb.stb,
            i_mmap_m_0_ack   = mmap_wb.ack,
            o_mmap_m_0_we    = mmap_wb.we,
            o_mmap_m_0_cti   = mmap_wb.cti,
            o_mmap_m_0_bte   = mmap_wb.bte,
            i_mmap_m_0_err   = mmap_wb.err,

            #MMAP | Scratchpad
            o_mmap_m_1_adr=mmap_sp1.adr[:24],  # CHECKME/FIXME: Base address
            o_mmap_m_1_dat_w=mmap_sp1.dat_w,
            i_mmap_m_1_dat_r=mmap_sp1.dat_r,
            o_mmap_m_1_sel=mmap_sp1.sel,
            o_mmap_m_1_cyc=mmap_sp1.cyc,
            o_mmap_m_1_stb=mmap_sp1.stb,
            i_mmap_m_1_ack=mmap_sp1.ack,
            o_mmap_m_1_we=mmap_sp1.we,
            o_mmap_m_1_cti=mmap_sp1.cti,
            o_mmap_m_1_bte=mmap_sp1.bte,
            i_mmap_m_1_err=mmap_sp1.err,
        )
        self.bus.add_master(master=mmap_wb)

        # Standalone SoC2.
        # ----------

        # Generate standalone SoC.
        os.system(f"litex_soc_gen --cpu-type={name_2} --bus-standard=wishbone "
                  f"--sys-clk-freq=50e6 --n_master_i=2 --name={soc_name_2} "
                  f"--integrated-rom-size={rom_2_size} "
                  f"--integrated-main-ram-size={ram_2_size} "
                  f"--integrated-sram-size={sram_2_size} "
                  f"--output-dir={os.path.join(build_dir, soc_name_2) if build_dir else ''} "
                  f"--build")
        # Add standalone SoC sources.
        platform.add_source(
            f"{os.path.join(build_dir, f'{soc_name_2}', 'gateware', f'{soc_name_2}.v') if build_dir else f'build/{soc_name_2}/gateware/{soc_name_2}.v'}")
        platform.add_source(
            f"{os.path.join(build_dir, f'{soc_name_2}', 'gateware', f'{soc_name_2}_rom.init') if build_dir else f'build/{soc_name_2}/gateware/{soc_name_2}_rom.init'}",
            copy=True)

        # Add CPU sources.
        CPUS[name_2].add_sources(platform, "standard")

        # Do standalone SoC instance.
        mmap_wb = wishbone.Interface()
        self.specials += Instance(f"{soc_name_2}",
            # Clk/Rst.

            i_clk     = ClockSignal("sys"),
            i_rst     = ResetSignal("sys"),

            # UART.

            o_uart_tx = uart_mux_pads[1].tx,
            i_uart_rx = uart_mux_pads[1].rx,

            # MMAP.

            o_mmap_m_0_adr   = mmap_wb.adr[:24], # CHECKME/FIXME: Base address.
            o_mmap_m_0_dat_w = mmap_wb.dat_w,
            i_mmap_m_0_dat_r = mmap_wb.dat_r,
            o_mmap_m_0_sel   = mmap_wb.sel,
            o_mmap_m_0_cyc   = mmap_wb.cyc,
            o_mmap_m_0_stb   = mmap_wb.stb,
            i_mmap_m_0_ack   = mmap_wb.ack,
            o_mmap_m_0_we    = mmap_wb.we,
            o_mmap_m_0_cti   = mmap_wb.cti,
            o_mmap_m_0_bte   = mmap_wb.bte,
            i_mmap_m_0_err   = mmap_wb.err,

            #MMAP | Scratchpad

            o_mmap_m_1_adr=mmap_sp2.adr[:24],  # CHECKME/FIXME: Base address
            o_mmap_m_1_dat_w=mmap_sp2.dat_w,
            i_mmap_m_1_dat_r=mmap_sp2.dat_r,
            o_mmap_m_1_sel=mmap_sp2.sel,
            o_mmap_m_1_cyc=mmap_sp2.cyc,
            o_mmap_m_1_stb=mmap_sp2.stb,
            i_mmap_m_1_ack=mmap_sp2.ack,
            o_mmap_m_1_we=mmap_sp2.we,
            o_mmap_m_1_cti=mmap_sp2.cti,
            o_mmap_m_1_bte=mmap_sp2.bte,
            i_mmap_m_1_err=mmap_sp2.err,
        )
        self.bus.add_master(master=mmap_wb)

# Build --------------------------------------------------------------------------------------------

def extract_config(config_file, config):
    configuration = {}
    print(config_file)
    assert os.path.exists(config_file)
    with open(config_file, 'r') as f:
        data = json.load(f)
    print(data)
    config_dict = data[config]
    #print(config_dict)
    configuration['shared_ram_size'] = int(config_dict.get('shared_ram'), 0)
    configuration["name_1"]          = config_dict.get('core_1').get("name")
    configuration["name_2"]          = config_dict.get('core_2').get("name")
    configuration['sram_1_size']     = int(config_dict.get("core_1").get("sram_s"), 0)
    configuration['sram_2_size']     = int(config_dict.get("core_2").get("sram_s"), 0)
    configuration['ram_1_size']      = int(config_dict.get("core_1").get("ram_s"), 0)
    configuration['ram_2_size']      = int(config_dict.get("core_2").get("ram_s"), 0)
    configuration['rom_1_size']      = int(config_dict.get("core_1").get("rom_s"), 0)
    configuration['rom_2_size']      = int(config_dict.get("core_2").get("rom_s"), 0)
    configuration['sp_1_size']       = int(config_dict.get("core_1").get("sp_s"), 0)
    configuration['sp_2_size']       = int(config_dict.get("core_2").get("sp_s"), 0)
    return configuration

def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="LiteX AMP Dual-Core SoC generator on De10Lite")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--platform",       default=terasic_de10lite.Platform())
    target_group.add_argument("--toolchain",      default="quartus",           help="FPGA toolchain (vivado, symbiflow or yosys+nextpnr).")
    target_group.add_argument("--sys-clk-freq",   default=50e6,                help="System clock frequency.")
    target_group.add_argument("--bus_data_width", default=16,                  help="Super SoC bus data width.")
    target_group.add_argument('--config_file',    help='Configuration file',   required=True)
    target_group.add_argument('--config',         help='Configuration number', required=True)
    target_group.add_argument("--build",          action="store_true",         help="Build bitstream.")
    target_group.add_argument("--build_dir",      default='',                  help="Base output directory.")
    target_group.add_argument("--load",           action="store_true",         help="Load bitstream.")
    target_group.add_argument("--mux",            default=False,               help="use uart mux.")
    builder_args(parser)
    args = parser.parse_args()

    configuration = extract_config(args.config_file, args.config)

    soc = BaseSoC(
        platform_name  = 'De10Lite',
        platform       = args.platform,
        toolchain      = args.toolchain,
        sys_clk_freq   = int(float(args.sys_clk_freq)),
        bus_data_width=  int(args.bus_data_width),
        mux            = args.mux,
        build_dir      = args.build_dir,
        shared_ram_size= configuration['shared_ram_size'],
        name_1         = configuration['name_1'],
        name_2         = configuration['name_2'],
        sram_1_size    = configuration['sram_1_size'],
        sram_2_size    = configuration['sram_2_size'],
        ram_1_size     = configuration['ram_1_size'],
        ram_2_size     = configuration['ram_2_size'],
        rom_1_size     = configuration['rom_1_size'],
        rom_2_size     = configuration['rom_2_size'],
        sp_1_size      = configuration['sp_1_size'],
        sp_2_size      = configuration['sp_2_size'],
    )
    args.output_dir = os.path.join(args.build_dir, soc.platform.name) if args.build_dir else ''
    builder = Builder(soc, **builder_argdict(args))
    builder_kwargs = {}
    builder.build(**builder_kwargs, run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    lxsocdoc.generate_docs(soc, f"{os.path.join(args.build_dir, 'documentation') if args.build_dir else 'build/documentation'}", project_name="Assymetric Multi-Processing SoC", author="Joseph W. FAYE")

if __name__ == "__main__":
    main()