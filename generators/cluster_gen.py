#!/usr/bin/env python3
# Copyright (c) 2023 Joseph Wagane Faye <joseph-wagane.faye@insa-rennes.fr>
# SPDX-License-Identifier: BSD-2-Clause

import json
import time
import argparse
from migen import *
from litex.soc.cores.cpu import CPUS
from litex.build.generic_platform import *
from litex.soc.interconnect import wishbone
from litex.soc.integration.builder import *
from litex.soc.integration.soc_core import *

# IOs/Interfaces -----------------------------------------------------------------------------------

def get_common_ios():
    return [
        # Clk/Rst.
        ("clk", 0, Pins(1)),
        ("rst", 0, Pins(1)),
    ]


def get_uart_ios():
    return [
        # Serial
        ("uart", 0,
         Subsignal("tx", Pins(1)),
         Subsignal("rx", Pins(1)),
         ),
        ("uart", 1,
         Subsignal("tx", Pins(1)),
         Subsignal("rx", Pins(1)),
         )
    ]


def get_debug_ios(debug_width=8):
    return [
        ("debug", 0, Pins(debug_width)),
    ]


# Platform -----------------------------------------------------------------------------------------

class Platform(GenericPlatform):
    def build(self, fragment, build_dir, build_name, **kwargs):
        os.makedirs(build_dir, exist_ok=True)
        os.chdir(build_dir)
        conv_output = self.get_verilog(fragment, name=build_name)
        conv_output.write(f"{build_name}.v")


# LiteX SoC Generator ------------------------------------------------------------------------------
class Cluster(SoCMini):
    mem_map = {
        "csr": 0x10000000,
        "rom": 0x00000000,
        "sram": 0x01000000,
        "main_ram": 0x01010000,
    }

    def __init__(self, n_master_i, n_slave_i, cluster_file, name="litex_soc", sys_clk_freq=int(50e6), build_dir='',
                 **kwargs):
        cluster = self.get_config(cluster_file)
        # Platform ---------------------------------------------------------------------------------
        platform = Platform(device="", io=get_common_ios())
        platform.name = name
        platform.add_extension(get_uart_ios())
        self.kwargs = kwargs

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(
            clk=platform.request("clk"),
            rst=platform.request("rst"),
        )

        # SoC --------------------------------------------------------------------------------------
        if self.kwargs["uart_name"] == "serial":
            self.kwargs["uart_name"] = "uart"
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq, ident=f"LiteX Standalone Cluster SoC - {name}")
        # Shared RAM.
        self.add_ram("shared_ram", 0x0000_0000, int(cluster["shared_ram"], 0), contents=[i for i in range(16)])

        # Debug ------------------------------------------------------------------------------------
        platform.add_extension(get_debug_ios())
        debug_pads = platform.request("debug")
        self.comb += [
            # Export Signal(s) for debug.
            debug_pads[0].eq(0),  # 0.
            debug_pads[1].eq(1),  # 1.
            # Etc...
        ]

        # Cluster CPUS Integration

        #print(f"{cluster}")
        #time.sleep(5)
        for i in range(int(cluster['n_cpus'], 0)):
            name = cluster[f'core_{i}']['name']
            soc_name = f'{name}_soc_{i}'
            rom_size = int(cluster[f'core_{i}'].get('rom'), 0)
            ram_size = int(cluster[f'core_{i}'].get('ram'), 0)
            sram_size = int(cluster[f'core_{i}'].get('sram'), 0)
            #print(f"{name}, {soc_name}, {rom_size}, {ram_size}, {sram_size}")
            #time.sleep(5)
            #print(f"{build_dir}")
            #time.sleep(5)
            os.system(f"litex_soc_gen --cpu-type={name} --n_master_i=2 --bus-standard=wishbone "
                  f"--sys-clk-freq=50e6 --name={soc_name} "
                  f"--integrated-rom-size={rom_size} "
                  f"--integrated-main-ram-size={ram_size} "
                  f"--integrated-sram-size={sram_size} "
                  f"--output-dir={os.path.join(build_dir, soc_name) if build_dir else ''} "
                  f"--build")
            # Add standalone SoC sources.
            platform.add_source(
                f"{os.path.join(build_dir, f'{soc_name}', 'gateware', f'{soc_name}.v') if build_dir else f'build/{soc_name}/gateware/{soc_name}.v'}")
            platform.add_source(
                f"{os.path.join(build_dir, f'{soc_name}', 'gateware', f'{soc_name}_rom.init') if build_dir else f'build/{soc_name}/gateware/{soc_name}_rom.init'}",
                copy=True)
            # Add CPU sources.
            CPUS[name].add_sources(platform, "standard")
            # Do standalone SoC instance.
            uart_pads = platform.request("uart", i)
            mmap_ext = wishbone.Interface()
            mmap_wb = wishbone.Interface()
            self.specials += Instance(f"{soc_name}",
                                      # Clk/Rst.
                                      i_clk=ClockSignal("sys"),
                                      i_rst=ResetSignal("sys"),

                                      # UART.
                                      o_uart_tx=uart_pads.tx,
                                      i_uart_rx=uart_pads.rx,

                                      # MMAP.
                                      o_mmap_m_0_adr=mmap_wb.adr[:24],  # CHECKME/FIXME: Base address
                                      o_mmap_m_0_dat_w=mmap_wb.dat_w,
                                      i_mmap_m_0_dat_r=mmap_wb.dat_r,
                                      o_mmap_m_0_sel=mmap_wb.sel,
                                      o_mmap_m_0_cyc=mmap_wb.cyc,
                                      o_mmap_m_0_stb=mmap_wb.stb,
                                      i_mmap_m_0_ack=mmap_wb.ack,
                                      o_mmap_m_0_we=mmap_wb.we,
                                      o_mmap_m_0_cti=mmap_wb.cti,
                                      o_mmap_m_0_bte=mmap_wb.bte,
                                      i_mmap_m_0_err=mmap_wb.err,

                                      # MMAP | Scratchpad
                                      o_mmap_m_1_adr=mmap_ext.adr[:24],  # CHECKME/FIXME: Base address
                                      o_mmap_m_1_dat_w=mmap_ext.dat_w,
                                      i_mmap_m_1_dat_r=mmap_ext.dat_r,
                                      o_mmap_m_1_sel=mmap_ext.sel,
                                      o_mmap_m_1_cyc=mmap_ext.cyc,
                                      o_mmap_m_1_stb=mmap_ext.stb,
                                      i_mmap_m_1_ack=mmap_ext.ack,
                                      o_mmap_m_1_we=mmap_ext.we,
                                      o_mmap_m_1_cti=mmap_ext.cti,
                                      o_mmap_m_1_bte=mmap_ext.bte,
                                      i_mmap_m_1_err=mmap_ext.err,
                                      )
            self.bus.add_master(master=mmap_wb)
            #wb_region = SoCRegion(origin=origin[i], size=0x10000000, cached=False)
            #self.bus.add_slave(name=f"mmap_m_{i}", slave=mmap_ext, region=wb_region)
            self.platform.add_extension(mmap_ext.get_ios(f"mmap_m_{i}"))
            wb_pads = self.platform.request(f"mmap_m_{i}".format(mmap_ext))
            self.comb += mmap_ext.connect_to_pads(wb_pads, mode="master")

    def get_config(self, config_file):
        assert os.path.exists(config_file)
        with open(config_file, 'r') as f:
            data = json.load(f)
        return data
# Build --------------------------------------------------------------------------------------------
def main():
    # Arguments.
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="LiteX Cluster Generator")
    target_group = parser.add_argument_group(title="Generator options")
    target_group.add_argument("--name", default="Cluster", help="SoC Name.")
    target_group.add_argument("--cluster_file", default={'n_cpus': '2', 'core_0': {'name': 'femtorv', 'sram': '0x2000', 'ram': '0x0', 'rom': '0x8000'}, 'core_1': {'name': 'firev', 'sram': '0x2000', 'ram': '0x0', 'rom': '0x8000'}, 'shared_ram': '0x100'})
    target_group.add_argument("--build_dir", default='')
    target_group.add_argument("--n_master_i", default=int(2), help="Number of master interfaces.")
    target_group.add_argument("--n_slave_i", default=int(0), help="Number of slave interfaces.")
    target_group.add_argument("--build", action="store_true", help="Build SoC.")
    target_group.add_argument("--sys-clk-freq", default=int(50e6), help="System clock frequency.")
    builder_args(parser)
    soc_core_args(parser)
    args = parser.parse_args()
    #print(f'We are in ram_soc.py cluster = {args.cluster_file}')
    #time.sleep(5)
    # SoC.
    soc = Cluster(
        name=args.name,
        cluster_file=args.cluster_file,
        build_dir=args.build_dir,
        sys_clk_freq=int(float(args.sys_clk_freq)),
        n_master_i=int(args.n_master_i),
        n_slave_i=int(args.n_slave_i),
        **soc_core_argdict(args)
    )

    # Build.
    builder = Builder(soc, **builder_argdict(args))
    builder.build(build_name=args.name, run=args.build)


if __name__ == "__main__":
    main()
