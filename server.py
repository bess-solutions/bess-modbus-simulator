#!/usr/bin/env python3
"""
server.py
=========
Universal Modbus TCP Simulator for BESS Solutions.
Loads any device profile JSON and dynamically populates holding registers,
simulating live telemetry with data-type and endianness awareness.
"""
import os
import sys
import json
import argparse
import asyncio
import logging
import struct
import math
import random
import time

from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusDeviceContext
from pymodbus.pdu.device import ModbusDeviceIdentification
from pymodbus.server import StartAsyncTcpServer

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("bess-sim")

class UniversalBessSimulator:
    def __init__(self, profile_path: str):
        self.profile_path = profile_path
        self.profile = self.load_profile()
        self.device = self.profile.get("device", {})
        self.conn_cfg = self.profile.get("connection", {})
        self.registers = self.profile.get("registers", {})
        
        # Endianness setup
        self.byte_order = self.conn_cfg.get("byte_order", "BIG").upper()
        self.word_order = self.conn_cfg.get("word_order", "BIG").upper()
        
        # Find maximum register address to size the block
        max_addr = 1000
        for reg in self.registers.values():
            addr = reg.get("address", 0)
            reg_type = reg.get("type", "INT16").upper()
            size = 2 if "32" in reg_type else 1
            max_addr = max(max_addr, addr + size)
            
        self.block_size = max_addr + 10
        self.data_block = [0] * self.block_size
        
        # Initialize default register values
        self.init_register_values()
        
    def load_profile(self) -> dict:
        log.info(f"Loading device profile: {self.profile_path}")
        with open(self.profile_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def write_val(self, address: int, val: float, reg_type: str, scale: float):
        # Apply inverse scale
        scaled_val = int(val / scale)
        reg_type = reg_type.upper()
        
        # Determine struct format based on type and endianness
        fmt_char = "h"  # default signed 16-bit
        if reg_type == "UINT16":
            fmt_char = "H"
        elif reg_type == "INT32":
            fmt_char = "i"
        elif reg_type == "UINT32":
            fmt_char = "I"
        elif reg_type == "FLOAT32":
            fmt_char = "f"

        prefix = ">" if self.byte_order == "BIG" else "<"
        
        try:
            packed = struct.pack(f"{prefix}{fmt_char}", scaled_val if reg_type != "FLOAT32" else float(scaled_val))
        except Exception as e:
            # Fallback to unsigned default on packing overflow
            packed = struct.pack(f"{prefix}h", 0)
            
        if len(packed) == 2:
            self.data_block[address] = int.from_bytes(packed, byteorder="big" if self.byte_order == "BIG" else "little", signed=False)
        elif len(packed) == 4:
            # Split 32-bit into two 16-bit registers
            r1 = int.from_bytes(packed[0:2], byteorder="big" if self.byte_order == "BIG" else "little", signed=False)
            r2 = int.from_bytes(packed[2:4], byteorder="big" if self.byte_order == "BIG" else "little", signed=False)
            
            if self.word_order == "BIG":
                self.data_block[address] = r1
                self.data_block[address + 1] = r2
            else:
                self.data_block[address] = r2
                self.data_block[address + 1] = r1

    def init_register_values(self):
        """Set logical defaults based on typical register tags."""
        for name, reg in self.registers.items():
            addr = reg.get("address")
            reg_type = reg.get("type", "INT16")
            scale = reg.get("scale", 1.0)
            tag = name.lower()
            
            # Default values depending on tag name
            val = 0.0
            if "soc" in tag or "stateofcharge" in tag:
                val = 75.0
            elif "soh" in tag or "stateofhealth" in tag:
                val = 98.5
            elif "frequency" in tag:
                val = 50.00
            elif "temp" in tag:
                val = 24.5
            elif "voltage" in tag or "volt" in tag:
                val = 380.0 if addr > 30000 else 230.0
            elif "activepower" in tag:
                val = 1500.0
            elif "running" in tag or "state" in tag:
                val = 256.0 # standard running enum for many inverters
                
            self.write_val(addr, val, reg_type, scale)

    async def simulation_loop(self, datablock_wrapper: ModbusSequentialDataBlock):
        """Simulate physical grid fluctuations and battery dynamics."""
        start_time = time.monotonic()
        while True:
            elapsed = time.monotonic() - start_time
            
            # 1. Fluctuating grid frequency around 50.00 Hz
            freq = 50.00 + 0.02 * math.sin(elapsed / 10.0) + random.uniform(-0.005, 0.005)
            
            # 2. Charging/Discharging SOC cycle (discharges during simulation)
            soc = 75.0 - (elapsed / 60.0) % 55.0
            
            # 3. Dynamic active power
            power = 1200.0 + 300.0 * math.sin(elapsed / 15.0)

            # 4. Temperature rising slightly
            temp = 24.5 + 2.0 * math.sin(elapsed / 120.0)

            for name, reg in self.registers.items():
                addr = reg.get("address")
                reg_type = reg.get("type", "INT16")
                scale = reg.get("scale", 1.0)
                tag = name.lower()
                
                val = None
                if "soc" in tag or "stateofcharge" in tag:
                    val = soc
                elif "frequency" in tag:
                    val = freq
                elif "activepower" in tag:
                    val = power
                elif "temp" in tag:
                    val = temp
                
                if val is not None:
                    self.write_val(addr, val, reg_type, scale)
            
            # Sync local datablock with the Modbus running store
            # ModbusSequentialDataBlock uses 1-based address offset matching start address
            datablock_wrapper.values = self.data_block[1:]
            
            await asyncio.sleep(2.0)

def main():
    parser = argparse.ArgumentParser(description="Universal Modbus BESS Simulator")
    parser.add_argument("--profile", type=str, default="profiles/huawei_sun2000.json", help="Path to device profile JSON")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Binding host address")
    parser.add_argument("--port", type=int, default=5020, help="Binding TCP port")
    args = parser.parse_args()

    # Validate path exists
    if not os.path.exists(args.profile):
        # Attempt to resolve from subdirectory
        local_path = os.path.join(os.path.dirname(__file__), args.profile)
        if os.path.exists(local_path):
            args.profile = local_path
        else:
            log.error(f"Profile path '{args.profile}' not found.")
            sys.exit(1)

    sim = UniversalBessSimulator(args.profile)
    
    # pymodbus holding registers offset: start at address 1 to allow direct 1-to-1 index matching
    store = ModbusSequentialDataBlock(1, sim.data_block[1:])
    slave1 = ModbusSlaveContext(di=store, co=store, hr=store, ir=store)
    
    # Create independent block and context for slave 3 to avoid share exceptions
    store3 = ModbusSequentialDataBlock(1, sim.data_block[1:])
    slave3 = ModbusSlaveContext(di=store3, co=store3, hr=store3, ir=store3)
    
    server_context = ModbusServerContext(devices={1: slave1, 3: slave3}, single=False)
    
    identity = ModbusDeviceIdentification()
    identity.VendorName = sim.device.get("manufacturer", "BESS Solutions")
    identity.ProductCode = "BESS-SIM-V2"
    identity.ModelName = sim.device.get("model", "Universal Inverter")

    # Start simulation loop task
    loop = asyncio.get_event_loop()
    loop.create_task(sim.simulation_loop(store))

    log.info(f"Starting Universal Modbus TCP Server on {args.host}:{args.port}")
    log.info(f"Simulating: {identity.VendorName} - {identity.ModelName}")
    
    loop.run_until_complete(
        StartAsyncTcpServer(
            context=server_context,
            identity=identity,
            address=(args.host, args.port)
        )
    )

# Compatibility fix: ModbusSlaveContext name can be import-dependent in some pymodbus versions
try:
    from pymodbus.datastore import ModbusSlaveContext
except ImportError:
    # Fallback to ModbusDeviceContext if SlaveContext doesn't exist
    ModbusSlaveContext = ModbusDeviceContext

if __name__ == "__main__":
    main()
