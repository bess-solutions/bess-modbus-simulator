#!/usr/bin/env python3
"""
server.py
=========
Universal Modbus TCP Simulator for BESS Solutions.
Loads any device profile JSON and dynamically populates holding registers,
simulating live telemetry with data-type and endianness awareness.
Fully compatible across pymodbus 3.8.x - 3.15.x.
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

# --- Pymodbus Cross-Version Compatibility Layer ---
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext
try:
    from pymodbus.datastore import ModbusDeviceContext as _ContextClass
except ImportError:
    from pymodbus.datastore import ModbusSlaveContext as _ContextClass

# Export both names to prevent NameError in all versions
ModbusDeviceContext = _ContextClass
ModbusSlaveContext = _ContextClass

try:
    from pymodbus.pdu.device import ModbusDeviceIdentification
except ImportError:
    try:
        from pymodbus.device import ModbusDeviceIdentification
    except ImportError:
        ModbusDeviceIdentification = None

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
            size = 2 if any(k in reg_type for k in ["32", "FLOAT"]) else 1
            max_addr = max(max_addr, addr + size)
            
        self.block_size = max_addr + 50
        self.data_block = [0] * self.block_size
        
        # Initialize default register values
        self.init_register_values()
        
    def load_profile(self) -> dict:
        log.info(f"Loading device profile: {self.profile_path}")
        with open(self.profile_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def write_val(self, addr: int, val: float, reg_type: str, scale: float = 1.0):
        if addr is None or addr <= 0:
            return
        
        reg_type = reg_type.upper()
        scaled_val = val / scale if scale != 0 else val
        
        bo = ">" if self.byte_order == "BIG" else "<"
        
        try:
            if reg_type in ["UINT16", "ENUM16"]:
                raw = int(max(0, min(65535, round(scaled_val))))
                self.data_block[addr] = raw
            elif reg_type == "INT16":
                raw = int(max(-32768, min(32767, round(scaled_val))))
                packed = struct.pack(f"{bo}h", raw)
                self.data_block[addr] = struct.unpack(f"{bo}H", packed)[0]
            elif reg_type in ["UINT32", "INT32"]:
                fmt = "I" if reg_type == "UINT32" else "i"
                raw = int(round(scaled_val))
                packed = struct.pack(f"{bo}{fmt}", raw)
                w0, w1 = struct.unpack(f"{bo}HH", packed)
                if self.word_order == "LITTLE":
                    w0, w1 = w1, w0
                self.data_block[addr] = w0
                self.data_block[addr + 1] = w1
            elif reg_type == "FLOAT32":
                packed = struct.pack(f"{bo}f", float(scaled_val))
                w0, w1 = struct.unpack(f"{bo}HH", packed)
                if self.word_order == "LITTLE":
                    w0, w1 = w1, w0
                self.data_block[addr] = w0
                self.data_block[addr + 1] = w1
        except Exception as e:
            log.warning(f"Failed to encode value {val} at address {addr} ({reg_type}): {e}")

    def init_register_values(self):
        """Populate initial holding register baseline values."""
        for name, reg in self.registers.items():
            addr = reg.get("address")
            reg_type = reg.get("type", "INT16")
            scale = reg.get("scale", 1.0)
            tag = name.lower()
            
            val = 0.0
            if "voltage" in tag:
                val = 400.0 if "ac" in tag or "grid" in tag else 800.0
            elif "current" in tag:
                val = 25.0
            elif "frequency" in tag:
                val = 50.00
            elif "soc" in tag or "stateofcharge" in tag:
                val = 75.0
            elif "soh" in tag or "stateofhealth" in tag:
                val = 98.5
            elif "temp" in tag:
                val = 24.5
            elif "activepower" in tag:
                val = 1500.0
            elif "running" in tag or "state" in tag:
                val = 256.0
                
            self.write_val(addr, val, reg_type, scale)

    async def simulation_loop(self, datablock_wrapper: ModbusSequentialDataBlock):
        """Simulate physical grid fluctuations and battery dynamics."""
        start_time = time.monotonic()
        while True:
            elapsed = time.monotonic() - start_time
            freq = 50.00 + 0.02 * math.sin(elapsed / 10.0) + random.uniform(-0.005, 0.005)
            soc = 75.0 - (elapsed / 60.0) % 55.0
            power = 1200.0 + 300.0 * math.sin(elapsed / 15.0)
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
            
            # Update values in datablock
            datablock_wrapper.values = self.data_block
            await asyncio.sleep(2.0)

def build_server_context(sim: UniversalBessSimulator):
    store1 = ModbusSequentialDataBlock(1, sim.data_block)
    slave1 = ModbusDeviceContext(di=store1, co=store1, hr=store1, ir=store1)
    
    store3 = ModbusSequentialDataBlock(1, sim.data_block)
    slave3 = ModbusDeviceContext(di=store3, co=store3, hr=store3, ir=store3)
    
    try:
        ctx = ModbusServerContext(devices={1: slave1, 3: slave3}, single=False)
    except TypeError:
        ctx = ModbusServerContext(slaves={1: slave1, 3: slave3}, single=False)
    return ctx, store1

async def run_server(args, sim, store, server_context, identity):
    loop_task = asyncio.create_task(sim.simulation_loop(store))
    log.info(f"Starting Universal Modbus TCP Server on {args.host}:{args.port}")
    if identity:
        log.info(f"Simulating: {getattr(identity, 'VendorName', 'BESS')} - {getattr(identity, 'ModelName', 'Sim')}")
    
    kwargs = {
        "context": server_context,
        "address": (args.host, args.port),
    }
    if identity is not None:
        kwargs["identity"] = identity
        
    try:
        await StartAsyncTcpServer(**kwargs)
    finally:
        loop_task.cancel()
        try:
            await loop_task
        except asyncio.CancelledError:
            pass

def main():
    parser = argparse.ArgumentParser(description="Universal Modbus BESS Simulator")
    parser.add_argument("--profile", type=str, default="profiles/huawei_sun2000.json", help="Path to device profile JSON")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Binding host address")
    parser.add_argument("--port", type=int, default=5020, help="Binding TCP port")
    args = parser.parse_args()

    if not os.path.exists(args.profile):
        local_path = os.path.join(os.path.dirname(__file__), args.profile)
        if os.path.exists(local_path):
            args.profile = local_path
        else:
            log.error(f"Profile path '{args.profile}' not found.")
            sys.exit(1)

    sim = UniversalBessSimulator(args.profile)
    server_context, store = build_server_context(sim)
    
    identity = None
    if ModbusDeviceIdentification is not None:
        identity = ModbusDeviceIdentification()
        identity.VendorName = sim.device.get("manufacturer", "BESS Solutions")
        identity.ProductCode = "BESS-SIM-V3"
        identity.ModelName = sim.device.get("model", "Universal Inverter")

    try:
        asyncio.run(run_server(args, sim, store, server_context, identity))
    except KeyboardInterrupt:
        log.info("Server terminated by user.")

if __name__ == "__main__":
    main()
