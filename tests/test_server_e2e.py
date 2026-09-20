"""Test E2E real: arranca el servidor Modbus y lee registros por TCP."""
import sys
import os
import asyncio
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from server import UniversalBessSimulator, build_server_context, StartAsyncTcpServer
from pymodbus.client import AsyncModbusTcpClient

@pytest.mark.asyncio
async def test_server_starts_and_responds_to_modbus_client():
    prof_path = os.path.join(os.path.dirname(__file__), "..", "profiles", "huawei_sun2000.json")
    assert os.path.exists(prof_path)
    sim = UniversalBessSimulator(prof_path)
    ctx, store = build_server_context(sim)

    port = 25028
    server_task = asyncio.create_task(
        StartAsyncTcpServer(context=ctx, address=("127.0.0.1", port))
    )
    await asyncio.sleep(0.3)

    client = AsyncModbusTcpClient("127.0.0.1", port=port)
    connected = await client.connect()
    assert connected is True, "El cliente Modbus no pudo conectar al simulador"

    try:
        # address 32069 (ac_voltage, inicializado en 400V / 0.1 scale = 4000)
        res = await client.read_holding_registers(32069, count=1)
        assert not res.isError(), f"Error leyendo registro 32069: {res}"
        assert len(res.registers) == 1
        assert res.registers[0] == 4000
    finally:
        client.close()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass
