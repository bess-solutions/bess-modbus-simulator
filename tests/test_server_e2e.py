"""Test E2E real: arranca el servidor Modbus y lee registros por TCP con puerto efímero."""
import sys
import os
import asyncio
import socket
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from server import UniversalBessSimulator, build_server_context, StartAsyncTcpServer
from pymodbus.client import AsyncModbusTcpClient

def get_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]

@pytest.mark.asyncio
async def test_server_starts_and_responds_to_modbus_client():
    prof_path = os.path.join(os.path.dirname(__file__), "..", "profiles", "huawei_sun2000.json")
    assert os.path.exists(prof_path)
    sim = UniversalBessSimulator(prof_path)
    ctx, store = build_server_context(sim)

    # Asignación de puerto efímero para evitar colisiones y estados TIME_WAIT en CI
    port = get_free_port()
    server_task = asyncio.create_task(
        StartAsyncTcpServer(context=ctx, address=("127.0.0.1", port))
    )
    await asyncio.sleep(0.3)

    client = AsyncModbusTcpClient("127.0.0.1", port=port)
    connected = await client.connect()
    assert connected is True, f"El cliente Modbus no pudo conectar al simulador en puerto {port}"

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
