from pymodbus.client.sync import ModbusSerialClient
import time

client = ModbusSerialClient(
    method='rtu', 
    port='/dev/ttyUSB0',  
    baudrate=115200,
    parity = 'N', 
    stopbits = 1, 
    bytesize = 8,  
    timeout=1
    )

client.connect()

reply = client.write_register(8328, 100, unit=1)
if reply.isError():
    print("Failed")
else:
    print("Success")
    print(reply)

reply = client.read_holding_registers(8363, 2, unit=1)
if reply.isError():
    print("Failed")
else:
    print("Success")
    high = reply.registers[0]
    low = reply.registers[1]
    combined = (high << 16) + low

    if combined >= 0x80000000:
        combined -= 0x100000000

    print(f"Actual motor speed: {combined}")
    print(reply)
