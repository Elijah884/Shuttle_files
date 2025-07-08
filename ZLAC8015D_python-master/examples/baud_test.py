from pymodbus.client.sync import ModbusSerialClient
import time
baud_list = [19200, 38400, 57600, 115200, 12800, 25600]
addr = 0x20AB
count = 2

for id in range(1, 128):
    for i in range(0, 5):

        client = ModbusSerialClient(
            method='rtu', 
            port='/dev/ttyUSB0',  
            baudrate=baud_list[i],
            parity = 'N', 
            stopbits = 1, 
            bytesize = 8,  
            timeout=1
            )

        client.connect()

        reply = client.read_holding_registers(
            addr, count, unit=id
        )
        if not reply.isError():
            print("Success")
            print(reply)
