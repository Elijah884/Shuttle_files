import can

def hex_conversion(hex_str):
	hex_list = list(hex_str)
	buffer_list = []
	output_list = []
	for i in range (len(hex_list)):
		buffer_list.append(hex_list[i].upper())
		if i%2 != 0:
			output_list.append(str(buffer_list[0])+str(buffer_list[1]))
			buffer_list.clear()
	output_str = " ".join(output_list)
	return output_str

def send_msg(bus):
	finished = False
	output_data = ""

	while not(finished):
		comm_type = input("Read (1) or Write (2)? ")
		if comm_type == "Read" or comm_type == "1":
			reg_size = input("Sizes: int8 (1), int16 (2), int32 (3), or float (4): ")
			if reg_size == "int8" or reg_size == "1":
				output_header = "10"
			elif reg_size == "int16" or reg_size == "2":
				output_header = "14"
			elif reg_size == "int32" or reg_size ==  "3":
				output_header = "18"
			elif reg_size == "float" or reg_size == "4":
				output_header = "1C"
			num_registers = input("Number of registers to read from: ")
			output_header = str(hex(int(output_header, 16) + int(num_registers)))[2:4]
			start_reg = input("Starting register number in hex format: ")
			output_data = str(str(output_data)+str(output_header)+str(start_reg))
			finished_input = input("Finished? False (0) or True (1): ")
			if int(finished_input) == 0:
				finished = False
			else:
				finished = True
		if comm_type == "Write" or comm_type == "2":
			reg_size = input("Sizes: int8 (1), int16 (2), int32 (3), or float (4): ")
			if reg_size == "int8" or reg_size ==  "1":
				output_header = "00"
			elif reg_size == "int16" or reg_size == "2":
				output_header = "04"
			elif reg_size == "int32" or reg_size ==  "3":
				output_header = "08"
			elif reg_size == "float" or reg_size ==  "4":
				output_header = "0C"
			num_registers = input("Number of registers to write to: ")
			output_header = str(hex(int(output_header, 16)+int(num_registers)))[2:4]
			if len(output_header) == 1:
				output_header = str("0"+output_header)
			start_reg = input("Starting register number in hex format: ")
			data_to_write = input("Value to write to these registers in hex format: ")
			output_data = str(str(output_data)+str(output_header)+str(start_reg)+str(data_to_write))
			finished_input = input("Finished? False (0) or True (1): ")
			if int(finished_input) == 0:
				finished = False
			else:
				finished = True

	print(output_data)

	msg = can.Message(
		arbitration_id=0x00008001,
		is_extended_id=True,
		is_fd=True,
		data=bytes.fromhex(output_data)
	)

	bus.send(msg)
	print("Sent: ", hex_conversion(output_data))

	response = bus.recv(timeout=1.0)
	if response:
		print("Received: ", hex_conversion(response.data.hex()))
	else:
		print("No Response :(")
	return response.data.hex()

def main():
	bus = can.interface.Bus(channel='can0', interface='socketcan', fd=True)
	send_msg(bus)
	bus.shutdown()

main()
