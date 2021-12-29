import serial
import serial.tools.list_ports

CELL_INIT_COMMAND = "43 45 4C 4C 5F 45 4E 5F 49 4E 49 54 0D 0A"
CELL_SELECT_COMMAND = "43 45 4C 4C 5F 53 45 4C 0D 0A"
DOCK_CELL_empty_ON_COMMAND = "43 45 4C 4C %s 5F 4F 4E 0D 0A"
CELL_HW_INFORMATION = "AC C0 01 10 7D"
CELL_HW_INFORMATION_RESP_SIZE = 23
CELL_ENTER_DFU = "AC C0 01 D2 BF"
DOCK_CELL_RESET_FOR_ENTER_DFU = "43 45 4C 4C 5F 44 4F 46 0D 0A"
CELL_ALL_OFF_COMMAND = "43 45 4C 4C 5F 4F 46 32 0D 0A"

DEBUG_LOG_PRINT = 1


def hex_to_ascii(hex_name):
    bytes_object = bytes.fromhex(hex_name)
    ascii_name = bytes_object.decode("ASCII")
    return ascii_name


def get_hw_info(usart):
    # print("get hw info")
    retry_cnt = 0
    hex_buf = bytes.fromhex(CELL_HW_INFORMATION)
    while retry_cnt < 3:
        # try:
        # print("try here")
        usart.write(hex_buf)
        in_bin = usart.read(CELL_HW_INFORMATION_RESP_SIZE)
        # print("Print length of in_bin", len(in_bin))
        if len(in_bin) != CELL_HW_INFORMATION_RESP_SIZE:
            # print("Retry hw info")
            usart.write(hex_buf)
            in_bin = usart.read(CELL_HW_INFORMATION_RESP_SIZE)
            retry_cnt += 1
        else:
            # print("Get cell hw info")
            break
        # except usart.SerialTimeoutException:
        #     print("Didn't get any information from cell")
        #     retry_cnt += 1
    in_hex = hex(int.from_bytes(in_bin, byteorder='big'))

    # print("Printed all hw info : ", in_hex)

    serial_number = int(in_hex[10:14], 16)
    # firm_ver = str(int(in_hex[14:18], 16))
    major_firm_ver = str(int(in_hex[14:16], 16))
    minor_firm_ver = str(int(in_hex[16:18], 16))
    product_id = in_hex[18:22]
    version_id = in_hex[22:26]
    product_version = in_hex[30:34]

    product_id_ascii_string = hex_to_ascii(product_id)
    version_id_ascii_string = hex_to_ascii(version_id)
    product_version_ascii_string = hex_to_ascii(product_version)

    serial_number = str(serial_number)
    firm_ver = str(major_firm_ver) + "." + str(minor_firm_ver)
    # print(serial_number)
    # print("fimr_Ver check = ", firm_ver)

    # print(product_id_ascii_string)
    # print(version_id_ascii_string)
    # print(product_version_ascii_string)
    # print(serial_number)
    # print(firm_ver)

    return product_id_ascii_string, version_id_ascii_string, product_version_ascii_string, serial_number, firm_ver


def get_hub_com_port(target_vendor_id):
    hub_port_name = ''
    for port in serial.tools.list_ports.comports():
        if port.vid == target_vendor_id:
            hub_port_name = port.device
    if hub_port_name == '':
        print("Please make sure USB port is plug in.")
        raise Exception("Dock USB Port Not Found.")

    return hub_port_name


def get_cell_com_port(cell_vendor_id):
    cell_port_name = ''
    cell_port_list = []
    for port in serial.tools.list_ports.comports():
        if DEBUG_LOG_PRINT:
            print(port.vid)
            print(port.device)
            print(port.pid)
        if port.vid == cell_vendor_id:
            if DEBUG_LOG_PRINT:
                print("Find cell COM port")
            cell_port_name = port.device
            if DEBUG_LOG_PRINT:
                print(cell_port_name)
            cell_port_list.append(cell_port_name)
    if cell_port_name == '':
        print("Please make sure Cell plug into docking")
        # raise Exception("Cell USB port not found")

    return cell_port_list
