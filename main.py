#!/usr/bin/env python
# This file is part of the OpenMV project.
# Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
# This work is licensed under the MIT license, see the file LICENSE for
# details.

"""This module implements enough functionality to program the STM32F4xx over
DFU, without requiring dfu-util.
See app note AN3156 for a description of the DFU protocol.
See document UM0391 for a dscription of the DFuse file.
"""
from __future__ import print_function

"""
cell power on (1번 경우)
CELL1_ON\r\n 43 45 4C 4C 31 5F 4F 4E 5C 72 5C 6E / 43 45 4C 4C 32 5F 4F 4E 5C 72 5C 6E
2. hw information
AC C0 01 10 7D
3. cell dfu entry
AC C0 01 D2 BF
4. dock board usb off/on
CELL_DOF\r\n(문자 전송) (edited)  43 45 4C 4C 5F 44 4F 46 5C 72 5C 6E
3:35
dfu 관련 명령어 입니다.
"""

import time
import argparse
import collections
import curses
import inspect
import re
import struct
import sys
from binascii import hexlify

import usb.core
import usb.util
import zlib
import serial
from tkinter import filedialog

from cell_dock_port import *

cell_cnt = 1
dfu_fail_num = []
dfu_fail_flag = 0

# USB request __TIMEOUT
__TIMEOUT = 25

# DFU commands
__DFU_DETACH = 0
__DFU_DNLOAD = 1
__DFU_UPLOAD = 2
__DFU_GETSTATUS = 3
__DFU_CLRSTATUS = 4
__DFU_GETSTATE = 5
__DFU_ABORT = 6

# DFU status
__DFU_STATE_APP_IDLE = 0x00
__DFU_STATE_APP_DETACH = 0x01
__DFU_STATE_DFU_IDLE = 0x02
__DFU_STATE_DFU_DOWNLOAD_SYNC = 0x03
__DFU_STATE_DFU_DOWNLOAD_BUSY = 0x04
__DFU_STATE_DFU_DOWNLOAD_IDLE = 0x05
__DFU_STATE_DFU_MANIFEST_SYNC = 0x06
__DFU_STATE_DFU_MANIFEST = 0x07
__DFU_STATE_DFU_MANIFEST_WAIT_RESET = 0x08
__DFU_STATE_DFU_UPLOAD_IDLE = 0x09
__DFU_STATE_DFU_ERROR = 0x0A

_DFU_DESCRIPTOR_TYPE = 0x21

_empty_offset = 0
_vector_table_size = 0

__DFU_STATUS_STR = {
    __DFU_STATE_APP_IDLE: "STATE_APP_IDLE",
    __DFU_STATE_APP_DETACH: "STATE_APP_DETACH",
    __DFU_STATE_DFU_IDLE: "STATE_DFU_IDLE",
    __DFU_STATE_DFU_DOWNLOAD_SYNC: "STATE_DFU_DOWNLOAD_SYNC",
    __DFU_STATE_DFU_DOWNLOAD_BUSY: "STATE_DFU_DOWNLOAD_BUSY",
    __DFU_STATE_DFU_DOWNLOAD_IDLE: "STATE_DFU_DOWNLOAD_IDLE",
    __DFU_STATE_DFU_MANIFEST_SYNC: "STATE_DFU_MANIFEST_SYNC",
    __DFU_STATE_DFU_MANIFEST: "STATE_DFU_MANIFEST",
    __DFU_STATE_DFU_MANIFEST_WAIT_RESET: "STATE_DFU_MANIFEST_WAIT_RESET",
    __DFU_STATE_DFU_UPLOAD_IDLE: "STATE_DFU_UPLOAD_IDLE",
    __DFU_STATE_DFU_ERROR: "STATE_DFU_ERROR",
}

# USB device handle
__dev = None

# Configuration descriptor of the device
__cfg_descr = None

__verbose = None

# USB DFU interface
__DFU_INTERFACE = 0

# Python 3 deprecated getargspec in favour of getfullargspec, but
# Python 2 doesn't have the latter, so detect which one to use
getargspec = getattr(inspect, "getfullargspec", inspect.getargspec)

if "length" in getargspec(usb.util.get_string).args:
    # PyUSB 1.0.0.b1 has the length argument
    def get_string(dev, index):
        return usb.util.get_string(dev, 255, index)


else:
    # PyUSB 1.0.0.b2 dropped the length argument
    def get_string(dev, index):
        return usb.util.get_string(dev, index)


def make_ff_by_counter(cnt):
    sum = b''
    for i in range(0, cnt):
        sum += b'\xff'
    return sum


def ascii2hex(source):
    return hex(ord(source))


def find_dfu_cfg_descr(descr):
    if len(descr) == 9 and descr[0] == 9 and descr[1] == _DFU_DESCRIPTOR_TYPE:
        nt = collections.namedtuple(
            "CfgDescr",
            [
                "bLength",
                "bDescriptorType",
                "bmAttributes",
                "wDetachTimeOut",
                "wTransferSize",
                "bcdDFUVersion",
            ],
        )
        return nt(*struct.unpack("<BBBHHH", bytearray(descr)))
    return None


def init(**kwargs):
    """Initializes the found DFU device so that we can program it."""
    global __dev, __cfg_descr
    devices = get_dfu_devices(**kwargs)
    print("len dev =", len(devices))
    if not devices:
        print("No DFU device found")
    if len(devices) > 1:
        print("Multiple DFU devices found")
    try:
        __dev = devices[0]
        __dev.set_configuration()
    except:
        return len(devices)

    # Claim DFU interface
    usb.util.claim_interface(__dev, __DFU_INTERFACE)

    # Find the DFU configuration descriptor, either in the device or interfaces
    __cfg_descr = None
    for cfg in __dev.configurations():
        __cfg_descr = find_dfu_cfg_descr(cfg.extra_descriptors)
        if __cfg_descr:
            break
        for itf in cfg.interfaces():
            __cfg_descr = find_dfu_cfg_descr(itf.extra_descriptors)
            if __cfg_descr:
                break

    # Get device into idle state
    for attempt in range(4):
        status = get_status()
        if status == __DFU_STATE_DFU_IDLE:
            break
        elif status == __DFU_STATE_DFU_DOWNLOAD_IDLE or status == __DFU_STATE_DFU_UPLOAD_IDLE:
            abort_request()
        else:
            clr_status()

    print("%s \n ABOVE CELL INIT DONE ! " % __dev)
    return len(devices)


def abort_request():
    """Sends an abort request."""
    __dev.ctrl_transfer(0x21, __DFU_ABORT, 0, __DFU_INTERFACE, None, __TIMEOUT)


def clr_status():
    """Clears any error status (perhaps left over from a previous session)."""
    __dev.ctrl_transfer(0x21, __DFU_CLRSTATUS, 0, __DFU_INTERFACE, None, __TIMEOUT)


def get_status():
    """Get the status of the last operation."""
    try:
        stat = __dev.ctrl_transfer(0xA1, __DFU_GETSTATUS, 0, __DFU_INTERFACE, 6, 25)

        # firmware can provide an optional string for any error
        if stat[5]:
            message = get_string(__dev, stat[5])
            if message:
                print(message)
        return stat[4]
    except:
        pass

    # return stat[4]


def check_status(stage, expected):
    global cell_cnt
    global dfu_fail_num
    global dfu_fail_flag

    status = get_status()
    if status != expected:
        time.sleep(0.025)
        # try one more check
        status = get_status()
        if status != expected:
            print("dfu failed cell num = ", cell_cnt)
            dfu_fail_num.append(cell_cnt)
            print("DFU: %s failed (%s)" % (stage, __DFU_STATUS_STR.get(status, status)))
            dfu_fail_flag = 1




def mass_erase():
    """Performs a MASS erase (i.e. erases the entire device)."""
    # Send DNLOAD with first byte=0x41
    __dev.ctrl_transfer(0x21, __DFU_DNLOAD, 0, __DFU_INTERFACE, "\x41", __TIMEOUT)

    # Execute last command
    check_status("erase", __DFU_STATE_DFU_DOWNLOAD_BUSY)

    # Check command state
    check_status("erase", __DFU_STATE_DFU_DOWNLOAD_IDLE)


def page_erase(addr):
    global cell_cnt
    global dfu_fail_num
    """Erases a single page."""
    if __verbose:
        print("Erasing page: 0x%x..." % (addr))

    # Send DNLOAD with first byte=0x41 and page address
    buf = struct.pack("<BI", 0x41, addr)
    try:
        __dev.ctrl_transfer(0x21, __DFU_DNLOAD, 0, __DFU_INTERFACE, buf, __TIMEOUT)
    except:
        dfu_fail_num.append(cell_cnt)
        pass

    # Execute last command
    check_status("erase", __DFU_STATE_DFU_DOWNLOAD_BUSY)

    # Check command state
    check_status("erase", __DFU_STATE_DFU_DOWNLOAD_IDLE)


def set_address(addr):
    global cell_cnt
    global dfu_fail_num
    """Sets the address for the next operation."""
    # Send DNLOAD with first byte=0x21 and page address
    buf = struct.pack("<BI", 0x21, addr)
    try:
        __dev.ctrl_transfer(0x21, __DFU_DNLOAD, 0, __DFU_INTERFACE, buf, __TIMEOUT)
    except:
        dfu_fail_num.append(cell_cnt)
        pass

    # Execute last command
    check_status("set address", __DFU_STATE_DFU_DOWNLOAD_BUSY)

    # Check command state
    check_status("set address", __DFU_STATE_DFU_DOWNLOAD_IDLE)


def write_memory(addr, buf, progress=None, progress_addr=0, progress_size=0):
    """Writes a buffer into memory. This routine assumes that memory has
    already been erased.
    """

    xfer_count = 0
    xfer_bytes = 0
    xfer_total = len(buf)
    xfer_base = addr

    global cell_cnt
    global dfu_fail_num

    while xfer_bytes < xfer_total:
        if __verbose and xfer_count % 512 == 0:
            print(
                "Addr 0x%x %dKBs/%dKBs..."
                % (xfer_base + xfer_bytes, xfer_bytes // 1024, xfer_total // 1024)
            )
        if progress and xfer_count % 2 == 0:
            progress(progress_addr, xfer_base + xfer_bytes - progress_addr, progress_size)

        # Set mem write address
        set_address(xfer_base + xfer_bytes)

        # Send DNLOAD with fw data
        chunk = min(8192, xfer_total - xfer_bytes)
        print("chunk size = ", chunk)

        try:
            __dev.ctrl_transfer(
                0x21, __DFU_DNLOAD, 2, __DFU_INTERFACE, buf[xfer_bytes: xfer_bytes + chunk], 25
            )
        except:
            dfu_fail_num.append(cell_cnt)
            break
        time.sleep(0.02)

        # Execute last command
        check_status("write memory", __DFU_STATE_DFU_DOWNLOAD_BUSY)

        # Check command state
        check_status("write memory", __DFU_STATE_DFU_DOWNLOAD_IDLE)

        xfer_count += 1
        xfer_bytes += chunk


def write_page(buf, xfer_offset):
    """Writes a single page. This routine assumes that memory has already
    been erased.
    """

    xfer_base = 0x08000000

    # Set mem write address
    set_address(xfer_base + xfer_offset)

    # Send DNLOAD with fw data
    __dev.ctrl_transfer(0x21, __DFU_DNLOAD, 2, __DFU_INTERFACE, buf, __TIMEOUT)

    # Execute last command
    check_status("write memory", __DFU_STATE_DFU_DOWNLOAD_BUSY)

    # Check command state
    check_status("write memory", __DFU_STATE_DFU_DOWNLOAD_IDLE)

    if __verbose:
        print("Write: 0x%x " % (xfer_base + xfer_offset))


def exit_dfu():
    """Exit DFU mode, and start running the program."""
    # Set jump address
    set_address(0x08000000)

    # Send DNLOAD with 0 length to exit DFU
    # __dev.ctrl_transfer(0x21, __DFU_DNLOAD, 0, __DFU_INTERFACE, None, __TIMEOUT)

    try:
        __dev.ctrl_transfer(0x21, __DFU_DNLOAD, 0, __DFU_INTERFACE, None, __TIMEOUT)
        # Execute last command
        if get_status() != __DFU_STATE_DFU_MANIFEST:
            print("Failed to reset device")

        # Release device
        usb.util.dispose_resources(__dev)
    except:
        pass


def named(values, names):
    """Creates a dict with `names` as fields, and `values` as values."""
    return dict(zip(names.split(), values))


def consume(fmt, data, names):
    """Parses the struct defined by `fmt` from `data`, stores the parsed fields
    into a named tuple using `names`. Returns the named tuple, and the data
    with the struct stripped off."""

    size = struct.calcsize(fmt)
    return named(struct.unpack(fmt, data[:size]), names), data[size:]


def cstring(string):
    """Extracts a null-terminated string from a byte array."""
    print("jaeuk1 ", str(hexlify(string), "utf-8"))
    return "ST..."


def compute_crc(data):
    """Computes the CRC32 value for the data passed in."""
    return 0xFFFFFFFF & -zlib.crc32(data) - 1


def read_dfu_file(filename):
    """Reads a DFU file, and parses the individual elements from the file.
    Returns an array of elements. Each element is a dictionary with the
    following keys:
        num     - The element index.
        address - The address that the element data should be written to.
        size    - The size of the element data.
        data    - The element data.
    If an error occurs while parsing the file, then None is returned.
    """
    _flag = 1
    vector_table_size = 0

    print("File: {}".format(filename))
    with open(filename, "rb") as fin:
        data = fin.read()
    crc = compute_crc(data[:-4])
    elements = []

    # Decode the DFU Prefix
    #
    # <5sBIB
    #   <   little endian           Endianness
    #   5s  char[5]     signature   "DfuSe"
    #   B   uint8_t     version     1
    #   I   uint32_t    size        Size of the DFU file (without suffix)
    #   B   uint8_t     targets     Number of targets
    dfu_prefix, data = consume("<5sBIB", data, "signature version size targets")
    print(
        "    %(signature)s v%(version)d, image size: %(size)d, "
        "targets: %(targets)d" % dfu_prefix
    )
    for target_idx in range(dfu_prefix["targets"]):
        # Decode the Image Prefix
        #
        # <6sBI255s2I
        #   <       little endian           Endianness
        #   6s      char[6]     signature   "Target"
        #   B       uint8_t     altsetting
        #   I       uint32_t    named       Bool indicating if a name was used
        #   255s    char[255]   name        Name of the target
        #   I       uint32_t    size        Size of image (without prefix)
        #   I       uint32_t    elements    Number of elements in the image
        img_prefix, data = consume(
            "<6sBI255s2I", data, "signature altsetting named name " "size elements"
        )
        print(img_prefix)
        img_prefix["num"] = target_idx
        print(img_prefix["named"])
        if img_prefix["named"]:
            print(img_prefix["name"])
            img_prefix["name"] = cstring(img_prefix["name"])
        else:
            img_prefix["name"] = ""
        print(
            "    %(signature)s %(num)d, alt setting: %(altsetting)s, "
            'name: "%(name)s", size: %(size)d, elements: %(elements)d' % img_prefix
        )

        target_size = img_prefix["size"]
        target_data = data[:target_size]
        data = data[target_size:]
        for elem_idx in range(img_prefix["elements"]):
            # Decode target prefix
            #
            # <2I
            #   <   little endian           Endianness
            #   I   uint32_t    element     Address
            #   I   uint32_t    element     Size

            elem_prefix, target_data = consume("<2I", target_data, "addr size")
            # print("elem_prefix, target_data ", elem_prefix, target_data)
            elem_prefix["num"] = elem_idx
            print("      %(num)d, address: 0x%(addr)08x, size: %(size)d" % elem_prefix)

            elem_size = elem_prefix["size"]
            print("###", elem_size)

            if _flag == 1:
                print("inhere")
                vector_table_size = elem_size
                _flag = 0

            elem_data = target_data[:elem_size]
            target_data = target_data[elem_size:]
            elem_prefix["data"] = elem_data
            elements.append(elem_prefix)

        if len(target_data):
            print("target %d PARSE ERROR" % target_idx)

    # Decode DFU Suffix
    #
    # <4H3sBI
    #   <   little endian           Endianness
    #   H   uint16_t    device      Firmware version
    #   H   uint16_t    product
    #   H   uint16_t    vendor
    #   H   uint16_t    dfu         0x11a   (DFU file format version)
    #   3s  char[3]     ufd         "UFD"
    #   B   uint8_t     len         16
    #   I   uint32_t    crc32       Checksum
    dfu_suffix = named(
        struct.unpack("<4H3sBI", data[:16]), "device product vendor dfu ufd len crc"
    )
    print(
        "    usb: %(vendor)04x:%(product)04x, device: 0x%(device)04x, "
        "dfu: 0x%(dfu)04x, %(ufd)s, %(len)d, 0x%(crc)08x" % dfu_suffix
    )
    if crc != dfu_suffix["crc"]:
        print("CRC ERROR: computed crc32 is 0x%08x" % crc)
        return
    data = data[16:]
    if data:
        print("PARSE ERROR")
        return

    return elements, vector_table_size


class FilterDFU(object):
    """Class for filtering USB devices to identify devices which are in DFU
    mode.
    """

    def __call__(self, device):
        for cfg in device:
            for intf in cfg:
                return intf.bInterfaceClass == 0xFE and intf.bInterfaceSubClass == 1


def get_dfu_devices(*args, **kwargs):
    """Returns a list of USB devices which are currently in DFU mode.
    Additional filters (like idProduct and idVendor) can be passed in
    to refine the search.
    """

    # Convert to list for compatibility with newer PyUSB
    return list(usb.core.find(*args, find_all=True, custom_match=FilterDFU(), **kwargs))


def get_memory_layout(device):
    """Returns an array which identifies the memory layout. Each entry
    of the array will contain a dictionary with the following keys:
        addr        - Address of this memory segment.
        last_addr   - Last address contained within the memory segment.
        size        - Size of the segment, in bytes.
        num_pages   - Number of pages in the segment.
        page_size   - Size of each page, in bytes.
    """

    cfg = device[0]
    intf = cfg[(0, 0)]
    mem_layout_str = get_string(device, intf.iInterface)
    mem_layout = mem_layout_str.split("/")
    result = []
    for mem_layout_index in range(1, len(mem_layout), 2):
        addr = int(mem_layout[mem_layout_index], 0)
        segments = mem_layout[mem_layout_index + 1].split(",")
        seg_re = re.compile(r"(\d+)\*(\d+)(.)(.)")
        for segment in segments:
            seg_match = seg_re.match(segment)
            num_pages = int(seg_match.groups()[0], 10)
            page_size = int(seg_match.groups()[1], 10)
            multiplier = seg_match.groups()[2]
            if multiplier == "K":
                page_size *= 1024
            if multiplier == "M":
                page_size *= 1024 * 1024
            size = num_pages * page_size
            last_addr = addr + size - 1
            result.append(
                named(
                    (addr, last_addr, size, num_pages, page_size),
                    "addr last_addr size num_pages page_size",
                )
            )
            addr += size
    return result


def list_dfu_devices(*args, **kwargs):
    """Prints a lits of devices detected in DFU mode."""
    devices = get_dfu_devices(*args, **kwargs)
    # if not devices:
    # raise SystemExit("No DFU capable devices found")

    for device in devices:
        print(
            "Bus {} Device {:03d}: ID {:04x}:{:04x}".format(
                device.bus, device.address, device.idVendor, device.idProduct
            )
        )
        layout = get_memory_layout(device)
        print("Memory Layout")
        for entry in layout:
            print(
                "    0x{:x} {:2d} pages of {:3d}K bytes".format(
                    entry["addr"], entry["num_pages"], entry["page_size"] // 1024
                )
            )


def write_elements(elements, mass_erase_used, progress=None):
    """Writes the indicated elements into the target memory,
    erasing as needed.
    """
    global dfu_fail_flag

    mem_layout = get_memory_layout(__dev)
    # for elem in elements:
    # print("@@@@@@@@", elements)
    addr = elements['addr']
    print("jaeuk 1", addr, type(addr))
    size = elements['size']
    data = elements['data']
    elem_size = size
    elem_addr = addr
    if progress and elem_size:
        progress(elem_addr, 0, elem_size)
    while size > 0 and dfu_fail_flag == 0:
        write_size = size
        if not mass_erase_used:
            print("page erase ")
            for segment in mem_layout:
                if addr >= segment['addr'] and addr <= segment['last_addr']:
                    # We found the page containing the address we want to
                    # write, erase it
                    page_size = segment['page_size']
                    page_addr = addr & ~(page_size - 1)
                    if addr + write_size > page_addr + page_size:
                        write_size = page_addr + page_size - addr
                    page_erase(page_addr)
                    break
        time.sleep(0.02)
        write_memory(addr, data[:write_size], progress, elem_addr, elem_size)
        data = data[write_size:]
        addr += write_size
        size -= write_size
        if progress:
            progress(elem_addr, addr - elem_addr, elem_size)


def cli_progress(addr, offset, size):
    """Prints a progress report suitable for use on the command line."""
    width = 25
    done = offset * width // size
    print(
        "\r0x{:08x} {:7d} [{}{}] {:3d}% ".format(
            addr, size, "=" * done, " " * (width - done), offset * 100 // size
        ),
        end="",
    )
    try:
        sys.stdout.flush()
    except OSError:
        pass  # Ignore Windows CLI "WinError 87" on Python 3.6
    if offset == size:
        print("")


def cell_usb_port_name_update():
    print()


def dfu_start(filepath):
    elements, vector_table_size = read_dfu_file(filepath)
    print("***", vector_table_size)
    if not elements:
        print("No data in dfu file")
        return
    print("Writing memory...")
    # print("elements = ", elements[0]['data'].type)

    _empty_offset = elements[1]['addr'] - elements[0]['addr']
    print("!!!", _empty_offset)
    empty_offset = _empty_offset - vector_table_size
    ff_offset_add_val = make_ff_by_counter(empty_offset)

    elements[0]['data'] += ff_offset_add_val

    print("___", elements[0]['data'])

    print("empty_offset = ", empty_offset)
    print(type(elements[0]['addr']))
    total_element = {'addr': elements[0]['addr'], 'size': elements[0]['size'] + elements[1]['size'],
                     'num': elements[0]['num'], 'data': elements[0]['data'] + elements[1]['data']}

    print("total_element", total_element['addr'], type(total_element['addr']), total_element['size'])
    write_elements(total_element, 0, progress=cli_progress)

    print("Exiting DFU...")
    exit_dfu()
    command_run = True
    return True


def main():
    global cell_cnt
    global dfu_fail_flag
    cell_dfu_on_process_flag = 0
    """Test program for verifying this files functionality."""

    # dfu_file_path = filedialog.askopenfilename()
    dfu_file_path = "/Users/jaeuklee/Downloads/X4-fifa-v1.1.dfu"

    while True:

        if cell_dfu_on_process_flag == 0:
            print("====== Start cell dfu mode process ======")

            print("====== Find dock port ======")
            hub_com_port = get_hub_com_port(1160)
            print("hub com list = ", hub_com_port)
            time.sleep(0.1)
            print("====== Turn on cell serial comm ======")
            hub_com_port_open = serial.Serial(hub_com_port, 115200)
            print("--cell init--")
            hub_to_cell_command = bytes.fromhex(CELL_INIT_COMMAND)
            hub_com_port_open.write(hub_to_cell_command)
            time.sleep(0.1)
            print("--cell select--")
            hub_to_cell_command = bytes.fromhex(CELL_SELECT_COMMAND)
            hub_com_port_open.write(hub_to_cell_command)
            time.sleep(0.1)
            print("--cell on--")
            hex_val_string = ascii2hex(str(cell_cnt))
            print("cell count next num = ", hex_val_string[2:])
            DOCK_CELL_ON_COMMAND = (DOCK_CELL_empty_ON_COMMAND % hex_val_string[2:])
            hub_to_cell_command = bytes.fromhex(DOCK_CELL_ON_COMMAND)
            hub_com_port_open.write(hub_to_cell_command)
            time.sleep(5)

            print("====== Read cell infomation ======")
            cell_com_port = get_cell_com_port(1155)
            print(cell_com_port)
            if len(cell_com_port) == 0:
                for i in range(0, 5):
                    hub_to_cell_off_command = bytes.fromhex(CELL_ALL_OFF_COMMAND)
                    hub_com_port_open.write(hub_to_cell_off_command)
                    time.sleep(0.2)
                print("Cell OFF")
                raise SystemExit("Next cell place is empty exit DFU program")

            cell_com_port_open = serial.Serial(cell_com_port[0], 115200)
            product_id, version_id, product_version, production_number, firm_ver = get_hw_info(cell_com_port_open)
            print(product_id)
            print(version_id)
            print(product_version)
            print(production_number)
            print(firm_ver)
            time.sleep(5)
            print("====== Enter to cell dfu mode ======")
            hex_buf1 = bytes.fromhex(CELL_ENTER_DFU)
            cell_com_port_open.write(hex_buf1)
            time.sleep(5)
            print("====== Cell power reset ======")
            hex_buf2 = bytes.fromhex(DOCK_CELL_RESET_FOR_ENTER_DFU)
            hub_com_port_open.write(hex_buf2)
            time.sleep(5)

        print("check dfu device on?")
        list_dfu_devices()
        device_len_check = init()
        if device_len_check > 0:
            cell_dfu_on_process_flag = 1

        if cell_dfu_on_process_flag:
            dfu_fail_flag = 0
            print("loop count = ", cell_cnt)

            list_dfu_devices()
            # print("here1")
            device_len = init()
            # print("here2")

            if device_len == 1:
                print("One cell - [ Start DFU process ] ")
                if dfu_start(dfu_file_path):
                    print("CELL DFU SUCCESS!")
                    print("move to next cell dfu")
                    cell_dfu_on_process_flag = 0

                # Finish one cell DFU process, ping to dock for next cell update
            elif device_len > 1:
                print("DFU mode cell > 1")
            else:
                print("len(devices) = 0")

            time.sleep(3)
            cell_cnt += 1

        # command_run = False
        # if args.mass_erase:
        #     print("Mass erase...")
        #     mass_erase()
        #     command_run = True
        #
        # if args.path:
        #     elements, vector_table_size = read_dfu_file(args.path)
        #     print("***", vector_table_size)
        #     if not elements:
        #         print("No data in dfu file")
        #         return
        #     print("Writing memory...")
        #     # print("elements = ", elements[0]['data'].type)
        #
        #     _empty_offset = elements[1]['addr'] - elements[0]['addr']
        #     print("!!!", _empty_offset)
        #     empty_offset = _empty_offset - vector_table_size
        #     ff_offset_add_val = make_ff_by_counter(empty_offset)
        #
        #     elements[0]['data'] += ff_offset_add_val
        #
        #     print("___", elements[0]['data'])
        #
        #     print("empty_offset = ", empty_offset)
        #     print(type(elements[0]['addr']))
        #     total_element = {'addr':elements[0]['addr'], 'size':elements[0]['size']+elements[1]['size'],
        #                                      'num':elements[0]['num'], 'data':elements[0]['data'] + elements[1]['data']}
        #
        #     print("total_element", total_element['addr'], type(total_element['addr']), total_element['size'])
        #     write_elements(total_element, args.mass_erase, progress=cli_progress)
        #
        #     print("Exiting DFU...")
        #     exit_dfu()
        #     command_run = True
        #
        # if args.exit:
        #     print("Exiting DFU...")
        #     exit_dfu()
        #     command_run = True
        #
        # if command_run:
        #     print("Finished")
        # else:
        #     print("No command specified")


if __name__ == "__main__":
    main()
