"""
===============================================================================
Filename:  prepare_image.py

Brief: Script to convert raw binary file to OTAIMG format

Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.


Subject to your compliance with these terms, you may use Microchip
software and any derivatives exclusively with Microchip products.
It is your responsibility to comply with third party license terms applicable
to your use of third party software (including open source software) that
may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.


Support and FAQ:
    visit <a href="https://www.microchip.com/support/">Microchip Support</a>

===============================================================================
"""

#*******************************************************************************
# TYPE: "python prepare_image.py" for help (without quotes)
#*******************************************************************************


#
# Input: .BIN file compiled with relocated start address
#
# Output: .otaimg file to be used with actiltity added with
#         FotaImageHeader_t and appended with CRC8
#

# Imports
import sys
import os
import struct

#
# CONSTANTS
#
IMAGE_MAGIC = 0xCEFA34ED
FLASH_PAGE_SIZE = 64
#
IS_IMAGE_ENCRYPTED = False
IS_IMAGE_SIGNED = False
#
IMAGE_FLAGS_BITFIELD = 0x00000000
IMAGE_FLAGS_BITFIELD |= (0x000000001 & int(IS_IMAGE_ENCRYPTED))
IMAGE_FLAGS_BITFIELD |= (0x000000002 & int(IS_IMAGE_SIGNED))
#
STACK_VERSION = 0x01050000 # MLS_SDK_1_0_P_4
APP_VERSION = 0x000000002
HARDWARE_VERSION = 0x000000001

#
# typedef struct _FotaImageHeader
# {
    # uint32_t magic;             // I
    # uint8_t  fotaImgHdrLen;     // B
    # uint8_t  pad1;              // B
    # uint8_t  pad2;              // B
    # uint8_t  pad3;              // B
    # uint32_t stackVersion;      // I
    # uint32_t appVersion;        // I
    # uint32_t hwVersion;         // I
    # uint32_t flags;             // I
    # uint32_t fotaImgSize;       // I
    # uint8_t  crc8;              // B
    # uint8_t  pad4;              // B
    # uint8_t  pad5;              // B
    # uint8_t  pad6;              // B
# } FotaImageHeader_t;
#
FOTA_IMAGE_HEADER_FORMAT = ('<' +
                            'I'    + # magic
                            'BBBB' + # fotaImgHdrLen & padding
                            'I'    + # stackVersion
                            'I'    + # appVersion
                            'I'    + # hwVersion
                            'I'    + # flags
                            'I'    + # fotaImgSize
                            'BBBB'   # crc8 & padding
                           )

#
#
#
def crc8(text):
    """Calculates CRC8 for a list of 8-bit values.

    Keyword arguments:
    text -- (list) List of 8-bit text to generate CRC

    Return:
    (unsigned 8-bit integer) CRC8 for the list
    """
    crc = 0x00
    for i in text:
        # print(hex(i))
        crc ^= crc_chain(i, crc & 0xFF)
    return crc

#
#
#
def crc_chain(in_text, in_crc=0x00):
    """Computes the CRC for the given 8-bit text with given CRC8.

    Keyword arguments:
    in_text -- (unsigned 8-bit integer) input text for which CRC to be generated
    in_crc -- (unsigned 8-bit integer) initial CRC8 value

    Return:
    (unsigned 8-bit integer) CRC8 value for the given 8-bit text
    """
    crc = in_crc
    crc ^= in_text
    i = 0
    while i < 8:
        crc = (((crc << 1) & 0xFF) ^ 0x31) if bool(crc & 0x80) else ((crc << 1) & 0xFF)
        i += 1
    return crc

#
#
#
def file_crc8(bin_file):
    """Calculate CRC8 for the given bin file.

    Keyword arguments:
    bin_file -- (file path) full path of the binary file e.g.: /home/sample.bin (file_path)

    Return:
    (unsigned 8-bit integer) CRC8 for the input BIN file
    """
    crc = 0x00
    binary_file = open(bin_file, 'rb')
    binary_len = binary_file.seek(0, os.SEEK_END)
    binary_file.seek(0)
    index = 0
    while index < binary_len:
        a_byte = ord(binary_file.read(1))
        # print(a_byte)
        crc = crc_chain(a_byte, crc)
        index += 1
    return crc

#
#
#
def prep_image(file_path):
    """Prepares and writes the .OTAIMG file for the given .BIN file.

    Keyword arguments:
    file_path -- Absolute path of .BIN file

    Return:
        None
    """
    #
    image_crc = file_crc8(file_path)
    #
    binary_file = open(file_path, 'rb')
    binary_len = binary_file.seek(0, os.SEEK_END)
    #
    otaimg_file = os.path.dirname(file_path) + os.sep + \
                  os.path.basename(os.path.splitext(file_path)[0]) + '.otaimg'
    # print(otaimg_file)
    otaimg_file = open(otaimg_file, 'wb')
    #
    print("BIN size =", binary_len, "bytes (without header and CRC)")
    fota_image_header = struct.pack(FOTA_IMAGE_HEADER_FORMAT,
                                    IMAGE_MAGIC,
                                    struct.calcsize(FOTA_IMAGE_HEADER_FORMAT) - 4, 0xff, 0xff, 0xff,
                                    STACK_VERSION,
                                    APP_VERSION,
                                    HARDWARE_VERSION,
                                    IMAGE_FLAGS_BITFIELD,
                                    binary_len,
                                    0xff, 0xff, 0xff, 0xff
                                   )
    header_crc = crc8(fota_image_header[:-4])
    fota_image_header = struct.pack(FOTA_IMAGE_HEADER_FORMAT,
                                    IMAGE_MAGIC,
                                    struct.calcsize(FOTA_IMAGE_HEADER_FORMAT) - 4, 0xff, 0xff, 0xff,
                                    STACK_VERSION,
                                    APP_VERSION,
                                    HARDWARE_VERSION,
                                    IMAGE_FLAGS_BITFIELD,
                                    binary_len,
                                    header_crc, 0xff, 0xff, 0xff
                                   )
    #
    # print(list(map(hex, list(fota_image_header))))
    #
    otaimg_file.write(fota_image_header)
    page_fill = b'\xff' * (FLASH_PAGE_SIZE - struct.calcsize(FOTA_IMAGE_HEADER_FORMAT))
    otaimg_file.write(page_fill)
    #
    binary_file.close()
    binary_file = open(sys.argv[1], 'rb')
    binary_len = binary_file.seek(0, os.SEEK_END)
    binary_file.seek(0)
    index = 0
    while index < binary_len:
        otaimg_file.write(binary_file.read(1))
        index += 1
    #
    otaimg_file.write(bytes([image_crc]))
    print("OTAIMG size =", otaimg_file.seek(0, os.SEEK_END), "bytes (with header[64] and CRC[1])")
    otaimg_file.close()
    print("OTAIMG written")
    print("Image CRC =", hex(image_crc), "(without header)")
    #
    binary_file.close()

#
# Main block. Execution of this script starts here
#
if __name__ == '__main__':
    #
    if len(sys.argv) <= 1:
        print("""
Help:
$ python prepare_image <bin_file_path or bin_file>

bin_file_path -> absolute path to .bin file
bin_file      -> .bin file name"""
             )
        sys.exit()
    #
    else:
        prep_image(os.path.normpath(os.path.abspath(sys.argv[1])))

# EOF prepare_image.py
