import sys
import os
import struct
import crcmod

# Check if the binary path is passed as a command-line argument
if len(sys.argv) < 3:
    print("Error: No binary file path provided.")
    sys.exit(1)

binary_file_path = sys.argv[1]  # Get the binary path from the command-line argument
arg2 = sys.argv[2]
start_address = int(arg2, 16)   # Get the vector table start address (physical)

# Print the binary path for debugging
print("Binary file path:", binary_file_path)

# Check if the file exists
if not os.path.exists(binary_file_path):
    print(f"Error: The file {binary_file_path} does not exist.")
    sys.exit(1)

# Constants (adjust as needed)
APP_HEADER_OFFSET = 0x188

# Function to calculate CRC32
def calculate_crc(data):
    # Initial CRC value for STM32
    crc = 0xFFFFFFFF
    polynomial = 0x04C11DB7

    # Process data in 32-bit chunks, as STM32 hardware does
    length = len(data)
    for i in range(0, length, 4):
        # Read 4 bytes as a 32-bit word in little-endian format
        word = struct.unpack('<I', data[i:i + 4].ljust(4, b'\x00'))[0]
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF  # Ensure CRC remains a 32-bit value

    return crc

# Read the binary file
with open(binary_file_path, "rb") as f:
    firmware_data = bytearray(f.read())

# Extract the address of the app_metadata from app_header
metadata_address_bytes = firmware_data[APP_HEADER_OFFSET:APP_HEADER_OFFSET + 4]
metadata_address = struct.unpack("<I", metadata_address_bytes)[0]

# Calculate the offset of app_metadata in the binary
metadata_offset = metadata_address - start_address

# Print the length of the data used for CRC calculation
print("Data length for CRC calculation:", metadata_offset)

# Calculate CRC excluding the metadata section
crc_value = calculate_crc(firmware_data[:metadata_offset])

# Update the CRC value in the metadata section
firmware_data[metadata_offset:metadata_offset + 4] = struct.pack("<I", crc_value)

# Write the updated binary back to the file
with open(binary_file_path, "wb") as f:
    f.write(firmware_data)

# Print the updated CRC as a full 32-bit value
print(f"Updated CRC: {crc_value:#010x} at address: {hex(metadata_address)}")
