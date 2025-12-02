#!/usr/bin/env python3
"""
Simple UF2 file format converter
Based on: https://github.com/microsoft/uf2
"""

import struct
import sys
import os

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30

def bytes_to_uint32_le(b):
    return struct.unpack('<I', b)[0]

def uint32_to_bytes_le(v):
    return struct.pack('<I', v)

class UF2File:
    def __init__(self, family_id=0xe48bff56):
        self.blocks = []
        self.family_id = family_id

    def add_file(self, binfile, base_addr):
        """Read binary file and create UF2 blocks"""
        with open(binfile, 'rb') as f:
            data = f.read()
        
        # Each UF2 block holds 256 bytes of data
        block_size = 256
        num_blocks = (len(data) + block_size - 1) // block_size
        
        for block_no in range(num_blocks):
            offset = block_no * block_size
            chunk = data[offset:offset + block_size]
            
            # Pad to 256 bytes
            chunk += b'\x00' * (block_size - len(chunk))
            
            target_addr = base_addr + offset
            self.blocks.append({
                'target_addr': target_addr,
                'payload': chunk,
                'block_no': block_no,
                'num_blocks': num_blocks,
            })

    def write(self, output_file):
        """Write UF2 file"""
        with open(output_file, 'wb') as f:
            for block in self.blocks:
                uf2_block = bytearray(512)
                
                # Magic numbers
                uf2_block[0:4] = uint32_to_bytes_le(UF2_MAGIC_START0)
                uf2_block[4:8] = uint32_to_bytes_le(UF2_MAGIC_START1)
                
                # Flags
                uf2_block[8:12] = uint32_to_bytes_le(0x00000000)
                
                # Target address
                uf2_block[12:16] = uint32_to_bytes_le(block['target_addr'])
                
                # Payload size (256 bytes)
                uf2_block[16:20] = uint32_to_bytes_le(256)
                
                # Block number
                uf2_block[20:24] = uint32_to_bytes_le(block['block_no'])
                
                # Total blocks
                uf2_block[24:28] = uint32_to_bytes_le(block['num_blocks'])
                
                # File size (family ID for RP2040)
                uf2_block[28:32] = uint32_to_bytes_le(self.family_id)
                
                # Data
                uf2_block[32:32+256] = block['payload']
                
                # End magic
                uf2_block[508:512] = uint32_to_bytes_le(UF2_MAGIC_END)
                
                f.write(uf2_block)

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Convert binary to UF2')
    parser.add_argument('-f', '--family', type=lambda x: int(x, 16), 
                        default=0xe48bff56, help='Family ID (hex)')
    parser.add_argument('-b', '--base', type=lambda x: int(x, 16),
                        default=0x10000000, help='Base address (hex)')
    parser.add_argument('input', help='Input binary file')
    parser.add_argument('-o', '--output', help='Output UF2 file')
    
    args = parser.parse_args()
    
    if not args.output:
        args.output = os.path.splitext(args.input)[0] + '.uf2'
    
    print(f"[UF2Conv] Input: {args.input}")
    print(f"[UF2Conv] Output: {args.output}")
    print(f"[UF2Conv] Base address: 0x{args.base:08X}")
    print(f"[UF2Conv] Family ID: 0x{args.family:08X}")
    
    uf2 = UF2File(family_id=args.family)
    uf2.add_file(args.input, args.base)
    uf2.write(args.output)
    
    print(f"[UF2Conv] Done! Wrote {len(uf2.blocks)} blocks")

if __name__ == '__main__':
    main()
