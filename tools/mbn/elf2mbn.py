#!/usr/bin/python3

import os
import argparse
import mbn_tools

parser = argparse.ArgumentParser()
parser.add_argument("-i", dest="in_file", help="Input file (.elf)", required=True)
parser.add_argument("-o", dest="out_file", help="Output file (.mbn)", required=True)
options = parser.parse_args()

base = os.path.splitext(str(options.out_file))[0]
hash = base + ".hash"
hash_header = base + ".hash_header"
hash_seg = base + ".combined_hash"
pboot_elf = base + ".pboot"

gen_dict = {}
gen_dict['IMAGE_KEY_IMAGE_ID'] = mbn_tools.ImageType.SBL1_IMG
gen_dict['IMAGE_KEY_MBN_TYPE'] = 'elf'

# Create stripped elf and hash segment
rv = mbn_tools.pboot_gen_elf([],
                options.in_file,
                hash,
                elf_out_file_name = pboot_elf,
                secure_type = 'non_secure',
                header_version = 6)
if rv:
    raise(RuntimeError, "Failed to run pboot_gen_elf")

# Create hash segment header
rv = mbn_tools.image_header([],
                gen_dict,
                hash,
                hash_header,
                secure_type = 'non_secure',
                elf_file_name = pboot_elf,
                header_version = 6)
if rv:
    raise(RuntimeError, "Failed to create image header for hash segment")

# Create combined hash segment
mbn_tools.concat_files (hash_seg, [hash_header, hash])

# Add the hash segment into the ELF
rv = mbn_tools.pboot_add_hash([], pboot_elf, hash_seg, options.out_file)
if rv:
    raise(RuntimeError, "Failed to add hash segment to elf")

