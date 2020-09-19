#!/usr/bin/env python3

"""
This program converts the hex output of objcopy to a format verilogs readmemh
task understands. This also includes reversing the byteorder in each memory
word.

Usage: fmt_hex.py filename [word_length_in_byte]
"""

import sys
from textwrap import wrap
import re


def group_by_word(word_length_in_byte: int,
                  current_grouping: int,
                  bytestream_as_hex_str: str) -> str:
    """
    :param word_length_in_byte: Number of bytes in a word in the resulting
                                output
    :param current_grouping: Current word length in half bytes (the same as
                             amount of hexadecimal numbers per word)
    :param bystestream_as_hex_str: Current bytestream like it was put out by
                                   objcopy minus the @ parts
    :returns: bytestream as hex str, with one word in each line.
    """
    bytestream_as_word_list = wrap(bytestream_as_hex_str,
                                   (word_length_in_byte * 2)
                                    + ((word_length_in_byte * 2)
                                    / current_grouping))
    return '\n'.join(bytestream_as_word_list).replace(' ', '')


def reverse_endian(formatted_bytestream: str) -> str:
    """
    :param formatted_bytestream: The bytestream as str with one hex word in
                                 each line
    :returns: The bytestream as str with one hex word in each line but with
              it's byte order reversed
    """
    bytestream_as_word_list = formatted_bytestream.split()
    reversed_bytestream = ''
    for word in bytestream_as_word_list:
        reversed_bytestream += ''.join([word[x:x + 2] for x in range(0,
                                                                     len(word),
                                                                     2)][::-1])
        reversed_bytestream += '\n'
    return reversed_bytestream


def get_current_grouping(bytestream_as_hex_str: str) -> int:
    """
    :param bytestream_as_hex_str: objcopy hex output minus the @ parts
    :returns: Word length in half bytes (The same as hexadecimal numbers per
              word)
    """
    hex_in_group = 0
    for char in bytestream_as_hex_str:
        if char != ' ':
            hex_in_group += 1
        else:
            break
    return hex_in_group


def pad_bytestream(formatted_bytestream: str,
                   amount_to_pad: int,
                   word_length_in_byte: int) -> str:
    """
    :param formatted_bytestream: processed bytestream as one hex word per line
    :param amount_to_pad: amount of words to add before the program
    :param word_length_in_bytes: word length in byte
    :returns: padded bytestream
    """
    line = ''
    for i in range(word_length_in_byte):
        line += '00'
    for i in range(int(amount_to_pad / word_length_in_byte)):
        formatted_bytestream = line + '\n' + formatted_bytestream
    return formatted_bytestream


# process arguments
filename = str(sys.argv[1])
if len(sys.argv) > 2:
    word_length_in_byte = int(sys.argv[2])
else:
    word_length_in_byte = 4

# open file
with open(filename) as f:
    hex_file = f.read()

# determine amount to pad in front of the actual program
amount_to_pad = ''
if (hex_file[0] == '@'):
    for char in hex_file:
        if (char == '\n'):
            break
        else:
            amount_to_pad += char

# remove adress information (the @ parts)
hex_file = re.sub('@[a-zA-Z0-9_]*\n', '', hex_file)

# process bytestream
hex_file = group_by_word(word_length_in_byte,
                         get_current_grouping(hex_file),
                         hex_file)

hex_file = reverse_endian(hex_file)

if (amount_to_pad != ''):
    hex_file = pad_bytestream(hex_file,
                              int(amount_to_pad[1:], 16),
                              word_length_in_byte)

# write bytestream to file
with open(filename, "w") as f:
    f.write(hex_file)
