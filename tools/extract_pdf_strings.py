import sys
import re
from pathlib import Path

p = Path('K-MD7 - Datasheet.pdf')
b = p.read_bytes()

# find printable ascii substrings of length >=4
import string
printable = set(bytes(string.printable, 'ascii'))
subs = []
cur = bytearray()
for byte in b:
    if byte in printable and byte >= 32:
        cur.append(byte)
    else:
        if len(cur) >= 4:
            subs.append(cur.decode('ascii', errors='ignore'))
        cur = bytearray()
if len(cur) >= 4:
    subs.append(cur.decode('ascii', errors='ignore'))

# print lines that look relevant
keywords = ['protocol','frame','CRC','checksum','start','preamble','RESP','RESA','RES','CMD','command','payload','length','baud','COM']
for s in subs:
    low = s.lower()
    for k in keywords:
        if k.lower() in low:
            print(repr(s))
            break

# also print top 200 printable substrings for manual inspection
print('\n--- TOP 200 PRINTABLE SUBSTRINGS ---')
for i, s in enumerate(subs[:200]):
    print(i, repr(s))
