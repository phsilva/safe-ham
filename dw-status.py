import sys

reg = int(sys.argv[1], 16)

if len(sys.argv) >= 3:
    size = int(sys.argv[2])
else:
    size = 32

for i in range(size - 1, -1, -1):
    print("%02d" % i, end='|')
print()

for i in range(size - 1, -1, -1):
    print("%2d" % ((reg >> i) & 0x1), end='|')
print()