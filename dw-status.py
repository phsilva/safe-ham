import sys


def red(msg, end=None):
    print("\033[1;31m%s\033[0m" % msg, end=end)


reg = int(sys.argv[1], 16)

if len(sys.argv) >= 3:
    size = int(sys.argv[2])
else:
    size = 32

for i in range(size - 1, -1, -1):
    on = reg & (1 << i)
    if on:
        red("%02d" % i, end='|')
    else:
        print("%02d" % i, end='|')
print()

for i in range(size - 1, -1, -1):
    print("%2d" % ((reg >> i) & 0x1), end='|')
print()