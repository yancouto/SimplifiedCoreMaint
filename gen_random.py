import random

with open("experiment/graph/random-1m.txt", "w") as f:
    for i in range(1, 1000000):
        f.write(str(random.randint(0, (1 << 31) - 1)) + '\n')
