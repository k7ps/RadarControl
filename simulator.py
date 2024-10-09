import os
import subprocess
import time
import sys

separator = ';'

def print_to_pipe(pipename, message):
    with open("input.fifo", "w") as pipe:
        pipe.write(message + separator)

print(f"Pyt: started")

pid = os.fork()
if pid == 0:
    os.system("cd build && make")
    os.system("./build/RadarControl")
else:
    time.sleep(1)

    for i in "12340":
        with open("input.fifo", "w") as pipe:
            print(f"Pyt: print {i}")
            pipe.write(i)
        time.sleep(1)

    print(f"Pyt: stopped")
