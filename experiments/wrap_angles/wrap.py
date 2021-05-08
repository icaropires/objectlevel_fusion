import math

i = -15
while i < 15:
    two_pi = math.pi * 2
    angle = math.pi*i

    rem = (angle + math.pi) % two_pi - math.pi
    # print(f"{angle:5.5f} {rem:5.5f} {math.cos(angle):5.5f} {math.cos(rem):5.5f}")
    print(f"{rem:5.5f}")

    i += 0.1
