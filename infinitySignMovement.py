from PixhawkOrange.PixhawkOrange import PixhawkOrange
from math import degrees
from time import sleep

from sys import argv

print(argv)


arac = PixhawkOrange()

arac.set_arm(True)

while arac.set_mod("Acro") != 0:
    continue

print(arac.getData())

ilk_yaw = (degrees(arac.get_attitude()["yaw"]))
yon = 1
yon_degisti_mi = False
try:
    while True:
        current_yaw = degrees(arac.get_attitude().get('yaw'))
        fark = abs(ilk_yaw - (current_yaw))
        print(current_yaw, "-", ilk_yaw, "=", fark, "yon:", yon_degisti_mi)

        if not yon_degisti_mi and fark < 5:
            yon *= -1
            yon_degisti_mi = True
        else:
            if fark > 50:
                yon_degisti_mi = False
        
        arac.hareket_et(
            1000,
            0,
            0,
            500 * yon,
            1,
        )
except KeyboardInterrupt:
    pass

arac.set_arm(False)

print(arac.current_arm_state)

print(arac.getData())