import PixhawkOrange
import time

arac = PixhawkOrange.Dolunay("USB")
arac.set_mod("ACRO")
time.sleep(5)
try:
    arac.set_arm(1)
    arac.hareket_et(0, 0, 0, 500, 3)
    arac.hareket_et(0, 0, 0, -500, 3)
    arac.set_arm(0)
    time.sleep(1)
    arac.kapat()
except Exception as e:
    print(e)
    arac.set_arm(0)
    arac.kapat()
arac.kapat()