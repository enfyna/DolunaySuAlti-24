#
#                           Gorev 1
#
#   Aracı havuzun zemininde bulunan kırmızı daireye kondur.
#
#
#

from os.path import abspath
from sys import path, argv

# Python normalde kendi bulundugu klasorun icindeki modulleri import edebilir.
# Ama suanda PixhawkOrange klasorundeki Dolunay sınıfını almak istiyoruz bu yuzden
# pythona o klasoru bulabilecegi dosya yolunu gostermemiz lazim.
# Komut satırında 'cd ..' komutunu kullanıyormus gibi dusunulebilinir.

# DolunaySuAlti-24 klasorunu bul
ROOT_PATH = '/'.join(abspath(__file__).split('/')[0:-2]) # linux

# path'a ekle
path.append(ROOT_PATH)

# Dolunayı simdi import edebiliriz
if len(argv) == 1:
	from PixhawkOrange.PixhawkOrange import Dolunay
else:
	from DMS.DolunaySim import Dolunay

arac = Dolunay()

from M1 import M1, State

mission = M1()

# Gorev Kodu

arac.set_arm(True)

arac.set_mod('ACRO')
try:
	while True:
		# Gorev algoritmasını burada calıstıracagız

		is_front_cam, frame_front = arac.get_front_cam()
		is_bottom_cam, frame_bottom = arac.get_bottom_cam()

		if not is_front_cam or not is_bottom_cam:
			continue

		move = mission.FindRed(frame_front, frame_bottom)
		print(f'{State(mission.current_state)}')

		if move is not None:
			# print(f"mission move -> {move}")
			arac.hareket_et(*move, 1, 0)
		else:
			# print("search  move -> (0, -1000, 500, 0)")
			arac.hareket_et(0, -1000, 500, 0, 1, 0)
except Exception as e:
	print(f'Exception: {e}')

arac.set_arm(False)

arac.release_cams()
# connStation.close()