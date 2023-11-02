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

# Dolunayı simdi import edebiliriz
if len(argv) == 1:

	# DolunaySuAlti-24/PixhawkOrange klasorunu bul
	PO_PATH = '/'.join(abspath(__file__).split('/')[0:-2]) + '/PixhawkOrange' # linux

	# path'a ekle
	path.append(PO_PATH)
	
	from Dolunay import Dolunay
else:

	# DolunaySuAlti-24/DMS klasorunu bul
	DMS_PATH = '/'.join(abspath(__file__).split('/')[0:-2]) + '/DMS' # linux

	# path'a ekle
	path.append(DMS_PATH)

	from DolunaySim import Dolunay

arac = Dolunay()

from M1 import M1, State

mission = M1()

mission.SetMissionVehicle(arac)

# Gorev Kodu

arac.Pixhawk.set_arm(True)

arac.Pixhawk.set_mod('ACRO')
try:
	while True:
		# Gorev algoritmasını burada calıstıracagız
		move = mission.FindRed()
		# print(f'{State(mission.current_state)}')

		if move is not None:
			# print(f"mission move -> {move}")
			arac.Pixhawk.hareket_et(*move, 1, 0)
		else:
			# print("search  move -> (0, -1000, 500, 0)")
			arac.Pixhawk.hareket_et(0, -1000, 500, 0, 1, 0)
except Exception as e:
	import traceback
	traceback.print_exception(e)

arac.Pixhawk.set_arm(False)

arac.Camera.release_cams()
# connStation.close()