#
#           Kısa bir deneme gorevi
#
#   Aracın kamerasından kırmızı renkli bir cisim
#   goruldugunde aracı bulundugu yerde sag-sol
#   yaparak cisme dogru yonlendir
#
#   Bu kodu atölyenin yanındaki su deposunda,
# 	aracın kamerasıyla hem görüntü işleme hem de
# 	aracın dr hassasiyetini test etmek için yazdım.
#

from os.path import abspath
from sys import path

# Python normalde kendi bulundugu klasorun icindeki modulleri import edebilir.
# Ama suanda PixhawkOrange klasorundeki Dolunay sınıfını almak istiyoruz bu yuzden
# pythona o klasoru bulabilecegi dosya yolunu gostermemiz lazim.
# Komut satırında 'cd ..' komutunu kullanıyormus gibi dusunulebilinir.

# DolunaySuAlti-24 klasorunu bul
ROOT_PATH = '/'.join(abspath(__file__).split('/')[0:-2]) # linux

# path'a ekle
path.append(ROOT_PATH)

# Dolunayı simdi import edebiliriz
from PixhawkOrange.PixhawkOrange import Dolunay

import numpy as np
import cv2

cap = cv2.VideoCapture(0)
# kamerayı aç

window_x_half = int(cap.get(3) / 2)
# kameradan alınan görüntünün genişliğini al

# Aracı sadece saga sola cevirecegiz o yuzden yukseklige gerek yok

lower_cyan = np.array([40, 50, 0], int)
upper_cyan = np.array([100, 255, 255], int)
# Renk aralıgı

def TurnToRed() -> int:
	"""
	Kamerada görülen en büyük kırmızı cismi bulup
	araca ne kadar dönmesi gerektiğini söyle
	"""
	_, dispframe = cap.read()
	# Kameradan goruntu al
	# frame = cv2.resize(dispframe,(200,150))
	# kucult
	frame = cv2.bitwise_not(dispframe)
	# Renkleri tersine cevir
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	# HSV'ye donustur
	frame = cv2.inRange(frame, lower_cyan, upper_cyan)
	# inRange ile camgobegi rengini bul
	cv2.imshow('inrange',frame)
	# frame = cv2.threshold(frame, 40, 200, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
	# cv2.imshow('threshold',frame)
	

	# cv2.imshow('cam',dispframe)
	cv2.waitKey(24)

	contours, _ = cv2.findContours(
		frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# contour listesini al

	if len(contours) == 0:
		return None
		# c bulamadık o yüzden None döndür

	anlik_max_c_alani, secilen_c = 0.0, None

	for c in contours:
		M = cv2.moments(c)
		# Moment hesabından c'nin alanını bul.

		# Anlik karedeki piksel alani en buyuk c'yi bul
		if M['m00'] > anlik_max_c_alani:
			secilen_c , scm , anlik_max_c_alani = c , M , M['m00']
			# Bulunan en buyuk c'yi anlik olarak kaydet
			# Not : scm -> secilen c momenti

	cx = int(scm['m10'] / scm['m00'])
	# Secilen c'nin momentinden merkezini hesapla
	cv2.drawContours(dispframe,[secilen_c],-1, 0xFF0, cv2.FILLED)
	# cv2.circle(dispframe,(cx,50),1,0xFFF,3)
	cv2.imshow('cont',dispframe)
	return cx - window_x_half
	# cisimle kameranın orta noktasındaki farkı gönder


arac = Dolunay('USB')
# aracı olustur

arac.set_arm(1)
# arm et

arac.set_mod('ACRO')
# mod ayarla

try:
	# Gorev algoritmasını burada calıstıracagız

	hassasiyet = 1

	while True:
		# Aracı sonsuza kadar kırmızı
		# cismin oldugu yöne doğru çevir

		dr = TurnToRed()

		if dr is not None:
			arac.hareket_et(0, 0, 500, dr * hassasiyet, 1, 0)
			# Aracın yönünü cisme doğru çevir
		else:
			arac.hareket_et(0, 0, 500, 0, 1, 0)
			# kırmızı cisim bulamamısız bekle

except KeyboardInterrupt:
	# Ctrl - C basarak görevi bitir
	pass

arac.set_arm(0)
# Aracı disarm et

arac.kapat()
# Baglantıyı kapat