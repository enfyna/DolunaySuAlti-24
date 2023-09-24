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

from os import sys, path

# Python normalde kendi bulundugu klasorun icindeki modulleri import edebilir.
# Ama suanda PixhawkOrange klasorundeki Dolunay sınıfını almak istiyoruz bu yuzden
# pythona o klasoru bulabilecegi dosya yolunu gostermemiz lazim.
# Komut satırında 'cd ..' komutunu kullanıyormus gibi dusunulebilinir.

# DolunaySuAlti-24 klasorunu bul
ROOT_PATH = '/'.join(path.abspath(__file__).split('/')[0:-2]) # linux

# path'a ekle
sys.path.append(ROOT_PATH)

# Dolunayı simdi import edebiliriz
from PixhawkOrange.PixhawkOrange import Dolunay

import cv2

cap = cv2.VideoCapture(0)
# kamerayı aç

window_x = int(cap.get(3))
# kameradan alınan görüntünün genişliğini al
# Aracı sadece saga sola cevirecegiz o yuzden yukseklige gerek yok

def TurnToRed() -> int:
	"""
	Kamerada görülen en büyük kırmızı cismi bulup
	araca ne kadar dönmesi gerektiğini söyle
	"""
	_, dispframe = cap.read()
	# Kameradan goruntu al
	frame = cv2.bitwise_not(dispframe)
	# Renkleri tersine cevir
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	# HSV'ye donustur
	frame = cv2.inRange(frame, self.lower_cyan, self.upper_cyan)
	# inRange ile camgobegi rengini bul

	contours, _ = cv2.findContours(
		frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# contour listesini al

	for c in contours:
		M = cv2.moments(c)
		# Moment hesabından c'nin alanını bul.

		# Anlik karedeki piksel alani en buyuk c'yi bul
		if M['m00'] > anlik_max_c_alani:
			secilen_c , scm , anlik_max_c_alani = c , M , M['m00']
			# Bulunan en buyuk c'yi anlik olarak kaydet
			# Not : scm -> secilen c momenti

	if secilen_c is not None:
		cx , cy = scm['m10']//scm['m00'] , scm['m01']//scm['m00']
		# Secilen c'nin momentinden merkezini hesapla

		return int(window_x - cx)
		# cisimle kameranın orta noktasındaki farkı gönder

	return None
	# c bulamadık o yüzden None döndür


def sign(x) -> int:
	return (x > 0) - (x < 0)


arac = Dolunay('USB')
# aracı olustur

arac.set_arm(1)
# arm et

try:
	# Gorev algoritmasını burada calıstıracagız

	hassasiyet = 1

	son_yon = 0
	bekleme_hizi = 100
	while True:
		# Aracı sonsuza kadar kırmızı
		# cismin oldugu yöne doğru çevir

		dr = TurnToRed()

		if dr is None:
			arac.hareket_et(0,0,0,son_yon * bekleme_hizi,1)
			# Kamera kırmızı bir cisim bulamamıs
			# o yüzden aracın dondugu son yone dogru
			# bekleme hizi ile don
		else:
			arac.hareket_et(0,0,0,dr * hassasiyet,1)
			# Aracın yönünü cisme doğru çevir

			son_yon = sign(dr)
			# Aracın dondugu son yonu kaydet

except KeyboardInterrupt:
	# Ctrl - C basarak görevi bitir
	pass

arac.set_arm(0)
# Aracı disarm et

arac.kapat()
# Baglantıyı kapat