from pymavlink import mavutil
import sys
import time
import math

class Dolunay():

    SUCCESS = 0
    ERROR_OUT_OF_LOOP = 1

    def __init__(self, baglanti_modu):
        def baglan():
            platform = sys.platform.lower()
            print("İsletim Sistemi : " + str(platform))
            i = 0
            if baglanti_modu == "USB":
                while True:
                    print(i)
                    if platform == "linux":
                        port = '/dev/ttyACM' + str(i)
                    elif platform == "win32":
                        port = 'COM' + str(i)
                    else:
                        raise Exception("Bilinmeyen isletim sistemi ?")
                    try:
                        master = mavutil.mavlink_connection(port)
                        master.wait_heartbeat(blocking=False)
                        print("Arac ile baglanti kuruldu.")
                        break
                    except:
                        i += 1
                        if i > 20:
                            raise Exception(
                                "Arac ile baglanti kurulamadi.Port bilgisinde hata var yada kablo duzgun takilmamis olabilir.")
                        continue
            elif baglanti_modu == "SITL":
                port = 'udp:127.0.0.1:14550'
                try:
                    master = mavutil.mavlink_connection(port)
                    master.wait_heartbeat()
                    print("SITL ile baglanti kuruldu.")
                except:
                    raise Exception(
                        "SITL ile bağlanti kurulamadi port bilgisini kontrol edin.")
            else:
                raise Exception(
                    "Araca USB ya da SITL ile baglanmadan hareket komutlarini calistiramazsiniz.")
            return master
        self.master = baglan()
        pass

    def  hareket_et(self, x, y, z, r,t):
        """
        x- ileri ve geri hareket,[-1000,1000] araligi(0 degeri hareket vermez)
        y- saga ve sola hareket,[-1000,1000] araligi(0 degeri hareket vermez)
        z- yukari ve asagi hareket,[-1000,1000] araligi(500 degeri hareket vermez)
        r- kendi etrafinda saga ve sola hareket,[-1000,1000] araligi(0 degeri hareket vermez)
        t- hareketin kac saniye islecegi
        """
        # t kac saniye hareketi yapacagi
        while(True):
            self.master.mav.manual_control_send(
                self.master.target_system,
                x,#ileri,geri
                y,#sag gitme,sol gitme
                z,#yukari asagi default 500 degeri
                r,#etrafinda donme
                0  # Buton parametresi eger butona basılacaksa buton numarasina denk gelen bit degeri gonderilmeli
            )
            time.sleep(1)
            t=t-1
            if(t==0):
                break
        return

    def yunusbaligi(self):
        #duzenlecek
        return

    def set_arm(self, arm : bool = True, max_try : int = 3):
        """
        (DISARM) arm = False icin arac kendini disarm eder ve komutlar calismaz
        (ARM) arm = True icin arac kendini arm eder ve komut almaya hazirdir
        (max_try) bu degisken ile en fazla kac defa arm etme denemesi yapilmasi
        verilebilir. Eger verilen deneme sayısında istenilen sonuca ulasilamazsa
        0, basarılı olursa 1 geri dondurur
        """
        for _ in range(max_try):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                arm, 0, 0, 0, 0, 0, 0)

            self.master.wait_heartbeat(timeout=1)

            if arm and self.master.motors_armed() == mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("-> Armed")
                return self.SUCCESS
            elif not arm and self.master.motors_armed() == 0:
                print("-> Disarmed")
                return self.SUCCESS
        return self.ERROR_OUT_OF_LOOP

    def set_mod(self, mode):
        """
        Aracin modunu degistirmek icin kullanilir
        Ornek set_mod("ACRO") -> araci ACRO moda alir
        """
        mode = mode.upper()
        if mode not in self.master.mode_mapping():
            print(
                "'{}' modu bulunamadi.Yazim hatasina dikkat et.\nSTABILIZE moda gecilecek.".format(mode))
            mode = "MANUAL"

        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(self.master.target_system,
                                      mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                      mode_id)
        heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True)
        mode1 = list(self.master.mode_mapping().keys())[
            list(self.master.mode_mapping().values()).index(heartbeat.custom_mode)]
        print("Mod -> ", mode1)
        if(mode!=mode1):
            self.set_mod(mode)
        return

    def kapat(self):
        """
        Pixhawk ile olan baglanitiyi kapatir.
        """
        self.master.close()
        print("-> Baglanti kapatildi.")
