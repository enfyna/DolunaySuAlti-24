from pymavlink import mavutil
import sys
import time
import math

class Dolunay():
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

    def set_arm(self, i):
        """
        (DISARM)i=0 icin arac kendini disarm eder ve komutlar calismaz
        (ARM)i=1 icin arac kendini arm eder ve komut almaya hazirdir
        """
        conttime=3
        while True:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                i, 0, 0, 0, 0, 0, 0)
            time.sleep(1)
            conttime=conttime-1
            if(conttime==0):
                break
        if i == 0:
            self.master.motors_disarmed_wait()
            print("-> Disarm")
        else:
            self.master.motors_armed_wait()
            print("-> Arm")
        return

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
