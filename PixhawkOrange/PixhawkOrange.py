from pymavlink import mavutil
from time import sleep

class Dolunay():

    SUCCESS = 0
    ERROR_OUT_OF_LOOP = 1

    def __init__(self, baglanti_modu):
        if baglanti_modu == "USB":

            from sys import platform

            OS = platform.lower()
            print(f"İsletim Sistemi : {OS}")

            if OS == "linux":
                port = '/dev/ttyACM'
            elif OS == "win32":
                port = 'COM'
            else:
                raise Exception("Bilinmeyen isletim sistemi ?")

            for idx in range(21):
                try:
                    master = mavutil.mavlink_connection(f'{port}{idx}')
                    master.wait_heartbeat(blocking=False)
                    print("Arac ile baglanti kuruldu.")
                    break
                except:
                    continue
            else:
                raise Exception(
                    "Arac ile baglanti kurulamadi."
                    "Port bilgisinde hata var yada kablo duzgun takilmamis olabilir."
                )
        elif baglanti_modu == "SITL":
            port = 'udp:127.0.0.1:14550'
            master = mavutil.mavlink_connection(port)
            master.wait_heartbeat(timeout = 3)
            if master.mav_count > 0:
                # sitl ile baglandiysak heartbeat mesajını almış olmamız gerek
                # heartbeat mesajı aldık isek 'mav_count' 0'dan buyuk olmalı
                print("SITL ile baglanti kuruldu.")
            else:
                raise Exception(
                    "SITL ile bağlanti kurulamadi port bilgisini kontrol edin."
                )
        else:
            raise Exception(
                "Araca sadece USB ya da SITL ile baglanabilirsin."
            )

        self.master = master

        self.mode_map = self.master.mode_mapping()
        self.mode_map_keys = tuple(self.mode_map.keys())
        self.mode_map_values = tuple(self.mode_map.values())
        return

    def hareket_et(self, x, y, z, r, t = 1, i = 1):
        """
        x- ileri ve geri hareket,[-1000,1000] araligi(0 degeri hareket vermez)
        y- saga ve sola hareket,[-1000,1000] araligi(0 degeri hareket vermez)
        z- yukari ve asagi hareket,Uyarı! [0,1000] araligi(500 degeri hareket vermez)
        r- kendi etrafinda saga ve sola hareket,[-1000,1000] araligi(0 degeri hareket vermez)
        t- komutun kac defa gonderilecegi
        i- her komut arası beklenecek olan sure
        """
        for _ in range(t):
            self.master.mav.manual_control_send(
                self.master.target_system,
                x, #ileri,geri
                y, #sag gitme,sol gitme
                z, #yukari asagi default 500 degeri
                r, #etrafinda donme
                0  # Buton parametresi eger butona basılacaksa buton numarasina denk gelen bit degeri gonderilmeli
            )
            sleep(i)
        return self.SUCCESS

    def yunusbaligi(self):
        ...

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

    def set_mod(self, mode : str = 'DEPTH_HOLD', max_try : int = 3):
        """
        Aracin modunu degistirmek icin kullanilir
        Ornek set_mod("DEPTH_HOLD") -> araci DEPTH_HOLD moda alir
        """
        mode = mode.upper()

        if mode not in self.mode_map:
            print(
                f"{mode} modu bulunamadi."
                " DEPTH_HOLD moda gecilecek."
            )
            mode = 'DEPTH_HOLD'

        mode_id = self.mode_map[mode]

        for _ in range(max_try):
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

            hb = self.master.wait_heartbeat(timeout=1)
            if hb is None:
                print('Couldnt receive heartbeat')
                continue

            current_mode = self.mode_map_keys[self.mode_map_values.index(hb.custom_mode)]

            print("Mod -> ", current_mode)

            if mode == current_mode:
                return self.SUCCESS
        return self.ERROR_OUT_OF_LOOP

    def kapat(self):
        """
        Pixhawk ile olan baglanitiyi kapatir.
        """
        self.master.close()
        print("-> Baglanti kapatildi.")
