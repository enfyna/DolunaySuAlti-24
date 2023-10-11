from pymavlink import mavutil
from time import sleep

class Dolunay():

    SUCCESS = 0
    ERROR_OUT_OF_LOOP = 1

    def __init__(self):
        """
        Araca baglanmak için sırasıyla USB->SITL portlarını dene
        """
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
                print("USB ile baglanti kuruldu.")
                break
            except:
                continue
        else:
            port = 'udp:127.0.0.1:14550'
            master = mavutil.mavlink_connection(port)
            master.wait_heartbeat(timeout = 3)
            if master.mav_count > 0:
                # sitl ile baglandiysak heartbeat mesajını almış olmamız gerek
                # heartbeat mesajı aldık isek 'mav_count' 0'dan buyuk olmalı
                print("SITL ile baglanti kuruldu.")
            else:
                raise Exception(
                    "Arac ile baglanti kurulamadi."
                    "Port bilgisinde hata var yada kablo duzgun takilmamis olabilir."
                )

        self.master = master

        self.mode_map = self.master.mode_mapping()
        self.mode_map_keys = tuple(self.mode_map.keys())
        self.mode_map_values = tuple(self.mode_map.values())

        hb = self.master.wait_heartbeat(blocking=True)

        if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            self.current_arm_state = 'ARM'
        else:
            self.current_arm_state = 'DISARM'

        self.current_mode = self.mode_map_keys[self.mode_map_values.index(hb.custom_mode)]
        return

    def hareket_et(self, x, y, z, r, t = 1, i = 1) -> int:
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

    def set_arm(self, arm : bool = True, max_try : int = 7) -> int:
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
                self.current_arm_state = 'ARM'
                return self.SUCCESS
            elif not arm and self.master.motors_armed() == 0:
                print("-> Disarmed")
                self.current_arm_state = 'DISARM'
                return self.SUCCESS
        return self.ERROR_OUT_OF_LOOP

    def set_mod(self, mode : str = 'ALT_HOLD', max_try : int = 7) -> int:
        """
        Aracin modunu degistirmek icin kullanilir
        Ornek set_mod("ALT_HOLD") -> araci ALT_HOLD moda alir
        """
        mode = mode.upper()

        if mode not in self.mode_map:
            print(
                f"{mode} modu bulunamadi."
                " ALT_HOLD moda gecilecek."
            )
            mode = 'ALT_HOLD'

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

            self.current_mode = self.mode_map_keys[self.mode_map_values.index(hb.custom_mode)]

            print("Mod -> ", self.current_mode)

            if mode == self.current_mode:
                return self.SUCCESS
        return self.ERROR_OUT_OF_LOOP

    def getData(self) -> dict:
        lines = {
            **self.get_attitude(),
            **self.get_pressure(),
            **self.motors(),
            **self.get_mod()
        }
        return lines

    def get_attitude(self) -> dict:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            30,0,0,0,0,0,0)
        attitude = self.master.recv_match(type='ATTITUDE', blocking=True)
        data = {
            "yaw": attitude.yaw,
            "roll": attitude.roll,
            "pitch": attitude.pitch
        }
        return data

    def get_pressure(self) -> dict:
        self.master.mav.command_long_send(
            self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,
            29,0,0,0,0,0,0)
        pressure = self.master.recv_match(type='SCALED_PRESSURE', blocking=True)
        pressure_abs = pressure.press_abs #hPa
        pressure_diff = pressure.press_diff #hPa
        temperature = pressure.temperature
        #Depth (meters) = (Differential Pressure (hPa)) / (Density (kg/m³) * Gravity (m/s²) * 100)
        #In this formula, 100 is used to convert hPa to Pa because 1 hPa = 100 Pa.
        p = 1000.0
        g = 9.83
        depth = pressure_diff/(p*g*100)
        data = {
            "pressure": depth
        }
        return data

    def get_motors(self) -> dict:
        self.master.mav.command_long_send(
            self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            36,0,0,0,0,0,0)
        engine = self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        # The SERVO_OUTPUT_RAW message contains data for 8 servo/engine outputs
        # You can access the data for each output using the following indices:
        # engine.servo1_raw, engine.servo2_raw, engine.servo3_raw, engine.servo4_raw
        # engine.servo5_raw, engine.servo6_raw, engine.servo7_raw, engine.servo8_raw
        data = {
            "servo1":engine.servo1_raw,"servo2":engine.servo2_raw,
            "servo3":engine.servo3_raw,"servo4":engine.servo4_raw,
            "servo5":engine.servo5_raw,"servo6":engine.servo6_raw,
            "servo7":engine.servo7_raw,"servo8":engine.servo8_raw
        }
        return data

    def get_mod(self) -> dict:
        data = {
            "mode": self.current_mode,
            "arm": self.current_arm_state
        }
        return data

    def kapat(self) -> None:
        """
        Pixhawk ile olan baglantiyi kapatir.
        """
        self.set_arm(False)

        self.master.close()
        print("-> Baglanti kapatildi.")