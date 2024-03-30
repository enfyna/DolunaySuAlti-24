from pymavlink import mavutil
from time import time#, sleep

class PixhawkOrange():

    MOTOR_GUC = [
        0.7, # X
        0.7, # Y
        0.7, # Z
        0.7, # R
    ]

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
            # ilk önce windows yada linux portunu ara
            try:
                master = mavutil.mavlink_connection(f'{port}{idx}')
                master.wait_heartbeat(blocking=False)
                print("USB ile baglanti kuruldu.")
                break
            except:
                continue
        else:
            # sitl portunu ara
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

        hb = self.master.wait_heartbeat(blocking=True)
        
        self.mode_map = self.master.mode_mapping()
        self.mode_map_keys = tuple(self.mode_map.keys())
        self.mode_map_values = tuple(self.mode_map.values())

        if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            self.current_arm_state = 'ARM'
        else:
            self.current_arm_state = 'DISARM'

        self.current_mode = self.mode_map_keys[self.mode_map_values.index(hb.custom_mode)]

        self.boot_time = time()
        return

    def hareket_et(self, X=0.0, Y=0.0, Z=0.0, R=0.0) -> int:
        """
        Her eksen [1.0, -1.0] arasında değer alır.
        X - ileri ve geri hareket
        Y - saga ve sola hareket
        Z - yukari ve asagi hareket
        R - kendi etrafinda saga ve sola hareket
        """

        motor_kuvvetleri = []
        for i, eksen in enumerate([X,Y,Z,R]):
            kuvvet = 0
            if i == 2: # Z ekseni özel
                kuvvet = 500
                kuvvet += (eksen * 500) * self.MOTOR_GUC[i]
            else:
                kuvvet += (eksen * 1000) * self.MOTOR_GUC[i]
            motor_kuvvetleri.append(kuvvet)

        self.master.mav.manual_control_send(
            self.master.target_system,
            *motor_kuvvetleri,
            0  # Buton parametresi eger butona basılacaksa buton numarasina denk gelen bit degeri gonderilmeli
        )
        return self.SUCCESS

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

    def set_mod(self, mode : str = 'ALT_HOLD', max_try : int = 100) -> int:
        """
        Aracin modunu degistirmek icin kullanilir
        Ornek set_mod("ALT_HOLD") -> araci ALT_HOLD moda alir
        """
        mode = mode.upper()
        self.mode_map = self.master.mode_mapping()
        self.mode_map_keys = tuple(self.mode_map.keys())
        self.mode_map_values = tuple(self.mode_map.values())

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
            print(mode)
            print("Mod -> ", self.current_mode)

            if mode == self.current_mode:
                return self.SUCCESS
        return self.ERROR_OUT_OF_LOOP

    def set_target_depth(self, target : float) -> int:
        """ Sets the target depth while in depth-hold mode.

        Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

        'depth' is technically an altitude, so set as negative meters below the surface
            -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

        """
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=( # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ), lat_int=0, lon_int=0, alt=target, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
        return self.SUCCESS

    def getData(self) -> dict:
        lines = {}
        lines.update(self.get_attitude())
        lines.update(self.get_pressure())
        lines.update(self.get_motors())
        lines.update(self.get_mod())
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
            178,0,0,0,0,0,0)
        ahrs2 = self.master.recv_match(type='AHRS2', blocking=True)
        data = {
            "pressure": ahrs2.altitude
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
