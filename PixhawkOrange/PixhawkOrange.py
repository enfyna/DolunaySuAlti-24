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

    def get_yaw_roll_pitch_rad(self):
        """
        Sirasiyla yaw roll pitch degerlerini radyan cinsinden verir.
        """
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                          0,
                                          30, 0, 0, 0, 0, 0, 0)
        attitude = self.master.recv_match(type='ATTITUDE', blocking=True)
        return [attitude.yaw, attitude.roll, attitude.pitch]

    def get_yaw_roll_pitch_deg(self):
        """
        Sirasiyla yaw, yaw hizi, roll, roll hizi, pitch, pitch hizi degerlerini derece ve derece/saniye cinsinden verir.
        """
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                          0,
                                          30, 0, 0, 0, 0, 0, 0)
        attitude = self.master.recv_match(type='ATTITUDE', blocking=True)
        return [math.degrees(attitude.yaw), math.degrees(attitude.yawspeed),
                math.degrees(attitude.roll), math.degrees(attitude.rollspeed),
                math.degrees(attitude.pitch), math.degrees(attitude.pitchspeed)]

    def get_pressure(self):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                          0,
                                          29, 0, 0, 0, 0, 0, 0)
        pressure = self.master.recv_match(
            type='SCALED_PRESSURE', blocking=True)
        pressure_abs = float(pressure.press_abs)
        pressure_diff = pressure.press_diff
        temperature = pressure.temperature
        # Depth (m) = ((P0/ρ) - P) / g
        p0 = 1013.25
        p = 1000.0
        P = pressure_abs/1000.0
        g = 9.83  # -> NOT : Arac yukari yada asagiya giderkenki ivmesiyle toplanmasi lazim galiba denemek lazim
        pressuremeter = ((p0/p) - P)/g
        pressuremeter2 = pressure_abs / (9.8 * 997.0)
        return pressure_abs, pressuremeter*100, pressuremeter2

    def get_servo(self):
        """
        The SERVO_OUTPUT_RAW message contains data for 8 servo/engine outputs
        You can access the data for each output using the following indices:
        engine.servo1_raw, engine.servo2_raw, engine.servo3_raw, engine.servo4_raw
        engine.servo5_raw, engine.servo6_raw, engine.servo7_raw, engine.servo8_raw
        """
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                          0,
                                          36, 0, 0, 0, 0, 0, 0)
        engine = self.master.recv_match(
            type='SERVO_OUTPUT_RAW', blocking=True).to_dict()
        return [engine[f'servo{i+1}_raw'] for i in range(8)]

    def set_arm(self, i):
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

    def get_heartbeat(self):
        return self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

    def get_battery_percentage(self):
        return self.master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1).battery_remaining

    def get_attitude_fast(self):
        return self.master.recv_match(type='ATTITUDE', blocking=False, timeout=0.5)

    def get_message(self):
        return self.master.messages

    def kapat(self):
        self.master.close()
        print("-> Baglanti kapatildi.")
