from pymavlink import mavutil

def getData(master):
    lines = {}
    lines.update(data(master))
    lines.update(pressure(master))
    lines.update(motors(master))
    lines.update(mod(master))
    return lines


def data(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     30,0,0,0,0,0,0)
    attitude = master.recv_match(type='ATTITUDE', blocking=True)
    yaw = attitude.yaw
    roll = attitude.roll
    pitch = attitude.pitch
    data = {
        "yaw": yaw,
        "roll": roll,
        "pitch": pitch}
    return data

def pressure(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,
                                     29,0,0,0,0,0,0)
    pressure = master.recv_match(type='SCALED_PRESSURE', blocking=True)
    pressure_abs = pressure.press_abs#hPa
    pressure_diff = pressure.press_diff#hPa
    temperature = pressure.temperature
    #Depth (meters) = (Differential Pressure (hPa)) / (Density (kg/m³) * Gravity (m/s²) * 100)
    #In this formula, 100 is used to convert hPa to Pa because 1 hPa = 100 Pa.
    p = 1000.0
    g = 9.83
    depth=pressure_diff/(p*g*100)
    data =  { "pressure": depth
             }
    return data

def motors(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     36,0,0,0,0,0,0)
    engine = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
    # The SERVO_OUTPUT_RAW message contains data for 8 servo/engine outputs
    # You can access the data for each output using the following indices:
    # engine.servo1_raw, engine.servo2_raw, engine.servo3_raw, engine.servo4_raw
    # engine.servo5_raw, engine.servo6_raw, engine.servo7_raw, engine.servo8_raw
    data ={"servo1":engine.servo1_raw,"servo2":engine.servo2_raw,
           "servo3":engine.servo3_raw,"servo4":engine.servo4_raw,
           "servo5":engine.servo5_raw,"servo6":engine.servo6_raw,
           "servo7":engine.servo7_raw,"servo8":engine.servo8_raw}
    return data

def decode_mode(base_mode, custom_mode):
    """Decode flight mode from base_mode and custom_mode fields of HEARTBEAT message"""
    mode = 'UNKNOWN'
    if base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        if custom_mode == 0:
            mode = 'STABILIZE'
        elif custom_mode == 1:
            mode = 'ACRO'
        elif custom_mode == 2:
            mode = 'ALT_HOLD'
        elif custom_mode == 3:
            mode = 'AUTO'
        elif custom_mode == 4:
            mode = 'GUIDED'
        elif custom_mode == 5:
            mode = 'LOITER'
        elif custom_mode == 6:
            mode = 'RTL'
        elif custom_mode == 7:
            mode = 'CIRCLE'
        elif custom_mode == 8:
            mode = 'POSITION'
        elif custom_mode == 9:
            mode = 'LAND'
        elif custom_mode == 10:
            mode = 'OF_LOITER'
        elif custom_mode == 11:
            mode = 'DRIFT'
        elif custom_mode == 13:
            mode = 'SPORT'
        elif custom_mode == 14:
            mode = 'FLIP'
        elif custom_mode == 15:
            mode = 'AUTOTUNE'
        elif custom_mode == 16:
            mode = 'POSHOLD'
        elif custom_mode == 17:
            mode = 'BRAKE'
        elif custom_mode == 18:
            mode = 'THROW'
        elif custom_mode == 19:
            mode = 'MANUAL'
        elif custom_mode == 20:
            mode = 'GUIDED_NOGPS'
    return mode
def mod(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     0,0,0,0,0,0,0)
    mode = master.recv_match(type='HEARTBEAT', blocking=True)
    mode_name = decode_mode(mode.base_mode, mode.custom_mode)
    armed = mode.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    armedtext = ""
    if armed ==128:
        armedtext = "ARMED"
    else:
        armedtext = "DISARM"
    data={"mode":mode_name,
          "arm":armedtext}
    return data
            