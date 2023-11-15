import json

def joystickControl(json_joystick,vehicle):
    joystickData=json.loads(json_joystick)

    left_x = int(joystickData["LeftThumbX"])
    left_y = int(joystickData["LeftThumbY"])
    right_x = int(joystickData["RightThumbX"])
    right_y = int(joystickData["RightThumbY"])
    move( left_x, left_y, right_x, right_y, vehicle)

    arm = bool(joystickData["MenuButton"])
    disarm = bool(joystickData["BackButton"])
    armAndDisarm(1 if arm else 0, vehicle)

    stabilize_mode=bool(joystickData["YButton"])
    depthhold_mode=bool(joystickData["XButton"])
    manual_mode=bool(joystickData["BButton"])
    acro_mode=bool(joystickData["AButton"])
    if(stabilize_mode):
        changeMode("STABILIZE", vehicle)
    elif(depthhold_mode):
        changeMode("DEPTH_HOLD", vehicle)
    elif(manual_mode):
        changeMode("MANUAL", vehicle)
    elif(acro_mode):
        changeMode("ACRO", vehicle)

def move(lx,ly,rx,ry,vehicle):
    vehicle.hareket_et(ly,rx,min(ry+500,1000),lx,1)

def armAndDisarm(armmode,vehicle):
    vehicle.set_arm(armmode)

def changeMode(mode,vehicle):
    vehicle.set_mod(mode)