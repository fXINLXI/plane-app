from flask import Flask, jsonify, render_template
import pygame
import threading
import time
import serial
import serial.tools.list_ports

app = Flask(__name__)

state = [0, 0, 0]
controlls = [0, 0, 0, 0]

ser = None
PORT_NAME = '/dev/ttyACM0'

pygame.init()
pygame.joystick.init()
pygame.event.pump()
pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)

joystick = None

armed=0
flaps=0
ccs=0
release=0
sas=0
controlls=[0,0,0,0,0]
packet_loss=0

radio_alarm=0
sounds = {
    "event_on": pygame.mixer.Sound("sounds/event_on.wav"),
    "event_off": pygame.mixer.Sound("sounds/event_off.wav"),
    "radio_weak": pygame.mixer.Sound("sounds/radio_weak.wav"),
    "radio_lost": pygame.mixer.Sound("sounds/radio_lost.wav"),
}
playing=[0,0]

def play(sound_name):
    sounds[sound_name].play()

def serial_loop():
    global ser
    global controlls
    global packet_loss
    global state
    global playing

    while True:
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            if PORT_NAME not in ports:
                if ser:
                    try:
                        ser.close()
                    except:
                        pass
                    ser = None
                state[1]=0
                state[2]=0
                packet_loss=100
                if not playing[1]:
                    sounds["radio_lost"].play(loops=-1)
                sounds["radio_weak"].stop()
                playing[0]=0
                playing[1]=1
            else:
                if not ser:
                    try:
                        ser = serial.Serial(PORT_NAME, 9600, timeout=0.1)
                    except Exception as e:
                        print(f"ERROR: {e}")
                        ser = None
                    state[1]=2
                    state[2]=0
                    controlls[0]=0
                    packet_loss=100
                    if not playing[1]:
                        sounds["radio_lost"].play(loops=-1)
                    sounds["radio_weak"].stop()
                    playing[0]=0
                    playing[1]=1
                else:
                    state[1]=1
                    ser.write(bytes([
                        int(controlls[0]*1.8),#motor
                        int(controlls[1]*1.8)&(0xfe)|release,#yaw
                        int(controlls[2]*1.8)&(0xfe)|flaps,#pitch
                        int(controlls[3]*1.8)&(0xfe)|sas #roll
                    ]))
                    if ser.in_waiting> 0:
                        packet_loss = float(ser.readline().decode().strip())*100
                        if packet_loss<30:
                            state[2]=1
                            sounds["radio_lost"].stop()
                            sounds["radio_weak"].stop()
                            playing[0]=0
                            playing[1]=0
                        elif packet_loss<80:
                            state[2]=2
                            if not playing[0]:
                                sounds["radio_weak"].play(loops=-1)
                            sounds["radio_lost"].stop()
                            playing[0]=1
                            playing[1]=0
                        else:
                            state[2]=0
                            if not playing[1]:
                                sounds["radio_lost"].play(loops=-1)
                            sounds["radio_weak"].stop()
                            playing[0]=0
                            playing[1]=1
                        
        except:
            pass
                

        time.sleep(0.01)


def joystick_loop():
    global controlls
    global joystick
    global armed
    global flaps
    global ccs
    global release
    global sas

    while True:
        global controlls

        number=pygame.joystick.get_count()
        if joystick and number:
            x_left=joystick.get_axis(0)
            y_left=joystick.get_axis(1)
            x_right=joystick.get_axis(4)
            y_right=joystick.get_axis(3)

            x_left=x_left*(abs(x_left)>0.16)
            y_left=y_left*(abs(y_left)>0.16)
            x_right=x_right*(abs(x_right)>0.16)
            y_right=y_right*(abs(y_right)>0.16)

            if not ccs:
                controlls[0] = max(-y_left*100,0)
            else:
                controlls[0]+=joystick.get_axis(5)-joystick.get_axis(2)
                controlls[0]=min(max(controlls[0],0),100)
            controlls[1] = x_left*50+50

            controlls[2] = -x_right*50+50
            controlls[3] = y_right*50+50
            if armed:
                if joystick.get_button(2) and joystick.get_axis(1)<=0.3:
                    armed=0
                    play("event_off")
            else:
                controlls[0]=0
                if joystick.get_button(2) and joystick.get_axis(1)>=0.3:
                    armed=1
                    play("event_on")
            events=pygame.event.get()
            for event in events:
                if event.type==pygame.JOYBUTTONDOWN:
                    if event.button==0:
                        if flaps==0:
                            play("event_on")
                            flaps=1
                        else:
                            flaps=0
                            play("event_off")
                    if event.button==3:
                        if ccs==0:
                            play("event_on")
                            ccs=1
                        else:
                            ccs=0
                            play("event_off")
                    if event.button==1:
                        if release==0:
                            play("event_on")
                            release=1
                        else:
                            release=0
                            play("event_off")
                    if event.button==4:
                        if sas==0:
                            play("event_on")
                            sas=1
                        else:
                            sas=0
                            play("event_off")
                    
        else:
            controlls[0]=0
            armed=0
            state[0]=0
            sas=1
            ccs=0
            while number<=0:
                pygame.event.pump()
                number=pygame.joystick.get_count()
                time.sleep(0.05)
            sas=0
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            pygame.event.pump()
            print(f"Pad detected: {joystick.get_name()}")
            state[0]=1
        pygame.event.pump()
        time.sleep(0.01)

        


threading.Thread(target=joystick_loop, daemon=True).start()

threading.Thread(target=serial_loop, daemon=True).start()

@app.route("/")
def home():
    return render_template("index.html")

@app.route("/data")
def data():
    return jsonify({"state": state, "controlls": controlls,"packet_loss":packet_loss,"armed":armed,"flaps":flaps,"ccs":ccs,"release":release,"sas":sas,"voltage":"X "})

if __name__ == "__main__":
    app.run(debug=False)