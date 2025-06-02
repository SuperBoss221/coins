#
#  Play a note on a buzzer
#
from machine import Pin, PWM
import time

BUZZER_PIN = 18 # Piezo buzzer + is connected to GP6, - is connected to the GND right beside GP6
buzzer = PWM(Pin(BUZZER_PIN, Pin.OUT))


def playNote(frequency, duration, pause) :
    global buzzer
    buzzer.duty_u16(5000)  # adjust loudness: smaller number is quieter.
    buzzer.freq(frequency)
    time.sleep(duration)
    buzzer.duty_u16(0) # loudness set to 0 = sound off
    time.sleep(pause)
    

def SUCCESS():
    #notes = [440, 494, 523, 587, 659, 698, 784]
    notes = [440, 523, 659, 784]
    for note in notes :
        playNote(note, 0.1, 0.01)

def Start():
    notes = [440, 494, 523, 587, 659, 698, 784]
    for note in notes :
        playNote(note, 0.1, 0.01)
def Error():
    notes = [65000, 1000, 65000, 65000, 1000]
    for note in notes :
        playNote(note, 0.1, 0.01)
        
def CLICK():
    notes = [1500,1200]
    for note in notes :
        playNote(note, 0.1, 0.01)