# https://www.printables.com/model/818975-compact-robot-arm-arduino-3d-printed/files de esta web se obtuvo el código original en C++

"""
Este programa controla un brazo robótico utilizando servomotores a través de un bvus de conexión I2C con una placa Jetson. 
El código permite mover los servos del brazo en función de las lecturas de los potenciómetros, que se simulan con entradas digitales,
también permite controlar el agarre de una pinza mediante la pulsación de un botón.

Módulos utilizados:
    'board': Para gestionar los pines de la placa Jetson.
    'busio': Para establecer el bus de comunicación I2C.
    'Jetson.GPIO': Para controlar la GPIO de la placa Jetson.
    'adafruit_pca9685': Para gestionar el controlador PCA9685 que controla los servomotores.
    'adafruit_servokit': Para gestionar los servomotores.
    'time': Para realizar pausas entre comandos.

Constantes:
    'MIN_PULSE_WIDTH': Ancho mínimo de pulso para el control de los servos.
    'MAX_PULSE_WIDTH': Ancho máximo de pulso para el control de los servos.
    'FREQUENCY': Frecuencia de operación para el controlador de servos. En esta caso es 50, porque al estar en europa la frecuencia es de 50 Hz.

Variables globales:
    'pwm': Instancia del controlador PCA9685.
    'kit': Instancia del controlador ServoKit.
    'hand', 'wrist', 'elbow', 'shoulder', 'base', 'potWrist', 'potElbow', 'potShoulder', 'potBase':
    Instancias de servos para controlar los distintos movimientos del brazo robótico y la pinza.

Proceso:
1. Se inicializan los pines GPIO y el controlador de servos.
2. Se configura el valor de frecuencia del controlador PWM.
3. El código entra en un bucle continuo donde lee los valores de los potenciómetros y mueve los motores en consecuencia.
4. Además, el código controla un botón conectado al pin GPIO 7 de la Jetson, para simular el agarre o liberación de la pinza del robot.

El programa continúa ejecutándose indefinidamente hasta que se detiene manualmente o se apaga el sistema.
"""

#import Wire
#import Adafruit_PWMServoDriver
import board 
import busio 
import Jetson.GPIO as GPIO
import adafruit_pca9685 
import time 
i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_servokit import ServoKit


#Declaro variables globales
MIN_PULSE_WIDTH = 650
MAX_PULSE_WIDTH = 2350
FREQUENCY = 50

#Instancio el Driver del controlador de servos
#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver()  esta queda asi selecionada y la linea de abajo es su traduccion a pyton
pwm = adafruit_pca9685.PCA9685(i2c)
kit = ServoKit(channels=16)

#Configuro el SetUp
time.sleep(5)     #So i have time to get controller to starter position
pwm.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = adafruit_motor.servo.Servo(0)  
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)             #Set Gripper to 90 degrees (Close Gripper)
pwm.begin()
GPIO.setup(7, GPIO.IN)     # Channel tiene que ser un pin valido para en Jetson

def MoveMotor(controlIn, motorOut):
    
    """Indice o descripción de la función: Mueve el motor de acuerdo con el valor leído del potenciómetro o entrada digital. El valor de control determina el ancho del pulso que se envía al servo correspondiente.
    Parámetros: 
    controlIn = es la entrada del valor del potenciómetro que se genera al mover el robot manipulador y deriva dicho movimiento al robot grande
    motorOut = esta variable nos da como resultado un número entero que es el resultado de pulse_wide dividido entre 1000000 por la frecuencia, que en este caso es 50, por 4096
    """
    pulse_wide, pulse_width, potVal = -7

    #potVal = analogRead(controlIn);  #lectura en C
    potVal = GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096)
    #pwm.setPWM(motorOut, 0, pulse_width);     #lectura en C
    pwm = GPIO.PWM(motorOut, 0, pulse_width)

while(True):
    MoveMotor(potWrist, wrist)
    MoveMotor(potElbow, elbow)
    MoveMotor(potShoulder, shoulder)
    MoveMotor(potBase, base)
    pushButton = GPIO.input(7)
    if(pushButton == GPIO.LOW):
        pwm.setPWM(hand, 0, 180)               
        print("Grab")
    else:
        pwm.setPWM(hand, 0, 90)
        print("Release")

GPIO.cleanup()