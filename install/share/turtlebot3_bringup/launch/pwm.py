import RPi.GPIO as GPIO
from time import sleep

ledpin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)

pi_pwm = GPIO.PWM(ledpin,100)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 



sleep(0.2)
# Đặt mức PWM cố định (0-100)
duty_cycle = 7  # Thay đổi giá trị này để điều chỉnh độ sáng (0-100)
pi_pwm.ChangeDutyCycle(duty_cycle)




try:
    while True:
        sleep(1)  # Giữ chương trình chạy
except KeyboardInterrupt:
    pi_pwm.stop()
    GPIO.cleanup()