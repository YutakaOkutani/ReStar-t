import pigpio
import time

# --- ピン定義 ---
AIN1 = 8 #8
AIN2 = 25 #25
PWMA = 18 #18
BIN1 = 9 #9
BIN2 = 11 #11
PWMB = 19 #19
STBY = 22 

pi = pigpio.pi()
if not pi.connected:
    exit()

# --- ピンモード設定 ---
pins = [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY]
for pin in pins:
    pi.set_mode(pin, pigpio.OUTPUT)

# --- PWM周波数を5000Hzに設定 ---
pi.set_PWM_frequency(PWMA, 5000)
pi.set_PWM_frequency(PWMB, 5000)

# STBYをHIGHにしてドライバーを有効化
pi.write(STBY, 1)

try:
    print("モーターを同時にソフトスタートさせ、3秒間走行します。")

    # 回転方向の指定
    pi.write(AIN1, 1)
    pi.write(AIN2, 0)
    pi.write(BIN1, 0)
    pi.write(BIN2, 1)

    # ソフトスタート (0 -> 106へ)
    for duty in range(128):
        pi.set_PWM_dutycycle(PWMA, duty)
        pi.set_PWM_dutycycle(PWMB, duty)
        time.sleep(0.01)

    print("設定速度に到達。3秒間維持します。")
    time.sleep(10)

    print("ソフトストップします。")
    # ソフトストップ (106 -> 0へ)
    for duty in range(128, -1, -1):
        pi.set_PWM_dutycycle(PWMA, duty)
        pi.set_PWM_dutycycle(PWMB, duty)
        time.sleep(0.01)

finally:
    print("モーターを停止し、リソースを解放します。")
    pi.write(AIN1, 0)
    pi.write(AIN2, 0)
    pi.write(BIN1, 0)
    pi.write(BIN2, 0)
    pi.set_PWM_dutycycle(PWMA, 0)
    pi.set_PWM_dutycycle(PWMB, 0)
    pi.write(STBY, 0)
    pi.stop()
