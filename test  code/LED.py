import time
import pigpio

# --- ピン定義 ---
# LEDのピン番号をリストにまとめる
LEDS = [20, 21]

# --- pigpioの初期化 ---
# pigpioデーモンに接続
pi = pigpio.pi() 
if not pi.connected:
    print("pigpioデーモンに接続できませんでした。")
    print("ターミナルで 'sudo pigpiod' を実行してください。")
    exit()

try:
    # --- ピンモード設定 ---
    # リスト内のすべてのピンを出力モードに設定
    for led_pin in LEDS:
        pi.set_mode(led_pin, pigpio.OUTPUT)
        pi.write(led_pin, 0) # 初期状態を消灯にする

    print("LEDの点滅を開始します。Ctrl+Cで停止します。")
    
    # --- メインループ ---
    while True:
        # リストをループ処理することで、コードを簡潔にする
        for led_pin in LEDS:
            pi.write(led_pin, 1)  # 点灯
            time.sleep(0.5)
            pi.write(led_pin, 0)  # 消灯

except KeyboardInterrupt:
    # Ctrl+Cが押されたらループを抜ける
    print("\nプログラムが中断されました。")

finally:
    # --- 終了処理 ---
    print("リソースを解放しています...")
    if pi.connected:
        # 念のため、すべてのLEDを消灯
        for led_pin in LEDS:
            pi.write(led_pin, 0)
        # pigpioの接続を終了
        pi.stop()
    print("処理を終了しました。")
