# Re-Start
# ファイル
- 本番用コード：main.py
- センサー用ライブラリ：bno055.py,bmp180.py,mycropyGPS.py
- テストコード：moter_test.py,camera_detection_test.py
- カメラフェーズ用：capture_roi_image.py,detect_corn.py
- パラシュート投下試験：open_parachute.py
- 着地衝撃試験：ground_shock.py

# 搭載計器一覧
|分類|型番|購入サイト|備考|
| ---- | ---- |---|---|
|マイコン |Raspberrypizero2W|マルツ（協賛）|OSは、Raspberry Pi OS Lite(64bit)|
|マイクロSD |LMEX1L032GG2/LMEX1L032GG4|秋月|16GBでも余裕で足りる、8GBでもいいかも|
|9軸センサ|BNO055|秋月|
|気圧センサ|BMP180|電子工作ステーション|
|モーター|75:1 Metal Gearmotor 25Dx54L mm HP 6V |POLOLU|ReStartはPWM制御降圧|
|モータードライバ|TB6612FNG(ピンヘッダver)|秋月|ストール電流に耐えられないので、これは不適|
|超音波センサ|HC-SR04|秋月|
|GPS|GEP-M 10 nano|アリエク|
|カメラ|KEYESTUDIO 5MP |Yahooショッピング|互換品であるが十分|
|バッテリー|LP-3S1P360RE|Amazon|360mAhでも完走できるが、すぐに充電が切れるので、720mAhのほうが良いかも|
|DC-DCコンバータ|M78AR05-1|秋月|1000mAしか流せないので、ラズパイの定格電流（2.5A）を満たさず、これは不適|

# 環境構築
## 0. アップデートする
```bash
sudo apt-get update
sudo apt-get upgrade
```

## 1. raspi-configからI2CとSerialを有効化する
```bash
raspi-config
```

## 2. リポジトリをクローンする

```bash
git clone https://github.com/YutakaOkutani/ReStar-t
cd ReStar-t
```

## 3. Pythonの仮想環境を作成する

```bash
python -m venv env
source ./env/bin/activate
pip install --upgrade pip
```

## 4. 必要なライブラリをインストールする

```bash
sudo apt-get install -y i2c-tools python3-smbus ibatlas-base-dev python3-picamera2
pip install pyserial numpy
pip install opencv-python==4.6.0.66 #opencvは最新バージョンをインストールしようとするとバグりがち
```

## 5. カメラを有効化
参照:https://raspida.com/rpi-camera-module-matome/#index_id5  
```bash
lsb_release -a #バージョン確認
nano /boot/firmware/config.txt #bookwormの場合
#config.txtを編集
dtoverlay=OV5647
#canera_auto_detect=1
camera_auto_detect=0
```

##  GPSの動作確認方法

```bash
sudo apt-get install screen
screen /dev/serial0 9600
```
1.真っ暗な画面が表示される場合：ラズパイがGPSを認識できていない\
2.エラーがでる場合：/dev/serial0を変えてみる　参照：https://tech-and-investment.com/raspberrypi2-5-uart-setting/ \
3.NMEAフォーマットが出力されるが、緯度経度の値が----になっている場合：GPSは認識されているがGPSが人工衛星を認識できていない。GPS受信機は屋内では人工衛星を捕捉できないので、壁や天井に遮られない場所に出て、しばらく放置してみる\
4.NMEAフォーマットが出力され、緯度経度の値が妥当な場合：成功!

## BNO055,BMP180の接続確認
```bash
sudo i2cdetecy -y 1
#0x28,0x77が表示されれば、それぞれBNO055,BMP180を、ラズパイが認識していることを示す。
```

## カメラの動作確認
```bash
rpicam-hello -t 0　#bookwormの場合
```