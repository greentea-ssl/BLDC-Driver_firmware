# BLDC-Driver
## 概要
- ブラシレスDCモータ(BLDC)のドライバ、コントローラです。
- モータのトルク(トルク分電流)、速度、位置を制御します。
- CANインターフェースにより指令値を送ります。
- ロータの位置、速度は磁気エンコーダ([AS5048A](https://ams.com/ja/as5048a))で検出します。
- モータの線電流は3シャント方式により検出します。
  * [参考資料](http://www.tij.co.jp/jp/lit/ug/tiducy7/tiducy7.pdf)

## 使用モータの指定
Inc/parameters.h 内の10行目付近で使用モータの型番を選んでコメントアウトを解除してください。

（例: SUNNYSKY V2806-KV400を使う場合）
```c
//#define _MOTOR_QUANUM_MT4108_KV370_
//#define _MOTOR_SUNNYSKY_V4006_KV320_
#define _MOTOR_SUNNYSKY_V2806_KV400_
```

## エンコーダのキャリブレーション
磁気エンコーダの原点(α軸方向)を取得してMD内のflashに書き込みます。
初回起動時とモーター・MDの組み合わせを変更した場合に実行してください。
### 方法
1. 電源を入れ、LEDが点滅している間にDIPSWの値を適当に変える
2. モータに触らず開放状態にする
3. なんか電流が流れて１秒経たずに終わる
4. ここからは通常時の動作

## 使用方法
1. 配線を接続する
2. モータの番号をDIPSWで指定する
3. 電源を入れる
4. LEDが点滅して、CAN受信が始まる
4. CANでデータ送るとまわる

## 部品概要
### マイコン
- [STM32F446RET6](https://www.stmcu.jp/stm32/stm32f4/stm32f446/12361/)

### ゲートドライバ
- [DRV8323RH](http://www.ti.com/lit/ds/symlink/drv8323r.pdf)
  * MOSFETx6個分の回路内蔵
  * シャント抵抗用のアンプ付き
  * 各種保護回路付き

### MOSFET
- [PSMN011-60ML](https://assets.nexperia.com/documents/data-sheet/PSMN011-60ML.pdf)

## ソフトウェア仕様
### 位置センサ(磁気エンコーダ)
- [AS5048A](https://ams.com/ja/as5048a)使用
- SPIインターフェースにより読み込み
- データシート記載の 4wire modeで使用

### 電流検出
- ゲートドライバ内蔵のアンプを通してADCに入力される
- ADCは3つ使用しPWM同期のトリガで同時に読み込む

### 電流制御器(ACR, Auto Current Regulator)
- センサ付き電流ベクトル制御を行い、界磁電流<img src="https://latex.codecogs.com/gif.latex?i_d"/>とトルク分電流<img src="https://latex.codecogs.com/gif.latex?i_q"/>を制御します。
- <img src="https://latex.codecogs.com/gif.latex?i_d"/>, <img src="https://latex.codecogs.com/gif.latex?i_q"/>の制御にはPI制御器を用います。
- 非干渉化制御は未実装

### 速度制御器(ASR, Auto Speed Regulator)
- モータの角速度を制御します。
- ここでいう角速度とは機械角の角速度<img src="https://latex.codecogs.com/svg.latex?\omega_m"/>です。
- PI制御器を用いています。


### 位置制御器(APR, Auto Position Regulator)
- モータの回転子位置（角度）を制御します。
- PD制御器を用いています。



