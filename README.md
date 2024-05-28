# BLDC-Driver
## 概要
- ブラシレスDCモータ(BLDC)のドライバ、コントローラです。
- モータのトルク(トルク分電流)
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

## キャリブレーション
磁気エンコーダの原点と電流センスのオフセットを取得してMD内のflashに書き込みます。
初回起動時，モータ/MDの組み合わせを変えた時，ファームウェア変更時に必ず実行してください。

### 方法
1. DIPスイッチの1番をONにする(キャリブレーションモード)
2. 回転部分から手を離す
3. 電源を入れる
4. LEDが点滅→点灯に変わるまで待つ
5. DIPスイッチを元に戻して再起動(通常モード)

## 部品概要
### マイコン
- [STM32F446RET6](https://www.stmcu.jp/stm32/stm32f4/stm32f446/12361/)

### ゲートドライバ
- [DRV8323RS](http://www.ti.com/lit/ds/symlink/drv8323r.pdf)
  * MOSFETx6個分の回路内蔵
  * シャント抵抗用のアンプ付き
  * 各種保護回路付き

### MOSFET
- [PSMN011-60ML](https://assets.nexperia.com/documents/data-sheet/PSMN011-60ML.pdf)


### 位置センサ(磁気エンコーダ)
- [AS5048A](https://ams.com/ja/as5048a)使用
- SPIインターフェースにより読み込み
- データシート記載の 4wire modeで使用

## ソフトウェア

### 電流検出
- ゲートドライバ内蔵のアンプを通してADCに入力される
- ADCは3つ使用しPWM同期のトリガで同時に読み込む

### 電流制御
- センサ付き電流ベクトル制御でId, Iqを制御します。
- 非干渉化制御は未実装

### PWM
- 中間電圧1/2重畳法を基本的に利用
- 高速域ではスイッチングタイミングを電流検出タイミングから遠ざける

