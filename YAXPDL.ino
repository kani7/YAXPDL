//------------------------------------------------------------
// Yet Another XPDL-1
// Horizontal paddle controller for X68k (ATmega32U4)
//------------------------------------------------------------
// 2021-10-26 初版
// 2022-07-24 MSDATAの出力段の構成を純正マウス準拠に変更
//------------------------------------------------------------
// このスケッチのコンパイルには以下のライブラリが必要です:
//   Encoder (https://www.pjrc.com/teensy/td_libs_Encoder.html)
//------------------------------------------------------------

// X68kマウスコネクタ配線(本体側)
//
//   5   4
//  3     2
//      1
//
// 端子割り当て状況
//
//  X68k本体側        Arduino側         備考
//  --------------------------------------------------------------------
//  1:Vcc1 5V(out) -> PD4(4)           電源としてではなくX68k接続判定に使う
//  2:MSCTRL(out)  -> RX(1)            マウス宛データ
//  3:MSDATA(in)   <- TX(0)            マウスからのデータ
//  4:GND(--)      -- GND
//  5:GND(--)      -- GND

//#define MYDEBUG 3
#define ENCODER_OPTIMIZE_INTERRUPTS   //include行より先に指定する
#include <Mouse.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <avr/power.h>

#define RXA 2           //ロータリーエンコーダX軸A相
#define RXB 3           //ロータリーエンコーダX軸B相

#define X68_MSDATA 1    //X68kのMSDATA端子
#define X68_MSCTRL 0    //X68kのMSCTRL端子(UARTのRXで代用できない場合は5かなあ)
#define X68_MODE 4      //X68kマウスとして動作するか否か

// 以下ボタンやスイッチ類は負論理
#define LEFTBUTTON 6    //マウス左ボタン
#define RIGHTBUTTON 7   //マウス右ボタン

#define DIPSW_BIT0 15   //倍率・方向設定用 16進ロータリーDIPスイッチ
#define DIPSW_BIT1 14
#define DIPSW_BIT2 16
#define DIPSW_BIT3 10

Encoder Enc(RXA, RXB);
SoftwareSerial msSerial(X68_MSCTRL, X68_MSDATA, true);  //RX, TX, inverse_logic

uint8_t leftButtonReleased;       //チャタリング防止用のカウンタ
uint8_t rightButtonReleased;
boolean USB_EN;


void setup() 
{
  pinMode(LEFTBUTTON,INPUT_PULLUP);
  pinMode(RIGHTBUTTON,INPUT_PULLUP);
  pinMode(X68_MODE,INPUT);        //外付け抵抗でプルダウンしておくこと(内蔵プルダウンはできない)
  pinMode(X68_MSCTRL,INPUT_PULLUP);
  pinMode(DIPSW_BIT0,INPUT_PULLUP);
  pinMode(DIPSW_BIT1,INPUT_PULLUP);
  pinMode(DIPSW_BIT2,INPUT_PULLUP);
  pinMode(DIPSW_BIT3,INPUT_PULLUP);
  leftButtonReleased=0;
  rightButtonReleased=0;
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  power_usart1_disable();
  if (digitalRead(X68_MODE) == HIGH) {
    USB_EN = false;
    msSerial.begin(4800); //MSDATA送信用(8N2, inverse_logic)
#ifndef MYDEBUG
    power_usb_disable();
#endif
  } else {
    USB_EN = true;
    Mouse.begin();
  }
#if MYDEBUG
  Serial.begin(115200);             //おなじみのシリアルモニタ
  Serial.println(F("HELLO. __DATE__ __TIME__"));
#endif
}

void loop() {
  long ENCODER_DIV;       //エンコーダ倍率・方向

  //ロータリーDIPSW指示値   0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
  //             X軸倍率  +1  +2  +3  +4  +5  +6  +7  +8  -8  -7  -6  -5  -4  -3  -2  -1
  //
  //リアルコードではなくコンプリメンタリコードのDIPSWを使った場合 LOW/HIGHが逆になる
  if (digitalRead(DIPSW_BIT3) == HIGH){
    ENCODER_DIV = 1;
    if (digitalRead(DIPSW_BIT0) == LOW) ENCODER_DIV = ENCODER_DIV + 1;
    if (digitalRead(DIPSW_BIT1) == LOW) ENCODER_DIV = ENCODER_DIV + 2;
    if (digitalRead(DIPSW_BIT2) == LOW) ENCODER_DIV = ENCODER_DIV + 4;
  } else {
    ENCODER_DIV = -1;
    if (digitalRead(DIPSW_BIT0) == HIGH) ENCODER_DIV = ENCODER_DIV - 1;
    if (digitalRead(DIPSW_BIT1) == HIGH) ENCODER_DIV = ENCODER_DIV - 2;
    if (digitalRead(DIPSW_BIT2) == HIGH) ENCODER_DIV = ENCODER_DIV - 4;
  }

  if (USB_EN) {
    usb_mouse_send(ENCODER_DIV);
  } else {
    x68_mouse_send(digitalRead(X68_MSCTRL), ENCODER_DIV);
  }
}

void usb_mouse_send(long ENCODER_DIV) {
  uint8_t leftButton = digitalRead(LEFTBUTTON);
  uint8_t rightButton = digitalRead(RIGHTBUTTON);
  long counts = Enc.read() * ENCODER_DIV; //前回からのカウント数が返ってくる

  delay(20);                              //USBメッセージ頻度調整用

  if(counts!=0){
    Mouse.move(counts,0,0);
    Enc.write(0);                         //カウンタをリセットする
  }
  
  if(leftButton == LOW) {
    Mouse.press(MOUSE_LEFT); 
    leftButtonReleased = 0; 
  } else {
    if(leftButtonReleased >= 3) {         //上記delayの3倍つまり60msec経過しないとボタンを離したと判定されない
      Mouse.release(MOUSE_LEFT);
    } else { 
      leftButtonReleased++;
    }
  }
    
  if(rightButton == LOW) {
    Mouse.press(MOUSE_RIGHT); 
    rightButtonReleased = 0; 
  } else {
    if(rightButtonReleased >= 3) {        //上記delayの(以下略)
      Mouse.release(MOUSE_RIGHT);
    } else { 
      rightButtonReleased++;
    }
  }
}


void x68_mouse_send(uint8_t MSCTRL, long ENCODER_DIV) {
  static uint8_t oldCTRL;             //前回届いたMSCTRLを保管して比較に使う用(staticなのが必須)
  uint8_t leftButton = digitalRead(LEFTBUTTON);
  uint8_t rightButton = digitalRead(RIGHTBUTTON);
  long dx = Enc.read()*ENCODER_DIV;   //前回送信時からのカウント数が返ってくる
  const long dy = 0;                  //Y軸側エンコーダが存在しないので常に0

  if (MSCTRL == LOW && oldCTRL == HIGH) {  //highからlowになった
#if MYDEBUG == 3
    //Serial.print(F("MSCTRL = ")); Serial.print(MSCTRL);
    Serial.print(F(" LEFT  = ")); Serial.print(leftButton); Serial.print(F(" RIGHT = ")); Serial.print(rightButton);
    Serial.print(F(" dX = ")); Serial.print(dx, HEX); Serial.print(F(" 2dX = ")); Serial.print(dx, BIN);
    Serial.print(F(" dX = ")); Serial.print(dx >> 1, HEX); Serial.print(F(" 2dX = ")); Serial.println(dx >> 1, BIN);
    //Serial.print(F(" dY = ")); Serial.print(dy, HEX); Serial.print(F(" 2dY = ")); Serial.print(dy, BIN);
    //Serial.print(F(" dY = ")); Serial.print(dy >> 1, HEX); Serial.print(F(" 2dY = ")); Serial.println(dy >> 1, BIN);
#endif
    delayMicroseconds(700);
    char MSDATA = B00000000;
    if (leftButton == LOW) {  //左ボタン押下
      MSDATA   |= B00000001;
      leftButtonReleased = 0;
    } else {
      if (leftButtonReleased < 3) {
        MSDATA |= B00000001;
        leftButtonReleased++;
      }
    }
    if (rightButton == LOW) { //右ボタン押下
      MSDATA   |= B00000010;
      rightButtonReleased = 0;
    } else {
      if (rightButtonReleased < 3) {
        MSDATA |= B00000010;
        rightButtonReleased++;
      }
    }
      
    if (dx > 127) {
      MSDATA |= B00010000;    //X軸オーバーフロー(プラス方向)
      dx = 127;
    }
    if (dx < -128) {
      MSDATA |= B00100000;    //X軸オーバーフロー(マイナス方向)
      dx = -128;
    }

    //Y軸は存在しないのでオーバーフロー判定は不要

    //Serial.print(F("MSDATA = ")); Serial.println(MSDATA);
    msSerial.write(MSDATA);
    delayMicroseconds(420);   //stop bitが足りないので2bits分待つ
    msSerial.write((int8_t)dx);
    delayMicroseconds(420);   //stop bitが足りないので2bits分待つ
    msSerial.write((int8_t)dy);

    Enc.write(0);             //カウンタをリセット
  }
  oldCTRL = MSCTRL;     //次回用に今回データを保存
}
