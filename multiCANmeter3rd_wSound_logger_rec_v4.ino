#include <Arduino.h>
#include "mcp_can.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "s660.h"
#include "neon_ring.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_PCF8574.h>
#include <SdFat.h>
#include "I2C_BM8563.h"

#define USE_TFT_ESPI_LIBRARY
#include "lv_xiao_round_screen.h"

I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);

TFT_eSprite time_diff_sprite = TFT_eSprite(&tft);
TFT_eSprite boost_sprite = TFT_eSprite(&tft);
TFT_eSprite wtemp_sprite = TFT_eSprite(&tft);
TFT_eSprite airtemp_sprite = TFT_eSprite(&tft);
TFT_eSprite fuel_sprite = TFT_eSprite(&tft);

TFT_eSprite oiltemp_sprite = TFT_eSprite(&tft);
TFT_eSprite oilpres_sprite = TFT_eSprite(&tft);

lv_coord_t touchX, touchY;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define XIAO_BL D6
#define CAN0_INT D2 //original D6                              // Set INT to pin 21 D6
MCP_CAN CAN0(D0);

Adafruit_PCF8574 pcf;
const uint8_t mcp_cs_pcf8574_pin = 0;     // P0をCSに使う
const uint8_t sdfat_cs_pcf8574_pin = 1;

int val_wtemp = 0;
int val_atemp = 0;
int val_fuel = 0;
float val_bst =0.0f;

int pre_wtemp = 0;
int pre_atemp = 0;
int pre_fuel = 99;
float pre_bst =0.0f;
int pre_otemp = 0;
float pre_opres = 9.99f;

int p_wtemp=0;
int p_atemp = 0;
int p_fuel = 99;
float p_bst =-0.99f;
int p_otemp = 0;
float p_opres = 9.99f;

int val_light = 0;
int pre_light = 0;

float g =0;

int BL_lvl=77;
int Day_BL_lvl =77;
int Night_BL_lvl =2;
int touch_flag =0;
int ph_mode =0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int temp = 0;
  float pres = 9.99f;
} struct_message;

// Create a struct_message called myData
struct_message oilstatus;

// ==== SdFatオブジェクト & 設定 ====
SdFat SD;
SdSpiConfig sdConfig(D0, SHARED_SPI, SD_SCK_MHZ(10));  // -1 = 手動CS, SHARED_SPI = 複数SPIデバイス対応

I2C_BM8563_DateTypeDef dateStruct;

// ===== PWM設定 =====
const int BUZZER_PIN = D0;   // ブザーをつなぐピン（XIAO ESP32-C3のGPIOに合わせる）
const int CH = 1;            // LEDCチャンネル番号（0〜15）
const int RES = 8;          // PWM分解能(ビット数) 10bit=最大1023
const int DUTY = 127;        // デューティ(0〜1023)
const int FREQ = 2700;       // PWM周波数 [Hz]

// ===== 断続パターン =====
// ON 150ms → OFF 150ms → ON 150ms → OFF 1000ms の繰り返し
const bool PAT_STATE1[]  = {1, 0};            // 1=ON, 0=OFF
const uint16_t PAT_MS1[] = {50, 500};   // 各状態の時間[ms]
const int PAT_LEN1 = sizeof(PAT_MS1) / sizeof(PAT_MS1[0]);

const bool PAT_STATE2[]  = {1, 0, 1, 0};            // 1=ON, 0=OFF
const uint16_t PAT_MS2[] = {50, 50, 50, 40};   // 各状態の時間[ms]
const int PAT_LEN2 = sizeof(PAT_MS2) / sizeof(PAT_MS2[0]);

const bool PAT_STATE3[]  = {1, 0, 1, 0, 1, 0};            // 1=ON, 0=OFF
const uint16_t PAT_MS3[] = {50, 50, 50, 50, 50, 300};   // 各状態の時間[ms]
const int PAT_LEN3 = sizeof(PAT_MS3) / sizeof(PAT_MS3[0]);

const bool PAT_STATE4[]  = {1, 0, 1, 0, 1, 0, 1, 0};            // 1=ON, 0=OFF
const uint16_t PAT_MS4[] = {50, 50, 50, 50, 50, 50, 50, 200};   // 各状態の時間[ms]
const int PAT_LEN4 = sizeof(PAT_MS4) / sizeof(PAT_MS4[0]);

const bool PAT_STATE5[]  = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};            // 1=ON, 0=OFF
const uint16_t PAT_MS5[] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 200};   // 各状態の時間[ms]
const int PAT_LEN5 = sizeof(PAT_MS4) / sizeof(PAT_MS4[0]);

const bool PAT_STATE6[]  = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};            // 1=ON, 0=OFF
const uint16_t PAT_MS6[] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 200};   // 各状態の時間[ms]
const int PAT_LEN6 = sizeof(PAT_MS4) / sizeof(PAT_MS4[0]);

int idx = 0;
uint32_t tp = 0;
bool stt =0;


struct BeepPat { const bool* s; const uint16_t* t; int n; };
BeepPat P1{ PAT_STATE1, PAT_MS1, PAT_LEN1 };
BeepPat P2{ PAT_STATE2, PAT_MS2, PAT_LEN2 };
BeepPat P3{ PAT_STATE3, PAT_MS3, PAT_LEN3 };
BeepPat P4{ PAT_STATE4, PAT_MS4, PAT_LEN4 };
BeepPat P5{ PAT_STATE5, PAT_MS5, PAT_LEN5 };
BeepPat P6{ PAT_STATE6, PAT_MS6, PAT_LEN6 };

BeepPat cur = P1;  // ←ここを変えるだけで切替

void sel_beep(int g){

  stt=0;  
 
  if(g ==1){
    cur = P1;
    stt=1;
  }
  else if(g ==2){
    cur = P2;
    stt=1;
  }
  else if(g ==3){
    cur = P3;
    stt=1;
  }
  else if(g ==4){
    cur = P4;
    stt=1;
  }
  else if(g >4){
    cur = P5;
    stt=1;
  }

}

char makeLogName(char *dst, size_t n,
                 const I2C_BM8563_DateTypeDef &d) {
  // 8.3形式: LOG_YYYYMMDD.TXT 例: 2025/08/11 → LOG_20250811.TXT
  snprintf(dst, n, "Log_%04d%02d%02d.txt", d.year, d.month, d.date);
  return 0;
}

void setting_sprite(){
  oilpres_sprite.setColorDepth(8);
  oilpres_sprite.setTextWrap(false);  // 改行をしない（画面をはみ出す時自動改行する場合はtrue）
  oilpres_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  oilpres_sprite.createSprite(95,40);

  oiltemp_sprite.setColorDepth(8);
  oiltemp_sprite.setTextWrap(false);  // 改行をしない（画面をはみ出す時自動改行する場合はtrue）
  oiltemp_sprite.setTextFont(1);
  oiltemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  oiltemp_sprite.createSprite(80,40);
  
  wtemp_sprite.setColorDepth(8);
  wtemp_sprite.setTextWrap(false);  // 改行をしない（画面をはみ出す時自動改行する場合はtrue）
  wtemp_sprite.setTextSize(1);
  wtemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  wtemp_sprite.createSprite(80,40);
  
  boost_sprite.setColorDepth(8);
  boost_sprite.setTextWrap(false);  // 改行をしない（画面をはみ出す時自動改行する場合はtrue）
  boost_sprite.setTextSize(1);
  boost_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  boost_sprite.createSprite(140,48);
  
  airtemp_sprite.setColorDepth(8);
  airtemp_sprite.setTextWrap(false);  // 改行をしない（画面をはみ出す時自動改行する場合はtrue）
  airtemp_sprite.setTextSize(1);
  airtemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  airtemp_sprite.createSprite(80,40);

  fuel_sprite.setColorDepth(8);
  fuel_sprite.setTextWrap(false);  // 改行をしない（画面をはみ出す時自動改行する場合はtrue）
  fuel_sprite.setTextSize(1);
  fuel_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  fuel_sprite.createSprite(55,40);


  time_diff_sprite.setColorDepth(8);
  time_diff_sprite.setTextWrap(false);  // 改行をしない（画面をはみ出す時自動改行する場合はtrue）
  time_diff_sprite.setTextSize(1);
  time_diff_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
//  time_diff_sprite.createSprite(32,15);
  time_diff_sprite.createSprite(40,15);
}

void SD_CSL(){
  pcf.digitalWrite(sdfat_cs_pcf8574_pin, LOW);
} 
void SD_CSH(){
  pcf.digitalWrite(sdfat_cs_pcf8574_pin, HIGH);
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&oilstatus, incomingData, sizeof(oilstatus));
//  tft.drawString("Received", 100, 120);
}

void init_pcf8574(){
  if(!pcf.begin(0x20, &Wire)) {
  //  Serial.println("Couldn't find PCF8574");
    while (1);
  }
//  else Serial.println("PCF8574 init success");
  
  pcf.pinMode(mcp_cs_pcf8574_pin, OUTPUT);
//  Serial.println("pinMode set");
  pcf.digitalWrite(mcp_cs_pcf8574_pin, HIGH);    
}

void init_can(){
  
  // MCP2515の初期化に成功した場合（ビットレート500kb/s ）
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK){
    // 特になにもしない    
  }else{  // 初期化に失敗した場合
    // 特になにもしない    
  }

  CAN0.init_Mask(0, 0, 0x07FF0000);   // there are 2 mask in mcp2515, you need to set both of them
  CAN0.init_Filt(0, 0, 0x018B0000);                          // there are 6 filter in mcp2515
  CAN0.init_Filt(1, 0, 0x01EA0000);                          // there are 6 filter in mcp2515

  CAN0.init_Mask(1, 0, 0x07FF0000);
  CAN0.init_Filt(2, 0, 0x03240000);                          // there are 6 filter in mcp2515
  CAN0.init_Filt(3, 0, 0x01A60000);                          // there are 6 filter in mcp2515
  CAN0.init_Filt(4, 0, 0x00000000);                          // there are 6 filter in mcp2515
  CAN0.init_Filt(5, 0, 0x00000000);                          // there are 6 filter in mcp2515

  // MCP2515を通常モードに設定
  CAN0.setMode(MCP_NORMAL);  
}

void receiveData(){

  String datas = "None";

//  if (CAN0.checkReceive() == CAN_MSGAVAIL) 
//  while (!digitalRead(CAN0_INT))
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    // バッファに格納する。
    if(rxId == 395){ //18B boost
      val_bst = ((rxBuf[0]*256+rxBuf[1])/10-760)/7.5/100;
      if(val_bst>p_bst) p_bst=val_bst;
      
      if(val_bst != pre_bst){
        if(ph_mode ==0){
          boost_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
          datas = String(val_bst);
        }
        if(ph_mode ==2){
          boost_sprite.setTextColor(TFT_RED, TFT_BLACK);
          datas = String(p_bst);
        }
        boost_sprite.fillScreen(TFT_BLACK);
        boost_sprite.drawRightString(datas, 140, 0,7);
        boost_sprite.pushSprite(40, 114);  // メモリ内に描画したcanvasを座標を指定して表示する
        
        pre_bst =val_bst;
      }
    }
    
    else if(rxId == 490){ //324 water temp& intake temp

      uint16_t rgx = ( (uint16_t)rxBuf[0]<<8)| (uint16_t)rxBuf[1];
      uint16_t rgy = ( (uint16_t)rxBuf[2]<<8)| (uint16_t)rxBuf[3];
      int16_t gx = (int16_t)rgx;
      int16_t gy = (int16_t)rgy;
      float gg = sqrt(gx*gx + gy*gy)*0.0015;
      g = (int)(gg/9.81*10);
      sel_beep(g);
    }
      
    else if(rxId == 804){ //324 water temp& intake temp
      val_wtemp = rxBuf[0]-40;
      val_atemp = rxBuf[1]-40;
      
      if(val_wtemp !=pre_wtemp){
        if(val_wtemp>p_wtemp) p_wtemp=val_wtemp;

        if(ph_mode ==0){
          wtemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
          datas = String(val_wtemp);
        }
        if(ph_mode ==2){
          wtemp_sprite.setTextColor(TFT_RED, TFT_BLACK);
          datas = String(p_wtemp);
        }
        wtemp_sprite.fillScreen(TFT_BLACK);
        if(val_wtemp < 100) wtemp_sprite.drawRightString(datas, 68,0,6); //2桁位置
        else wtemp_sprite.drawRightString(datas, 80,0,6); //3桁位置
        wtemp_sprite.pushSprite(120,65);

        pre_wtemp =val_wtemp;
      }
      
      if(val_atemp !=pre_atemp){
        if(val_atemp>p_atemp) p_atemp=val_atemp;
        
        if(ph_mode ==0){
          airtemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
          datas = String(val_atemp);
        }
        if(ph_mode ==2){
          airtemp_sprite.setTextColor(TFT_RED, TFT_BLACK);
          datas = String(p_atemp);
        }
        airtemp_sprite.fillScreen(TFT_BLACK);
        if(val_atemp<100) airtemp_sprite.drawRightString(datas, 65,0,6);//2桁位置
        else airtemp_sprite.drawRightString(datas, 80,0,6);//3桁位置
        airtemp_sprite.pushSprite(50, 170);
        
        pre_atemp =val_atemp; 
      }
    }
    
    else if(rxId == 422){ //1A6 Fuel stock and Day light
      val_light = rxBuf[0]&0b00000011;

      if(val_light !=pre_light){
        if(val_light ==0) BL_lvl = Day_BL_lvl;
        else if(val_light >0 && val_light < 3) BL_lvl = Night_BL_lvl;
        analogWrite(XIAO_BL, BL_lvl);
        pre_light = val_light;
      }
        
      val_fuel = int(25*(rxBuf[3] -5)/100);
      if(val_fuel<p_fuel) p_fuel=val_fuel;
      
      if(val_fuel != pre_fuel){
        if(val_fuel<p_fuel) p_fuel=val_fuel;
        
        if(ph_mode ==0){
          fuel_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
          datas = String(val_fuel);
        }
        if(ph_mode ==2){
          fuel_sprite.setTextColor(TFT_RED, TFT_BLACK);
          datas = String(p_fuel);
        }
        fuel_sprite.fillScreen(TFT_BLACK);
        fuel_sprite.drawRightString(datas, 55,0,6);
        fuel_sprite.pushSprite(136, 170);

        pre_bst =val_bst;
      }
    }
    
  }
}

void receiveESPdata(){

  String datas = "None";
  
  if(oilstatus.temp != pre_otemp){

    if(oilstatus.temp > p_otemp) p_otemp=oilstatus.temp;
    pre_otemp =oilstatus.temp;
    
    if(ph_mode ==0){
      oiltemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
      datas = String(oilstatus.temp);      
    }
    else if(ph_mode ==2){
      oiltemp_sprite.setTextColor(TFT_RED, TFT_BLACK);
      datas = String(p_otemp);
    }
    
    oiltemp_sprite.fillScreen(TFT_BLACK);
    if(oilstatus.temp < 100) oiltemp_sprite.drawRightString(datas, 75,0,6); //2桁位置
    else oiltemp_sprite.drawRightString(datas, 80,0,6); //3桁位置 
    oiltemp_sprite.pushSprite(35, 65);   
  }

  if(oilstatus.pres != pre_opres){

    if(oilstatus.pres < p_opres) p_opres=oilstatus.pres;
    pre_opres =oilstatus.pres;

    if(ph_mode ==0){
      oilpres_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
      datas = String(oilstatus.pres);
    }
    else if(ph_mode ==2){
      oilpres_sprite.setTextColor(TFT_RED, TFT_BLACK);
      datas = String(p_opres);      
    }
    oilpres_sprite.fillScreen(TFT_BLACK);
    oilpres_sprite.drawRightString(datas, 95,0,6);
    oilpres_sprite.pushSprite(73, 16);
  }

  
}



void settouchflag(){
      touch_flag=1;    
}

void show_peak(){
  
  String datas = "None";

  oilpres_sprite.setTextColor(TFT_RED, TFT_BLACK);
  oilpres_sprite.setTextFont(1);
  datas = String(p_opres);
  oilpres_sprite.fillScreen(TFT_BLACK);
  oilpres_sprite.drawRightString(datas, 95,0,6);
  oilpres_sprite.pushSprite(73, 16);

  oiltemp_sprite.setTextColor(TFT_RED, TFT_BLACK);
  datas = String(p_otemp);
  oiltemp_sprite.fillScreen(TFT_BLACK);
  if(p_otemp < 100) oiltemp_sprite.drawRightString(datas, 76,0,6); //2桁位置
  else oiltemp_sprite.drawRightString(datas, 80,0,6); //3桁位置 
  oiltemp_sprite.pushSprite(35, 65);
  
  wtemp_sprite.setTextColor(TFT_RED, TFT_BLACK);
  datas = String(p_wtemp);
  wtemp_sprite.fillScreen(TFT_BLACK);
  if(p_wtemp < 100) wtemp_sprite.drawRightString(datas, 68,0,6); //2桁位置
  else wtemp_sprite.drawRightString(datas, 80,0,6); //3桁位置
  wtemp_sprite.pushSprite(120,65);
  
  boost_sprite.setTextColor(TFT_RED, TFT_BLACK);
  datas = String(p_bst);
  boost_sprite.fillScreen(TFT_BLACK);
  boost_sprite.drawRightString(datas, 140, 0,7);
  boost_sprite.pushSprite(40, 114);  // メモリ内に描画したcanvasを座標を指定して表示する

  airtemp_sprite.setTextColor(TFT_RED, TFT_BLACK);
  datas = String(p_atemp);
  airtemp_sprite.fillScreen(TFT_BLACK);
  if(p_atemp<100) airtemp_sprite.drawRightString(datas, 65,0,6);//2桁位置
  else airtemp_sprite.drawRightString(datas, 80,0,6);//3桁位置
  airtemp_sprite.pushSprite(50, 170);
  
  fuel_sprite.setTextColor(TFT_RED, TFT_BLACK);
  datas = String(p_fuel);
  fuel_sprite.fillScreen(TFT_BLACK);
  fuel_sprite.drawRightString(datas, 55,0,6);
  fuel_sprite.pushSprite(136, 170);
  
}

void reshow_screen(){
  
  String datas = "None";

  oilpres_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  oilpres_sprite.setTextFont(1);
  datas = String(oilstatus.pres);
  oilpres_sprite.fillScreen(TFT_BLACK);
  oilpres_sprite.drawRightString(datas, 95,0,6);
  oilpres_sprite.pushSprite(73, 16);

  oiltemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  oiltemp_sprite.setTextFont(1);
  datas = String(oilstatus.temp);
  oiltemp_sprite.fillScreen(TFT_BLACK);
  if(oilstatus.temp < 100) oiltemp_sprite.drawRightString(datas, 76,0,6); //2桁位置
  else oiltemp_sprite.drawRightString(datas, 80,0,6); //3桁位置 
  oiltemp_sprite.pushSprite(35, 65);
  
  wtemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  datas = String(val_wtemp);
  wtemp_sprite.fillScreen(TFT_BLACK);
  if(val_wtemp < 100) wtemp_sprite.drawRightString(datas, 68,0,6); //2桁位置
  else wtemp_sprite.drawRightString(datas, 80,0,6); //3桁位置
  wtemp_sprite.pushSprite(120,65);
  
  boost_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  datas = String(val_bst);
  boost_sprite.fillScreen(TFT_BLACK);
  boost_sprite.drawRightString(datas, 140, 0,7);
  boost_sprite.pushSprite(40, 114);  // メモリ内に描画したcanvasを座標を指定して表示する

  airtemp_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  datas = String(val_atemp);
  airtemp_sprite.fillScreen(TFT_BLACK);
  if(val_atemp<100) airtemp_sprite.drawRightString(datas, 55,0,6);//2桁位置
  else airtemp_sprite.drawRightString(datas, 80,0,6);//3桁位置
  airtemp_sprite.pushSprite(50, 170);
  
  fuel_sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  datas = String(val_fuel);
  fuel_sprite.fillScreen(TFT_BLACK);
  fuel_sprite.drawRightString(datas, 55,0,6);
  fuel_sprite.pushSprite(136, 170);
  
}

void setup() {
  // Open serial communications and wait for port to open:
//  Serial.begin(115200);
//  Serial1.begin(115200, SERIAL_8N1, D7, D6);

// Initialise the screen
//  tft.setRotation(0);
  analogWrite(XIAO_BL, 0);
  Wire.begin();
  tft.init();
//  tft.fillScreen(TFT_BLACK);
  tft.pushImage(0, 0, 240, 240, (uint16_t *)s660_logo);
  tft.setTextColor(TFT_WHITE);
  tft.setTextWrap(false);
  tft.setTextDatum(MC_DATUM);
  pinMode(TOUCH_INT, INPUT_PULLUP);
  analogWrite(XIAO_BL, 10);

  setting_sprite();
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() == ESP_OK) {
    ;
  } else {
    ESP.restart();
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  pinMode(D1, OUTPUT);
  digitalWrite(D1, HIGH);

  init_pcf8574(); 
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  init_can();
  digitalWrite(D1, HIGH);

  rtc.begin();
  // tft.init();

  tft.pushImage(0, 0, 240, 240, (uint16_t *)neon_ring);
  tft.setRotation(0);
  analogWrite(XIAO_BL, BL_lvl);
  
  tft.fillCircle(120,120,115,TFT_BLACK);
  tft.setTextFont(2); //default:2
  tft.setTextDatum(MC_DATUM);
  tft.setTextWrap(false);
  
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.drawString("Oil pressure", 121, 60);
  tft.drawString("Oil temp", 85, 109);
  tft.drawString("Water temp", 162, 109);
  tft.drawString("Boost", 121, 166);
  tft.drawString("Intake", 90, 214);
  tft.drawString("Fuel", 163, 214);

  
//  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
//  tft.drawString("x100kPa", 180, 50);
//  tft.drawString("deg", 130, 99);
//  tft.drawString("deg", 192, 99);
//  tft.drawString("x100kPa", 181, 146);
//  tft.drawString("deg", 120, 204);
//  tft.drawString("L", 193, 204);
  
  attachInterrupt(TOUCH_INT, settouchflag, CHANGE);

    // RTC取得
  rtc.getDate(&dateStruct);

  ledcSetup(CH, FREQ, RES);
  ledcAttachPin(BUZZER_PIN, CH);
  tp = millis();

}

uint32_t cur_ms = 0; // 符号なし32bit整数型
uint32_t pre_ms = 0; // 符号なし32bit整数型
uint32_t diff_ms = 0; // 符号なし32bit整数型
int pre_tx =0;
int pre_ty =0;

void loop() {


  String datas = "None";
  cur_ms = micros();
  diff_ms =cur_ms -pre_ms;
  pre_ms =cur_ms;
  datas = String(diff_ms);

  time_diff_sprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
  time_diff_sprite.fillScreen(TFT_BLACK);
  time_diff_sprite.setTextFont(0);
  time_diff_sprite.drawRightString(datas, 40, 0, 2);
  time_diff_sprite.pushSprite(105,220);

  receiveData();
  receiveESPdata();

  if(ph_mode == 1 || ph_mode ==2){
    show_peak();
    ph_mode = 2;
  }
  else if(ph_mode ==3){
    reshow_screen();
    ph_mode = 0;
  }

  if(touch_flag ==1){
    chsc6x_get_xy(&touchX, &touchY);
    if(touchX != pre_tx && touchY !=pre_ty){
      if(touchX <40){
        if(BL_lvl == Night_BL_lvl){
          BL_lvl -= 1;
          if(BL_lvl<0) BL_lvl=0;
          Night_BL_lvl =BL_lvl;
        }
        if(BL_lvl == Day_BL_lvl){
          BL_lvl -= 25;
          if(BL_lvl<0) BL_lvl=0;
          Day_BL_lvl =BL_lvl;
        }
        analogWrite(XIAO_BL, BL_lvl);
      }
      
      else if(touchX >200){
        if(BL_lvl == Night_BL_lvl){
          BL_lvl += 1;
          if(BL_lvl>255) BL_lvl=255;
          Night_BL_lvl =BL_lvl;
        }
        if(BL_lvl == Day_BL_lvl){
          BL_lvl += 50;
          if(BL_lvl>255) BL_lvl=255;
          Day_BL_lvl =BL_lvl;
        }
         analogWrite(XIAO_BL, BL_lvl);       
      }
      else if(touchX>=40&&touchX<=200&&touchY >=40){        
        if(ph_mode ==0) ph_mode=1;
        else if(ph_mode ==2) ph_mode=3;
      }
      else{
        p_wtemp=0;
        p_atemp = 0;
        p_fuel = 25;
        p_bst =-0.99f;
        p_otemp = 0;
        p_opres = 9.99;
        
        pre_wtemp=0;
        pre_atemp = 0;
        pre_fuel = 25;
        pre_bst =-0.99f;
        pre_otemp = 0;
        pre_opres = 9.99;
      }

      pre_tx =touchX;
      pre_ty =touchY;
    }
    touch_flag =0;
  }

  uint32_t now = millis();
  if ((now - tp >= cur.t[idx]) && (stt == 1)) {
    idx = (idx + 1) % cur.n;
    ledcWrite(CH, cur.s[idx] ? DUTY : 0);
    tp = now;
  }
  else{
    ledcWrite(CH, 0);
  }

}
