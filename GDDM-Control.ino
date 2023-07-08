#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

//yard-meter換算
#define METER_TO_YARDS 0.9144

//GPSシリアル通信
#define GPS_RXPIN 2
#define GPS_TXPIN 3
#define GPS_BAUD 9600

//SDカード
#define SD_CHIPSELECT 10
#define MAPDATAFILE "mapdata.csv"

//Nextionディスプレイ
#define NEX_BAUD 9600
#define NEX_RCVING_TIMEOUT 100

//プッシュボタン
#define SW_DISPONOFF_PIN 21
#define SW_FILTERTIME 1000

SoftwareSerial gpsSerial(GPS_RXPIN, GPS_TXPIN);

class MapDataHandler{
  public:
    typedef struct HoleData{
      int hole;
      int par;
      float LatLeft;
      float LngLeft;
      float LatRight;
      float LngRight;      
    };

    HoleData currentHoleData;

    int initialize(){
      File MapFile;
      if(!SD.begin(SD_CHIPSELECT)){
        return 1;
      }
      MapFile = SD.open(MAPDATAFILE);
      if(!MapFile){
        MapFile.close();
        return 2;
      }
      int HoleDataCounter = 0;
      while(MapFile.available()){
        HoleDataCounter++;
        MapFile.readStringUntil('\n');
      }
      if(HoleDataCounter!=18){
        MapFile.close();
        Serial.println("Illegal MapData");
        return 3;
      }
      getHoleDataSet(1);
      MapFile.close();      
      return 0;
    }
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
     * マップデータの構成
     * ファイル名：mapdata.csv
     * 1行あたり1ホール分の情報，全18行(ホール)分
     * 1行のデータ構成：
     *  (パー数),(左側ピン緯度),(左側ピン経度),(右側ピン緯度),(右側ピン経度)
     */
    void getHoleDataSet(int hole){
      currentHoleData.par = parseHoleDataString(getHoleDataString(hole),0).toInt();
      currentHoleData.LatLeft = parseHoleDataString(getHoleDataString(hole),1).toFloat();
      currentHoleData.LngLeft = parseHoleDataString(getHoleDataString(hole),2).toFloat();
      currentHoleData.LatRight = parseHoleDataString(getHoleDataString(hole),3).toFloat();
      currentHoleData.LngRight = parseHoleDataString(getHoleDataString(hole),4).toFloat();
    }

    String parseHoleDataString(String SourceString, int TargetElementNumber){
      String TargetString;
      int ElementStringBeginIndex = 0;
      int ElementStringEndIndex;
      for(int elementCounter = 0; elementCounter <= TargetElementNumber; elementCounter++){
        ElementStringEndIndex = SourceString.indexOf(',',ElementStringBeginIndex);
        TargetString = SourceString.substring(ElementStringBeginIndex,ElementStringEndIndex);
        ElementStringBeginIndex = ElementStringEndIndex + 1;
      }
      Serial.println("MDH DS:"+TargetString);
      return TargetString;
    }

    String getHoleDataString(int TargetHoleNumber){
      File MapFile;
      String TargetString;
      if(TargetHoleNumber > 18){
        return "Invalid Hole Number" ;
      }
      MapFile = SD.open(MAPDATAFILE);
      if(!MapFile){
        MapFile.close();
        return "MapFile is Invalid";
      }
      //while(MapFile.available()){
      for(int counter = 1; counter <= TargetHoleNumber; counter++){
        TargetString = MapFile.readStringUntil('\n');
      }
      //}
      MapFile.close();
      return TargetString;
    }
      
}mapdata;

class GeoSensorControl{
  public:
    TinyGPSPlus gps;
    DFRobot_QMC5883 cps;

    struct target{
      float lat;
      float lng;
    }target;

    struct cpsCalib{
      float ofsXAxis;
      float ofsYAxis;
      float magDeclination;
    }cpsCalib;
    
    typedef struct jpnTime{
      int hr;
      int min;
    };

    float initialize(){
      gpsSerial.begin(GPS_BAUD);
      cps.begin();
      cps.setRange(QMC5883_RANGE_2GA);
      cps.setMeasurementMode(QMC5883_CONTINOUS);
      cps.setDataRate(QMC5883_DATARATE_50HZ);
      cps.setSamples(QMC5883_SAMPLES_8);
    }

    bool getGPSData(){
      if(gpsSerial.available()>0){
        if(gps.encode(gpsSerial.read())){
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    }

    void changeTarget(float lat, float lng){
      target.lat = lat;
      target.lng = lng;
    }

    jpnTime getJpnTime(){
      jpnTime time;
      time.hr = gps.time.hour() + 9;
      time.min = gps.time.minute();
      if(time.hr > 23){
        time.hr -=24;
      }
      return time;
    } 

    float calcCpsNorthDirection(){
    //前方に対する北の方向の角度を計算
    //左：0deg 正面：90deg 右：180deg 後方：270deg
      float angle;
      float X = cpsCalib.ofsXAxis - cps.readRaw().XAxis;
      float Y = cpsCalib.ofsYAxis - cps.readRaw().YAxis;
      angle = atan2(Y, -X) * 180/PI;
      if(angle < 0){
        angle += 360;
      }
      return angle - cpsCalib.magDeclination + 0.5;
    }

    float calcCpsForwardDirection(float NorthDirection){
    //前方の方角を計算
    //西：0deg 北：90deg 東：180deg 南：270deg
      float ret;
      ret = 180 - NorthDirection;
      if(ret < 0){
        ret += 360;
      }
      return ret + 0.5;
    }

    float calcGpsTarget(){
    //目標の方角を計算
    //西：0deg 北：90deg 東：180deg 南：270deg
      float ret;
      ret = gps.courseTo(gps.location.lat(), gps.location.lng(), target.lat, target.lng);
      ret +=90;
      if(ret > 360){
        ret -= 360;
      }
      return ret + 0.5;
    }

    float calcTargetDirection(){
      float ret;
      ret = calcGpsTarget() - calcCpsForwardDirection(calcCpsNorthDirection()) + 90;
      if(ret < 0){
        ret += 360;
      } else if (ret > 360){
        ret -= 360;
      }
      return ret + 0.5;
    }

    float calcDistance(){
      float ret;
      ret = gps.distanceBetween(gps.location.lat(), gps.location.lng(), target.lat, target.lng);
      return ret * METER_TO_YARDS + 0.5;
    }
}sensor;

class NexControl{
  #define nexSerial Serial1
  public:
    struct routineData{
      int satellite;
      int hr;
      int min;
    }routineData;

    struct mainData{
      float distance;
      float direction;
      float forward;
      float target;
      float lat;
      float lng;
    }mainData;

    struct mapData{
      int hole;
      int par;
      int green; //1:左,2:右
    }mapData;

    bool gotoSleepMode(){
      nexSerial.print("sleep=1");
      sendTerminate();
      return true;
    }

    bool diactivateSleepMode(){
      nexSerial.print("sleep=0");
      sendTerminate();
      return false;
    }

    void setRoutineData(int satellite, int hr, int min){
      routineData.satellite = satellite;
      routineData.hr = hr;
      routineData.min = min;
    }

    void setMainData(float distance, float direction, float forward, float target, float lat, float lng){
      mainData.distance = distance;
      mainData.direction = direction;
      mainData.forward = forward;
      mainData.target = target;
      mainData.lat = lat;
      mainData.lng = lng;
    }

    void setMapData(int hole, int par, int green){
      mapData.hole = hole;
      mapData.par = par;
      mapData.green = green;
    }

    void initialize(int initMapDataCheck){
      nexSerial.begin(9600);
      delay(1500);
      initErrPageTransition(initMapDataCheck);
    }

    void initErrPageTransition(int init_state){
      switch(init_state){
        case 1: //SDカードがないとき
          sendPageTrans("initerr");
          sendObjHide("tINIERR2");
          break;
        case 2: //マップデータがないとき
          sendPageTrans("initerr");
          sendObjHide("tINIERR1");
          break;
        default:
          sendPageTrans("main");
          break;
      }
    }

    void sendRoutineData(bool isValidTime){
      sendCnvObjValue("sat",routineData.satellite);
      if(isValidTime){
        sendCnvObjValue("hour",routineData.hr);
        sendCnvObjValue("min",routineData.min);
      }else{
        sendCnvObjValue("hour",-1);
        sendCnvObjValue("min",-1);
      }
    }

    void sendMainData(bool isValidLocation){
      long dispLat = mainData.lat * 1000000;
      long dispLng = mainData.lng * 100000;
      sendCnvObjValue("compass",mainData.forward);
      if(isValidLocation){
        sendObjHide("msgPosInvalid");
        sendCnvObjValue("distance",mainData.distance);
        sendCnvObjValue("direction",mainData.direction);
        //sendCnvObjValue(" ",data.target);
        sendCnvObjValue("lat",dispLat);
        sendCnvObjValue("lng",dispLng);
      }else{
        sendObjVisible("msgPosInvalid");        
        sendCnvObjValue("distance",-1);
        //sendCnvObjValue("direction",data.direction);
        //sendCnvObjValue(" ",data.target);
        sendCnvObjValue("lat",0);
        sendCnvObjValue("lng",0);
      }
    }

    void sendMapData(){
      sendCnvObjValue("hole",mapData.hole);
      sendCnvObjValue("par",mapData.par);
      if(mapData.green == 1){
        sendObjVisible("cgL"); //指示場所変える
        sendObjHide("cgR");
      } else if(mapData.green == 2){
        sendObjVisible("cgR");
        sendObjHide("cgL");
      }
    }

  private:
    void sendPageTrans(String pageName){
      nexSerial.print("page ");
      nexSerial.print(pageName);
      sendTerminate();
    }

    void sendObjVisible(String objectName){
      nexSerial.print("vis ");
      nexSerial.print(objectName);
      nexSerial.print(",1");
      sendTerminate();
    }

    void sendObjHide(String objectName){
      nexSerial.print("vis ");
      nexSerial.print(objectName);
      nexSerial.print(",0");
      sendTerminate();
    }

    void sendCnvObjValue(String objectName, float value){
      long Lvalue = (long) value;
      nexSerial.print("cov \"");
      nexSerial.print(Lvalue);
      nexSerial.print("\",");
      nexSerial.print(objectName);
      nexSerial.print(".val,0");
      sendTerminate();
    }

    void sendTerminate(){
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
    }
}display;

class SystemControl{
  public:
    struct InternalData{
      uint8_t MainMode;
      uint8_t SubMode;
      int Hole;
      int Green; //1:Left 2:Right
      bool SleepState;
    }InternalData;    

    struct Command{
      bool shouldUpdateMapData; 
    }Command;

    /* * * * * * * * * * * * * * * * * * * * *
     * Nextionからの指令信号の構成
     * 1～3バイト目：指令開始信号
     *  [0xF0 0xF1 0xF2]であれば指令開始
     * 4バイト目：MainMode選択
     *  0x00(0)：メイン画面(距離・方角表示)
     *  0x01(1)：選択画面(ホール，グリーン)
     * 5バイト目：SubMode選択
     *  MainMode:0
     *   ->0x01：ホール変更してメイン画面へ
     *   ->0x02：グリーン変更してメイン画面へ
     *  MainMode:1
     *   ->0x01：ホール選択画面へ
     *   ->0x02：グリーン選択画面へ
     * 6バイト目：指令値
     *  Main:0/Sub:1
     *   ->ホール番号
     *  Main:0/Sub:1
     *   ->グリーン選択(1:左/2:右)
     */

    void recieveCommand(){
      if(isCommand()){
        InternalData.MainMode = recieveWord();
        InternalData.SubMode = recieveWord();
        int CmdValue = recieveWord();  
        executeCommand(InternalData.MainMode,InternalData.SubMode,CmdValue);
      }
      wasteMessage();
    }

  private:
    void executeCommand(uint8_t MainMode, uint8_t SubMode, int CmdValue){
      switch(MainMode){
        case 0:
          setInternalData(SubMode, CmdValue);
          break;
        case 1:
          switch(SubMode){
            case 1:
              Serial.println("Sys:hole select");
              break;
            case 2:
              Serial.println("Sys:green select");
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    }

    void setInternalData(uint8_t SubMode, int CmdValue){
      switch(SubMode){
        case 1:
          InternalData.Hole = CmdValue;
          Command.shouldUpdateMapData = true;
          break;         
        case 2:
          InternalData.Green = CmdValue;
          Command.shouldUpdateMapData = true;
          break;
        default:
          break;
      }
    }

    bool isCommand(){
      byte header[3];
      if(nexSerial.available()){
        for(int counter = 0;counter <= 2;counter++){
          Serial.print("Sys Rcv:h");
          header[counter] = recieveWord();
        }
      } else {
        return false;
      }
      if(header[0] == 0xF0 && header[1] == 0xF1 && header[2] == 0xF2){   
        Serial.println("Sys Rcv:Command Rcvd");
        return true;
      } else {
        wasteMessage();
        return false;
      }
    }

    uint8_t recieveWord(){
      uint8_t word;
      uint32_t start = millis();
      while(millis() - start <= NEX_RCVING_TIMEOUT){
        if(nexSerial.available()){
          word = nexSerial.read();
          Serial.println(word);
          return word;
        }
      }
      Serial.print("Sys Rcv:Timeout now:");
      Serial.print(millis()); 
      Serial.print(" /start:");
      Serial.println(start);
      return 99;
    }

    void wasteMessage(){
      while(nexSerial.available()){
        Serial.print("Sys Rcv:w");
        uint8_t waste = recieveWord();
      }
    }

}System;

volatile uint32_t LastTimePushingSW;

void Interrupt_DisplayOnOff(){
  if(millis() - LastTimePushingSW >= SW_FILTERTIME){
    if(System.InternalData.SleepState){
      System.InternalData.SleepState = display.diactivateSleepMode();
    } else {
      System.InternalData.SleepState = display.gotoSleepMode();
    }
    LastTimePushingSW = millis();
  }
}

void setup() {

  Serial.begin(9600);
  sensor.initialize();
  display.initialize(mapdata.initialize());

  System.InternalData.Hole = 1;
  System.InternalData.Green = 1;
  System.Command.shouldUpdateMapData = true;

  pinMode(2,INPUT);
  attachInterrupt(digitalPinToInterrupt(SW_DISPONOFF_PIN),Interrupt_DisplayOnOff,RISING);
  System.InternalData.SleepState = false;

}

void loop() {
  if(!System.InternalData.SleepState){
    if(sensor.getGPSData()){
      display.setRoutineData(
        sensor.gps.satellites.value(), 
        sensor.getJpnTime().hr, 
        sensor.getJpnTime().min
      );
      display.sendRoutineData(sensor.gps.time.isValid());

      if(System.InternalData.MainMode == 0){
        display.setMainData(
          sensor.calcDistance(), 
          sensor.calcTargetDirection(), 
          sensor.calcCpsNorthDirection(), 
          sensor.calcGpsTarget(), 
          sensor.gps.location.lat(), 
          sensor.gps.location.lng()
        );
        display.sendMainData(sensor.gps.location.isValid());
      }
    }
  }

  System.recieveCommand();
  if(System.Command.shouldUpdateMapData){
    mapdata.getHoleDataSet(System.InternalData.Hole);
    display.setMapData(
      System.InternalData.Hole,
      mapdata.currentHoleData.par,
      System.InternalData.Green
    );
    display.sendMapData();
    Serial.print("Main:Updated MapData:hole=");
    Serial.print(System.InternalData.Hole);
    if(System.InternalData.Green == 1){
      sensor.target.lat = mapdata.currentHoleData.LatLeft;
      sensor.target.lng = mapdata.currentHoleData.LngLeft;
      long printlat = sensor.target.lat * 10000000;
      long printlng = sensor.target.lng * 10000000;
      Serial.print(",green=L target=");
      Serial.print(printlat);
      Serial.print("/");
      Serial.println(printlng);
    } else {
      sensor.target.lat = mapdata.currentHoleData.LatRight;
      sensor.target.lng = mapdata.currentHoleData.LngRight;
      long printlat = sensor.target.lat * 10000000;
      long printlng = sensor.target.lng * 10000000;
      Serial.print(",green=R target=");
      Serial.print(printlat);
      Serial.print("/");
      Serial.println(printlng);
    }
    System.Command.shouldUpdateMapData = false;
  }
}