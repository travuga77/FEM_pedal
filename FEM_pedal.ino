
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10


int lSensorData, rSensorData, lSensorConv, rSensorConv;
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

const byte lSensorPin = 1;
const byte rSensorPin = 6;

unsigned long timer = 0;
int errCode;

void CanMsgSend(int value, int err)
{
  byte sndStat;
  data[0] = value%256;
  data[1]= value>>8;
  sndStat = CAN0.sendMsgBuf(0x150, 0, 2, data);
 // Serial.print(value);
  if (millis()-timer>1000)
  {
     data[0] = err%256;
    data[1]= err>>8;
    sndStat = CAN0.sendMsgBuf(0x151, 0, 2, data);
    timer = millis();
    Serial.println(err);
  }
  /*if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
    } else {
    Serial.println("Error Sending Message...");
  }*/
}

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
  pinMode(A6, INPUT);
  digitalWrite(A6, LOW);
  pinMode(A1, INPUT);
  digitalWrite(A1, LOW);
  
  //Serial.println("MCP2515 Library Receive Example...");

}



void loop() {
 // put your main code here, to run repeatedly:
  delay(45);
  
 //timer = millis();
 lSensorData = analogRead(lSensorPin);
 rSensorData = analogRead(rSensorPin);
 /*if (rSensorData>100&&rSensorData<900&&lSensorData>100&&lSensorData<900)
 {
    errCode = 0;
   // CanMsgSend(0, 601);
  //  return;
 }
 if (rSensorData<100&&lSensorData>100&&lSensorData<900)
 {
    errCode = 601;
    CanMsgSend(0, 601);
    return;
 }
 if (lSensorData<1&&rSensorData>100&&rSensorData<900)
 {
    errCode = 602;
    CanMsgSend(0, 602);
    return;
 }
 if (lSensorData<100&&rSensorData<100)
 {
    errCode = 603;
    CanMsgSend(0, 603);
    return;
 }
 if (rSensorData>900&&lSensorData>100&&lSensorData<900)
 {
    errCode = 611;
    CanMsgSend(0, 611);
    return;
 }
 if (lSensorData>900&&rSensorData>100&&rSensorData<900)
 {
    errCode = 612;
    CanMsgSend(0, 612);
    return;
 }
 if (lSensorData>900&&rSensorData>900)
 {
    errCode = 613;
    CanMsgSend(0, 613);
    return;
 }
 if (lSensorData>900&&rSensorData<100)
 {
    errCode = 607;
    CanMsgSend(0, 607);
    return;
 }
 if (lSensorData<100&&rSensorData>900)
 {  
    errCode = 606;
    CanMsgSend(0, 606);
    return;
 }
 */
 
 lSensorConv = map(lSensorData, 695, 1010, 0, 4096);
 rSensorConv = map(rSensorData, 30, 310, 0, 4096);
 lSensorConv = constrain(lSensorConv, 0, 4095);
 rSensorConv = constrain(rSensorConv,0, 4095);
 int minim;
 if (lSensorConv<rSensorConv) minim=lSensorConv; else minim=rSensorConv;
 if(abs(lSensorConv-rSensorConv)>820)
 {
    CanMsgSend(0, 604);
    minim = 0;
    return;
 }
 CanMsgSend(minim, 0);
 
  String debug;
  debug = String(minim) + ";" + String(lSensorData) + ";" + String(rSensorData) + ";" + String(lSensorConv) + ";" + String(rSensorConv);
  Serial.println(debug);
  debug = "";
 //Serial.println();
 //data[0] = minim%256;
 //data[1]=minim>>8;

 //data = analogRead(rSensorPin);
 //sndStat = CAN0.sendMsgBuf(0x100, 0, 2, data);
 //Serial.println(rSensorData);
 //Serial.println(lSensorData);
}
