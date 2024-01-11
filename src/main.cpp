#include <Arduino.h>
#include <string.h>
#include <FastLED.h>
#include "main.h"


#define uchar unsigned char
#define LED_PIN   13
#define RGB_PIN   2         /*RGB灯使用IO17进行控制*/
#define NUM_LEDS  15         /*RGB灯是串联的，RGB灯的数量*/
CRGB leds[NUM_LEDS];        /*CRGB是结构体类型*/


/*
 *功能:RGB灯初始化
 *参数:无
 *返回值:无
 */
void RGB_Init(void)
{
  FastLED.addLeds<WS2812, RGB_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(12);  // 设置初始亮度为50%，数值范围在0到255之间

}

// 清除串口缓冲区
void clearSerialBuffer(void) {
    while (Serial.available() > 0) {
        char t = Serial.read(); // 读取并丢弃缓冲区中的数据
    }
}

//////////////////////////////////////////////////////////////////////////////////////


#define BUFFER_SIZE 262
#define PACKET_SIZE 262


// 检查奇校验
bool checkOddParity(uint8_t *packet, size_t length) {
  bool parity_result = false;
  for (size_t i = 0; i < length; i++) {
      uint8_t byte = packet[i];
      bool bit_parity = false;
      for (int j = 0; j < 8; j++) {
          bit_parity ^= (byte >> j) & 0x01;
      }
      parity_result ^= bit_parity;
  }
  return parity_result;
}

// Function to check even parity
bool check_parity(uint8_t byte) {
    uint8_t parity = 0;
    for (int i = 0; i < 8; i++) {
        parity ^= (byte >> i) & 0x01;
    }
    return parity;
}

// 读取和处理数据包
bool readAndProcessPacket(HardwareSerial Serial_p, uint8_t *bufferPacket_p, size_t *bufferLength_p) {
  Serial.println("readAndProcessPacket");
  // 等待缓冲区有足够的数据
  if (Serial.available() < 6) {

    return false;
  }

  // 读取包头
  uint8_t header1 = Serial.read();
  if (header1 != 0x2E) {
    Serial.println("header1 error\n");
    return false; // 包头不匹配
  }
  uint8_t header2 = Serial.read();
  if (header2 != 0x13) {
    Serial.println("header2 error\n");
    return false; // 包头不匹配
  }

  // 读取包体长度位，最大256（不包括包体长度位和校验位）
  uint8_t payloadLength = Serial.read();

  // 读取包体和数据长度位
  uint8_t packet[PACKET_SIZE];
  packet[0] = header1;
  packet[1] = header2;
  packet[2] = payloadLength;
  for (int i = 3; i < (payloadLength + 3); i++) {
    packet[i] = Serial.read();
  }

  packet[3 + payloadLength] = Serial.read(); // 读入校验位
  // 检查奇校验
  if (!checkOddParity(packet, (payloadLength + 3)) == packet[3 + payloadLength]) {
    Serial.println("odd parity error\n");
    return false; // 校验失败
  }

  // 读取并检查包尾
  uint8_t footer1 = Serial.read();
  if (footer1 != 0xF2) {
    Serial.println("footer1 error\n");
    return false; // 包尾不匹配
  }
  else {
    packet[4 + payloadLength] = footer1;
  }
  uint8_t footer2 = Serial.read();
  if (footer2 != 0x13) {
    Serial.println("footer2 error\n");
    return false; // 包尾不匹配
  }
  else {
    packet[5 + payloadLength] = footer2;
  }

  // for (int i = (4 + payloadLength); i < (4 + payloadLength + 2); i++) {
  //   packet[i] = Serial.read();
  // }

  // 将数据包存储到buffer中,总大小最大262
  memcpy(bufferPacket_p, packet, (payloadLength + 6)); // 包头2位 + 包体长度位1位 + 包体长度 + 奇校验1位 + 包尾2位
  *bufferLength_p = payloadLength + 6;

  Serial.println("packet_OK");
  return true;
}

// 打印完整数据包
void printReceivedPacket(const uint8_t* packet, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (i > 0) Serial.print(" ");
        Serial.print(packet[i], HEX); // 以十六进制格式打印
    }
    Serial.println(); // 换行
}

void createPacket(const uint8_t* payload, size_t payloadLength, uint8_t* packet, size_t* packetLength) {
    Serial.println("create packet...");
    // 定义包头、包尾
    const uint8_t header1 = 0x2E, header2 = 0x13;
    const uint8_t footer1 = 0xF2, footer2 = 0x13;

    // 计算数据包长度
    *packetLength = payloadLength + 6; // 包头2位 + 包体长度位1位 + 包体长度 + 奇校验位1位 + 包尾2位

    // 填充包头
    packet[0] = header1;
    packet[1] = header2;

    // 填充包体长度位
    packet[2] = static_cast<uint8_t>(payloadLength);

    // 填充包体
    memcpy(&packet[3], payload, payloadLength);

    // 计算并填充奇校验位
    // uint8_t parity = 0;
    // for (size_t i = 0; i < payloadLength + 3; i++) {
    //     parity ^= packet[i];
    // }
    // parity ^= parity >> 4;
    // parity ^= parity >> 2;
    // parity ^= parity >> 1;
    packet[payloadLength + 3] = checkOddParity(packet, (payloadLength + 3));
    // 填充包尾
    packet[payloadLength + 4] = footer1;
    packet[payloadLength + 5] = footer2;
}

void changeLed()
{

}


/*
 *功能：RGB灯控制
 *参数：
  cresset：灯号，控制哪个LED灯
  red:三原色中的红色(0-255)
  green:三原色中的绿色(0-255)
  blue:三原色中的蓝色(0-255)
 *返回值:无
 */
void RGB_Control(uchar cresset, uchar red, uchar green, uchar blue)
{
  leds[cresset] = CRGB ( red, green, blue);
  FastLED.show();
}

void setup() 
{
  Serial.begin(115200);                         /*设置串口波特率*/
  delay(10);
  RGB_Init();                                   /*RGB灯初始化*/
    for(int i = 0; i < NUM_LEDS; i++)
  {
    RGB_Control(i, 0, 0, 0);
  }

  // 初始化 bufferPacket 为空值
  // memset(bufferPacket, 0, sizeof(bufferPacket));

  pinMode(13, OUTPUT);
  // Serial.print("hello world!\r\n");   /*串口打印函数*/


  uint8_t payload[] = {1,1,4,5,1,4};
  size_t payloadLength = sizeof(payload) / sizeof(payload[0]);

  uint8_t packet[PACKET_SIZE]; // 确保这个数组足够大以容纳完整的数据包
  size_t packetLength;

  createPacket(payload, payloadLength, packet, &packetLength);

  // 通过串口发送数据包
  Serial.write(packet, packetLength);
  // 确保数据包已完全发送
  Serial.flush();
  delay(100);
}

void loop() 
{
  uint8_t bufferPacket[BUFFER_SIZE];
  size_t bufferLength = 0;
  // 初始化 bufferPacket 为空值
  // memset(bufferPacket, 0, sizeof(bufferPacket));

  Serial.println("gogogo");
  digitalWrite(LED_PIN, LOW);

  if(readAndProcessPacket(Serial, bufferPacket, &bufferLength))
  {
    digitalWrite(LED_PIN, HIGH);
    clearSerialBuffer();
    // 通过串口发送数据包
    Serial.write(bufferPacket, bufferLength);
    // 确保数据包已完全发送
    Serial.flush();

    for(int i = 0; i < ((int)bufferPacket[2])/3; i++)
    {
      int led_data_head = 3 + i*3;
      RGB_Control(i, (int)bufferPacket[led_data_head], (int)bufferPacket[led_data_head + 1], (int)bufferPacket[led_data_head + 2]);             /*灯2亮红色*/
    }

    delay(100);
  }
  else
  {
    Serial.println("readAndProcessPacket failed");
  }
  // clearSerialBuffer();


  delay(100);
}

// void loop() {
// 	// Serial.print(int32_t(Serial.read()), HEX);
//   Serial.write(Serial.read());
// 	delay(500);
// }

// void loop() {
//   static int pos = 0; // 记录当前点亮的LED位置

//   // 将所有LED设为关闭状态
//   fill_solid(leds, NUM_LEDS, CRGB::Black);

//   // 点亮当前位置的LED
//   leds[pos] = CRGB::Red;  // 可以更改颜色

//   // 更新LED条
//   FastLED.show();

//   // 更新位置，实现流水效果
//   pos = (pos + 1) % NUM_LEDS;

//   // 延时来控制流水速度
//   delay(50);
// }