/*
    Copyright(C) 2018 by Yukiya Ishioka
*/

#include <freertos/FreeRTOS.h>    /* FreeRTOSを用いるためのヘッダファイル */

byte a = 0;

void  task1( void *param )
{
  Serial.begin(115200);
  Serial.print("setup()\n");
  while ( 1 ) {
    Serial.println(a);
    vTaskDelay(100);
  }
}

void  task2( void *param )
{
  while ( 1 ) {
    a++;
    vTaskDelay(1000);
  }
}

//void setup()
//{
//  /* create task */
//  xTaskCreatePinnedToCore( task1,   /* タスクの入口となる関数名 */
//                           "TASK1", /* タスクの名称 */
//                           0x800,   /* スタックサイズ */
//                           NULL,    /* パラメータのポインタ */
//                           1,       /* プライオリティ */
//                           NULL,    /* ハンドル構造体のポインタ */
//                           0 );     /* 割り当てるコア (0/1) */
//
//  xTaskCreatePinnedToCore( task2,
//                           "TASK2",
//                           0x800,
//                           NULL,
//                           2,
//                           NULL,
//                           0 );
//}
//
//void loop()
//{
//
//}

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>


#define MODE_STA // phone connects directly to ESP

// For AP mode:
const char *ssid = "aterm-82f586-gw";  // You will connect your phone to this Access Point
const char *pw = "4u2V58F#"; // and this is the password
IPAddress ip(192, 168, 10, 30); // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);

/*************************  COM Port 0 *******************************/
#define UART_BAUD0 19200            // Baudrate UART0
#define SERIAL_PARAM0 SERIAL_8N1    // Data/Parity/Stop UART0
#define SERIAL0_RXPIN 16            // receive Pin UART0
#define SERIAL0_TXPIN 17



// transmit Pin UART0
#define SERIAL0_TCP_PORT 8880       // Wifi Port UART0


#define bufferSize 1024

HardwareSerial Serial_one(1);

WiFiServer server(SERIAL0_TCP_PORT);
WiFiClient TCPClient;

uint8_t buf1[bufferSize];
uint16_t i1 = 0;

uint8_t buf2[bufferSize];
uint16_t i2 = 0;

uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;

void setup() {

  delay(500);

  Serial_one.begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  Serial.begin(115200, SERIAL_8N1, 3, 1);
#ifdef MODE_AP
  //AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  delay(2000); // VERY IMPORTANT
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP
#endif

#ifdef MODE_STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  server.begin(); // start TCP server
  server.setNoDelay(true);

  esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
}



void loop()
{
  if (server.hasClient())
  {
    //find free/disconnected spot
    if (!TCPClient || !TCPClient.connected()) {
      if (TCPClient) {
        TCPClient.stop();
      }
      TCPClient = server.available();
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = server.available();
    TmpserverClient.stop();
  }

  if (TCPClient)
  {
    while (TCPClient.available())
    {
      buf1[i1] = TCPClient.read(); // read char from client (LK8000 app)
      if (i1 < bufferSize - 1) {
        i1++;
      }
    }

    Serial_one.write(buf1, i1); // now send to UART(num):
    i1 = 0;
  }

  if (Serial_one.available())
  {
    while (Serial_one.available())
    {
      buf2[i2] = Serial_one.read(); // read char from UART(num)
      if (i2 < bufferSize - 1) {
        i2++;
      }
    }
    // now send to WiFi:
    if (TCPClient) {
      TCPClient.write(buf2, i2);
    }
    i2 = 0;
  }
}
