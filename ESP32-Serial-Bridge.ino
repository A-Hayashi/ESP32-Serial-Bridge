#include <freertos/FreeRTOS.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <NetBIOS.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

//#define DEBUG (1)
#define DEBUG (0)

//NetBIOS library only works in STA mode, and not in AP mode. #5300
//https://github.com/espressif/arduino-esp32/issues/5300

//#define MODE_AP // phone connects directly to ESP
#define MODE_STA

#ifdef MODE_AP
// For AP mode:
const char *ssid = "LK8000";  // You will connect your phone to this Access Point
const char *pw = "Flightcomputer"; // and this is the password
IPAddress ip(192, 168, 1, 30); // connect to this IP
IPAddress netmask(255, 255, 255, 0);
#endif

#ifdef MODE_STA
const char *ssid = "aterm-82f586-gw";  // You will connect your phone to this Access Point
const char *pw = "4u2V58F#"; // and this is the password
#endif

const char *netBios = "Bridge1"; // NetBIOS Name
#define bufferSize (1024)

struct DATA_SET {
  bool tcp;
  char check[10];
};
DATA_SET eepData;
void manager(void *params);
void load_eeprom();
void save_eeprom();
void init_eeprom();

class Bridge
{
  public:
    Bridge(int uartNo, int baudRate, int serialParam, int rxdPin, int txdPin, int port);
    static void runner(void *params);
    void start();
    void tcpLoop();
    void udpLoop();
    void serialBegin(int baudRate, int serialParam);
  private:
    HardwareSerial serial;
    int port;
    int baudRate;
    int serialParam;
    int rxdPin;
    int txdPin;

    WiFiServer server;
    WiFiClient TCPClient;
    WiFiUDP udp;
    IPAddress remoteIp;
    uint16_t remotePort;

    uint8_t buf1[bufferSize];
    uint16_t i1;

    uint8_t buf2[bufferSize];
    uint16_t i2;
};

Bridge::Bridge(int uartNo, int baudRate, int serialParam, int rxdPin, int txdPin, int port):
  serial(uartNo)
{
  this->port = port;
  this->baudRate = baudRate;
  this->serialParam = serialParam;
  this->rxdPin = rxdPin;
  this->txdPin = txdPin;
}

void Bridge::start(void)
{
  /* create task */
  xTaskCreatePinnedToCore( this->runner,   /* タスクの入口となる関数名 */
                           "TASK1", /* タスクの名称 */
                           1024 * 3, /* スタックサイズ */
                           this,    /* パラメータのポインタ */
                           1,       /* プライオリティ */
                           NULL,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */

}

void Bridge::serialBegin(int baudRate, int serialParam)
{
  this->baudRate = baudRate;
  this->serialParam = serialParam;
  serial.end();
  serial.begin(baudRate, serialParam, rxdPin, txdPin);
}

void Bridge::runner(void *params)
{
  Bridge* ref = static_cast<Bridge *>(params);
  if (eepData.tcp == 1) {
    ref->tcpLoop();
  } else {
    ref->udpLoop();
  }
}

void Bridge::tcpLoop()
{
  serial.end();
  serial.begin(baudRate, serialParam, rxdPin, txdPin);
  server.begin(port); // start TCP server
  server.setNoDelay(true);

  Serial.print("start TCP server: ");
  Serial.println(port);
  while (1) {
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
      if (i1 > 0) {
        Serial.print("from\t");
        Serial.print(port);
        Serial.print(":");
        Serial.write(buf1, i1);
        Serial.println("");
        serial.write(buf1, i1); // now send to UART(num):
      }
      i1 = 0;
    }

    if (serial.available())
    {
      while (serial.available())
      {
        buf2[i2] = serial.read(); // read char from UART(num)
        if (i2 < bufferSize - 1) {
          i2++;
        }
      }
      // now send to WiFi:
      if (i2 > 0) {
        if (TCPClient) {
          Serial.print("to\t");
          Serial.print(port);
          Serial.print(":");
          Serial.write(buf2, i2);
          Serial.println("");
          TCPClient.write(buf2, i2);
        }
      }
      i2 = 0;
    }
    vTaskDelay(1);
  }
}

void Bridge::udpLoop()
{
  serial.end();
  serial.begin(baudRate, serialParam, rxdPin, txdPin);
  udp.begin(port); // start UDP server
  
  Serial.print("start UDP server: ");
  Serial.println(port);
  while (1) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      remoteIp = udp.remoteIP(); // store the ip of the remote device
      remotePort = udp.remotePort(); // store the port of the remote device
      udp.read(buf1, bufferSize);

      Serial.print("from\t");
      Serial.print(port);
      Serial.print(":");
      Serial.write(buf1, packetSize);
      Serial.println("");
      serial.write(buf1, packetSize); // now send to UART(num):
    }

    if (serial.available()) {
      while (serial.available())
      {
        buf2[i2] = serial.read(); // read char from UART(num)
        if (i2 < bufferSize - 1) {
          i2++;
        }
      }
      // now send to WiFi:
      if (i2 > 0) {
        Serial.print("to\t");
        Serial.print(remotePort);
        Serial.print(":");
        Serial.write(buf2, i2);
        Serial.println("");

        udp.beginPacket(remoteIp, remotePort); // remote IP and port
        udp.write(buf2, i2);
        udp.endPacket();
      }
      i2 = 0;
    }
    vTaskDelay(1);
  }
}

#if (DEBUG==1)
Bridge a(3, 9600, SERIAL_8E1, 3, 1, 8880);
#else
Bridge a(0, 9600, SERIAL_8E1, 3, 1, 8880);
#endif
Bridge b(1, 9600, SERIAL_8E1, 26, 27, 8881);
Bridge c(2, 9600, SERIAL_8E1, 16, 17, 8882);

void manager(void *params)
{
  a.start();
  b.start();
  c.start();

  WiFiServer server;
  WiFiClient TCPClient;
  server.begin(8883); // start TCP server
  server.setNoDelay(true);

  Serial.println("start TCP server: 8883");
  while (1) {
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
      char buf1[bufferSize];
      uint16_t i1;
      while (TCPClient.available())
      {
        buf1[i1] = TCPClient.read(); // read char from client (LK8000 app)
        if (i1 < bufferSize - 1) {
          i1++;
        }
      }
      if (i1 > 0) {
        Serial.print("from\t");
        Serial.print("8883");
        Serial.print(":");
        Serial.write(buf1, i1);
        Serial.println("");

        bool ack = true;
        switch (buf1[0]) {
          case 's': {
              switch (buf1[1]) {
                case '0':
                  a.serialBegin(9600, SERIAL_8E1 );
                  b.serialBegin(9600, SERIAL_8E1 );
                  c.serialBegin(9600, SERIAL_8E1 );
                  break;
                case '1':
                  a.serialBegin(460800, SERIAL_8N1 );
                  b.serialBegin(460800, SERIAL_8N1 );
                  c.serialBegin(460800, SERIAL_8N1 );
                  break;
                default:
                  ack = false;
                  break;
              }
              break;
            }
          case 't':
            eepData.tcp = 1;
            save_eeprom();
            ESP.restart();
            break;
          case 'u':
            eepData.tcp = 0;
            save_eeprom();
            ESP.restart();
            break;
          case 'r':
            ESP.restart();
            break;
          default:
            ack = false;
            break;
        }

        String res;
        if (ack == true) {
          res = "ack";
        } else {
          res = "nak";
        }

        Serial.print("to\t");
        Serial.print("8883");
        Serial.print(":");
        Serial.print(res);
        Serial.println("");

        TCPClient.print(res);
      }
      i1 = 0;
    }
    vTaskDelay(1);
  }
}

void setup() {
  delay(500);
  Serial.begin(9600);
  Serial.println("");
  init_eeprom();

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

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
  NBNS.begin(netBios);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

#if (DEBUG==0)
  Serial.end();
#endif

  xTaskCreatePinnedToCore( manager,   /* タスクの入口となる関数名 */
                           "MANAGER", /* タスクの名称 */
                           1024 * 100, /* スタックサイズ */
                           NULL,    /* パラメータのポインタ */
                           1,       /* プライオリティ */
                           NULL,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */
}

void loop() {
  ArduinoOTA.handle();
}

#define DEFAULT_TCP     (1)
#define DATA_VERSION    "DATA1.0"

void load_eeprom() {
  EEPROM.get<DATA_SET>(0, eepData);
  if (strcmp(eepData.check, DATA_VERSION)) {
    eepData.tcp = DEFAULT_TCP;
  }
}

void save_eeprom() {
  strcpy(eepData.check, DATA_VERSION);
  EEPROM.put<DATA_SET>(0, eepData);
  EEPROM.commit();
}

void init_eeprom() {
  EEPROM.begin(1024);
  load_eeprom();
}
