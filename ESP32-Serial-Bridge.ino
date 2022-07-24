#include <freertos/FreeRTOS.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <NetBIOS.h>

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
#define bufferSize 1024

class Bridge
{
  public:
    Bridge(int uartNo, int baudRate, int serialParam, int rxdPin, int txdPin, int port);
    static void runner(void *params);
    void start();
    void loop();
    xTaskHandle getTaskhandle();
    void serialBegin(int baudRate, int serialParam);
  private:
    HardwareSerial serial;
    int port;
    int baudRate;
    int serialParam;
    int rxdPin;
    int txdPin;
    xTaskHandle taskHandle;

    WiFiServer server;
    WiFiClient TCPClient;

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
                           &taskHandle,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */

}

void Bridge::serialBegin(int baudRate, int serialParam)
{
  this->baudRate = baudRate;
  this->serialParam = serialParam;
  serial.end();
  serial.begin(baudRate, serialParam, rxdPin, txdPin);
}


xTaskHandle Bridge::getTaskhandle(void)
{
  return taskHandle;
}

void Bridge::runner(void *params)
{
  Bridge* ref = static_cast<Bridge *>(params);
  ref->loop();
}

void Bridge::loop()
{
  serial.end();
  serial.begin(baudRate, serialParam, rxdPin, txdPin);

  server.begin(port); // start TCP server
  server.setNoDelay(true);

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
        serial.write(buf1, i1); // now send to UART(num):
        Serial.println("");
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

xTaskHandle managerTaskHandle;

Bridge a(3, 9600, SERIAL_8E1, 3, 1, 8880);
//Bridge a(0, 9600, SERIAL_8E1, 3, 1, 8880);
Bridge b(1, 9600, SERIAL_8E1, 26, 27, 8881);
Bridge c(2, 9600, SERIAL_8E1, 16, 17, 8882);
void manager(void *params)
{
  //  Serial.end();

  a.start();
  b.start();
  c.start();

  WiFiServer server;
  WiFiClient TCPClient;
  char buf1[bufferSize];
  uint16_t i1;
  bool ack;

  server.begin(8883); // start TCP server
  server.setNoDelay(true);
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
        Serial.print("8883");
        Serial.print(":");
        Serial.write(buf1, i1);
        Serial.println("");

        ack = true;
        char *ptr = strtok(buf1, ",");
        if (strcmp(ptr, "SERIAL")) {
          int baudRate;
          int serialParam;
          ptr = strtok(NULL, ",");
          baudRate = atoi(ptr);
          Serial.print("baudRate:");
          Serial.println(baudRate);

          ptr = strtok(NULL, ",");
          Serial.print("serialParam:");
          if (strcmp(ptr, "8N1") == 0) {
            serialParam = SERIAL_8N1;
            Serial.println("SERIAL_8N1");
          }  else if (strcmp(ptr, "8E1") == 0) {
            serialParam = SERIAL_8E1;
            Serial.println("SERIAL_8E1");
          } else if (strcmp(ptr, "8O1") == 0) {
            serialParam = SERIAL_8O1;
            Serial.println("SERIAL_8O1");
          } else {
            ack = false;
            serialParam = SERIAL_8N1;
            Serial.println("SERIAL_8N1");
          }
          a.serialBegin(baudRate, serialParam);
          b.serialBegin(baudRate, serialParam);
          c.serialBegin(baudRate, serialParam);
        } else {
          ack = false;
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
    //    Serial.println("Start");
    //    Serial.println(uxTaskGetStackHighWaterMark(b.getTaskhandle()));
    //    Serial.println(uxTaskGetStackHighWaterMark(c.getTaskhandle()));
    //    Serial.println(uxTaskGetStackHighWaterMark(managerTaskHandle));
  }
}

void setup() {
  delay(500);
  Serial.begin(9600);

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

  xTaskCreatePinnedToCore( manager,   /* タスクの入口となる関数名 */
                           "MANAGER", /* タスクの名称 */
                           1024 * 100, /* スタックサイズ */
                           NULL,    /* パラメータのポインタ */
                           1,       /* プライオリティ */
                           &managerTaskHandle,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */
}

void loop() {
  ArduinoOTA.handle();
}
