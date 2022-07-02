#include <freertos/FreeRTOS.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

#define MODE_STA // phone connects directly to ESP

// For AP mode:
const char *ssid = "aterm-82f586-gw";  // You will connect your phone to this Access Point
const char *pw = "4u2V58F#"; // and this is the password
IPAddress ip(192, 168, 10, 30); // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);
#define bufferSize 1024

class Bridge
{
  public:
    Bridge(int uartNo, int baudRate, int serialParam, int rxdPin, int txdPin, int port);
    static void runner(void *params);
    void start();
    void loop();
  private:
    HardwareSerial serial;
    int port;

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
  serial.end();
  serial.begin(baudRate, serialParam, rxdPin, txdPin);
  this->port = port;
}

void Bridge::start(void)
{
  /* create task */
  xTaskCreatePinnedToCore( this->runner,   /* タスクの入口となる関数名 */
                           "TASK1", /* タスクの名称 */
                           0x800,   /* スタックサイズ */
                           this,    /* パラメータのポインタ */
                           1,       /* プライオリティ */
                           NULL,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */

}

void Bridge::runner(void *params)
{
  Bridge* ref = static_cast<Bridge *>(params);
  ref->loop();
}

void Bridge::loop()
{
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

      serial.write(buf1, i1); // now send to UART(num):
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
      if (TCPClient) {
        TCPClient.write(buf2, i2);
      }
      i2 = 0;
    }
    vTaskDelay(1);
  }

}

Bridge a(0, 19200, SERIAL_8E1, 3, 1, 8880);
Bridge b(1, 19200, SERIAL_8E1, 26, 27, 8881);
Bridge c(2, 19200, SERIAL_8E1, 16, 17, 8882);

void setup() {
  delay(500);
  Serial.begin(115200);

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

  esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power

  a.start();
  b.start();
  c.start();
}

void loop() {

}
