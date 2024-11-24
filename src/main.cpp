
/*
   rocrail_CAN_TCP_GW

    Programme permettant le pilotage de locomotives à partir de Rocrail©
    et la mise a jour de la liste des locomotives en MFX en utilisant une liaison TCP (WiFi ou Ethernet).
*/

#define PROJECT "rocrail_can_tcp_gateway"
#define VERSION "1.3.1"
#define AUTHOR "Christophe BOBILLE - www.locoduino.org"

//----------------------------------------------------------------------------------------
//  Board Check
//----------------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------

#include <ACAN_ESP32.h> // https://github.com/pierremolinaro/acan-esp32.git
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

//----------------------------------------------------------------------------------------
//  Debug serial
//----------------------------------------------------------------------------------------

#define debug Serial // (only for TCP connection)

//----------------------------------------------------------------------------------------
//  CAN Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 250UL * 1000UL; // Marklin CAN baudrate = 250Kbit/s

//----------------------------------------------------------------------------------------
//  Buffers  : Rocrail always send 13 bytes
//----------------------------------------------------------------------------------------

static const uint8_t BUFFER_SIZE = 13;
byte cBuffer[BUFFER_SIZE]; // CAN buffer
byte sBuffer[BUFFER_SIZE]; // Serial buffer

//----------------------------------------------------------------------------------------
//  Marklin hash
//----------------------------------------------------------------------------------------

uint16_t rrHash; // for Rocrail hash

//----------------------------------------------------------------------------------------
//  Select a communication mode
//----------------------------------------------------------------------------------------
//#define ETHERNET
#define WIFI

IPAddress ip(192, 168, 1, 210);
const uint port = 15731;

//----------------------------------------------------------------------------------------
//  Ethernet
//----------------------------------------------------------------------------------------
#if defined(ETHERNET)
#include <Ethernet.h>
#include <SPI.h>
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF};
EthernetServer server(port);
EthernetClient client;

//----------------------------------------------------------------------------------------
//  WIFI
//----------------------------------------------------------------------------------------
#elif defined(WIFI)
#include <WiFi.h>


const char *ssid = "**********";
const char *password = "**********";
IPAddress gateway(192, 168, 1, 1);  // passerelle par défaut
IPAddress subnet(255, 255, 255, 0); // masque de sous réseau
WiFiServer server(port);
WiFiClient client;

#endif

//----------------------------------------------------------------------------------------
//  Queues + mutex
//----------------------------------------------------------------------------------------

QueueHandle_t canToTcpQueue;
QueueHandle_t tcpToCanQueue;
QueueHandle_t debugQueue; // Queue for debug messages

SemaphoreHandle_t tcpMutex;

//----------------------------------------------------------------------------------------
//  Debug declaration
//----------------------------------------------------------------------------------------

void debugFrame(const CANMessage *);

//----------------------------------------------------------------------------------------
//  Tasks
//----------------------------------------------------------------------------------------

void CANReceiveTask(void *pvParameters);
void TCPSendTask(void *pvParameters);
void TCPReceiveTask(void *pvParameters);
void CANSendTask(void *pvParameters);
#if defined(ETHERNET)
void ethernetMonitorTask(void *pvParameters);
#elif defined(WIFI)
void wifiMonitorTask(void *pvParameters);
#endif
void debugFrameTask(void *pvParameters); // Debug task

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup()
{
  //--- Start serial
  debug.begin(115200); // For debug

  while (!debug)
  {
    debug.print(".");
    delay(200);
  }
  delay(100);

  debug.printf("\nProject   :    %s", PROJECT);
  debug.printf("\nVersion   :    %s", VERSION);
  debug.printf("\nAuteur    :    %s", AUTHOR);
  debug.printf("\nFichier   :    %s", __FILE__);
  debug.printf("\nCompiled  :    %s", __DATE__);
  debug.printf(" - %s\n\n", __TIME__);
  debug.printf("-----------------------------------\n\n");

  debug.println("Start setup");

  //--- Configure ESP32 CAN
  debug.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mDriverReceiveBufferSize = 50;
  settings.mDriverTransmitBufferSize = 50;
  settings.mRxPin = GPIO_NUM_21; // Optional, default Rx pin is GPIO_NUM_5
  settings.mTxPin = GPIO_NUM_22; // Optional, default Tx pin is GPIO_NUM_4
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);

  if (errorCode)
  {
    debug.print("Cƒonfiguration error 0x");
    debug.println(errorCode, HEX);
  }
  else
    debug.print("Configuration CAN OK\n\n");

#if !defined(ETHERNET) && !defined(WIFI)
  Serial.print("Select a communication mode.");
  while (1)
  {
  }
#endif

#if defined(ETHERNET)
  Serial.println("Waiting for Ethernet connection : ");
  // Ethernet initialization
  Ethernet.init(5); // MKR ETH Shield (change depending on your hardware)
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("IP address = ");
  Serial.println(Ethernet.localIP());
  Serial.printf("Port = %d\n", port);

#elif defined(WIFI)
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.print("Waiting for WiFi connection : \n\n");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address : ");
  Serial.println(WiFi.localIP());
  Serial.printf("Port = %d\n", port);
  server.begin();

#endif

  debug.printf("\n\nWaiting for connection from Rocrail.\n");

  while (!client) // listen for incoming clients
    client = server.available();

  // extract the Rocrail hash
  if (client.connected())
  {                 // loop while the client's connected
    int16_t rb = 0; //!\ Do not change type int16_t See https://www.arduino.cc/reference/en/language/functions/communication/stream/streamreadbytes/
    while (rb != BUFFER_SIZE)
    {
      if (client.available()) // if there's bytes to read from the client,
        rb = client.readBytes(cBuffer, BUFFER_SIZE);
    }
    rrHash = ((cBuffer[2] << 8) | cBuffer[3]);

    debug.printf("New Client Rocrail : 0x");
    debug.println(rrHash, HEX);

    // --- register Rocrail on the CAN bus
    constexpr uint8_t MAX_RETRIES = 5;
    constexpr uint16_t RESTART_DELAY_MS = 10000;

    CANMessage frame;
    frame.id = (cBuffer[0] << 24) | (cBuffer[1] << 16) | rrHash;
    frame.ext = true;
    frame.len = cBuffer[4];
    for (byte i = 0; i < frame.len; i++)
      frame.data[i] = cBuffer[i + 5];

    // uint32_t ok = false;
    bool isSent = false;
    uint8_t attempts = 0;

    while (!isSent && attempts < MAX_RETRIES)
    {
      isSent = ACAN_ESP32::can.tryToSend(frame);
      ++attempts;
      delay(100);
    }
    if (isSent)
    {
      debugFrame(&frame);
    }
    else
    {
      debug.println("CAN frame failed to send.");
      debug.println("ESP32 will restart in 10 seconds.");
      debug.println("/!\\ Relaunch Rocrail.");
      delay(RESTART_DELAY_MS);
      ESP.restart();
    }

    // Create queues
    canToTcpQueue = xQueueCreate(50, sizeof(CANMessage));
    tcpToCanQueue = xQueueCreate(50, BUFFER_SIZE * sizeof(byte));
    debugQueue = xQueueCreate(50, sizeof(CANMessage)); // Create debug queue

    tcpMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreatePinnedToCore(CANReceiveTask, "CANReceiveTask", 4 * 1024, NULL, 3, NULL, 0); // priority 3 on core 0
    xTaskCreatePinnedToCore(TCPSendTask, "TCPSendTask", 4 * 1024, NULL, 5, NULL, 1);       // priority 5 on core 1
    xTaskCreatePinnedToCore(TCPReceiveTask, "TCPReceiveTask", 4 * 1024, NULL, 3, NULL, 1); // priority 3 on core 1
    xTaskCreatePinnedToCore(CANSendTask, "CANSendTask", 4 * 1024, NULL, 5, NULL, 0);       // priority 5 on core 0
#if defined(ETHERNET)
    xTaskCreatePinnedToCore(ethernetMonitorTask, "Ethernet Monitor", 4 * 1024, NULL, 1, NULL, 1); // priority 1 on core 1
#elif defined(WIFI)
    xTaskCreatePinnedToCore(wifiMonitorTask, "WiFi Monitor", 4 * 1024, NULL, 1, NULL, 1); // priority 1 on core 1
#endif
    xTaskCreatePinnedToCore(debugFrameTask, "debugFrameTask", 2 * 1024, NULL, 1, NULL, 1); // debug task with priority 1 on core 1
  }

} // end setup

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop()
{
  vTaskDelete(NULL);
} // Nothing to do

//----------------------------------------------------------------------------------------
//   CANReceiveTask
//----------------------------------------------------------------------------------------

void CANReceiveTask(void *pvParameters)
{
  CANMessage frameIn;
  while (true)
  {
    if (ACAN_ESP32::can.receive(frameIn))
    {
      xQueueSend(canToTcpQueue, &frameIn, portMAX_DELAY);
      xQueueSend(debugQueue, &frameIn, 10); // send to debug queue
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Avoid busy-waiting
  }
}

//----------------------------------------------------------------------------------------
//   TCPSendTask
//----------------------------------------------------------------------------------------

void TCPSendTask(void *pvParameters)
{
  CANMessage frameIn;
  while (true)
  {
    if (xQueueReceive(canToTcpQueue, &frameIn, portMAX_DELAY))
    {
      // clear sBuffer
      memset(sBuffer, 0, sizeof(sBuffer));

      // debug.printf("CAN -> TCP\n\n");
      xQueueSend(debugQueue, &frameIn, 10); // send to debug queue

      sBuffer[0] = (frameIn.id & 0xFF000000) >> 24;
      sBuffer[1] = (frameIn.id & 0xFF0000) >> 16;
      sBuffer[2] = (frameIn.id & 0xFF00) >> 8; // hash
      sBuffer[3] = (frameIn.id & 0x00FF);      // hash
      sBuffer[4] = frameIn.len;
      for (byte i = 0; i < frameIn.len; i++)
        sBuffer[i + 5] = frameIn.data[i];

      client.write(sBuffer, BUFFER_SIZE);
    }
  }
}

//----------------------------------------------------------------------------------------
//   TCPReceiveTask
//----------------------------------------------------------------------------------------

void TCPReceiveTask(void *pvParameters)
{
  while (true)
  {
    if (client.connected() && client.available())
    {
      if (client.readBytes(cBuffer, BUFFER_SIZE) == BUFFER_SIZE)
        xQueueSend(tcpToCanQueue, cBuffer, 10);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Avoid busy-waiting
  }
}

//----------------------------------------------------------------------------------------
//   CANSendTask
//----------------------------------------------------------------------------------------

void CANSendTask(void *pvParameters)
{
  byte buffer[BUFFER_SIZE];
  while (true)
  {
    if (xQueueReceive(tcpToCanQueue, buffer, portMAX_DELAY))
    {
      CANMessage frameOut;
      frameOut.id = (buffer[0] << 24) | (buffer[1] << 16) | rrHash;
      frameOut.ext = true;
      frameOut.len = buffer[4];
      for (byte i = 0; i < frameOut.len; i++)
        frameOut.data[i] = buffer[i + 5];

      const bool ok = ACAN_ESP32::can.tryToSend(frameOut);
      xQueueSend(debugQueue, &frameOut, 10); // send to debug queue
    }
  }
}

#if defined(ETHERNET)

//----------------------------------------------------------------------------------------
//   ethernetMonitorTask
//----------------------------------------------------------------------------------------

void ethernetMonitorTask(void *parameter)
{
  while (true)
  {
    if (!client.connected())
    {
      debug.println("Connexion au serveur TCP perdue. Tentative de reconnexion...");
      client = server.available();
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // Vérifier toutes les 5 secondes
  }
}

#elif defined(WIFI)

//----------------------------------------------------------------------------------------
//   wifiMonitorTask
//----------------------------------------------------------------------------------------

// Tâche pour surveiller la connexion WiFi et la reconnecter si nécessaire (Core 1)
void wifiMonitorTask(void *parameter)
{
  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      xSemaphoreTake(tcpMutex, portMAX_DELAY);
      Serial.println("Connexion au WiFi perdue. Tentative de reconnexion...");
      WiFi.begin(ssid, password);
    }
    xSemaphoreGive(tcpMutex);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Vérifier toutes les 5 secondes
  }
}

#endif

//----------------------------------------------------------------------------------------
//   debugFrameTask
//----------------------------------------------------------------------------------------

void debugFrameTask(void *pvParameters)
{
  CANMessage frame;
  while (true)
  {
    if (xQueueReceive(debugQueue, &frame, portMAX_DELAY))
    {
      debugFrame(&frame);
    }
  }
}

//----------------------------------------------------------------------------------------
//   debugFrame
//----------------------------------------------------------------------------------------

void debugFrame(const CANMessage *frame)
{
  uint16_t hash = frame->id & 0xFFFF;
  if (hash == rrHash)
    Serial.println("TCP -> CAN");
  else
    Serial.println("CAN -> TCP");
  debug.print("Hash : 0x");
  debug.println(hash, HEX);
  debug.print("Response : ");
  debug.println((frame->id & 0x10000) >> 16 ? "true" : "false");
  debug.print("Command : 0x");
  debug.println((frame->id & 0x1FE0000) >> 17, HEX);
  debug.print("Length : 0x");
  debug.println(frame->len, HEX);
  for (byte i = 0; i < frame->len; i++)
  {
    debug.printf("data[%d] = 0x", i);
    debug.print(frame->data[i], HEX);
    if (i < frame->len - 1)
      debug.print(" - ");
  }
  debug.printf("\n");
  debug.println("-----------------------------------------------------------------------------------------------------------------------------");
  debug.printf("\n");
}