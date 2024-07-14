/*
   rocrail_CAN_TCP_GW

    Programme permettant le pilotage de locomotives à partir de Rocrail©
    et la mise a jour de la liste des locomotives en MFX en utilisant une liaison TCP (WiFi ou Ethernet).
*/

#define PROJECT "rocrail_can_tcp_gateway"
#define VERSION "1.0.1"
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
#include <WiFi.h>
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

byte cBuffer[13]; // CAN buffer
byte sBuffer[13]; // Serial buffer

//----------------------------------------------------------------------------------------
//  Marklin hash
//----------------------------------------------------------------------------------------

uint16_t rrHash; // for Rocrail hash

//----------------------------------------------------------------------------------------
//  TCP/WIFI-ETHERNET
//----------------------------------------------------------------------------------------

const char *ssid = "**********";
const char *password = "**********";
const uint port = 15731;
WiFiServer server(port);
WiFiClient client;

//----------------------------------------------------------------------------------------
//  Queues
//----------------------------------------------------------------------------------------

QueueHandle_t canToTcpQueue;
QueueHandle_t tcpToCanQueue;

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

  debug.println("Start setup");

  //--- Configure ESP32 CAN
  debug.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mDriverReceiveBufferSize = 50;
  settings.mDriverTransmitBufferSize = 50;
  settings.mRxPin = GPIO_NUM_22; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_23; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);

  if (errorCode)
  {
    debug.print("Configuration error 0x");
    debug.println(errorCode, HEX);
  }
  else
    debug.println("Configuration CAN OK");
  debug.println("");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();

  while (!client) // listen for incoming clients
    client = server.available();

  // extract the Rocrail hash
  debug.printf("New Client Rocrail : 0x");
  if (client.connected())
  {                 // loop while the client's connected
    int16_t rb = 0; //!\ Do not change type int16_t See https://www.arduino.cc/reference/en/language/functions/communication/stream/streamreadbytes/
    while (rb != 13)
    {
      if (client.available()) // if there's bytes to read from the client,
        rb = client.readBytes(cBuffer, 13);
    }
    rrHash = ((cBuffer[2] << 8) | cBuffer[3]);
    debug.println(rrHash, HEX);

    // --- register Rocrail on the CAN bus
    CANMessage frame;
    frame.id = (cBuffer[0] << 24) | (cBuffer[1] << 16) | rrHash;
    frame.ext = true;
    frame.len = cBuffer[4];
    for (byte i = 0; i < frame.len; i++)
      frame.data[i] = cBuffer[i + 5];

    uint32_t ok = false;
    uint8_t compt = 0;
    while (!ok && compt < 5)
    {
      ok = ACAN_ESP32::can.tryToSend(frame);
      compt++;
    }
    if (ok && compt < 5)
      debugFrame(&frame);
    else
    {
      debug.println("CAN frame failed to send.");
      debug.println("ESP32 will restart in 10 seconds.");
      debug.println("Relaunch Rocrail.");
      delay(10);
      esp_restart;
    }
  }

  // Create queues
  canToTcpQueue = xQueueCreate(50, sizeof(CANMessage));
  tcpToCanQueue = xQueueCreate(50, 13 * sizeof(byte));

  // Create tasks
  xTaskCreatePinnedToCore(CANReceiveTask, "CANReceiveTask", 2 * 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TCPSendTask, "TCPSendTask", 2 * 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TCPReceiveTask, "TCPReceiveTask", 2 * 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(CANSendTask, "CANSendTask", 2 * 1024, NULL, 3, NULL, 0);
} // end setup

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop()
{
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
      xQueueSend(canToTcpQueue, &frameIn, portMAX_DELAY);
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

      debug.printf("CAN -> TCP\n\n");
      debugFrame(&frameIn);

      sBuffer[0] = (frameIn.id & 0xFF000000) >> 24;
      sBuffer[1] = (frameIn.id & 0xFF0000) >> 16;
      sBuffer[2] = (frameIn.id & 0xFF00) >> 8; // hash
      sBuffer[3] = (frameIn.id & 0x00FF);      // hash
      sBuffer[4] = frameIn.len;
      for (byte i = 0; i < frameIn.len; i++)
        sBuffer[i + 5] = frameIn.data[i];

      client.write(sBuffer, 13);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Avoid busy-waiting
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
      if (client.readBytes(cBuffer, 13) == 13)
      {
        xQueueSend(tcpToCanQueue, cBuffer, portMAX_DELAY);
        debug.printf("TCP -> CAN\n\n");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Avoid busy-waiting
  }
}

//----------------------------------------------------------------------------------------
//   CANSendTask
//----------------------------------------------------------------------------------------

void CANSendTask(void *pvParameters)
{
  byte buffer[13];
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

      debugFrame(&frameOut);
      const bool ok = ACAN_ESP32::can.tryToSend(frameOut);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Avoid busy-waiting
  }
}

//----------------------------------------------------------------------------------------
//   debugFrame
//----------------------------------------------------------------------------------------

void debugFrame(const CANMessage *frame)
{
  debug.print("Hash : 0x");
  debug.println(frame->id & 0xFFFF, HEX);
  debug.print("Response : ");
  debug.println((frame->id & 0x10000) >> 16 ? "true" : "false");
  debug.print("Commande : 0x");
  debug.println((frame->id & 0x1FE0000) >> 17, HEX);
  for (byte i = 0; i < frame->len; i++)
  {
    debug.printf("data[%d] = 0x", i);
    debug.print(frame->data[i], HEX);
    if (i < frame->len - 2)
      debug.print(" - ");
  }
  debug.printf("\n");
  debug.println("-----------------------------------------------------------------------------------------------------------------------------");
  debug.printf("\n");
}
