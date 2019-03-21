#include "WiFi.h"
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include "maxi.h"


maxiFilter filt;
//maxiOsc osc;

maxiSVF svf;
#define SERVER_ADVERT_STRING "%groundControlTo?"

const char* ssid     = "ListeningMirrors";
const char* password = "badgersbadgers";


const int GIVEREQ = 17;
//const char * udpAddress = "192.168.0.255";
const int udpAdvPort = 13252;
const int UDP_PORT = 13251;
const unsigned int CONTROL_SERVICE_PORT = 14672;
const char CONTROL_SERVICE_REPLY = 174;
const unsigned int STREAM_TARGET_PORT = 17232;
const unsigned int STREAM_SOURCE_PORT = 17233;

#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26
uint8_t m_i2s_num = I2S_NUM_0;
uint8_t         m_BCLK = 0;                     // Bit Clock
uint8_t         m_LRC = 0;                      // Left/Right Clock
uint8_t m_DOUT = 0; // Data Out

i2s_port_t I2S_PORT_MIC = I2S_NUM_1;

WiFiUDP udp, udpStreamOut;
enum networkState {DISCONNECTED, CONNECTING, WAITING_FOR_ADVERT, CONNECTED} netState;
boolean connected = false;
IPAddress LMServerIP;

float audioOutBuffer[512];
int audioBlockSize = 512;
unsigned long audioFrame = 0;
int blockPos = 0;

int i2s_block_size = 64;

void connectToAP() {
  netState = CONNECTING;
  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, password);

  Serial.println("Waiting for WIFI connection...");
}


int frame = 0;
uint16_t buf_len = 1024;
char buf[1024];

QueueHandle_t xQueueToDSP;
QueueHandle_t xQueueFromDSP;

void coreTask( void * pvParameters ) {

  String taskMessage = "Core task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  float data;
  while (true) {
    if (xQueueReceive( xQueueToDSP, &data, 0))
    {
      Serial.println(data);
      data = filt.lores(data, 500, 1);
      if (xQueueSendToFront(xQueueFromDSP, &data, 0) != pdPASS) {
        Serial.println("xQueueFromDSP send failed");
      } else {
        //        Serial.println("Queue send ok");
      }

    } else {
      Serial.println("queue fail");
    }
  }

}

void setup() {
  Serial.begin(115200);


  xQueueToDSP = xQueueCreate( 2048, sizeof(float));

  if ( xQueueToDSP == NULL )
  {
    Serial.println("Could not create xQueueToDSP");
  }
  else
  {
    Serial.println("xQueueToDSP created");
  }
  xQueueFromDSP = xQueueCreate( 2048, sizeof(float));
  if ( xQueueFromDSP == NULL )
  {
    Serial.println("Could not create xQueueFromDSP");
  }
  else
  {
    Serial.println("xQueueFromDSP created");
  }
  maxiSettings::setup(44100, 2,  64);
  svf.setCutoff(3000);
  svf.setResonance(10);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = i2s_block_size,   //Interrupt level 1
    .use_apll = 0,
    .tx_desc_auto_clear = true, // new in V1.0.1
    .fixed_mclk = -1
  };
  i2s_driver_install((i2s_port_t)m_i2s_num, &i2s_config, 0, NULL);

  m_BCLK = 26;                     // Bit Clock
  m_LRC = 25;                      // Left/Right Clock
  m_DOUT = 27;                     // Data Out
  i2s_pin_config_t pins = {
    .bck_io_num = m_BCLK,
    .ws_io_num =  m_LRC,              //  wclk,
    .data_out_num = m_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_set_pin((i2s_port_t)m_i2s_num, &pins);
  i2s_start((i2s_port_t) m_i2s_num);

  const i2s_config_t i2s_config_microphone = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
    .sample_rate = 44100,                         // 16KHz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // although the SEL config should be left, it seems to transmit on right
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
    .dma_buf_count = 8,                           // number of buffers
    .dma_buf_len = i2s_block_size                 // 8 samples per buffer (minimum)
  };

  const i2s_pin_config_t pin_config_mic = {
    .bck_io_num = 19,   // BCKL
    .ws_io_num = 5,    // LRCL
    .data_out_num = -1, // not used (only for speakers)
    .data_in_num = 18   // DOUT
  };

  esp_err_t err = i2s_driver_install(I2S_PORT_MIC, &i2s_config_microphone, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT_MIC, &pin_config_mic);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }


  for (int i = 0; i < audioBlockSize; i++) {
    audioOutBuffer[i] = 0.0;
  }
  netState = DISCONNECTED;
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  Serial.print("Connecting to ");
  Serial.println(ssid);
  delay(10);

  connectToAP();


  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(WiFi.status());
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  xTaskCreatePinnedToCore(
    coreTask,   /* Function to implement the task */
    "coreTask", /* Name of the task */
    10000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    0,          /* Priority of the task */
    NULL,       /* Task handle. */
    1);  /* Core where the task should run */


}

BaseType_t xStatus;
/* time to block the task until the queue has free space */
const TickType_t xTicksToWait = pdMS_TO_TICKS(100);


void loop() {

  switch (netState) {
    case DISCONNECTED:
      connectToAP();
      break;
    case WAITING_FOR_ADVERT:
      {
        int udpMsgLength = udp.parsePacket();
        if (udpMsgLength != 0) {
          byte udpPacket[udpMsgLength + 1];
          udp.read(udpPacket, udpMsgLength);
          udpPacket[udpMsgLength] = 0;
          udp.flush(); // empty UDP buffer for next packet
          String advMessage = String((char*)udpPacket);
          Serial.println("Received: " + advMessage);
          if (advMessage == "%groundControlTo?") {
            LMServerIP = udp.remoteIP();
            Serial.println("Server found, address: " + LMServerIP.toString());
            Serial.println("Requesting an audio stream destination");
            WiFiClient tcpClient;
            if (tcpClient.connect(LMServerIP, CONTROL_SERVICE_PORT)) {
              tcpClient.print((char)GIVEREQ);
              tcpClient.print((char)0);
              tcpClient.print((char)0);
              tcpClient.print((char)0);
              tcpClient.print('\n');
              uint8_t data[2];
              tcpClient.read(data, 2);
              Serial.println((char *)data);
              //              udpStreamOut.begin(LMServerIP, STREAM_SOURCE_PORT);
              Serial.println("Sending audio...");
              netState = CONNECTED;
            } else {
              Serial.println("Can't connect to LM Server control service");
            }
          }
        }
        delay(100);
      }
      break;
    case CONNECTED:
      {
        size_t bytes_read;
        esp_err_t readResponse = i2s_read(I2S_PORT_MIC, buf, buf_len, &bytes_read, 0);

        if (readResponse != ESP_OK) {
          Serial.println("i2s read error");
        }

        uint32_t samples_read = bytes_read  / (I2S_BITS_PER_SAMPLE_32BIT / 8);

        //        Serial.println(samples_read);

        int32_t *intPtr = (int32_t*) &buf[0];
        float mean = 0.f;
        float data = 103.4;
//        if (samples_read > 0) {
//          if (xQueueSendToFront(xQueueToDSP, &data, 0) != pdPASS) {
//            //            Serial.println("Queue send failed");
//          } else {
//            //            Serial.println("Queue send ok");
//          }
//          if (xQueueReceive( xQueueFromDSP, &data, 0))
//          {
//            Serial.println(data);
//          } else {
////            Serial.println("xQueueFromDSP fail");
//          }
//
//        }
        for (int i = 0; i < samples_read; i += 2) {
          float w = intPtr[i] / (float) 0x7fffffff;
          //    w = w * 1.5f;
          //        w = filt.lores(w, 300, 0.5);
          //        w = osc.saw(100);
          //    w = filt.lopass(w, 0.5);
          //    mean += w;
          //    w *= 0.05f;

          //          w = svf.play(w, 0.f, 1.f, 0.f, 0.f);
          if (xQueueSendToFront(xQueueToDSP, &w, 0) != pdPASS) {
            //            Serial.println("Queue send failed");
          } else {
            //            Serial.println("Queue send ok");
          }
          if (xQueueReceive( xQueueFromDSP, &data, 0))
          {
//            Serial.println(data);
//            audioOutBuffer[blockPos] = data * 50.0f;
//            blockPos++;
          } else {
//            Serial.println("xQueueFromDSP fail");
          }
          
          audioOutBuffer[blockPos] = w * 50.0f;
          blockPos++;
          if (blockPos == audioBlockSize) {
            blockPos = 0;
            udpStreamOut.beginPacket(LMServerIP, STREAM_SOURCE_PORT);
            udpStreamOut.write((uint8_t *) &audioOutBuffer[0], sizeof(float) * audioBlockSize);
            if (!udpStreamOut.endPacket()) {
              Serial.print("Error");
            }
            //            Serial.println("send");
          }

          //    if (i == 0) {
          //      Serial.println(w);
          //    }
          intPtr[i] = int32_t(w * (float)0x7fffffff);
        }
        //  Serial.println(mean / samples_read);
        //    Serial.println(intPtr[0]);


        size_t m_bytesWritten;
        esp_err_t err = i2s_write((i2s_port_t)m_i2s_num, (const char*)&buf, bytes_read, &m_bytesWritten, 0);
        if (err != ESP_OK) {
          Serial.print("ESP32 Errorcode ");
          Serial.println(err);
        }
      }
      break;

  };
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(udpAdvPort);
      netState = WAITING_FOR_ADVERT;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      netState = DISCONNECTED;
      break;
  }
}
