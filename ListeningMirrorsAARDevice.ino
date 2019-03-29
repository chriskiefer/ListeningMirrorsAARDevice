#include "WiFi.h"
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include "maxi.h"

maxiFilter filt;
maxiOsc osc, osc2;
maxiDCBlocker dcblock;
maxiDelayline dl;

//maxiSVF svf;
maxiDyn compressor;
#define SERVER_ADVERT_STRING "%groundControlTo?"

const char* ssid     = "ListeningMirrors";
const char* password = "badgersbadgers";


const int GIVEREQ = 17;
const int RECEIVEREQ = 18;
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

WiFiUDP udp, udpStreamOut, udpStreamIn;
enum networkState {DISCONNECTED, CONNECTING, WAITING_FOR_ADVERT, CONNECTED} netState;
boolean connected = false;
IPAddress LMServerIP;

#define AUDIOBLOCKSIZE 128

float audioOutBuffer[AUDIOBLOCKSIZE];
unsigned long audioFrame = 0;
int blockPos = 0;

int i2s_block_size = 128;

BaseType_t xStatus;
/* time to block the task until the queue has free space */
const TickType_t xTicksToWait = pdMS_TO_TICKS(100);


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


void mainLoop( void * pvParameters ) {
  String taskMessage = "Main loop task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  Serial.println("Finding a listening mirrors server...");
  while (1) {
    //    Serial.println(netState);
    //    delay(0.1);
    switch (netState) {
      case DISCONNECTED:
        connectToAP();
        break;
      case WAITING_FOR_ADVERT:
        {
          int udpMsgLength = udp.parsePacket();
          //          Serial.println("Parsing packet");
          //          Serial.println(udpMsgLength);
          if (udpMsgLength != 0) {
            byte udpPacket[udpMsgLength + 1];
            udp.read(udpPacket, udpMsgLength);
            udpPacket[udpMsgLength] = 0;
            udp.flush(); // empty UDP buffer for next packet
            udp.stop();
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
                tcpClient.read(&data[0], 2);
                Serial.println((char *)data);
                Serial.println("Sending audio...");
                Serial.println("Requesting an audio stream destination");
                if (tcpClient.connect(LMServerIP, CONTROL_SERVICE_PORT)) {
                  tcpClient.print((char)RECEIVEREQ);
                  tcpClient.print((char)0);
                  tcpClient.print((char)0);
                  tcpClient.print((char)0);
                  tcpClient.print('\n');
                  uint8_t data[2];
                  tcpClient.read(&data[0], 2);
                  Serial.println((char*)data);
                  Serial.println("Receiving audio...");
                } else {
                  Serial.println("Can't connect to LM Server control service");
                }
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

          //          if (samples_read > 0) {
          //            Serial.println(samples_read);
          //          } else {
          //            Serial.print(".");
          //          }

          int32_t *intPtr = (int32_t*) &buf[0];
          for (int i = 0; i < samples_read; i += 2) {
            float w = intPtr[i] / (float) 0x7fffffff;
            w = w * 50.f;
            w = dcblock.play(w, 0.99f);
            //            w = compressor.compress(w);
            //            w = w + osc.saw(300.f + (200.0f * osc2.sinewave(0.2)));

            //        w = osc.saw(100);
            //            w = filt.lores(w, 300, 1);
            //    mean += w;
            //    w *= 0.05f;

            //            w = svf.play(w, 1.f, 0.f, 0.f, 0.f);
            audioOutBuffer[blockPos] = w ;
            blockPos++;
            if (blockPos == AUDIOBLOCKSIZE) {
              blockPos = 0;
              int beginPacketResult = udpStreamOut.beginPacket(LMServerIP, STREAM_SOURCE_PORT);
              if (beginPacketResult) {
                size_t udpWriteResult = udpStreamOut.write((uint8_t *) &audioOutBuffer[0], sizeof(float) * AUDIOBLOCKSIZE);
                if (!udpStreamOut.endPacket()) {
                  Serial.print("udpStreamOut Error");
                }
                else {
//                  Serial.println("Audio sent");
                }
              } else {
                Serial.println("Error beginning udp packet");
              }
            }

            intPtr[i] = int32_t(w * (float)0x7fffffff);
          }

          //          size_t m_bytesWritten;
          //          esp_err_t err = i2s_write((i2s_port_t)m_i2s_num, (const char*)&buf, bytes_read, &m_bytesWritten, 0);
          //          if (err != ESP_OK) {
          //            Serial.print("ESP32 Errorcode ");
          //            Serial.println(err);
          //          }
        }
        break;

    };
  }
}




void netReceiveLoop( void * pvParameters ) {
  int32_t outBuf[512];
  Serial.println("Net receive loop started");
  if (udpStreamIn.begin(LMServerIP, STREAM_TARGET_PORT)) {
    while (1) {
      //      Serial.print("p");
      if (netState == CONNECTED) {
        int udpMsgLength = udpStreamIn.parsePacket();
        //                Serial.println(udpMsgLength);
        if (udpMsgLength != 0) {
//          Serial.print("Rx: ");
//          Serial.println(udpMsgLength);
          byte udpPacket[udpMsgLength];
          udpStreamIn.read(udpPacket, udpMsgLength);
          //          udpPacket[udpMsgLength] = 0;

          //output to DAC
          float *sampleBuf = (float*) &udpPacket[0];
          int nSamples = udpMsgLength / sizeof(float);
          //           Serial.println(nSamples);
          for (int i = 0; i < nSamples; i++) {
            outBuf[i * 2] = outBuf[(i * 2) + 1] = int32_t(sampleBuf[i] * (float)0x7fffffff);
            //            outBuf[i] = 0;
          }
          size_t m_bytesWritten;
          esp_err_t err = i2s_write((i2s_port_t)m_i2s_num, (const char*)&outBuf[0], 2 * nSamples * sizeof(int32_t), &m_bytesWritten, 0);
          if (err != ESP_OK) {
            Serial.print("ESP32 Errorcode ");
            Serial.println(err);
          }
          udpStreamIn.flush(); // empty UDP buffer for next packet

        }
      }
      delay(0.001);
    }

  } else {
    Serial.println("Could not initalise udpStreamIn");

  }
}

void setup() {
  Serial.begin(115200);
  compressor.setAttack(100);
  compressor.setRelease(100);
  compressor.setThreshold(0.8);
  compressor.setRatio(3);


  maxiSettings::setup(44100, 1,  64);
  //  svf.setCutoff(3000);
  //  svf.setResonance(10);

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


  for (int i = 0; i < AUDIOBLOCKSIZE; i++) {
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

  BaseType_t taskErr;

  taskErr = xTaskCreatePinnedToCore(
              mainLoop,   /* Function to implement the task */
              "mainLoop", /* Name of the task */
              8192 * 2,    /* Stack size in words */
              NULL,       /* Task input parameter */
              configMAX_PRIORITIES - 1,        /* Priority of the task */
              NULL,       /* Task handle. */
              1);  /* Core where the task should run */

  if (taskErr != pdPASS) {
    Serial.println("Error creating main loop task: " + taskErr);
  }

  taskErr = xTaskCreatePinnedToCore(
              netReceiveLoop,   /* Function to implement the task */
              "netReceiveLoop", /* Name of the task */
              8192 * 2,    /* Stack size in words */
              NULL,       /* Task input parameter */
              configMAX_PRIORITIES - 1,        /* Priority of the task */
              NULL,       /* Task handle. */
              1);  /* Core where the task should run */

  if (taskErr != pdPASS) {
    Serial.println("Error creating netReceiveLoop task: " + taskErr);
  }

}

void loop() {
  delay(1000);
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
