/*

  _     _  ____  _____  _____ _      _  _      _____   _      _  ____  ____  ____  ____  ____
  / \   / \/ ___\/__ __\/  __// \  /|/ \/ \  /|/  __/  / \__/|/ \/  __\/  __\/  _ \/  __\/ ___\
  | |   | ||    \  / \  |  \  | |\ ||| || |\ ||| |  _  | |\/||| ||  \/||  \/|| / \||  \/||    \
  | |_/\| |\___ |  | |  |  /_ | | \||| || | \||| |_//  | |  ||| ||    /|    /| \_/||    /\___ |
  \____/\_/\____/  \_/  \____\\_/  \|\_/\_/  \|\____\  \_/  \|\_/\_/\_\\_/\_\\____/\_/\_\\____/

  Chris Kiefer and Cecile Chevalier, 2019

*/

#include "WiFi.h"
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include "maxi.h"
#include "freertos/ringbuf.h"
#include "AsyncUDP.h"

#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(2, 17, NEO_GRB + NEO_KHZ800);

float masterVol = 0.4;

maxiDCBlocker dcblock;
maxiOsc osc;


#define SERVER_ADVERT_STRING "%groundControlTo?"

const char* ssid     = "ListeningMirrors";
const char* password = "badgersbadgers";




const int GIVEREQ = 17;
const int RECEIVEREQ = 18;
const int udpAdvPort = 13252;
const int UDP_PORT = 13251;
const uint16_t CONTROL_SERVICE_PORT = 14672;
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
AsyncUDP udpAsyncIn, udpAsyncOut;
enum networkState {DISCONNECTED, CONNECTING, WAITING_FOR_ADVERT, WAITING_FOR_GIVE_REQ, WAITING_FOR_RECEIVE_REQ, CONNECTED} netState;
boolean connected = false;
IPAddress LMServerIP;

#define AUDIOBLOCKSIZE 64

float audioOutBuffer[AUDIOBLOCKSIZE];
unsigned long audioFrame = 0;
int blockPos = 0;

RingbufHandle_t recvRing;
uint32_t recvRingSize = AUDIOBLOCKSIZE * 16 * sizeof(float);
//uint32_t recvRingMin = (uint32_t)recvRingSize * 0.75;
//uint32_t recvRingSizeThreshold = (recvRingSize / 2) + 1;
bool streamingStarted = 0;

int i2s_block_size = 64;

int lastAmpVal = 0;
int cxRed, cxBlue, cxGreen;

bool buffering = true;

size_t sentCount=0;
size_t recvCount=0;
size_t playedCount=0;
size_t audioInCount=0;

void connectToAP() {
  netState = CONNECTING;

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("Waiting for WIFI connection...");
}


int frame = 0;
uint16_t buf_len = AUDIOBLOCKSIZE * 2 * sizeof(float);
char buf[AUDIOBLOCKSIZE * 2 * sizeof(float)];

const float maxAmp32 = (float) 0x7fffffff;
WiFiClient tcpClient;

struct streamRequest {
  enum streamTypes {GIVE = 17, RECEIVE, EXIT} streamType;
  char endRequest = '\n';
  streamRequest(streamTypes T = streamTypes::RECEIVE) : streamType(T) {}
};

uint32_t serverAliveTimeStamp = 0;


uint32_t tcpResponseTimer = 0;

size_t reportCount=0;

void mainLoop( void * pvParameters ) {
  int32_t outBuf[1024];
  String taskMessage = "Main loop task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  Serial.println("Finding a listening mirrors server...");
  while (1) {
    //always read the mics, otherwise data will build up while the network is connecting
    size_t bytes_read;
    esp_err_t readResponse = i2s_read(I2S_PORT_MIC, buf, buf_len, &bytes_read, 0);
    if (readResponse != ESP_OK) {
      Serial.println("i2s read error");
    }

    //    Serial.println(netState);
    //    delay(0.1);
    switch (netState) {
      case DISCONNECTED:
        pixels.begin();
        pixels.setPixelColor(0, pixels.Color(0, 150, 0));
        pixels.show();
        connectToAP();
        break;
      case WAITING_FOR_ADVERT:
        {
          pixels.setPixelColor(0, pixels.Color(255, 0, 0));
          pixels.show();
          int udpMsgLength = udp.parsePacket();
          //          Serial.println("Parsing packet");
          //          Serial.println(udpMsgLength);
          if (udpMsgLength != 0) {
            byte udpPacket[udpMsgLength + 1];
            udp.read(udpPacket, udpMsgLength);
            udpPacket[udpMsgLength] = 0;
            String advMessage = String((char*)udpPacket);
            Serial.println("Received: " + advMessage);
            if (advMessage == "%groundControlTo?") {
              LMServerIP = udp.remoteIP();
              Serial.println("Server found, address: " + LMServerIP.toString());
              Serial.println("Requesting an audio stream destination");
              if (tcpClient.connect(LMServerIP, CONTROL_SERVICE_PORT)) {
                streamRequest reqData(streamRequest::GIVE);
                tcpClient.write((uint8_t*)&reqData, sizeof(streamRequest));
                //                tcpClient.print('\n');
                Serial.println("Sent control request, waiting for response...");
                tcpResponseTimer = millis();
                netState = WAITING_FOR_GIVE_REQ;
              } else {
                Serial.println("Can't connect to LM Server control service");
              }
            }
            udp.flush();
            udp.stop();
          }
          delay(100);
        }
        break;
      case WAITING_FOR_GIVE_REQ:
        {
          pixels.setPixelColor(0, pixels.Color(0, 0, 255));
          pixels.show();
          if (tcpClient.available()) {
            uint8_t data[2];
            tcpClient.read(&data[0], 2);
            Serial.println("Received give req response");
            Serial.print((char) data[0]);
            Serial.println((char) data[1]);
            if (data[0] == 'O' && data[1] == 'K') {
              Serial.println("Sending audio...");
              Serial.println("Requesting an audio stream source");
              tcpClient.stop();
              if (tcpClient.connect(LMServerIP, CONTROL_SERVICE_PORT)) {
                streamRequest reqData(streamRequest::RECEIVE);
                tcpClient.write((uint8_t*)&reqData, sizeof(streamRequest));
                tcpResponseTimer = millis();
                netState = WAITING_FOR_RECEIVE_REQ;

              } else {
                Serial.println("Can't connect to LM Server control service");
              }
            }
          } else {
            if (millis() - tcpResponseTimer > 1000) {
              netState = WAITING_FOR_ADVERT;
            }
          }
          delay(100);
          break;
        }
      case WAITING_FOR_RECEIVE_REQ:
        {
          pixels.setPixelColor(0, pixels.Color(255, 0, 255));
          pixels.show();
          if (tcpClient.available()) {
            uint8_t data[2];
            tcpClient.read(&data[0], 2);
            Serial.println("Received receive req response");
            Serial.print((char) data[0]);
            Serial.println((char) data[1]);
            if (data[0] == 'O' && data[1] == 'K') {
              tcpClient.stop();
              Serial.println("Starting to stream");
              streamingStarted = 0;
              serverAliveTimeStamp = millis();
              cxRed = 153 + random(-40, 40);
              cxGreen = 92 + random(-40, 40);
              cxBlue = 223 + random(-40, 30);
              pixels.setPixelColor(0, pixels.Color(cxRed, cxGreen, cxBlue));
              pixels.show();

              //setup udp comms for streaming
              Serial.println("Starting async udp");
              if (udpAsyncIn.listen(STREAM_TARGET_PORT)) {
                Serial.println("Async udp connected");
                udpAsyncIn.onPacket([&] (AsyncUDPPacket packet) {
                  //                Serial.println("p");
                  String taskMessage = "Main loop task running on core ";
                  taskMessage = taskMessage + xPortGetCoreID();

                  recvCount++;
                  if (netState == CONNECTED) {
                    int bufferSpace = xRingbufferGetCurFreeSize(recvRing);
                    if (bufferSpace > 0) {
                      int udpMsgLength = packet.length();
                      //output to DAC via ringbuffer
                      xRingbufferSend(recvRing, packet.data(), packet.length(), pdMS_TO_TICKS(0));
                      serverAliveTimeStamp = millis();
                      //                    Serial.print("Written to ringbuf: ");
                      //                    Serial.println(udpMsgLength);
                    }
                    if (buffering && bufferSpace < recvRingSize / 2) {
                      buffering = false;
                      Serial.println("Buffering complete");
                    }
                  }
                }); // end of packet receiver
  
                if (!udpAsyncOut.connect(LMServerIP, STREAM_SOURCE_PORT)) {
                  Serial.println("Connect error: stream out udp");
                  netState = WAITING_FOR_GIVE_REQ;
                }
                else
                {
                  netState = CONNECTED; 
                }
              } else {
                Serial.println("Could not initalise async udp client");
                netState = WAITING_FOR_GIVE_REQ;
              }
          }
            //                if (udpStreamIn.begin(LMServerIP, STREAM_TARGET_PORT)) {
            //                } else {
            //                  Serial.println("Error opening udpStreamIn");
            //                }

          } else {
            if (millis() - tcpResponseTimer > 1000) {
              netState = WAITING_FOR_GIVE_REQ;
            }

          }
//          delay(500);
          break;
        }
      case CONNECTED:
        {
//          if (reportCount++ == 100) {
//            reportCount=0;
//            Serial.print(audioInCount);
//            Serial.print("\t");
//            Serial.print(playedCount);
//            Serial.print("\t");
//            Serial.print(recvCount);
//            Serial.print("\t");
//            Serial.print(sentCount);
//            Serial.println("");
//          }
          uint32_t now = millis();
          uint32_t gap = now - serverAliveTimeStamp;
          if (gap > 5000) {
            Serial.println("Lost the server, restarting");
            ESP.restart();
          }


          if (bytes_read > 0) {
            audioInCount++;
            uint32_t samples_read = bytes_read  / (I2S_BITS_PER_SAMPLE_32BIT / 8);

            //          if (samples_read > 0) {
            //            Serial.println(samples_read);
            //          } else {
            //            Serial.print(".");
            //          }

            int32_t *intPtr = (int32_t*) &buf[0];
            for (int i = 0; i < samples_read; i += 2) {
              float w = intPtr[i] / maxAmp32;
              float w2 = intPtr[i + 1] / maxAmp32;
              w = w + w2;
              //            w = w * 10.f;
              // w = dcblock.play(w, 0.99f);
              //            if (i==0)
              //              Serial.println(w);
              w = w * 1.0f;

              //            w = compressor.compress(w);
              //            w = w + osc.saw(300.f + (200.0f * osc2.sinewave(0.2)));

//              w = osc.saw(200);
              //            w = filt.lores(w, 300, 1);
              //    mean += w;
              //    w *= 0.05f;

              audioOutBuffer[blockPos] = w ;
              blockPos++;
              if (blockPos == AUDIOBLOCKSIZE) {
                blockPos = 0;
                size_t udpWriteResult = udpAsyncOut.write((uint8_t *) &audioOutBuffer[0], sizeof(float) * AUDIOBLOCKSIZE);
                if (udpWriteResult != (sizeof(float) * AUDIOBLOCKSIZE)) {
                  Serial.print("udp stream out error");
                }
                else {
                  sentCount++;
                  //                                                      Serial.println("Audio sent");
                }


//                if (xRingbufferGetCurFreeSize(recvRing) > 0) {
                    
//                  if (udpStreamOut.begin(STREAM_SOURCE_PORT)) {
//                    int beginPacketResult = udpStreamOut.beginPacket(LMServerIP, STREAM_SOURCE_PORT);
//                    if (beginPacketResult) {
//                      size_t udpWriteResult = udpStreamOut.write((uint8_t *) &audioOutBuffer[0], sizeof(float) * AUDIOBLOCKSIZE);
//                      if (!udpStreamOut.endPacket()) {
//                        Serial.print("udpStreamOut Error");
//                      }
//                      else {
//                        sentCount++;
//                        //                                                      Serial.println("Audio sent");
//                      }
//                    } else {
//                      Serial.println("Error beginning udp packet");
//                    }
//                    udpStreamOut.stop();
//                  } else {
//                    Serial.println("udp not opening");
//                  }
//                } else {
//                  Serial.println("rebuffering");
//                }
              }

              //            intPtr[i] = int32_t(w * (float)0x7fffffff);
              //            if (samples_read > 0) {
              //              Serial.print("ring buf: " );
              //              Serial.println(xRingbufferGetCurFreeSize(recvRing));
              //            } else {
              //            }
              //            if (samples_read > 0) {
            }
          }

          if (!buffering && bytes_read>0) {
            float *sampleBuf;
            size_t recvRingCount;
            size_t bufFreeSpace = xRingbufferGetCurFreeSize(recvRing);
//            Serial.println(bufFreeSpace);
            if (bufFreeSpace < recvRingSize) {
              sampleBuf = (float*) xRingbufferReceiveUpTo(recvRing, &recvRingCount, pdMS_TO_TICKS(0), bytes_read /2);
              if (sampleBuf != NULL) {
                recvRingCount /= sizeof(float);
  
                for (int i = 0; i < recvRingCount; i++) {
                  int32_t w = int32_t(sampleBuf[i] * masterVol * (float)0x7fffffff);
                  outBuf[i * 2] = outBuf[(i * 2) + 1] = w;
                }
                size_t m_bytesWritten;
                esp_err_t err = i2s_write((i2s_port_t)m_i2s_num, (const char*)&outBuf[0], 2 * recvRingCount * sizeof(int32_t), &m_bytesWritten, 0);
                //              Serial.print(recvRingCount);
                //              Serial.print(" ");
                //              Serial.println(m_bytesWritten);
                playedCount++;
                if (err != ESP_OK) {
                  Serial.print("ESP32 Errorcode ");
                  Serial.println(err);
                }
                vRingbufferReturnItem(recvRing, (void*)sampleBuf);
              }
            }
          }
          //          } else if (bufFreeSpace == recvRingSize) {
          //            Serial.println("dac: buffer underrun");
          //          }
//          if (samples_read > 0) {
            //            int val = analogRead(A6);
            //            if (val < lastAmpVal - 200 || val > lastAmpVal + 200) {
            //              masterVol = powf(val / 4095.0f, 2.f);
            //              lastAmpVal = val;
            //              //              Serial.print("Amp: ");
            //              //              Serial.println(masterVol);
            //            };

//            frame++;
            //            int mod = (int) (sinf((float)frame * 0.01) * 80.0);
            //            pixels.setPixelColor(0, pixels.Color(cxRed + mod, cxGreen + mod, cxBlue + mod));
            //            pixels.show();

//          }


          //          size_t m_bytesWritten;
          //          esp_err_t err = i2s_write((i2s_port_t)m_i2s_num, (const char*)&buf, bytes_read, &m_bytesWritten, 0);
          //          if (err != ESP_OK) {
          //            Serial.print("ESP32 Errorcode ");
          //            Serial.println(err);
          //          }



        }
        break;

    };
    taskYIELD();

  } // end of loop
}



uint32_t underrunCount = 0;
//void netReceiveLoop( void * pvParameters ) {
//  //  int32_t outBuf[512];
//  Serial.println("Net receive thread waiting");
//  if (udpStreamIn.begin(LMServerIP, STREAM_TARGET_PORT)) {
//    while (1) {
//      //      Serial.print("p");
//      if (netState == CONNECTED) {
//        int bufferSpace = xRingbufferGetCurFreeSize(recvRing);
//        if (bufferSpace > 0) {
//          int udpMsgLength = udpStreamIn.parsePacket();
//          //                Serial.println(udpMsgLength);
//          if (udpMsgLength != 0) {
//            //          Serial.print("Rx: ");
//            //          Serial.println(udpMsgLength);
//            byte udpPacket[udpMsgLength];
//            udpStreamIn.read(udpPacket, udpMsgLength);
//            //          udpPacket[udpMsgLength] = 0;
//
//            //output to DAC via ringbuffer
//            float *sampleBuf = (float*) &udpPacket[0];
//            int nSamples = udpMsgLength / sizeof(float);
//            xRingbufferSend(recvRing, udpPacket, udpMsgLength, pdMS_TO_TICKS(0));
//            serverAliveTimeStamp = millis();
//            udpStreamIn.flush(); // empty UDP buffer for next packet
//            Serial.print("Written to ringbuf: ");
//            Serial.println(udpMsgLength);
//            uint32_t now = millis();
//            uint32_t gap = now - serverAliveTimeStamp;
//            if (gap > 5000) {
//              Serial.println("Lost the server, restarting");
//              ESP.restart();
//            }
//
//          }
//        } else {
//          Serial.print("buffer underrun ");
//          Serial.println(underrunCount++);
//          delay(0.01);
//        }
//        if (buffering && bufferSpace < recvRingSize / 2) {
//          buffering = false;
//          Serial.println("Buffering complete");
//        }
//      }
//      taskYIELD();
//    } // end of loop
//
//  } else {
//    Serial.println("Could not initalise udpStreamIn");
//
//  }
//}
//
void ampReadLoop( void * pvParameters ) {
  while (1) {
    int val = analogRead(A6);
//    Serial.println(val);
    if (val < lastAmpVal - 100 || val > lastAmpVal + 100) {
      masterVol = powf(val / 4095.0f, 2.f);
      lastAmpVal = val;
//      Serial.print("Amp: ");
//      Serial.println(masterVol);
    };
    delay(100);
  }
}


void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();

  recvRing = xRingbufferCreate(recvRingSize, RINGBUF_TYPE_BYTEBUF);
  if (recvRing == NULL) {
    Serial.println("Couldn't create recvRing");
  }

  //  compressor.setAttack(100);
  //  compressor.setRelease(100);
  //  compressor.setThreshold(0.8);
  //  compressor.setRatio(3);
  size_t SAMPLERATE=44100;

  maxiSettings::setup(SAMPLERATE, 1,  128);
  //  svf.setCutoff(3000);
  //  svf.setResonance(10);
  

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLERATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = i2s_block_size,   //Interrupt level 1
    .use_apll = 1,
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
    .sample_rate = SAMPLERATE,                         // 
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // although the SEL config should be left, it seems to transmit on right
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
    .dma_buf_count = 8,                           // number of buffers
    .dma_buf_len = i2s_block_size,                 // 8 samples per buffer (minimum)
    .use_apll = 1,

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
  }
  err = i2s_set_pin(I2S_PORT_MIC, &pin_config_mic);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
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


  //  WiFi.mode(WIFI_STA);
  //  WiFi.begin(ssid, password);

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
              8192*8,    /* Stack size in words */
              NULL,       /* Task input parameter */
              configMAX_PRIORITIES - 1,        /* Priority of the task */
              NULL,       /* Task handle. */
              1);  /* Core where the task should run */

  if (taskErr != pdPASS) {
    Serial.println("Error creating main loop task: " + taskErr);
  }

  //  taskErr = xTaskCreatePinnedToCore(
  //              netReceiveLoop,   /* Function to implement the task */
  //              "netReceiveLoop", /* Name of the task */
  //              8192,    /* Stack size in words */
  //              NULL,       /* Task input parameter */
  //              configMAX_PRIORITIES - 1,        /* Priority of the task */
  //              NULL,       /* Task handle. */
  //              1);  /* Core where the task should run */
  //
  //  if (taskErr != pdPASS) {
  //    Serial.println("Error creating netReceiveLoop task: " + taskErr);
  //  }

    taskErr = xTaskCreatePinnedToCore(
                ampReadLoop,   /* Function to implement the task */
                "ampReadLoop", /* Name of the task */
                2048,    /* Stack size in words */
                NULL,       /* Task input parameter */
                configMAX_PRIORITIES - 2,        /* Priority of the task */
                NULL,       /* Task handle. */
                1);  /* Core where the task should run */
  
    if (taskErr != pdPASS) {
      Serial.println("Error creating ampReadLoop task: " + taskErr);
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
