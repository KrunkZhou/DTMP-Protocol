#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <algorithm>
#include <iterator>

#define NETWORKID 0x43

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 100 // Define the payload size here

class LogData {
  public:
    int id;
    std::vector<int> seqNumbers;
    LogData(int id, std::vector<int> seqNums) : id(id), seqNumbers(seqNums) {}
};

class LogDataManager {
  private:
    std::map<std::string, std::vector<int>> items;
    std::default_random_engine generator;

  public:
    // Add sequence numbers to an item; if the item doesn't exist, create a new entry.
    void add(const std::string& id, const std::vector<int>& seqNumbers) {
      // Check if the item already exists
      auto it = items.find(id);
      if (it != items.end()) {
        // Item exists, append the new sequence numbers
        it->second.insert(it->second.end(), seqNumbers.begin(), seqNumbers.end());
      } else {
        // No item exists with this ID, create a new one
        items[id] = seqNumbers;
      }
    }

    // Search an item by ID and return its sequence of numbers
    std::vector<int> searchById(const std::string& id) {
      auto it = items.find(id);
      if (it != items.end()) {
        return it->second;
      } else {
        return {};
      }
    }

    // Remove a specific item from the sequence numbers array for a given ID
    bool removeItem(const std::string& id, int number) {
      if (items.find(id) != items.end()) {
        auto& vec = items[id];
        auto end_it = std::remove(vec.begin(), vec.end(), number);
        if (end_it != vec.end()) {
          vec.erase(end_it, vec.end());
          return true; // Item was found and removed
        }
      }
      return false; // Item not found or ID does not exist
    }

    std::string randomSelectSensorID() {
      if (items.empty()) {
        return "-1";
      }
      // Create a random distribution based on the number of items
      std::uniform_int_distribution<size_t> distribution(0, items.size() - 1);
      // Create an iterator and advance it to a random position
      auto it = items.begin();
      std::advance(it, distribution(generator));
      // Return the key at the random iterator position
      return it->first;
    }

    // Randomly select an item from the sequence numbers array for a given ID
    int randomSelect(const std::string& id) {
      if (items.find(id) != items.end() && !items[id].empty()) {
        std::uniform_int_distribution<int> distribution(0, items[id].size() - 1);
        int index = distribution(generator);
        return items[id][index];
      }
      return -1;
    }

    // Method to check if a specific key with a value exists
    bool keyWithValueExists(const std::string& id, int value) {
        // Check if the key exists in the map
        auto it = items.find(id);
        if (it != items.end()) {
            // Key exists, check if the value is in the vector
            auto& vec = it->second;
            return std::find(vec.begin(), vec.end(), value) != vec.end();
        }
        return false;
    }

    // Calculate and return the total count of all sequence numbers in all items
    size_t totalCount() {
      size_t count = 0;
      for (const auto& item : items) {
        count += item.second.size();
      }
      return count;
    }

    void displayItems() {
      for (const auto& pair : items) {
        std::cout << "ID: " << pair.first << " Sequence Numbers: ";
        for (int num : pair.second) {
          std::cout << num << " ";
        }
        std::cout << std::endl;
      }
    }
};

SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

uint64_t chipid;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
int16_t txNumber;
int16_t rssi,rxSize;
void OnTxDone( void );
void OnTxTimeout( void );
bool lora_idle = true;

LogDataManager ack_log;

void setup() {
    Serial.begin(115200);

    chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
    Serial.printf("ESP32ChipID=%016llX\n", chipid);

    VextON();
    delay(100);
    factory_display.init();
    factory_display.clear();
    factory_display.display();
  
    pinMode(LED ,OUTPUT);
    digitalWrite(LED, LOW);

    factory_display.clear();
    factory_display.display();       
    factory_display.setFont(ArialMT_Plain_10);
    factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
    factory_display.drawStringMaxWidth(0, 0, 128,"Starting Gateway" );
    factory_display.display();
    
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    txNumber=0;
    rssi=0;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                               LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
}

void loop()
{
  if(lora_idle)
  {
          lora_idle = false;
          Serial.println("into RX mode");
          factory_display.clear();
          factory_display.display();  
          factory_display.setFont(ArialMT_Plain_10);
          factory_display.setTextAlignment(TEXT_ALIGN_CENTER);
          factory_display.drawString(64, 0, "Gateway");
          factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
          factory_display.drawString(0, 10, "Ack");
          factory_display.setTextAlignment(TEXT_ALIGN_RIGHT);
          factory_display.drawString(128, 10, String(ack_log.totalCount()));
          factory_display.display();
          //Serial.println(ack_log.totalCount());
          Radio.Rx(0);
  }
  Radio.IrqProcess( );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep( );
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
    int msgtype = atoi(String(rxpacket).substring(0, 1).c_str());

    switch(msgtype) {
      case 0: {
        Serial.println("> DP Received");
        String net = String(rxpacket).substring(1, 3);
        if (net != String(NETWORKID, HEX)){
          break;
        }
        String id = String(rxpacket).substring(3, 19);
        //String seq = String(rxpacket).substring(19, String(rxpacket).indexOf(',', 19));
        int seq = atoi(String(rxpacket).substring(19, String(rxpacket).indexOf(',', 19)).c_str());
        String dp = String(rxpacket).substring(String(rxpacket).indexOf(',', 19) + 1, strlen(rxpacket));
        Serial.print(id);
        Serial.print("\n");
        Serial.print(seq);
        Serial.print("\n");

        if (!ack_log.keyWithValueExists(id.c_str(), seq)) {
          ack_log.add(id.c_str(), {seq});
        }
        
        sprintf(txpacket,"1%02X%s%s%i",NETWORKID, id.c_str(), id.c_str(), seq);
        
        Serial.print("> Sending ack\n");
        Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
        Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
        lora_idle = false;
        return;
      }
      case 1: {
        Serial.println("> Ack Received");
//        String net = String(rxpacket).substring(1, 3);
//        if (net != String(NETWORKID, HEX)){
//          break;
//        }
        break;
      }
      default:
        Serial.println("> Skip");
        break;
    }
    lora_idle = true;
}

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void OnTxDone( void )
{
  Serial.println("TX done......");
  lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}
