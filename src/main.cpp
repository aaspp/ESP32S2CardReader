/**
 * Author: Ammar Panjwani
 * Email: ammar.panj@gmail.com 
 * This Code is written with just the User ID of a 26 bit Weigand card in mind,
 * if your card is another standard you will need to make changes
 */
 

#include <Arduino.h>
#include <Wire.h>

// Config

// Wiegand pins
static const int PIN_D0 = 17; //18;
static const int PIN_D1 = 18; //19;

// I2C 
#define PN532_I2C_ADDR 0x24       
#define I2C_SDA_PIN    8 //21
#define I2C_SCL_PIN    9 //22
#define I2C_FREQ       100000

// Wiegand
#define MAX_BITS 100
#define WEIGAND_WAIT_TIME 30000UL  

// Packaging to be compaitbale with PN532
#define PN532_RDY 0x01


static const uint8_t PN532_ACK[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

// Wiegand state 
volatile uint8_t  g_bits[MAX_BITS];
volatile uint8_t  g_bitCount = 0;
volatile uint8_t  g_flagDone = 1;
volatile uint32_t g_wgCounter = 0;

volatile uint16_t g_lastCN = 0;      // 16-bit Card Number (lower 16 of 26 bits)
volatile bool     g_hasNewCN = false;

static inline void memsetVolatile(volatile uint8_t* dst, uint8_t v, size_t n){ 
  for (size_t i=0;i<n;i++) dst[i]=v; 
}


static inline void memcpyFromVolatile(uint8_t* d, const volatile uint8_t* s, size_t n){ 
  for(size_t i=0;i<n;i++) d[i]=s[i]; 
}



enum class TxPhase : uint8_t { IDLE, SEND_ACK, SEND_RESP };
volatile TxPhase g_phase = TxPhase::IDLE;


uint8_t  g_resp[64];
uint8_t  g_respLen = 0;
uint8_t  g_cmd[64];
uint8_t  g_cmdLen = 0;

static void buildPN532DataFrame(uint8_t* out, uint8_t& outLen, const uint8_t* payload, uint8_t payloadLen) {

  uint8_t idx = 0;
  out[idx++] = 0x00;  
  out[idx++] = 0x00;   
  out[idx++] = 0xFF;  

  const uint8_t LEN = payloadLen;           
  const uint8_t LCS = (uint8_t)(0x100 - LEN);

  out[idx++] = LEN;
  out[idx++] = LCS;

  uint8_t sum = 0;
  for (uint8_t i = 0; i < payloadLen; ++i) {
    out[idx++] = payload[i];
    sum += payload[i];
  }

  const uint8_t DCS = (uint8_t)(0x100 - sum);
  out[idx++] = DCS;
  out[idx++] = 0x00;   

  outLen = idx;
}

// Configures response to motherboard
static void buildRespSamConfiguration() {
  uint8_t pay[3] = {0xD5, 0x15, 0x00};
  buildPN532DataFrame(g_resp, g_respLen, pay, sizeof(pay));
}

// Configures response if there is no new tag
static void build_resp_inautopoll_no_tag() {
  uint8_t pay[2+1] = {0xD5, 0x61, 0x00};
  buildPN532DataFrame(g_resp, g_respLen, pay, 3);
}

// Configures response with a new id
static void build_resp_inautopoll_with_uid(uint16_t cn) {
  // Map 26-bit Wiegand CN (16-bit) → UID (2 bytes: MSB, LSB)
  uint8_t uid[2] = { (uint8_t)(cn >> 8), (uint8_t)(cn & 0xFF) };
  uint8_t uidLen = 2;


  const uint8_t card_type = 0x01;
  const uint8_t sens_res_L = 0x04, sens_res_H = 0x00, sel_res = 0x00, pad0 = 0x00;
  const uint8_t tLen = (uint8_t)(5 + uidLen);

  uint8_t data[2 /*TFI,cmd pre*/ + 3 /*hdr*/ + 5 /*fixed*/ + 2 /*uid max*/];
  uint8_t k = 0;

  data[k++] = 0xD5;
  data[k++] = 0x61;
  data[k++] = 0x01;              
  data[k++] = card_type;         
  data[k++] = tLen;              
  data[k++] = sens_res_L;       
  data[k++] = sens_res_H;        
  data[k++] = sel_res;           
  data[k++] = pad0;              
  data[k++] = uidLen;           
  for (uint8_t i = 0; i < uidLen; ++i) data[k++] = uid[i]; 

  buildPN532DataFrame(g_resp, g_respLen, data, k);
}

static void handle_command_and_prepare_response() {

  int start = -1;
  for (uint8_t i = 0; i + 1 < g_cmdLen; ++i) {
    if (g_cmd[i] == 0x00 && g_cmd[i+1] == 0xFF) { start = i; break; }
  }
  if (start < 0 || (start + 4) > g_cmdLen) { g_respLen = 0; return; }

  uint8_t LEN = g_cmd[start + 2];
  uint8_t* payload = &g_cmd[start + 4];
  if ((start + 4 + LEN) > g_cmdLen) { g_respLen = 0; return; }

  if (LEN < 2) { 
    g_respLen = 0; 
    return; 
  }

  uint8_t TFI = payload[0];
  uint8_t CMD = payload[1];

  if (TFI != 0xD4) { 
    g_respLen = 0; 
    return; 
  }

  switch (CMD) {
    case 0x14: // SAMConfiguration
      buildRespSamConfiguration();
      break;
    case 0x60: { // InAutoPoll
      bool have;
      uint16_t cn;
      noInterrupts();
      have = g_hasNewCN;
      cn   = g_lastCN;
      g_hasNewCN = false;  // consume once
      interrupts();
      if (have) build_resp_inautopoll_with_uid(cn);
      else      build_resp_inautopoll_no_tag();
      break;
    }
    default:
      g_respLen = 0;
      break;
  }
}


void IRAM_ATTR ISR_INT0() { // D0 => logical 0
  if (g_bitCount < MAX_BITS) g_bitCount++;
  g_flagDone = 0;
  g_wgCounter = WEIGAND_WAIT_TIME;
}
void IRAM_ATTR ISR_INT1() { // D1 => logical 1
  if (g_bitCount < MAX_BITS) {
    g_bits[g_bitCount] = 1;
    g_bitCount++;
  }
  g_flagDone = 0;
  g_wgCounter = WEIGAND_WAIT_TIME;
}


void onReceive_I2C(int nbytes) {
  // Read raw command into g_cmd
  uint8_t idx = 0;
  while (Wire.available() && idx < sizeof(g_cmd)) {
    g_cmd[idx++] = (uint8_t)Wire.read();
  }
  g_cmdLen = idx;

  handle_command_and_prepare_response();
  g_phase = TxPhase::SEND_ACK;
}

void onRequest_I2C() {
  if (g_phase == TxPhase::SEND_ACK) {
    // RDY + ACK frame
    Wire.write(PN532_RDY);
    Wire.write(PN532_ACK, sizeof(PN532_ACK));
    g_phase = TxPhase::SEND_RESP;
    return;
  }
  if (g_phase == TxPhase::SEND_RESP && g_respLen > 0) {
    Wire.write(PN532_RDY);
    Wire.write(g_resp, g_respLen);
    g_respLen = 0;
    g_phase = TxPhase::IDLE;
    return;
  }
  // Nothing pending -> say "no msg"
  Wire.write((uint8_t)0x00);
}

/*** ------------ helpers ------------ ***/
static inline void resetWiegand() {
  memsetVolatile(g_bits, 0, MAX_BITS);
  g_bitCount = 0;
}

/*** ------------ setup/loop ------------ ***/
void setup() {
  Serial.begin(115200);
  Serial.println("Daughter: Wiegand -> PN532 I2C emulator");

  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_D0), ISR_INT0, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_D1), ISR_INT1, FALLING);

  resetWiegand();
  g_flagDone = 1;
  g_wgCounter = WEIGAND_WAIT_TIME;

  // I2C daughter @ 0x24 on SDA/SCL
  Wire.begin(PN532_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ); // daughter mode on ESP32
  Wire.onReceive(onReceive_I2C);
  Wire.onRequest(onRequest_I2C);
}

void loop() {
  // Wiegand timing window close
  if (!g_flagDone) {
    if (--g_wgCounter == 0) g_flagDone = 1;
  }

  if (g_bitCount > 0 && g_flagDone) {
    // Snapshot bits
    uint8_t bits[MAX_BITS] = {0};
    uint8_t nbits;
    noInterrupts();
    nbits = g_bitCount;
    memcpyFromVolatile(bits, g_bits, nbits);
    resetWiegand();
    interrupts();

    if (nbits == 26) {
      // Extract 24 data bits (1..24), ignore parity bits
      uint32_t dataField = 0;
      for (uint8_t i = 1; i <= 24; i++) {
        dataField = (dataField << 1) | (bits[i] ? 1u : 0u);
      }
      uint16_t cardNum = (uint16_t)(dataField & 0xFFFF);

      noInterrupts();
      g_lastCN = cardNum;
      g_hasNewCN = true;
      interrupts();

      Serial.printf("Wiegand CN: %u (0x%04X)\n", cardNum, cardNum);
    } else {
      Serial.printf("Unsupported bit length: %u\n", nbits);
    }

    g_flagDone = 1;
    g_wgCounter = WEIGAND_WAIT_TIME;
  }
}
