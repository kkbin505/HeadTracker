/*
 * This file is part of the Head Tracker distribution (https://github.com/dlktdr/headtracker)
 * Copyright (c) 2022 Cliff Blackburn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "sbus.h"

#include <nrfx.h>
#include <nrfx_uarte.h>
#include <string.h>
#include <sys/ring_buffer.h>
#include <zephyr.h>

#include "auxserial.h"
#include "io.h"
#include "log.h"
#include "soc_flash.h"
#include "trackersettings.h"

#define SBUS_FRAME_LEN 25

static constexpr uint8_t HEADER_ = 0x0F;
static constexpr uint8_t FOOTER_ = 0x00;
static constexpr uint8_t FOOTER2_ = 0x04;
static constexpr uint8_t LEN_ = 25;
static constexpr uint8_t CH17_ = 0x01;
static constexpr uint8_t CH18_ = 0x02;
static constexpr uint8_t LOST_FRAME_ = 0x04;
static constexpr uint8_t FAILSAFE_ = 0x08;
static constexpr uint8_t CH17_MASK_ = 0x01;
static constexpr uint8_t CH18_MASK_ = 0x02;
static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
static bool failsafe_ = false, lost_frame_ = false, ch17_ = false, ch18_ = false;

volatile bool sbusTreadRun = false;
volatile bool sbusBuildingData = false; // TODO Replace me with a mutex
volatile bool sbusoutinv = false;
volatile bool sbusininv = false;
volatile bool sbusinsof = false;  // Start of Frame

uint8_t localTXBuffer[SBUS_FRAME_LEN];  // Local Buffer

// crc implementation from CRSF protocol document rev7
static uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++){
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

void sbus_Thread()
{
  while (1) {
    if (!sbusTreadRun || pauseForFlash) {
      rt_sleep_ms(50);
      continue;
    }
    rt_sleep_us((1.0 / (float)trkset.getSbRate()) * 1.0e6);

    // Has the SBUS inverted status changed
    if (sbusininv != !trkset.getSbInInv() ||
        sbusoutinv != !trkset.getSbOutInv()) {
      sbusininv = !trkset.getSbInInv();
      sbusoutinv = !trkset.getSbOutInv();

      // Close and re-open port with new settings
      AuxSerial_Close();
      uint8_t inversion = 0;
      if (sbusininv) inversion |= CONFINV_RX;
      if (sbusoutinv) inversion |= CONFINV_TX;
      AuxSerial_Open(BAUD100000, CONF8E2, inversion);
    }
    // Send SBUS Data
    SBUS_TX_Start();
  }
}

uint8_t buf_[SBUS_FRAME_LEN];
int bytesfilled = 0;
int8_t state_ = 0;
uint8_t prev_byte_ = FOOTER_;
uint8_t cur_byte_;

#ifdef DEBUG
uint64_t bytecount = 0;
#endif
bool SbusRx_Parse()
{
  /* Parse messages */
  while (AuxSerial_Read(&cur_byte_, 1)) {
    /*TODO fixme serialWriteHex(&cur_byte_,1);
    if ((cur_byte_ == HEADER_) && ((prev_byte_ == FOOTER_) ||
       ((prev_byte_ & 0x0F) == FOOTER2_))) {
      serialWriteln();
       }*/
#ifdef DEBUG
    bytecount++;
#endif
    if (state_ == 0) {
      if ((cur_byte_ == HEADER_) &&
          ((prev_byte_ == FOOTER_) || ((prev_byte_ & 0x0F) == FOOTER2_))) {
        buf_[state_++] = cur_byte_;
      } else {
        state_ = 0;
      }
    } else {
      if (state_ < SBUS_FRAME_LEN) {
        buf_[state_++] = cur_byte_;
      } else {
        state_ = 0;
        if ((buf_[SBUS_FRAME_LEN - 1] == FOOTER_) ||
            ((buf_[SBUS_FRAME_LEN - 1] & 0x0F) == FOOTER2_)) {
          return true;
        } else {
          return false;
        }
      }
    }
    prev_byte_ = cur_byte_;
  }
  return false;
}

#ifdef DEBUG
uint8_t sbusrate = 0;
uint64_t sbstarttime = 0;
uint64_t bytesread = 0;
#endif

/* FROM -----
 * Brian R Taylor
 * brian.taylor@bolderflight.com
 *
 * Copyright (c) 2021 Bolder Flight Systems Inc
 */

bool SBUS_Read_Data(uint16_t ch_[16])
{
#ifdef DEBUG_SBUS
  static bool toggle = false;
  pinMode(D_TO_32X_PIN(8), GPIO_OUTPUT);
  digitalWrite(D_TO_32X_PIN(8), toggle);
  toggle = !toggle;
#endif
  bool newdata = false;
  while (SbusRx_Parse()) {  // Get most recent data if more than 1 packet came in
    newdata = true;
  }
  if (newdata) {
    ch_[0] = static_cast<int16_t>(buf_[1] | ((buf_[2] << 8) & 0x07FF));
    ch_[1] = static_cast<int16_t>(buf_[2] >> 3 | ((buf_[3] << 5) & 0x07FF));
    ch_[2] = static_cast<int16_t>(buf_[3] >> 6 | ((buf_[4] << 2) | ((buf_[5] << 10) & 0x07FF)));
    ch_[3] = static_cast<int16_t>(buf_[5] >> 1 | ((buf_[6] << 7) & 0x07FF));
    ch_[4] = static_cast<int16_t>(buf_[6] >> 4 | ((buf_[7] << 4) & 0x07FF));
    ch_[5] = static_cast<int16_t>(buf_[7] >> 7 | ((buf_[8] << 1) | ((buf_[9] << 9) & 0x07FF)));
    ch_[6] = static_cast<int16_t>(buf_[9] >> 2 | ((buf_[10] << 6) & 0x07FF));
    ch_[7] = static_cast<int16_t>(buf_[10] >> 5 | ((buf_[11] << 3) & 0x07FF));
    ch_[8] = static_cast<int16_t>(buf_[12] | ((buf_[13] << 8) & 0x07FF));
    ch_[9] = static_cast<int16_t>(buf_[13] >> 3 | ((buf_[14] << 5) & 0x07FF));
    ch_[10] = static_cast<int16_t>(buf_[14] >> 6 | ((buf_[15] << 2) | ((buf_[16] << 10) & 0x07FF)));
    ch_[11] = static_cast<int16_t>(buf_[16] >> 1 | ((buf_[17] << 7) & 0x07FF));
    ch_[12] = static_cast<int16_t>(buf_[17] >> 4 | ((buf_[18] << 4) & 0x07FF));
    ch_[13] = static_cast<int16_t>(buf_[18] >> 7 | ((buf_[19] << 1) | ((buf_[20] << 9) & 0x07FF)));
    ch_[14] = static_cast<int16_t>(buf_[20] >> 2 | ((buf_[21] << 6) & 0x07FF));
    ch_[15] = static_cast<int16_t>(buf_[21] >> 5 | ((buf_[22] << 3) & 0x07FF));
    for (int i = 0; i < 16; i++) {  // Shift + Scale SBUS to PPM Range
      ch_[i] = (((float)ch_[i] - TrackerSettings::SBUS_CENTER) / TrackerSettings::SBUS_SCALE) +
               TrackerSettings::PPM_CENTER;
      if (ch_[i] > TrackerSettings::MAX_PWM) ch_[i] = TrackerSettings::MAX_PWM;
      if (ch_[i] < TrackerSettings::MIN_PWM) ch_[i] = TrackerSettings::MIN_PWM;
    }

#ifdef DEBUG_SBUS
    static bool toggle = false;
    pinMode(D_TO_32X_PIN(7), GPIO_OUTPUT);
    digitalWrite(D_TO_32X_PIN(7), toggle);
    toggle = !toggle;

    if (sbusrate++ == 0) {
      sbstarttime = millis64();  // Store start time
      bytesread = bytecount;

    } else if (sbusrate == 100) {  // After 100 samples, output the time taken
      float elapsed = (float)(millis64() - sbstarttime) / 1000.0f;
      uint32_t bytes = bytecount - bytesread;  // Bytes read in this time
      sbusrate = 0;
      LOGD("SBUS Rate - %d BytesRx - %d", (int)(elapsed * 1000.0f), (int)bytes);
    }
#endif

    return true;
  }

  return false;
}

void SBUS_TX_Start()
{
  if (sbusBuildingData) return;
  AuxSerial_Write(localTXBuffer, SBUS_FRAME_LEN);
}

void sbus_init()
{
  sbusininv = !trkset.getSbInInv();
  sbusoutinv = !trkset.getSbOutInv();
  uint8_t inversion = 0;
  if (sbusininv) inversion |= CONFINV_RX;
  if (sbusoutinv) inversion |= CONFINV_TX;
  AuxSerial_Open(SERIAL_BAUDRATE, CONF8E2, inversion);
  sbusTreadRun = true;
}

// Build Channel Data

/* FROM -----
 * Brian R Taylor
 * brian.taylor@bolderflight.com
 *
 * Copyright (c) 2021 Bolder Flight Systems Inc
 */

void SBUS_TX_BuildData(uint16_t ch_[16])
{
  sbusBuildingData = true;
  uint8_t *buf_ = localTXBuffer;
  buf_[0] = ELRS_ADDRESS; 
  buf_[1] = ELRS_FRAME_LENGTH; // length of type (24) + payload + crc = 24
  buf_[2] = TYPE_CHANNELS;
  buf_[3] = static_cast<uint8_t>((ch_[0] & 0x07FF));
  buf_[4] = static_cast<uint8_t>((ch_[0] & 0x07FF) >> 8 | (ch_[1] & 0x07FF) << 3);
  buf_[5] = static_cast<uint8_t>((ch_[1] & 0x07FF) >> 5 | (ch_[2] & 0x07FF) << 6);
  buf_[6] = static_cast<uint8_t>((ch_[2] & 0x07FF) >> 2);
  buf_[7] = static_cast<uint8_t>((ch_[2] & 0x07FF) >> 10 | (ch_[3] & 0x07FF) << 1);
  buf_[8] = static_cast<uint8_t>((ch_[3] & 0x07FF) >> 7 | (ch_[4] & 0x07FF) << 4);
  buf_[9] = static_cast<uint8_t>((ch_[4] & 0x07FF) >> 4 | (ch_[5] & 0x07FF) << 7);
  buf_[10] = static_cast<uint8_t>((ch_[5] & 0x07FF) >> 1);
  buf_[11] = static_cast<uint8_t>((ch_[5] & 0x07FF) >> 9 | (ch_[6] & 0x07FF) << 2);
  buf_[12] = static_cast<uint8_t>((ch_[6] & 0x07FF) >> 6 | (ch_[7] & 0x07FF) << 5);
  buf_[13] = static_cast<uint8_t>((ch_[7] & 0x07FF) >> 3);
  buf_[14] = static_cast<uint8_t>((ch_[8] & 0x07FF));
  buf_[15] = static_cast<uint8_t>((ch_[8] & 0x07FF) >> 8 | (ch_[9] & 0x07FF) << 3);
  buf_[16] = static_cast<uint8_t>((ch_[9] & 0x07FF) >> 5 | (ch_[10] & 0x07FF) << 6);
  buf_[17] = static_cast<uint8_t>((ch_[10] & 0x07FF) >> 2);
  buf_[18] = static_cast<uint8_t>((ch_[10] & 0x07FF) >> 10 | (ch_[11] & 0x07FF) << 1);
  buf_[19] = static_cast<uint8_t>((ch_[11] & 0x07FF) >> 7 | (ch_[12] & 0x07FF) << 4);
  buf_[20] = static_cast<uint8_t>((ch_[12] & 0x07FF) >> 4 | (ch_[13] & 0x07FF) << 7);
  buf_[21] = static_cast<uint8_t>((ch_[13] & 0x07FF) >> 1);
  buf_[22] = static_cast<uint8_t>((ch_[13] & 0x07FF) >> 9 | (ch_[14] & 0x07FF) << 2);
  buf_[23] = static_cast<uint8_t>((ch_[14] & 0x07FF) >> 6 | (ch_[15] & 0x07FF) << 5);
  buf_[24] = static_cast<uint8_t>((ch_[15] & 0x07FF) >> 3);
  buf_[25] = crsf_crc8(&packet[2], packet[1] - 1); // CRC
  sbusBuildingData = false;
}
