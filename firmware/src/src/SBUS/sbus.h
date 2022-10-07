/*
 * This file is part of the Head Tracker distribution (https://github.com/dlktdr/headtracker)
 * Copyright (c) 2021 Cliff Blackburn
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

#pragma once

#include <stdint.h>

#include "defines.h"

void SBUS_TX_BuildData(uint16_t ch_[16]);
void SBUS_TX_Start();
void sbus_init();
void sbus_Thread();
bool SBUS_Read_Data(uint16_t ch_[16]);
void SBUSIn_SetInverted(bool sbusininv);

// ELRS command
#define ELRS_ADDRESS                    0xEE
#define ELRS_BIND_COMMAND               0xFF
#define ELRS_WIFI_COMMAND               0xFE
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TLM_RATIO_COMMAND          0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x04
#define ELRS_POWER_COMMAND              0x06
#define ELRS_BLE_JOYSTIC_COMMAND        17
#define TYPE_SETTINGS_WRITE             0x2D
#define ADDR_RADIO                      0xEA //  Radio Transmitter
#define ELRS_FRAME_LENGTH               24 // length of type + payload + crc
#define TYPE_CHANNELS                   0x16

// Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MID 992
#define CRSF_DIGITAL_CHANNEL_MAX 1811