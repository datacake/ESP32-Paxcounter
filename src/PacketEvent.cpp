/**
 * Copyright 2018 Datacake GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "PacketEvent.h"

// ---- Local defines ----------------------------------------------------------

// ---- Local Functions --------------------------------------------------------

// ---- Public Functions -------------------------------------------------------

bool PacketEvent::fromBuffer( const uint8_t buffer[] )
{
    this->timestamp.tv_sec  =   buffer[0] << 24 |
                                buffer[1] << 16 |
                                buffer[2] << 8 |
                                buffer[3];
    this->timestamp.tv_usec =   buffer[4] << 24 |
                                buffer[5] << 16 |
                                buffer[6] << 8 |
                                buffer[7];
    this->mac[0] = buffer[8];
    this->mac[1] = buffer[9];
    this->mac[2] = buffer[10];
    this->mac[3] = buffer[11];
    this->mac[4] = buffer[12];
    this->mac[5] = buffer[13];
    this->rssi = buffer[14];
    this->channel = buffer[15];

    return true;
}

size_t PacketEvent::toBuffer( uint8_t buffer[], int max_length ) const
{
    if( max_length < PACKET_EVENT_PACKED_SIZE )
    {
        return 0;
    }

    buffer[0] = this->timestamp.tv_sec >> 24;
    buffer[1] = this->timestamp.tv_sec >> 16;
    buffer[2] = this->timestamp.tv_sec >> 8;
    buffer[3] = this->timestamp.tv_sec;
    buffer[4] = this->timestamp.tv_usec >> 24;
    buffer[5] = this->timestamp.tv_usec >> 16;
    buffer[6] = this->timestamp.tv_usec >> 8;
    buffer[7] = this->timestamp.tv_usec;
    buffer[8] = this->mac[0];
    buffer[9] = this->mac[1];
    buffer[10] = this->mac[2];
    buffer[11] = this->mac[3];
    buffer[12] = this->mac[4];
    buffer[13] = this->mac[5];
    buffer[14] = this->rssi;
    buffer[15] = this->channel;

    return PACKET_EVENT_PACKED_SIZE;
}
