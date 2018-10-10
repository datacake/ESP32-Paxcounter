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

#ifndef PACKETEVENT_H
#define PACKETEVENT_H

#include <stdint.h>
#include <sys/time.h>

#define PACKET_EVENT_PACKED_SIZE    16

// ---- Class definition -------------------------------------------------------

struct PacketEvent
{
    timeval  timestamp;
    uint8_t   mac[6];
    int8_t    rssi;
    uint8_t   channel;

    bool fromBuffer( const uint8_t buffer[] );
    size_t toBuffer( uint8_t buffer[], int max_length ) const;
};

#endif

