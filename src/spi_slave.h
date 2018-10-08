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

#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include "globals.h"

// ---- Function definition ----------------------------------------------------

void spi_slave_init( void );
void spi_slave_task(void *pvParameters);
void spi_slave_send( MessageBuffer_t* message );

#endif
