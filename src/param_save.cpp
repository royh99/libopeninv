/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/crc.h>
#include "params.h"
#include "param_save.h"
#include "hwdefs.h"
#include "my_string.h"

#define NUM_PARAMS ((PARAM_BLKSIZE - 8) / sizeof(PARAM_ENTRY))
#define PARAM_WORDS (PARAM_BLKSIZE / 4)
#define PARAM_DOUBLEWORDS (PARAM_WORDS / 2)                                       

typedef struct
{
   uint16_t key;
   uint8_t dummy;
   uint8_t flags;
   uint32_t value;
} PARAM_ENTRY;

typedef struct
{
   PARAM_ENTRY data[NUM_PARAMS];
   uint64_t crc;
   //uint32_t padding;
} PARAM_PAGE;

static uint32_t GetFlashAddress()
{
	return FLASH_CONF_BASE + PARAM_BLKOFFSET;
}

/**
* Save parameters to flash
*
* @return CRC of parameter flash page
*/
uint32_t parm_save()
{
   PARAM_PAGE parmPage;
   uint32_t idx;
   uint32_t paramAddress = GetFlashAddress();
   uint8_t flashpage = (paramAddress - FLASH_BASE) / FLASH_PAGE_SIZE;

   crc_reset();
   memset32((int*)&parmPage, 0xFFFFFFFF, PARAM_WORDS);

   // Copy parameter values and keys to block structure
   for (idx = 0; idx < NUM_PARAMS && idx < Param::PARAM_LAST; idx++)
   {
      if (Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM)
      {
         const Param::Attributes *pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
         parmPage.data[idx].flags = (uint8_t)Param::GetFlag((Param::PARAM_NUM)idx);
         parmPage.data[idx].key = pAtr->id;
         parmPage.data[idx].value = Param::Get((Param::PARAM_NUM)idx);
      }
   }

   // Calculate CRC over the data portion of the structure, excluding the CRC
   uint32_t* dataStart = (uint32_t*)&parmPage;
   uint32_t dataSize = (sizeof(PARAM_PAGE) - sizeof(parmPage.crc)) / sizeof(uint32_t);
   parmPage.crc = (uint64_t)crc_calculate_block(dataStart, dataSize);

   flash_unlock();

   // Always erase the page
   flash_clear_status_flags();
   flash_erase_page(flashpage);

   // Program the flash page
   for (idx = 0; idx < PARAM_DOUBLEWORDS; idx++)
   {
      uint64_t* pData = ((uint64_t*)&parmPage) + idx;
      flash_clear_status_flags();
      flash_program_double_word(paramAddress + idx * sizeof(uint64_t), *pData);
   }

   flash_lock();
   return parmPage.crc;
}


/**
* Load parameters from flash
*
* @retval 0 Parameters loaded successfully
* @retval -1 CRC error, parameters not loaded
*/
int parm_load()
{
   uint32_t paramAddress = GetFlashAddress();
 
   const PARAM_PAGE *parmPage = (const PARAM_PAGE *)paramAddress;

   // Use a 64-bit aligned buffer for reading flash data
   uint64_t buffer[sizeof(PARAM_PAGE) / sizeof(uint64_t)];
   const uint64_t* source = (const uint64_t*)parmPage;
   uint64_t* dest = buffer;

   // Read flash data into the buffer
   for (uint32_t i = 0; i < sizeof(PARAM_PAGE) / sizeof(uint64_t); i++)
   {
      *dest++ = *source++;
   }

   // Calculate CRC over the data portion of the buffer, excluding the CRC
   uint32_t* dataStart = (uint32_t*)buffer;
   uint32_t dataSize = 2 * NUM_PARAMS;
   crc_reset();
   uint32_t crc = crc_calculate_block(dataStart, dataSize);

   // Compare the calculated CRC with the stored CRC
   if (crc == (uint32_t)parmPage->crc)
   {
      // Load parameters if the CRC matches
      for (unsigned int idxPage = 0; idxPage < NUM_PARAMS; idxPage++)
      {
         Param::PARAM_NUM idx = Param::NumFromId(parmPage->data[idxPage].key);
         if (idx != Param::PARAM_INVALID && Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM)
         {
            Param::SetFixed(idx, parmPage->data[idxPage].value);
            Param::SetFlagsRaw(idx, parmPage->data[idxPage].flags);
         }
      }
      return 0; // Success
   }
   
   return -1; // CRC error
}
