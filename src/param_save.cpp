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
    uint32_t crc;
    uint32_t padding = 0;
} PARAM_PAGE;

static uint32_t GetFlashAddress()
{
    return FLASH_CONF_BASE; // 0x8004000  //+ PARAM_BLKOFFSET;
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
    uint8_t flashpage = (paramAddress - FLASH_BASE) / FLASH_PAGE_SIZE; // page 8, 0x8004000
    
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
    parmPage.crc = crc_calculate_block(((uint32_t*)&parmPage) , (2 * NUM_PARAMS));
    
    flash_unlock();
    
    // Always erase the flash page
    flash_clear_status_flags();
    flash_erase_page(flashpage);
    
    // Program the flash page
    flash_program(paramAddress, (uint8_t*)&parmPage, PARAM_BLKSIZE); // length is in bytes (multiple of 8)
    
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
    const uint64_t* flash_data = (const uint64_t*)parmPage;
    
    crc_reset();
    
	for (uint8_t i = 0; i < NUM_PARAMS; i++)   // 
	{
		uint64_t data64 = flash_data[i]; // read 64 bit data from flash
		CRC_DR = (uint32_t)data64;
		CRC_DR = (uint32_t)(data64>>32);
    }
    uint32_t crc = CRC_DR;
    
    // Compare the calculated CRC with the stored CRC
    if (crc == parmPage->crc)
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

