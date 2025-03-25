/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2016 Nail Güzel
 * Johannes Huebner <dev@johanneshuebner.com>
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
#include <stdint.h>
#include <cstddef>
#include "hwdefs.h"
#include "my_math.h"
#include "printf.h"
#include <libopencm3/stm32/fdcan.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "stm32_can.h"

#define MAX_INTERFACES        3
#define IDS_PER_BANK          2
#define EXT_IDS_PER_BANK      2


struct CANSPEED
{
   uint32_t ts1;
   uint32_t ts2;
   uint32_t prescaler;
};

Stm32Can* Stm32Can::interfaces[MAX_INTERFACES];

static const CANSPEED canSpeed[CanHardware::BaudLast] =
{  //ts1-1, ts2-1, prescale-1
	{ 11, 2, 23 }, //125kbps
	{ 11, 2, 11 }, //250kbps
	{ 11, 2, 5  }, //500kbps 48/6/( 1+12+3 ) = 0.5MHz
	{ 10, 2, 3  }, //800kbps 48/4/( 1+11+3 ) = 0.8MHz
	{ 11, 2, 2  }, //1000kbps
};

/** \brief Init can hardware with given baud rate
 * Initializes the following sub systems:
 * - CAN hardware itself
 * - Appropriate GPIO pins
 * - Enables appropriate interrupts in NVIC
 *
 * \param baseAddr base address of CAN peripheral, CAN1 or CAN2
 * \param baudrate enum baudrates
 * \param remap use remapped IO pins
 * \return void
 *
 */
Stm32Can::Stm32Can(uint32_t baseAddr, enum baudrates baudrate, bool remap)
   : sendCnt(0), canDev(baseAddr)
{
   switch (baseAddr)
   {
      case CAN1:
         if (remap)
         {
			gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
            gpio_set_af(GPIOB, GPIO_AF9, GPIO8 | GPIO9);
		}
		else
		{
            gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
            gpio_set_af(GPIOA, GPIO_AF9, GPIO11 | GPIO12);
		}

		//CAN1 RX and TX IRQs
	    nvic_enable_irq(NVIC_FDCAN1_IT0_IRQ); // CAN RX ints FIFO0 and FIFO1	
		nvic_set_priority(NVIC_FDCAN1_IT0_IRQ, 0xf << 4); //lowest priority		
        nvic_enable_irq(NVIC_FDCAN1_IT1_IRQ); // CAN TX  ints
	    nvic_set_priority(NVIC_FDCAN1_IT1_IRQ, 0xf << 4); //lowest priority
		interfaces[0] = this;
		break;
		case CAN2:
		if (remap)
		{
			gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13);
			gpio_set_af(GPIOB, GPIO_AF9, GPIO12 | GPIO13);
		}
		else
		{
			// Configure CAN pins
			gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13);
			gpio_set_af(GPIOB, GPIO_AF9, GPIO12 | GPIO13);
		}
		//CAN2 RX and TX IRQs
		nvic_enable_irq(NVIC_FDCAN2_IT0_IRQ); // CAN RX
		nvic_set_priority(NVIC_FDCAN2_IT0_IRQ, 0xf << 4); //lowest priority
		nvic_enable_irq(NVIC_FDCAN2_IT1_IRQ); //CAN TX
		nvic_set_priority(NVIC_FDCAN2_IT1_IRQ, 0xf << 4); //lowest priority
		interfaces[1] = this;
		break;	 
		case CAN3:
		// Configure CAN pins	
		gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4);
		gpio_set_af(GPIOB, GPIO_AF11, GPIO3 | GPIO4);
		
		//CAN3 RX and TX IRQs
		nvic_enable_irq(NVIC_FDCAN3_IT0_IRQ); //CAN RX
		nvic_set_priority(NVIC_FDCAN3_IT0_IRQ, 0xf << 4); //lowest priority
        nvic_enable_irq(NVIC_FDCAN3_IT1_IRQ); //CAN TX
		nvic_set_priority(NVIC_FDCAN3_IT1_IRQ, 0xf << 4); //lowest priority
		interfaces[2] = this;
		break;
	}
	FDCAN_ILS(canDev) &= ~(FDCAN_ILS_RxFIFO0| FDCAN_ILS_RxFIFO1); // select sources for int0 RX 
			 
	FDCAN_ILS(canDev) |= FDCAN_ILS_SMSG| FDCAN_ILS_TFERR; // select sources for int1 ( TX )	
	FDCAN_IE(canDev)  |= FDCAN_IE_RF0NE| FDCAN_IE_RF1NE; // enable interupt source for rx (int0)	new message
	FDCAN_IE(canDev)  |= FDCAN_IE_TFEE; // enable interupt source for tx fifo empty /int1
	
	SetBaudrate(baudrate);
	fdcan_init_filter(canDev, 28, 8); // set no. of filter ids, max  28 std, 8 ext
	ConfigureFilters();
	
	fdcan_start(canDev, 100);
	// Enable CAN RX interrupts.
	fdcan_enable_irq(canDev, FDCAN_ILE_INT0);

}

/** \brief Set baud rate to given value
 *
 * \param baudrate enum baudrates
 * \return void
 *
 */
void Stm32Can::SetBaudrate(enum baudrates baudrate)
{
	// CAN controller init.
	// Setting the bitrate to 500KBit. pllq = 48MHz,
	// prescaler = 6 -> 8MHz time quanta frequency.
	// 1tq sync + 12tq bit segment1 (TS1) + 3tq bit segment2 (TS2) =
	// 16time quanto per bit period, therefor 8MHz/16 = 500kHz
	//
	fdcan_set_can(canDev,
		false,          // auto_retry_disable?
		false,          // rx_fifo_locked?
		false,          // tx_queue_mode, false = use fifo
		false,          // silent?
		0,  	        // n_sjw-1
		canSpeed[baudrate].ts1,  	// n_ts1-1
		canSpeed[baudrate].ts2,  	// n_ts2-1
		canSpeed[baudrate].prescaler); // n_br_presc-1 : Baud rate prescaler
}

/** \brief Send a user defined CAN message
 *
 * \param canId uint32_t
 * \param data[2] uint32_t
 * \param len message length
 * \return void
 *
 */
void Stm32Can::Send(uint32_t canId, uint32_t data[2], uint8_t len)
{
   fdcan_disable_irq(canDev, FDCAN_ILE_INT1); // disable TX ints

   if (fdcan_transmit(canDev, canId, canId > 0x7FF, false, false, false, len, (uint8_t*)data) < 0 && sendCnt < SENDBUFFER_LEN)
   {
      /* enqueue in send buffer if all TX mailboxes are full */
      sendBuffer[sendCnt].id = canId;
      sendBuffer[sendCnt].len = len;
      sendBuffer[sendCnt].data[0] = data[0];
      sendBuffer[sendCnt].data[1] = data[1];
      sendCnt++;
   }

   if (sendCnt > 0)
   {
      fdcan_enable_irq(canDev, FDCAN_ILE_INT1); //enable TX ints
   }
}


Stm32Can* Stm32Can::GetInterface(int index)
{
   if (index < MAX_INTERFACES)
   {
      return interfaces[index];
   }
   return 0;
}

void Stm32Can::HandleMessage() // fifo not used. Checked in interupt flag register
{
   uint32_t id;
   uint8_t fifo = 0;
   bool ext, rtr;
   uint8_t length, fmi;
   uint32_t data[2];
	
   //if (FDCAN_IR(canDev) & (FDCAN_IR_RF0N)) fifo = 0;
   if (FDCAN_IR(canDev) & (FDCAN_IR_RF1N)) fifo = 1;
 
   while (fdcan_available_rx(canDev, fifo)) 
   {
	  fdcan_receive(canDev, fifo, true, &id, &ext, &rtr, &fmi, &length, (uint8_t*)data, NULL);
      HandleRx(id, data, length);
      lastRxTimestamp = systick_get_value();
   }
   FDCAN_IR(canDev) |= ( FDCAN_IR_RF0N | FDCAN_IR_RF1N); // set flags to "1" to clear them!
}

void Stm32Can::HandleTx()
{
   SENDBUFFER* b = sendBuffer; //alias

	while (sendCnt > 0 && fdcan_transmit(canDev, b[sendCnt - 1].id, b[sendCnt - 1].id > 0x7FF, false, false, false, b[sendCnt - 1].len, (uint8_t*)b[sendCnt - 1].data) >= 0)
	sendCnt--;
	
	if (sendCnt == 0)
	{
		fdcan_disable_irq(canDev, FDCAN_ILE_INT1);
	}
}

/****************** Private methods and ISRs ********************/

void Stm32Can::SetFilterBank(int& idIndex, uint32_t& filterId, uint32_t* idList)
{
	fdcan_set_std_filter(
		canDev,
		filterId,
		FDCAN_SFT_DUAL,   // id_list_mode, FDCAN_SFT_ID_MASK, FDCAN_SFT_RANGE, FDCAN_SFT_DUAL
		idList[0],
		idList[1],
		(filterId & 1) ? FDCAN_SFEC_FIFO1 : FDCAN_SFEC_FIFO0); // even FIFO0 , odd FIFO1	select/enable 	( 0 = disable )
	idIndex = 0;
	filterId++;
	idList[0] = idList[1] = 0;
}

void Stm32Can::SetFilterBankMask(int& idIndex, uint32_t& filterId, uint32_t* idMaskList)
{
	fdcan_set_std_filter(
		canDev,
		filterId,
		FDCAN_SFT_MASK,
		idMaskList[0], //id
		idMaskList[1], //mask
		(filterId & 1) ? FDCAN_SFEC_FIFO1 : FDCAN_SFEC_FIFO0);
   idIndex = 0;
   filterId++;
   idMaskList[0] = 0;
   idMaskList[1] = 0x7FF;
}

void Stm32Can::SetFilterBank29(int& idIndex, uint32_t& filterId, uint32_t* idList)
{
	fdcan_set_ext_filter(
        canDev,
        filterId,
		FDCAN_EFT_DUAL,	// match ID1 or ID2
        idList[0],
		idList[1],
   (filterId & 1) ? FDCAN_SFEC_FIFO1 : FDCAN_SFEC_FIFO0); // even FIFO0 , odd FIFO1 select/enable
	idIndex = 0;
	filterId++;
	idList[0] = idList[1] = 0;
}

void Stm32Can::ConfigureFilters()
{
	uint32_t idList[IDS_PER_BANK] = { 0, 0 };
	uint32_t idMaskList[IDS_PER_BANK] = { 0, 0x7FF };
	uint32_t extIdList[EXT_IDS_PER_BANK] = { 0, 0 };
	int idIndex = 0, idMaskIndex = 0, extIdIndex = 0;
	// G4 has 28 filter ids per CAN instance.
	uint32_t filterId = 0;
	uint32_t extfilterId = 0;

   for (int i = 0; i < nextUserMessageIndex; i++)
   {
      if (userIds[i] > 0x7ff)
      {
         extIdList[extIdIndex] = userIds[i] & 0x1FFFFFFF;
         extIdIndex++;
      }
      else if (userMasks[i] != 0)
      {
         idMaskList[idMaskIndex++] = userIds[i];
         idMaskList[idMaskIndex++] = userMasks[i];
      }
      else
      {
         idList[idIndex] = userIds[i];
         idIndex++;
      }

      if (idIndex == IDS_PER_BANK)
      {
         SetFilterBank(idIndex, filterId, idList);
      }
      if (idMaskIndex == EXT_IDS_PER_BANK)
      {
         SetFilterBankMask(idMaskIndex, filterId, idMaskList);
      }
      if (extIdIndex == EXT_IDS_PER_BANK)
      {
         SetFilterBank29(extIdIndex, extfilterId, extIdList);
      }
   }

   //loop terminates before adding last set of filters
   if (idIndex > 0)
   {
      SetFilterBank(idIndex, filterId, idList);
   }
   if (idMaskIndex > 0)
   {
      SetFilterBankMask(extIdIndex, filterId, idMaskList);
   }
   if (extIdIndex > 0)
   {
      SetFilterBank29(extIdIndex, extfilterId, extIdList);
   }
}

/* Interrupt service routines */
extern "C" void fdcan1_it0_isr(void)
{
	Stm32Can::GetInterface(0)->HandleMessage();
}

extern "C" void fdcan1_it1_isr()
{
	Stm32Can::GetInterface(0)->HandleTx();
}

extern "C" void fdcan2_it0_isr()
{
	Stm32Can::GetInterface(1)->HandleMessage();
}

extern "C" void fdcan2_it1_isr()
{
	Stm32Can::GetInterface(1)->HandleTx();
}

extern "C" void fdcan3_it0_isr()
{
	Stm32Can::GetInterface(2)->HandleMessage();
}

extern "C" void fdcan3_it1_isr()
{
	Stm32Can::GetInterface(2)->HandleTx();
}
