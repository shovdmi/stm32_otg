#include "stm32.h"

void _init(void)
{
	return;
}

int main(void)
{
	while(1)
	{
		;
		RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;   // 1: HSE oscillator bypassed with an external clock
		while ((RCC->CR & RCC_CR_HSERDY) == 0) {
		}

		uint32_t pllcfgr = RCC->PLLCFGR;
		pllcfgr = pllcfgr | RCC_PLLCFGR_PLLSRC; // 1: HSE oscillator clock selected as PLL and PLLI2S clock entry
		// The software has to set these bits correctly to ensure that the VCO input frequency
		// ranges from 1 to 2 MHz. It is recommended to select a frequency of 2 MHz to limit
		// PLL jitter. (2 ≤ PLLM ≤ 63 )
		// Set VCO = 8 Mhz / M = 8 Mhz / 4 = 2 Mhz
		pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLM_Msk) | (4 << RCC_PLLCFGR_PLLM_Pos); // PLL "/M" Division factor

		// The software has to set these bits correctly to ensure that the VCO output frequency is between 192 and 432 MHz
		// Set VCO Output frequency = 2 MHz * N = 2 Mhz * 216 = 432 Mhz  ( 192 ≤ PLLN ≤ 432 )
		pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLN_Msk) | (216 << RCC_PLLCFGR_PLLN_Pos); // PLL "xN" multiplication factor for VCO

		// The software has to set these bits correctly not to exceed 84 MHz on this domain.
		// PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8 (00: PLLP=2, 01: PLLP=4, 10: PLLP=6, 11: PLLP=8)
		// Set System Clock = 432 Mhz / P = 432 MHz / 8 = 48 MHz
		pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLP_Msk) | (RCC_PLLCFGR_PLLP_Msk << RCC_PLLCFGR_PLLP_Pos); // PLL "/P" division factor for main system clock

		// Set PLL48CK = 432 MHz / Q = 432 / 9 = 48Mhz ( 2 ≤ PLLQ ≤ 15 )
		pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLQ_Msk) | (9 << RCC_PLLCFGR_PLLQ_Pos); // PLL "/Q" division factor for USB OTG FS, SDIO

		RCC->PLLCFGR = pllcfgr;

		RCC->CR |= RCC_CR_PLLON;
		while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
		}


		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_1; // 10: PLL selected as system clock
		while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_1) {
		}

		// 84MHz * P : 84 * 1 = 84; 84 * 2 = 168; 84 * 3 = 252; 84 * 4 = 332 MHz
		// PLL48CK * Q   |  /N               |  
		// 48 * 2 =  96  |  96 / 48  = 2 MHz | 
		// 48 * 3 = 132  | 132 / 66          |
		// 48 * 4 = 192  | 192 / 96          |
		// 48 * 5 = 240  | 240 / 120         |
		// 48 * 6 = 288  | 288 / 144         |
		// 48 * 7 = 336  | 336 / 168         |
		// 48 * 8 = 384  | 384 / 192         |
		// 48 * 9 = 432  | 432 / 216         | 432 / 8 = 42 MHz

		RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
		USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;   // force OTG_FS core to work as a USB peripheral-only

		// 22.17 OTG_FS programming model. 22.17.1 Core initialization
		USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;    // Global interrupt mask bit GINTMSK = 1
		USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_RXFLVL;  // RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
		USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_TXFELVL; // Periodic TxFIFO empty level

		USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_HNPCAP;  // HNP capable bit (Host Negtiation Protocol)
		USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_SRPCAP;  // SRP capable bit 
		USB_OTG_FS->GUSBCFG |= 1 << USB_OTG_GUSBCFG_TRDT_Pos;  // FS timeout calibration
		USB_OTG_FS->GUSBCFG |= 1 << USB_OTG_GUSBCFG_TOCAL_Pos; // USB turnaround time

		// 22.17.3 Device initialization
#define USB_OTG_DEVICE          ((USB_OTG_DeviceTypeDef *) USB_OTG_DEVICE_BASE)
		//USB_OTG_DEVICE_BASE
		USB_OTG_DEVICE->DCFG |=  USB_OTG_DCFG_DSPD_Msk; // 11: Full speed (USB 1.1 transceiver clock is 48 MHz)
		USB_OTG_DEVICE->DCFG &= ~USB_OTG_DCFG_NZLSOHSK; // 0: Send the received OUT packet to the application (zero-length or nonzero-length)...

		USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSASEN;

		// Wait for the USBRST interrupt. A reset has been detected on the USB that lasts for about 10 ms
		while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBSUSP) == 0) {
		}

		// Wait for the end of reset on the USB
		while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) == 0) {
		}
		//Read the OTG_FS_DSTS to determine the enumeration speed

		// 22.17.5 Device programming model
		
	}
}
