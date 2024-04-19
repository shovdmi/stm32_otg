#include "stm32.h"

#if 1
void _init(void)
{
	return;
}
#else
void SystemInit()
{
}
#endif

void clock_initialization()
{
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
	// PLL48CK * Q   |  /N			   |  
	// 48 * 2 =  96  |  96 / 48  = 2 MHz | 
	// 48 * 3 = 132  | 132 / 66		  |
	// 48 * 4 = 192  | 192 / 96		  |
	// 48 * 5 = 240  | 240 / 120		 |
	// 48 * 6 = 288  | 288 / 144		 |
	// 48 * 7 = 336  | 336 / 168		 |
	// 48 * 8 = 384  | 384 / 192		 |
	// 48 * 9 = 432  | 432 / 216		 | 432 / 8 = 42 MHz

}

void otg_initialization()
{
#define USB_OTG_DEVICE          ((USB_OTG_DeviceTypeDef *)      (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_OUT_ENDPOINT0   ((USB_OTG_OUTEndpointTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE))
#define USB_OTG_IN_ENDPOINT0    ((USB_OTG_INEndpointTypeDef *)  (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE))

	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

	uint32_t device_status = USB_OTG_DEVICE->DSTS;

	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;          // force OTG_FS core to work as a USB peripheral-only

	// 22.17 OTG_FS programming model. 22.17.1 Core initialization
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;           // Global interrupt mask bit GINTMSK = 1
	USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_RXFLVL;         // RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_TXFELVL;        // Periodic TxFIFO empty level

	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_HNPCAP;         // HNP capable bit (Host Negtiation Protocol)
	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_SRPCAP;         // SRP capable bit 
	USB_OTG_FS->GUSBCFG |= 1 << USB_OTG_GUSBCFG_TRDT_Pos;  // FS timeout calibration
	USB_OTG_FS->GUSBCFG |= 1 << USB_OTG_GUSBCFG_TOCAL_Pos; // USB turnaround time

	// OTG interrupt mask, Mode mismatch interrupt mask
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTMSK_MMISM;
	volatile uint8_t current_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD;   // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode 
	(void)current_mode;


	// 22.17.3 Device initialization
	//USB_OTG_DEVICE_BASE
	USB_OTG_DEVICE->DCFG |=  USB_OTG_DCFG_DSPD_Msk;  // 11: Full speed (USB 1.1 transceiver clock is 48 MHz)
	USB_OTG_DEVICE->DCFG &= ~USB_OTG_DCFG_NZLSOHSK;  // 0: Send the received OUT packet to the application (zero-length or nonzero-length)...

	// unmask the following interrupts: USB reset, Enumeration done, Early suspend, USB suspend, SOF
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_ESUSPM | USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_SOFM;

	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
	//USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;

	// Wait for the USBRST interrupt. A reset has been detected on the USB that lasts for about 10 ms
	while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) == 0) {
	}

	// Wait for the end of reset on the USB
	while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) == 0) {
	}

	//Read the OTG_FS_DSTS to determine the enumeration speed
	current_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode 
	uint32_t device_status2 = USB_OTG_DEVICE->DSTS;
	(void) device_status;
	(void) device_status2;


	// 22.17.5 Device programming model
	// 1.  Set the NAK bit for all OUT endpoints. SNAK = 1 in OTG_FS_DOEPCTLx(for all OUT endpoints)
	// FIXME--> USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;

	// 2. Unmask the following interrupt bits –
	// INEP0 = 1, OUTEP0 = 1 in OTG_FS_DAINTMSK(control 0 IN/OUT endpoint), 
	USB_OTG_DEVICE->DAINTMSK |= (1 << USB_OTG_DAINTMSK_OEPM_Pos) | (1 << USB_OTG_DAINTMSK_IEPM_Pos); // EP0
	// STUP = 1, XFRC = 1 in DOEPMSK; 
	USB_OTG_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
	// XFRC = 1, TOC = 1 in DIEPMSK
	USB_OTG_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM;
	// 3.  Set up the Data FIFO RAM for each of the FIFOs. Program the OTG_FS_GRXFSIZ register
	//RXFD: RxFIFO depth This value is in terms of 32 - bit words.  Minimum value is 16, Maximum value is 256
	USB_OTG_FS->GRXFSIZ = 0x200; // 0x200 is Reset value
	//Program the OTG_FS_TX0FSIZ register (depending on the FIFO number chosen) to be able to transmit control IN data.
	//At a minimum, this must be equal to 1 max packet size of control endpoint 0
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = 0x200; // 0x200 is Reset value. Bits 31:16 TX0FD: Endpoint 0 TxFIFO depth This value is in terms of 32 - bit words

	//
	// Endpoint activation
	//
	// Program the following fields in the endpoint-specific registers for control OUT endpoint 0 to receive a SETUP packet, STUPCNT = 3 in OTG_FS_DOEPTSIZ0(to receive up to 3 back - to - back SETUP packets)
	USB_OTG_OUT_ENDPOINT0->DOEPTSIZ |= 3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos;

	USB_OTG_IN_ENDPOINT0->DIEPCTL  = (USB_OTG_IN_ENDPOINT0->DIEPCTL & ~USB_OTG_DIEPCTL_EPTYP_Msk) | (0 << USB_OTG_DIEPCTL_EPTYP_Pos); // 00 Control type
	// The maximum packet size for a control endpoint depends on the enumeration speed.
	USB_OTG_IN_ENDPOINT0->DIEPCTL |= 3 << USB_OTG_DIEPCTL_MPSIZ_Pos; // 00: 64 bytes, 01: 32 bytes, 10: 16 bytes, 11: 8 bytes
	USB_OTG_IN_ENDPOINT0->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;         // USB active endpoint

	USB_OTG_OUT_ENDPOINT0->DOEPCTL  = (USB_OTG_OUT_ENDPOINT0->DOEPCTL & ~USB_OTG_DOEPCTL_EPTYP_Msk) | (0 << USB_OTG_DOEPCTL_EPTYP_Pos); // 00 Control type
	USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;           // Clear NAK. A write to this bit clears the NAK bit for the endpoint.
	USB_OTG_OUT_ENDPOINT0->DOEPCTL |= 8 << USB_OTG_DOEPCTL_MPSIZ_Pos; // Maximum packet size. This value is in bytes
	USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;         // USB active endpoint

	// On catching an RXFLVL interrupt (OTG_FS_GINTSTS register), the application must read the Receive status pop register(OTG_FS_GRXSTSP).
	while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == 0) {
	}

	while (1) {
		current_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode 
	}
}

int main(void)
{
	while(1)
	{
		clock_initialization();
		otg_initialization();
	}
}
