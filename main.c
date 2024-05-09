#include "stm32.h"

#if 0
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
    // APB1 /2 prescaler!! Mustn't exceed 42MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_2; // 100: AHB clock divided by 2

    RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON; // 1: HSE oscillator bypassed with an external clock
    while ((RCC->CR & RCC_CR_HSERDY) == 0) {
        __asm__("nop");
    }

    // Quartz frequency is 16 MHz
    // (16MHz / PLLM) * PLLN / PLLP = (16MHz / 8) * 216 / 8 = 432MHz
    // "/M" Division factor : (2 <= PLLM <= 63).
    // "*N" multiplication factor for VCO ( 192 <= PLLN <= 432 ). VCO output frequency is between 192 and 432 MHz
    // "/P" division factor for main system clock. Value is 2, 4, 6 or 8. Frequency must not to exceed 120 MHz on this domain
    // "/Q" division factor for USB OTG FS, SDIO ( 2 <= PLLQ <= 15 )

#define QUARTZ_FREQ 8
#define PLLM  4
#define PLLQ  5
//#define PLLN ((48 * PLLQ) / (QUARTZ_FREQ / PLLM))
#define PLLN  120
#define PLLP  6
#define PLLPval (((PLLP) >> 1) - 1) // PLLP = 2, 4, 6, or 8. 00: PLLP=2, 01: PLLP=4, 10: PLLP=6, 11: PLLP=8

    uint32_t pllcfgr = RCC->PLLCFGR;
    pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE; // 1: HSE oscillator clock selected as PLL and PLLI2S clock entry
    pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLM) | (PLLM << 0);     // Bits 5:0   PLLM 
    pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLN) | (PLLN << 6);     // Bits 14:6  PLLN 
    pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLP) | (PLLPval << 16); // Bits 17:16 PLLP 
    pllcfgr = (pllcfgr & ~RCC_PLLCFGR_PLLQ) | (PLLQ << 24);    // Bits 27:24 PLLQ 
    RCC->PLLCFGR = pllcfgr;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
        __asm__("nop");
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_1; // 10: PLL selected as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_1) {
        __asm__("nop");
    }
}


static void delay_ms(int timeout_ms)
{
    for (int i = 0; i < (432 * 10) * 1000; ++i) {
        __asm__("nop");
    }
}

#define USB_OTG_DEVICE          ((USB_OTG_DeviceTypeDef *)      (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_OUT_ENDPOINT0   ((USB_OTG_OUTEndpointTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE))
#define USB_OTG_IN_ENDPOINT0    ((USB_OTG_INEndpointTypeDef *)  (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE))
#define USB_OTG_IN_PCGCCTL                        ((uint32_t *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))

void otg_detach()
{
	USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;    // 0: Power down activated
    //*USB_OTG_IN_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK); // FIXME
    // The DP pull-up resistor is removed by setting the soft disconnect bit in the device control register(SDIS bit in OTG_FS_DCTL)
    USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; //Table 134. Minimum duration for soft disconnect : 1ms + 2.5us
    delay_ms(10);

    //When no device is plugged in, the host will see both data lines low, as its 15 kohm resistors are pulling each data line low.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODE11 | GPIO_MODER_MODE12); // 00: Input (reset state)
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD11_1 | GPIO_PUPDR_PUPD12_1;// 10: Pull-down


    //*USB_OTG_IN_PCGCCTL |= USB_OTG_PCGCR_STPPCLK;
	//RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
}

void otg_initialization()
{
	
    RCC->AHB1ENR   |=  RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER   |=  GPIO_MODER_MODE11_1  | GPIO_MODER_MODE12_1; // 10: Alternate function mode; 01: General purpose output mode
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT11  | GPIO_OTYPER_OT12);     //  0: Output push-pull (reset state)
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDER_OSPEEDR11  | GPIO_OSPEEDER_OSPEEDR12);   //  11: 
    //GPIOA->PUPDR = GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD12_0;    // 01: Pull-up
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);    // 00: No pull-up, pull-down
    // AF10 (OTG_FS)
    GPIOA->AFR[1]  |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos); 

    //GPIOA->BSRR = GPIO_BSRR_BR4 | GPIO_BSRR_BR5;

    //GPIOA->OTYPER = ~(GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT5); //0: Output push-pull (reset state)
    //GPIOA->MODER = GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;// 01: General purpose output mode
    //GPIOA->BSRR = GPIO_BSRR_BR4 | GPIO_BSRR_BR5;
    //GPIOA->PUPDR = GPIO_PUPDR_PUPD4_1 | GPIO_PUPDR_PUPD5_1;// 10: Pull-down
    
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __asm__("nop"); //  Delay after an RCC peripheral clock enabling

    // Select FS Embedded PHY
    // This bit is always 1 with read-only access. 
    // USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    // Core soft reset
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) { // Indicates that the AHB master state machine is in the Idle condition.
        __asm__("nop");
    }

    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0) { 
        __asm__("nop");
    }

	// The software must also check that bit 31 in this register is set to 1 (AHB Master is Idle) before starting any operation
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) { // Indicates that the AHB master state machine is in the Idle condition.
        __asm__("nop");
    }

	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN;    // 1: Power down deactivated (“Transceiver active”)
    

	USB_OTG_FS->GUSBCFG  &= ~(USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_FHMOD);
	USB_OTG_FS->GUSBCFG  |=   USB_OTG_GUSBCFG_FDMOD;  // force OTG_FS core to work as a USB peripheral-only
	// When HNP or SRP is enabled the VBUS sensing pin (PA9) pin should be connected to VBUS. 
	// When HNP and SRP are both disabled, the VBUS sensing pin (PA9) should not be connected to VBUS.This pin can be can be used as GPIO.
	USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_HNPCAP;         // HNP capable bit (Host Negtiation Protocol)
	USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_SRPCAP;         // SRP capable bit 
	
    while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD) != 0) { // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode
        __asm__("nop");
    }

    {
        // Endpoint Init
    }

	// VBUS Sensing disable
    USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; //Table 134. Minimum duration for soft disconnect : 1ms + 2.5us
    delay_ms(10);
	USB_OTG_FS->GCCFG    |=   USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG    &= ~(USB_OTG_GCCFG_VBUSASEN | USB_OTG_GCCFG_VBUSBSEN);
    

	// PHY Clock restart
    *USB_OTG_IN_PCGCCTL = 0;

    USB_OTG_DEVICE->DCFG |=   0 << USB_OTG_DCFG_PFIVL_Pos;
    USB_OTG_DEVICE->DCFG |=   USB_OTG_DCFG_DSPD_Msk;  // 11: Full speed (USB 1.1 transceiver clock is 48 MHz)

    {
        // Flush FIFOs
    }
    

    //-------------------------------------------------------------------------------------
    
	uint32_t device_status = USB_OTG_DEVICE->DSTS;


	// 22.17 OTG_FS programming model. 22.17.1 Core initialization
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;           // Global interrupt mask bit GINTMSK = 1
    
    *USB_OTG_IN_PCGCCTL  &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK); //FIXME
    USB_OTG_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;

#if 0

	USB_OTG_FS->GINTSTS  = USB_OTG_GINTSTS_RXFLVL;         // RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_TXFELVL;        // Periodic TxFIFO empty level

	// When HNP or SRP is enabled the VBUS sensing pin (PA9) pin should be connected to VBUS. 
	// When HNP and SRP are both disabled, the VBUS sensing pin (PA9) should not be connected to VBUS.This pin can be can be used as GPIO.
	// USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_HNPCAP;         // HNP capable bit (Host Negtiation Protocol)
	// USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_SRPCAP;         // SRP capable bit 

    USB_OTG_FS->GUSBCFG |= 1 << USB_OTG_GUSBCFG_TRDT_Pos;  // USB turnaround time : https://www.usb.org/sites/default/files/Propagation_Delay_between_Host_Transcievers_and_Downstream_Ports.pdf
    USB_OTG_FS->GUSBCFG |= 1 << USB_OTG_GUSBCFG_TOCAL_Pos; // FS timeout calibration

#endif
	// OTG interrupt mask, Mode mismatch interrupt mask
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTMSK_MMISM;
	volatile uint8_t current_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD;   // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode 
	(void)current_mode;


	// 22.17.3 Device initialization
	//USB_OTG_DEVICE_BASE
	USB_OTG_DEVICE->DCFG &= ~USB_OTG_DCFG_NZLSOHSK;  // 0: Send the received OUT packet to the application (zero-length or nonzero-length)...

	// unmask the following interrupts: USB reset, Enumeration done, Early suspend, USB suspend, SOF
	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_ESUSPM | USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_SOFM;


	// Wait for the USBRST interrupt. A reset has been detected on the USB that lasts for about 10 ms
	while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) == 0) {
	}
    USB_OTG_FS->GINTSTS = (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST);

	// Wait for the end of reset on the USB
	while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) == 0) {
	}
    USB_OTG_FS->GINTSTS = (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE);

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
    USB_OTG_FS->GRXFSIZ = 16; //0x200; // 0x200 is Reset value
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
	//USB_OTG_IN_ENDPOINT0->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;         // USB active endpoint. Always is 1 for EP0
	USB_OTG_IN_ENDPOINT0->DIEPCTL |= USB_OTG_DIEPCTL_EPENA;            // Endpoint enable

	USB_OTG_OUT_ENDPOINT0->DOEPCTL  = (USB_OTG_OUT_ENDPOINT0->DOEPCTL & ~USB_OTG_DOEPCTL_EPTYP_Msk) | (0 << USB_OTG_DOEPCTL_EPTYP_Pos); // 00 Control type
	USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;           // Clear NAK. A write to this bit clears the NAK bit for the endpoint.
	USB_OTG_OUT_ENDPOINT0->DOEPCTL |= 8 << USB_OTG_DOEPCTL_MPSIZ_Pos; // Maximum packet size. This value is in bytes
	//USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;         // USB active endpoint. Always is 1 for EP0
	USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;          // Endpoint enable

	// On catching an RXFLVL interrupt (OTG_FS_GINTSTS register), the application must read the Receive status pop register(OTG_FS_GRXSTSP).
	while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == 0) {
	}

	while (1) {
		current_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode 

//    	USB_OTG_OUT_ENDPOINT0->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
//    	USB_OTG_OUT_ENDPOINT0->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

    	
#define USBx_DFIFO(i)   *(__IO uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))
    	volatile uint8_t __attribute__((aligned(32))) buf[64] ;
    	uint8_t *pDest = &buf[0];
    	uint32_t len = 8 + 4; // fifo header + packet
    	uint32_t count32b = (uint32_t)len >> 2U;
    	for (int i = 0U; i < len * sizeof(uint32_t); i += sizeof(uint32_t))
    	{
        	__UNALIGNED_UINT32_WRITE(&buf[i], USBx_DFIFO(0U));
    	}
    	//ep->xfer_count += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
    	USB_UNMASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
    	
    	(void)len++;
	}
}

int main(void)
{
	while(1)
	{
		clock_initialization();
    	otg_detach();
		otg_initialization();
	}
}
