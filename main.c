#include "stm32.h"
#include <stddef.h>

#define MIN(x,y) ((x) < (y) ? (x) : (y))

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

uint8_t usb_addr = 0;
volatile uint8_t usb_addr_old = 0;

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

void otg_hw_reset()
{
	// PHY Clock restart
    // see CSRST: Core soft reset. When you change the PHY, the corresponding clock for the PHY is selected and used in the
    // PHY domain.Once a new clock is selected, the PHY domain has to be reset for proper operation.
    *USB_OTG_IN_PCGCCTL = 0;

    // Select FS Embedded PHY
    // This bit is always 1 with read-only access. 
    // USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    // Core soft reset
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) { // Indicates that the AHB master state machine is in the Idle condition.
        __asm__("nop");
    }

    // All module state machines(except for the AHB slave unit) are reset to the Idle state, and 
    // all the transmit FIFOs and the receive FIFO are flushed
    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0) { 
        __asm__("nop");
    }

	// The software must also check that bit 31 in this register is set to 1 (AHB Master is Idle) before starting any operation
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) { // Indicates that the AHB master state machine is in the Idle condition.
        __asm__("nop");
    }

    // HCLK soft reset. The application uses this bit to flush the control logic in the AHB Clock domain.Only AHB Clock Domain pipelines are reset.
    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_HSRST;
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_HSRST) != 0) { 
        __asm__("nop");
    }

    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) { // Indicates that the AHB master state machine is in the Idle condition.
        __asm__("nop");
    }

}

void otg_initialization()
{
    RCC->AHB1ENR   |=  RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER   |=  GPIO_MODER_MODE11_1  | GPIO_MODER_MODE12_1; // 10: Alternate function mode; 01: General purpose output mode
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT11  | GPIO_OTYPER_OT12);     //  0: Output push-pull (reset state)
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDER_OSPEEDR11  | GPIO_OSPEEDER_OSPEEDR12);   //  11: 
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);    // 00: No pull-up, pull-down
    GPIOA->AFR[1]  |=  (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos); // AF10 (OTG_FS)

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    __asm__("nop"); //  Delay after an RCC peripheral clock enabling

    otg_hw_reset();

	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN;    // 1: Power down deactivated (“Transceiver active”)


	USB_OTG_FS->GUSBCFG  &= ~(USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_FHMOD);
	USB_OTG_FS->GUSBCFG  |=   USB_OTG_GUSBCFG_FDMOD;  // force OTG_FS core to work as a USB peripheral-only
	// When HNP or SRP is enabled the VBUS sensing pin (PA9) pin should be connected to VBUS. 
	// When HNP and SRP are both disabled, the VBUS sensing pin (PA9) should not be connected to VBUS.This pin can be can be used as GPIO.
	USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_HNPCAP;   // HNP capable bit (Host Negtiation Protocol)
	USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_SRPCAP;   // SRP capable bit 
	
    while ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD) != 0) { // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode
        __asm__("nop");
    }

    // ---------
    {
        // Endpoint Init
    }

	// VBUS Sensing disable
    USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; //Table 134. Minimum duration for soft disconnect : 1ms + 2.5us
    delay_ms(10);
	USB_OTG_FS->GCCFG    |=   USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG    &= ~(USB_OTG_GCCFG_VBUSASEN | USB_OTG_GCCFG_VBUSBSEN);
    

    USB_OTG_DEVICE->DCFG |=   0 << USB_OTG_DCFG_PFIVL_Pos;
    USB_OTG_DEVICE->DCFG |=   USB_OTG_DCFG_DSPD_Msk;  // 11: Full speed (USB 1.1 transceiver clock is 48 MHz)

    

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

    NVIC_EnableIRQ(OTG_FS_IRQn);

#if 0

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
    	//USB_UNMASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
    	
    	(void)len++;
	}
#endif
}

void  OTG_FS_WKUP_IRQHandler() 
{
}

enum USB_Descriptor_Type_t {
    DEVICE_DESCRIPTOR_TYPE        = 1,
    CONFIGURATION_DESCRIPTOR_TYPE = 2,

    INTERFACE_DESCRIPTOR_TYPE     = 4,
    ENDPOINT_DESCRIPTOR_TYPE      = 5,

    HID_DESCRIPTOR_TYPE           = 0x21,
};

#define EP0_MAX_PACKET_SIZE   64

struct USB_Device_Descriptor_t {
    uint8_t  bLength; // 0x12
    uint8_t  bDescriptorType; // 1  Device Descriptor
    uint16_t bcdUSB; // 0x00, 0x02 --  2.0

    uint8_t  bDeviceClass; //
    uint8_t  bDeviceSubClass; //
    uint8_t  bDeviceProtocol; //

    uint8_t  bMaxPacketSize0; // Maximum packet size for endpoint zero(only 8, 16, 32, or 64 are valid)
    uint16_t idVendor; // VID Vendor ID
    uint16_t idProduct; // PID Product ID
    uint16_t bcdDevice; // Device release number (assigned by manufacturer).
    uint8_t  iManufacturer; // Index of string descriptor describing manufacturer
    uint8_t  iProduct; // Index of string descriptor describing product
    uint8_t  iSerialNumber; // Index of string descriptor describing the device serial number
    uint8_t bNumConfigurations; // Number of possible configurations. 
};

const struct USB_Device_Descriptor_t DeviceDescriptor __attribute__((aligned(16))) = {
    .bLength = sizeof(struct USB_Device_Descriptor_t), // 0x12
    .bDescriptorType = DEVICE_DESCRIPTOR_TYPE, // 1  Device Descriptor  
    .bcdUSB = 0x200, // 0x00, 0x02 --  2.0

    .bDeviceClass = 0,
    //
    .bDeviceSubClass = 0, //
    .bDeviceProtocol = 0, //

    .bMaxPacketSize0 = EP0_MAX_PACKET_SIZE,
    // Maximum packet size for endpoint zero(only 8, 16, 32, or 64 are valid)
    .idVendor = 0x03EB,
    // VID Vendor ID
    .idProduct = 2009,
    // PID Product ID
    .bcdDevice = 0x0001, // Device release number (assigned by manufacturer).
    .iManufacturer = 1, // Index of string descriptor describing manufacturer
    .iProduct = 2,
    // Index of string descriptor describing product
    .iSerialNumber = 3,
    // Index of string descriptor describing the device serial number
    .bNumConfigurations = 1, // Number of possible configurations. 
};

void  OTG_FS_IRQHandler() 
{
    uint32_t current_mode = 0;

	if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) != 0) {
    	USB_OTG_FS->GINTSTS = (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST);
    	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
    }

	// Wait for the end of reset on the USB
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) != 0) {
        USB_OTG_FS->GINTSTS = (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE);

        //Read the OTG_FS_DSTS to determine the enumeration speed
        current_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode 
        uint32_t device_status2 = USB_OTG_DEVICE->DSTS;
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
        //USB_OTG_IN_ENDPOINT0->DIEPCTL |= 3 << USB_OTG_DIEPCTL_MPSIZ_Pos; // 00: 64 bytes, 01: 32 bytes, 10: 16 bytes, 11: 8 bytes
        USB_OTG_IN_ENDPOINT0->DIEPCTL |= (USB_OTG_IN_ENDPOINT0->DIEPCTL & ~USB_OTG_DIEPCTL_MPSIZ) | (0 << USB_OTG_DIEPCTL_MPSIZ_Pos); // 00: 64 bytes, 01: 32 bytes, 10: 16 bytes, 11: 8 bytes
        //USB_OTG_IN_ENDPOINT0->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;         // USB active endpoint. Always is 1 for EP0
        //USB_OTG_IN_ENDPOINT0->DIEPCTL |= USB_OTG_DIEPCTL_EPENA; // Endpoint enable

        USB_OTG_OUT_ENDPOINT0->DOEPCTL  = (USB_OTG_OUT_ENDPOINT0->DOEPCTL & ~USB_OTG_DOEPCTL_EPTYP_Msk) | (0 << USB_OTG_DOEPCTL_EPTYP_Pos); // 00 Control type
        USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_CNAK; // Clear NAK. A write to this bit clears the NAK bit for the endpoint.
        USB_OTG_OUT_ENDPOINT0->DOEPCTL |= 8 << USB_OTG_DOEPCTL_MPSIZ_Pos; // Maximum packet size. This value is in bytes
        //USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;         // USB active endpoint. Always is 1 for EP0
        USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_EPENA; // Endpoint enable
    }

    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0) {
#define USB_OTG_GRXSTSP_FRMNUM_Pos                (21U)
#define USB_OTG_GRXSTSP_FRMNUM_Msk                (0xFUL << USB_OTG_GRXSTSP_FRMNUM_Pos) /*!< 0x01E00000 */
#define USB_OTG_GRXSTSP_FRMNUM                    USB_OTG_GRXSTSP_FRMNUM_Msk            /*!< Frame number*/

        // On catching an RXFLVL interrupt (OTG_FS_GINTSTS register), the application must read the Receive status pop register(OTG_FS_GRXSTSP).
        // A read to the GRXSTSP additionally pops the top data entry out of the RxFIFO
        uint32_t rxstsp = USB_OTG_FS->GRXSTSP;
        volatile uint32_t ep_num        = (rxstsp & USB_OTG_GRXSTSP_EPNUM)  >> USB_OTG_EPNUM_Pos;
        volatile uint32_t data_pid      = (rxstsp & USB_OTG_GRXSTSP_DPID)   >> USB_OTG_DPID_Pos;
        volatile uint32_t packet_status = (rxstsp & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_PKTSTS_Pos;
        volatile uint32_t frame_num     = (rxstsp & USB_OTG_GRXSTSP_FRMNUM) >> USB_OTG_FRMNUM_Pos;
        volatile uint32_t bytes_count   = (rxstsp & USB_OTG_GRXSTSP_BCNT)   >> USB_OTG_BCNT_Pos;

        current_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // CMOD: Current mode of operation 0 : Device mode, 1 : Host mode 

        //    	USB_OTG_OUT_ENDPOINT0->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
        //    	USB_OTG_OUT_ENDPOINT0->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

    	
#define USBx_DFIFO(i)   *(__IO uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))
        volatile uint8_t __attribute__((aligned(32))) buf[64];
        for (int i = 0U; i < bytes_count; i += sizeof(uint32_t))
        {
            __UNALIGNED_UINT32_WRITE(&buf[i], USBx_DFIFO(0U));
        }
        
        // 80 06 00 01 00 00 40 00
        const uint8_t request1[] = { 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, }; // 0x40, 0x00};
        const uint8_t request2[] = { 0x00, 0x05 };
        if (memcmp(&buf[0], request1, MIN(bytes_count, sizeof(request1))) == 0) {
            size_t response_len = MIN(sizeof(DeviceDescriptor), buf[6]);

            USB_OTG_IN_ENDPOINT0->DIEPTSIZ |= USB_OTG_DIEPTSIZ_PKTCNT | response_len;
            USB_OTG_IN_ENDPOINT0->DIEPCTL  |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK; // Endpoint enable

            size_t wc = (response_len + 3) / 4; // we must push this many words
            for (size_t i = 0; i < wc; ++i) {
                USBx_DFIFO(0U) = __UNALIGNED_UINT32_READ((uint8_t*)&DeviceDescriptor + 4 * i); // ok if we read over boundary
            }
        }
        else if (memcmp(&buf[0], request2, MIN(bytes_count, sizeof(request2))) == 0) {
            usb_addr = buf[2];
            usb_addr_old = buf[2];
            size_t response_len = 0; // ZLP
            USB_OTG_IN_ENDPOINT0->DIEPTSIZ |= USB_OTG_DIEPTSIZ_PKTCNT | response_len;
            USB_OTG_IN_ENDPOINT0->DIEPCTL  |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK; // Endpoint enable
        }
        else {
            __asm__("bkpt 255");
        }
        USB_OTG_FS->GINTSTS = (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL); // TODO: is it RW? correct?
    	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    }
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_NPTXFE) != 0) {
        if (usb_addr != 0) {
            USB_OTG_DEVICE->DCFG = usb_addr << USB_OTG_DCFG_DAD_Pos;
            usb_addr = 0;
        }
        USB_OTG_FS->GINTSTS = (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_NPTXFE); // TODO: is it RW? correct?
    	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
    }

    if (USB_OTG_IN_ENDPOINT0->DIEPINT & USB_OTG_DIEPINT_XFRC){
        //FIXME: Why??
        USB_OTG_OUT_ENDPOINT0->DOEPCTL |= USB_OTG_DOEPCTL_CNAK; // Clear NAK. A write to this bit clears the NAK bit for the endpoint.

        USB_OTG_IN_ENDPOINT0->DIEPINT = USB_OTG_DIEPINT_XFRC;
    }
}


int main(void)
{
    clock_initialization();
    otg_detach();
    otg_initialization();
    while(1) {
	}
}
