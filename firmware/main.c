#include <stdint.h>
#include <stdio.h>

#include "stm8.h"
//#include "image.h"
#include "font_24x32.h"

#define EPD_CS_PIN      4
#define EPD_DC_PIN      3
#define EPD_RST_PIN     2
#define EPD_BUSY_PIN    1
#define FLASH_CS_PIN    0

#define RED_LED     (6)
#define GRN_LED     (5)

#define EPD_SIZE    5808

#define PB_HIGH(x)  (PB_ODR |= (1 << (x)))
#define PB_LOW(x)   (PB_ODR &= ~(1 << (x)))

#define PC_HIGH(x)  (PC_ODR |= (1 << (x)))
#define PC_LOW(x)   (PC_ODR &= ~(1 << (x)))

#define FLASH_CS_SetLow()   PB_LOW(FLASH_CS_PIN)
#define FLASH_CS_SetHigh()  PB_HIGH(FLASH_CS_PIN)
#define EPD_CS_SetLow()     PB_LOW(EPD_CS_PIN)
#define EPD_CS_SetHigh()    PB_HIGH(EPD_CS_PIN)
#define EPD_DC_SetLow()     PB_LOW(EPD_DC_PIN)
#define EPD_DC_SetHigh()    PB_HIGH(EPD_DC_PIN)
#define EPD_RST_SetLow()    PB_LOW(EPD_RST_PIN)
#define EPD_RST_SetHigh()   PB_HIGH(EPD_RST_PIN)
#define PWR2_SetHigh()      (PD_ODR |= 1)
#define PWR2_SetLow()       (PD_ODR &= ~(1))

#define Boost_On()         (PA_ODR |= (1<<3))
#define Boost_Off()        (PA_ODR &= ~(1<<3))

#define EPD_BUSY_GetValue() (PB_IDR &= (1 << (EPD_BUSY_PIN)))

#define T_COUNT(x) ((( F_CPU * x / 1000000UL )-5)/5)
#define HIGH_NIB(x) (((x) >> 4) & 0xf)
#define LOW_NIB(x) ((x) & 0xf)

void    Clock_Init(void);
void    SPI_Init(void);
void    SPI_Disable(void);
uint8_t SPI_ExchangeByte(uint8_t x);
void    SPI_WriteBlock(uint8_t* buf, size_t buflen);
void    SPI_ReadBlock(uint8_t* buf, size_t buflen);

void    EPD_Init(void);
void    EPD_Display(const uint8_t *imgdata);
void    EPD_StartDisplay(uint8_t id);
void    EPD_PushData(const uint8_t *imgdata, uint16_t len);
void    EPD_Refresh(void);
void    EPD_Sleep(void);

void    DisplayData(uint8_t count, uint16_t sec, uint16_t volt);
void    EPD_PushChar(char c);
void    EPD_PushString(const char* str);

void    W25qxx_Init(void);
uint32_t W25qxx_GetId(void);
void    W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void    W25qxx_Erase32(uint32_t Page_Address);
void    W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint16_t dataSize);
void    W25qxx_Sleep(void);
#define W25qxx_Spi SPI_ExchangeByte

void DisplayImage(int idx);


unsigned short xs = 1;

#define srand(X) {xs=X;}

unsigned short rand()
{
    xs ^= xs << 7;
    xs ^= xs >> 9;
    xs ^= xs << 8;
    return xs;
}

static inline void __delay_ms(uint16_t ms) {
    uint32_t i;
    for (i = 0; i < ((F_CPU / 18000UL) * ms); i++)
        __asm__("nop");
}

unsigned int clock(void)
{
	unsigned char h = TIM1_CNTRH;
	unsigned char l = TIM1_CNTRL;
	return((unsigned int)(h) << 8 | l);
}

void Clock_Init(void) {
    CLK_CKDIVR = 0x03; // Set the frequency to 2 MHz (default)
}

void RTC_Init(void) {
    while (CLK_CRTCR & 1);
    CLK_CRTCR = 0x04; // RTC uses LSI/1 (38Hz) RTCDIV = 0
    CLK_ICKCR |= 0x04; //Enable LSI
    while (CLK_CRTCR & 1);
    CLK_PCKENR2 |= 0x04; //RTC

    RTC_WPR = 0xCA; //Unlock RTC edits
    RTC_WPR = 0x53;

    
    RTC_ISR1 |= 0x80;
    while (!(RTC_ISR1 & 0x40));
    
    //RTCDIV = 0 , PREDIV_A = 124, PREDIV_S = 303
    //requires INITF
    RTC_SPRERH = 1;
    RTC_SPRERL = 47;
    RTC_APRER = 124;

    RTC_ISR1 &= ~0x80; 
    
    RTC_CR2 &= ~0x04; //Disable Wakeup timer
    while (!(RTC_ISR1 & 0x04)); // wait for Wakeup writing to be enabled
    //requires WUTWF and not WUTE
    RTC_CR1 = 0x04; //ck_spre source (1Hz)
    RTC_WUTRH = 0; //0x02;
    RTC_WUTRL = 0x5;//A;//0x58; //600 seconds
    
    RTC_CR2 = 0x44; //Enable Wakeup timer and interrupt

    RTC_WPR = 0xFF; // Lock RTC
}

void Tim4_Init() {
    CLK_PCKENR1 |= (1<<2); // Enable Tim4
    TIM4_PSCR = 7;
    /* Period = 5ms */
    TIM4_ARR = 77;
    TIM4_IER |= (1 << 0); // Enable Interrupt
    TIM4_CR1 |= (1 << 0); // enable Timer4
}

void Tim4_Disable() {
    TIM4_CR1 &= ~(1 << 0); // Disable Timer4
    CLK_PCKENR1 &= ~(1<<2); // Disable Tim4
    TIM4_SR = 0; // Clear interrupts
}

void RTC_ISR(void) __interrupt(4) {
    RTC_ISR2 = 0; // Clear interrupts
}

void TIM4_ISR(void) __interrupt(25) {
    TIM4_SR = 0; // Clear interrupts
}

void ADC_Init() {
    PC_ODR &= ~(1<<1); // Enable Vsignal
    CLK_PCKENR2 |= 1;
    ADC1_CR1 |= 1; // Enable ADC
    ADC1_CR2 = 0x04; //Sampling time = 48 ADC clock cycles
    ADC1_SQR1 |= 0x80; // DMA off
    ADC1_SQR4 = (1 << 4); // Configure ADC channel 4 (PC4)
    PC_ODR |= (1<<1);
}

// Disable Vrefint ADC channel and ADC module
void ADC_Disable() {
    PC_ODR |= (1<<1); //Disable Vsignal
    ADC1_SQR4 = 0;
    ADC1_CR1 &= ~1; // Disable ADC
    CLK_PCKENR2 &= ~1; // Disable ADC1
}

// Init Vreint ADC channel
void ADC_Vrefint_Init(void) {
    CLK_PCKENR2 |= 1;
    ADC1_CR1 |= 1; // Enable ADC
    ADC1_TRIGR1 |= 0x10; // Enable internal reference voltage
    ADC1_SQR1 |= 0x10; // Enable CHSEL_SVREFINT fast ADC channel
    ADC1_CR3 = 0x80; // Sampling time = 48 ADC clock cycles, disable analog watchdog channels
    ADC1_SQR1 |= 0x80; // DMA off
}

uint16_t ADC_Read(void) {
    uint16_t adc_res;
    uint16_t value = 0;
    uint8_t cntr;

    for (cntr = 0; cntr < 4; cntr++) {
        ADC1_CR1 |= 2; // Start ADC conversion, by software trigger
        while (!(ADC1_SR & 1)); // Wait for the conversion ends
        adc_res  = (ADC1_DRH << 8); // Get ADC converted data
        adc_res |= ADC1_DRL;
        value += adc_res;
        if (cntr) value >>= 1;
    }

    return value;
}

// Disable Vrefint ADC channel and ADC module
void ADC_Vrefint_Disable() {
    ADC1_TRIGR1 &= ~0x10; // Disable internal reference voltage
    ADC1_SQR1 &= ~0x10; // Disable CHSEL_SVREFINT fast ADC channel
    ADC1_CR1 &= ~0x01; // Disable ADC
    CLK_PCKENR2 &= ~1; // Disable ADC1
}

void memcpy(uint8_t *dst, uint8_t *src, uint16_t len) {
    for (uint16_t i=0; i< len; ++i)
        dst[i] = src[i];
}

void DisplayImage(int idx) {
    uint8_t buf[726];
    uint32_t address = (uint32_t)idx * 5888;

    EPD_Init();
    W25qxx_Init();

    /*
    EPD_Init();
    EPD_Display(gImage);
    EPD_Sleep();
    //*/
        
    EPD_StartDisplay(0);
    for (uint16_t i=0; i< EPD_SIZE; i+=sizeof(buf)) {
        //memcpy(buf, gImage+i, sizeof(buf));
        W25qxx_ReadBytes(buf, address+i, sizeof(buf));
        EPD_PushData(buf, sizeof(buf));
    }
    W25qxx_Sleep();
    EPD_Refresh();

    EPD_Sleep();
}

void opt_write() {
    FLASH_CR2 |= (1 << 7);
    /* write option byte and it's complement */
    *(volatile unsigned char *)0x480A = 1; //Enable BOR
    /* wait until programming is finished */
    FLASH_CR2 &= ~(1 << 7);
}

void main(void)
{
    uint8_t frame = 1;   
    uint16_t idx = 0;
    uint16_t checks=0; 
    uint16_t factory_vref = 0x0691;
    uint16_t last_voltage =0;

    uint8_t led = GRN_LED;

    //__delay_ms(1000);

    // SPI port setup: MISO is pullup in, MOSI & SCK are push-pull out
    PB_DDR |= (1<<0) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6); // SCK and MOSI
    PB_CR1 |= (1<<0) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6); //
    PD_DDR |= (1<<0);
    PD_CR1 |= (1<<0);
    PC_DDR |= (1<<5) | (1<<6); //LED cntl
    PC_CR1 |= (1<<5) | (1<<6);
    PA_CR1 |= (1<<3);
    PA_DDR |= (1<<3);
    Boost_Off();
    PWR2_SetLow();
    

    Clock_Init();
    RTC_Init();

    //while (!(PWR_CSR2 & 1));
    //PWR_CSR2 |= 2; // ULP
    __asm__ ("rim"); //enable interrupts

    while(1) {

        ADC_Vrefint_Init();
        //ADC_Init();
        uint16_t vrefint = ADC_Read();
        ADC_Vrefint_Disable();
        //ADC_Disable();
        vrefint =(((uint32_t)factory_vref*300)/vrefint);
        //vrefint = ((uint32_t)300*vrefint / 4095);

        PA_CR1 |= (1<<2);
        uint8_t unlocked = (PA_IDR & (1<<2) );
        PA_CR1 &= ~(1<<2);
        
        checks++;
        if (vrefint > 255)
        { 
            Boost_On();
            PWR2_SetHigh();
            __delay_ms(10);
            //__asm__ ("sim"); //disable interrupts
            SPI_Init();

            //DisplayData(frame++, checks, vrefint);
            DisplayImage(idx); // update display
            checks = 0;

            uint16_t nidx;
            do {
                nidx = rand() % 12;
                if (unlocked && (nidx==1)) {
                    nidx = 12 + rand() % 10;
                }
            } while (idx == nidx);
            idx = nidx;
        }
        else if (vrefint > last_voltage) {

            //VMeasure_On();
            //ADC_Init();
            //uint16_t vsolar = ADC_Read();
            //ADC_Disable();
            //vsolar = ((uint32_t)vsolar*vrefint / 4095);

            uint8_t led = unlocked?GRN_LED:RED_LED;
            
            //Tim4_Init();
            for (uint8_t i=0; i<5; ++i) {
                PC_HIGH(led); 
                __delay_ms(5);
                //__asm__("wfi");
                PC_LOW(led); 
                __delay_ms(5);
                //__asm__("wfi");
            } 
            //Tim4_Disable();
        }//*/

        xs = rand(); //seeding rand
        last_voltage = vrefint;
        PWR2_SetLow(); 
        Boost_Off();
        SPI_Disable();
        //Go to sleep
        //__asm__("wfi");
        __asm__ ("halt");
        __asm__ ("nop");

    }
}

/*
 * SPI pinout:
 * SCK  -> PB5
 * MOSI -> PB6
 * MISO -> PB7
 * no CS PIN
 */
void SPI_Init(void) {
    CLK_PCKENR1 |= 0x10; //SPI
    /* Initialize SPI master at 500kHz  */
    SPI_CR2 = SPI_CR2_SSM | SPI_CR2_SSI;
    SPI_CR1 = SPI_CR1_MSTR; // | SPI_CR1_BR(3);
    PB_ODR |= (1<<0) | (1<<2) | (1<<4); //CS high
    SPI_CR1 |= SPI_CR1_SPE; //SPI on    
}

void SPI_Disable(void) {
    //CLK_PCKENR1 &= ~0x10; // Disable SPI CLK
    SPI_CR1 &= ~SPI_CR1_SPE; //SPI off
    PB_ODR = 0; //Pins off
} 

void SPI_Wait(void) {
    while ((SPI_SR & SPI_SR_BSY));
}

uint8_t SPI_ExchangeByte(uint8_t x) {
    while (!(SPI_SR & SPI_SR_TXE));
    SPI_DR = x;
    while (!(SPI_SR & SPI_SR_RXNE));
    return SPI_DR;
}

void SPI_WriteByte(uint8_t x) {
    while (!(SPI_SR & SPI_SR_TXE));
    SPI_DR = x;
}

void SPI_WriteBlock(const uint8_t* buf, size_t buflen) {
    for (size_t i=0; i< buflen; ++i) {
        while (!(SPI_SR & SPI_SR_TXE));
        SPI_DR = buf[i];
    }
}

void SPI_ReadBlock(uint8_t* buf, size_t buflen) {
    for (size_t i=0; i< buflen; ++i) {
        while (!(SPI_SR & SPI_SR_TXE));
        SPI_DR = 0;
        while (!(SPI_SR & SPI_SR_RXNE));
        buf[i] = SPI_DR;
    }
}

static void EPD_SendCommand(uint8_t cmd, uint16_t datalen, const uint8_t *data) {
	EPD_DC_SetLow();
    EPD_CS_SetLow();
    SPI_WriteByte(cmd);
    SPI_Wait();
    EPD_CS_SetHigh();
    EPD_DC_SetHigh();
    //SPI1_WriteBlock(data, datalen);
    for (uint16_t i=0; i< datalen; ++i) {
        EPD_CS_SetLow();
        SPI_WriteByte(data[i]);
        SPI_Wait();
        EPD_CS_SetHigh();
    }
}

static void EPD_Wait(void) {
	for (int i=0; i<500; ++i){
        __delay_ms(10);
		if (EPD_BUSY_GetValue() == 0)
			return;
	}
	//printf("wait failed\n");
}

    static const uint16_t WIDTH = 176;
    static const uint16_t HEIGHT = 264;

void EPD_Init(void) {
    EPD_RST_SetLow();
    __delay_ms(10);
    EPD_RST_SetHigh();
	__delay_ms(10);

    //EPD 2.7v2
    EPD_SendCommand(0x12, 0, NULL);  //SWRESET
    EPD_Wait();
    //EPD_SendCommand(0x11, 1, "\x03"); //data entry mode       
    //EPD_SendCommand(0x3C, 1, "\x05"); //BorderWavefrom
    EPD_SendCommand(0x18, 1, "\x80"); //Read built-in temperature sensor
    EPD_SendCommand(0x22, 1, "\xb1"); // Power On
    EPD_SendCommand(0x20, 0, NULL);
    EPD_Wait();

    EPD_SendCommand(0x1A, 2, "\x64\x00"); //Read built-in temperature sensor
    EPD_SendCommand(0x22, 1, "\x91"); // Power On
    EPD_SendCommand(0x20, 0, NULL);
    EPD_Wait();
    
}

void EPD_Refresh(void) {

    EPD_SendCommand(0x22, 1, "\xc7");
    EPD_SendCommand(0x20, 0, NULL);
    //EPD_SendCommand(0x12, 0, NULL);//REFRESH
    EPD_Wait();
}

void EPD_Display(const uint8_t *imgdata) {
	//EPD_SendCommand(AREA, 7, "\0\x7F\0\0\x01\x27\x28");
    //EPD_SendCommand(0x10, EPD_SIZE, imgdata);
	//EPD_SendCommand(0x13, EPD_SIZE, imgdata);
    EPD_SendCommand(0x24, EPD_SIZE, imgdata);
    EPD_Refresh();
}

void EPD_StartDisplay(uint8_t page) {
    EPD_SendCommand(0x24,0, NULL);
}

void EPD_PushData(const uint8_t *imgdata, uint16_t len) {
    for (uint16_t i=0; i< len; ++i) {
        EPD_CS_SetLow();
        SPI_WriteByte(imgdata[i]);
        SPI_Wait();
        EPD_CS_SetHigh();
    }
    //SPI_WriteBlock(imgdata, len);
}

void EPD_Sleep(void) {
	//EPD_SendCommand(0x50, 1, "\xf7"); //VCOM_DATA FLOATING
    //EPD_SendCommand(0x02, 0, NULL); //POWER_OFF
    //EPD_SendCommand(0x07, 1, (uint8_t*)"\xA5"); //SLEEP
    EPD_SendCommand(0x10, 1, "\x01"); //SLEEP
}

void DisplayData(uint8_t count, uint16_t sec, uint16_t volt) {
    char str[12];
    str[2] = ' ';
    str[6] = ' ';
    str[8] = '.';
    str[11] = '\0';
    str[0] = '0' + (count / 10);
    str[1] = '0' + (count % 10);
    str[3] = '0' + ((sec / 100) % 10);
    str[4] = '0' + ((sec / 10) % 10);
    str[5] = '0' + ((sec) % 10);
    str[7] = '0' + ((volt /100) % 10);
    str[9] = '0' + ((volt /10)% 10);
    str[10] = '0' + ((volt)% 10);

    W25qxx_Sleep();
    EPD_Init();
    EPD_StartDisplay(0);
    EPD_PushString(str);
    EPD_StartDisplay(1);
    EPD_PushString(str);
    EPD_Refresh();
    EPD_Sleep();
}


const uint8_t pat[] = {0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF};
void EPD_PushChar(char c) {
    int c2 = c - '.';
    for (int i=0; i<24; ++i) {
        EPD_PushData(pat, 9);
        if (c >= '.' && c <= '9') {
            EPD_PushData(&F24x32[c2][i*4], 4);
        }
        else {
            EPD_PushData(&F24x32[1][i*4], 4);
        }
        EPD_PushData(pat, 9);
    }
}
void  EPD_PushString(const char* str) {
    for (int i=0; i< 11; ++i) {
        EPD_PushChar(str[10-i]);
    }
}

void W25qxx_Init(void) {
    FLASH_CS_SetLow();
    W25qxx_Spi(0xAB);
    SPI_Wait();
    FLASH_CS_SetHigh();
    __delay_ms(10);
}

uint32_t W25qxx_GetId(void) {
    uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
    FLASH_CS_SetLow();
	W25qxx_Spi(0x9F);
	Temp0 = W25qxx_Spi(0);
	Temp1 = W25qxx_Spi(0);
	Temp2 = W25qxx_Spi(0);
    SPI_Wait();
    FLASH_CS_SetHigh();
	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	return Temp;
}

void W25qxx_Sleep(void) {
    FLASH_CS_SetLow();
	W25qxx_Spi(0xB9);
    SPI_Wait();
    FLASH_CS_SetHigh();
}

void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
	FLASH_CS_SetLow();

	W25qxx_Spi(0x03);
	W25qxx_Spi((ReadAddr & 0xFF0000) >> 16);
	W25qxx_Spi((ReadAddr & 0xFF00) >> 8);
	W25qxx_Spi(ReadAddr & 0xFF);
	W25qxx_Spi(0); //shouldn't be here but needed to make reading line up
    SPI_ReadBlock(pBuffer, NumByteToRead);
    SPI_Wait();
	FLASH_CS_SetHigh();
}

void W25qxx_WriteEnable(void)
{
	FLASH_CS_SetLow();
	W25qxx_Spi(0x06);
    SPI_Wait();
	FLASH_CS_SetHigh();
}

void W25qxx_WaitForWriteEnd(void)
{
	uint8_t status = 1;
	FLASH_CS_SetLow();
	W25qxx_Spi(0x05);
	do
	{
        status = W25qxx_Spi(0xFF);
	} while ((status & 0x01) == 0x01);
    SPI_Wait();
	FLASH_CS_SetHigh();
}

void W25qxx_Erase32(uint32_t Page_Address)
{
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	FLASH_CS_SetLow();
	W25qxx_Spi(0x52);
	W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi((Page_Address & 0xFF00) >> 8);
	W25qxx_Spi(Page_Address & 0xFF);
    SPI_Wait();
    FLASH_CS_SetHigh();
	W25qxx_WaitForWriteEnd();
}

void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint16_t dataSize)
{
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	FLASH_CS_SetLow();
	//Page_Address = (Page_Address * 256);
	W25qxx_Spi(0x02);
	W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi((Page_Address & 0xFF00) >> 8);
	W25qxx_Spi(Page_Address & 0xFF);
    SPI_WriteBlock(pBuffer, dataSize);
    SPI_Wait();
	FLASH_CS_SetHigh();
	W25qxx_WaitForWriteEnd();
}