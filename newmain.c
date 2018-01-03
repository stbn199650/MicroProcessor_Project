// CONFIG1H
#pragma config OSC = INTIO67      // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = ON       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


/* SSPCON1 REGISTER */
#define   SSPENB    			0b00100000  	/* Enable serial port and configures SCK, SDO, SDI*/
#define   SLAVE_7   			0b00000110     	/* I2C Slave mode, 7-bit address*/
#define   SLAVE_10  			0b00000111    	/* I2C Slave mode, 10-bit address*/
#define   MASTER    			0b00001000     	/* I2C Master mode */
#define   MASTER_FIRMW			0b00001011		//I2C Firmware Controlled Master mode (slave Idle)
#define   SLAVE_7_STSP_INT 		0b00001110		//I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
#define   SLAVE_10_STSP_INT 	0b00001111		//I2C Slave mode, 10-bit address with Start and Stop bit interrupts enabled
#define   _XTAL_FREQ          4000000
/* SSPSTAT REGISTER */
#define   SLEW_OFF  			0b10000000  	/* Slew rate disabled for 100kHz mode */
#define   SLEW_ON   			0b00000000  	/* Slew rate enabled for 400kHz mode  */
#define USE_OR_MASKS

#include <xc.h>
#include <pic18f4520.h>
#include<stdint.h>
#include<string.h>
#include<stdbool.h>
#include<stdlib.h>

struct trellis_set
{
    uint8_t address,keys[6],last_keys[6];
    uint16_t display_buffer[8];
};
struct trellis_set matrices[4];
static const uint8_t ledLUT[16] =
    { 0x3A, 0x37, 0x35, 0x34,
      0x28, 0x29, 0x23, 0x24,
      0x16, 0x1B, 0x11, 0x10,
      0x0E, 0x0D, 0x0C, 0x02 },
    buttonLUT[16] =
    { 0x07, 0x04, 0x02, 0x22,
      0x05, 0x06, 0x00, 0x01,
      0x03, 0x10, 0x30, 0x21,
      0x13, 0x12, 0x11, 0x31 };

static const uint8_t Buttons[8][8] = {
    { 0, 1, 2, 3, 16, 17, 18, 19},
    { 4, 5, 6, 7, 20, 21, 22, 23},
    { 8, 9, 10, 11, 24, 25, 26, 27},
    {12, 13, 14, 15, 28, 29, 30, 31},
    {32, 33, 34, 35, 48, 49, 50, 51},
    {36, 37, 38, 39, 52, 53, 54, 55},
    {40, 41, 42, 43, 56, 57, 58, 59},
    {44, 45, 46, 47, 60, 61, 62, 63}
};
//animation0
static const uint8_t show0_1[40] = {0, 1, 2, 3, 16, 17, 18, 19, 22, 21, 20, 7, 6, 5, 10, 11, 24, 25, 28, 15, 35, 48, 53, 52, 39, 38, 41, 42, 43, 56, 57, 58, 63, 62, 61, 60, 47, 46, 45, 44};
static const uint8_t show0_2[40] = {0, 4, 8, 12, 32, 36, 40, 44, 41, 37, 33, 13, 9, 5, 10, 14, 34, 38, 35, 15, 28, 48, 53, 49, 29, 25, 22, 26, 30, 50, 54, 58, 63, 59, 55, 51, 31, 27, 23, 19};
static const uint8_t show0_3[6][20] = {
    {15, 28, 35, 48, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {11, 14, 34, 39, 24, 29, 49, 52, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {10, 11, 24, 25, 14, 34, 38, 39, 52, 53, 29, 49, 100, 100, 100, 100, 100, 100, 100, 100},
    {7, 10, 13, 33, 38, 43, 20, 25, 30, 50, 53, 56, 100, 100, 100, 100, 100, 100, 100, 100},
    {5, 6, 7, 20, 21, 22, 9, 13, 33, 37, 41, 42, 43, 56, 57, 58, 26, 30, 50, 54},
    {2, 5, 8, 36, 41, 46, 55, 58, 61, 17, 22, 27, 100, 100, 100, 100, 100, 100, 100, 100}
};
//animation1
static const uint8_t arrow[16][4][4] = {
    {
        {0, 100, 100, 100},
        {1, 4, 5, 100},
        {6, 9, 10, 100},
        {11, 14, 15, 100}
    },
    {
        {1, 100, 100, 100},
        {0, 2, 5, 100},
        {4, 6, 9, 100},
        {8, 10, 13, 100}
    },
    {
        {2, 100, 100, 100},
        {1, 3, 6, 100},
        {5, 7, 10, 100},
        {9, 11, 14, 100}
    },
    {
        {3, 100, 100, 100},
        {2, 6, 7, 100},
        {5, 9, 10, 100},
        {8, 12, 13, 100}
    },
    {
        {4, 100, 100, 100},
        {0, 5, 8, 100},
        {1, 6, 9, 100},
        {2, 7, 10, 100}
    },
    {
        {5, 100, 100, 100},
        {1, 4, 6, 9},
        {7, 13, 100, 100},
        { 100, 100, 100, 100}
    },
    {
        {6, 100, 100, 100},
        {2, 5, 7, 10},
        {4, 14, 100, 100},
        { 100, 100, 100, 100}
    },
    {
        {7, 100, 100, 100},
        {3, 6, 11, 100},
        {2, 5, 10, 100},
        {1, 4, 9, 100}
    },
    {
        {8, 100, 100, 100},
        {4, 9, 12, 100},
        {5, 10, 13, 100},
        {6, 11, 14, 100}
    },
    {
        {9, 100, 100, 100},
        {5, 8, 10, 13},
        {1, 11, 100, 100},
        { 100, 100, 100, 100}
    },
    {
        {10, 100, 100, 100},
        {6, 9, 11, 14},
        {2, 8, 100, 100},
        { 100, 100, 100, 100}
    },
    {
        {11, 100, 100, 100},
        {7, 10, 15, 100},
        {6, 9, 14, 100},
        {5, 8, 13, 100}
    },
    {
        {12, 100, 100, 100},
        {8, 9, 13, 100},
        {5, 6, 10, 100},
        {2, 3, 7, 100}
    },
    {
        {13, 100, 100, 100},
        {9, 12, 14, 100},
        {5, 8, 10, 100},
        {1, 4, 6, 100}
    },
    {
        {14, 100, 100, 100},
        {10, 13, 15, 100},
        {6, 9, 11, 100},
        {2, 5, 7, 100}
    },
    {
        {15, 100, 100, 100},
        {10, 11, 14, 100},
        {5, 6, 9, 100},
        {0, 1, 4, 100}
    }
};
//animation2
static const uint8_t horizental_flash[8][3] = {
    {0, 1, 2},
    {1, 2, 3},
    {2, 3, 16},
    {3, 16, 17},
    {16, 17, 18},
    {17, 18, 19},
    {18, 19, 100},
    {19, 100, 100}
};
//animation3
static const uint8_t circle[4][26] = {
    {0, 1, 2, 3, 16, 17, 18, 22, 26, 30, 50, 54, 58, 4, 8, 12, 32, 36, 40, 44, 45, 46, 47, 60, 61, 62},
    {5, 6, 7, 20, 21, 25, 29, 49, 53, 57, 9, 13, 33, 37, 41, 42, 43, 56, 100, 100, 100, 100, 100, 100, 100, 100},
    {10, 11, 24, 28, 14, 34, 38, 39, 52, 48, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {15, 35, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}
};
static const uint8_t show3_1[10][8] = {
    {0, 1, 2, 3, 47, 60, 61, 62},
    {1, 2, 3, 16, 46, 47, 60, 61},
    {2, 3, 16, 17, 45, 46, 47, 60},
    {3, 16, 17, 18, 44, 45, 46, 47},
    {16, 17, 18, 22, 40, 44, 45, 46},
    {17, 18, 22, 26, 36, 40, 44, 45},
    {18, 22, 26, 30, 32, 36, 40, 44},
    {22, 26, 30, 50, 12, 32, 36, 40},
    {26, 30, 50, 54, 8, 12, 32, 36},
    {30, 50, 54, 58, 4, 8, 12, 32}
};
static const uint8_t show3_2[7][6] = {
    {5, 6, 7, 43, 56, 57},
    {6, 7, 20, 42, 43, 56},
    {7, 20, 21, 41, 42, 43},
    {20, 21, 25, 37, 41, 42},
    {21, 25, 29, 33, 37, 41},
    {25, 29, 49, 13, 33, 37},
    {29, 49, 53, 9, 13, 33}
};
static const uint8_t show3_3[5][4] = {
    {10, 11, 39, 52},
    {11, 24, 38, 39},
    {24, 28, 34, 38},
    {28, 48, 14, 34},
    {10, 14, 48, 52}
};
//animation4
static const uint8_t show4_1[8][8] = {
    {19, 44, 100, 100, 100, 100, 100, 100},
    {18, 23, 40, 45, 100, 100, 100, 100},
    {17, 22, 27, 36, 41, 46, 100, 100},
    {16, 21, 26, 31, 32, 37, 42, 47},
    {20, 25, 30, 51, 33, 38, 43, 60},
    {24, 29, 50, 55, 34, 39, 56, 61},
    {28, 49, 54, 59, 35, 52, 57, 62},
    {48, 53, 58, 63, 100, 100, 100, 100}
};
static const uint8_t show4_4[8][8] = {
    {19, 44, 100, 100, 100, 100, 100, 100},
    {18, 23, 40, 45, 100, 100, 100, 100},
    {17, 22, 27, 36, 41, 46, 100, 100},
    {16, 21, 26, 31, 32, 37, 42, 47},
    {3, 20, 25, 30, 33, 38, 43, 12},
    {2, 7, 24, 29, 8, 13, 34, 39},
    {1, 6, 11, 28, 4, 9, 14, 35},
    {0, 5, 10, 15, 100, 100, 100, 100}
};
static const uint8_t show4_2[12][8] = {
    {0, 1, 2, 4, 5, 8, 100, 100},
    {2, 5, 8, 3, 6, 9, 12, 100},
    {3, 6, 9, 12, 7, 10, 13, 32},
    {7, 10, 13, 32, 11, 14, 33, 36},
    {11, 14, 33, 36, 15, 34, 37, 40},
    {15, 34, 37, 40, 35, 38, 41, 44},
    {35, 38, 41, 44, 48, 39, 42, 45},
    {48, 39, 42, 45, 49, 52, 43, 46},
    {49, 52, 43, 46, 50, 53, 56, 47},
    {50, 53, 56, 47, 51, 54, 57, 60},
    {51, 54, 57, 60, 55, 58, 61, 100},
    {55, 58, 61, 59, 62, 63, 100, 100}
};
static const uint8_t show4_3[12][8] = {
    {55, 58, 61, 59, 62, 63, 100, 100},
    {51, 54, 57, 60, 55, 58, 61, 100},
    {31, 50, 53, 56, 51, 54, 57, 60},
    {27, 30, 49, 52, 31, 50, 53, 56},
    {23, 26, 29, 48, 27, 30, 49, 52},
    {19, 22, 25, 28, 23, 26, 29, 48},
    {18, 21, 24, 15, 19, 22, 25, 28},
    {17, 20, 11, 14, 18, 21, 24, 15},
    {16, 7, 10, 13, 17, 20, 11, 14},
    {3, 6, 9, 12, 16, 7, 10, 13},
    {2, 5, 8, 3, 6, 9, 12, 100},
    {0, 1, 2, 4, 5, 8, 100, 100}
};
//animation5
static const uint8_t show5[16][4] = {
    {0, 1, 4, 5},
    {1, 2, 5, 6},
    {2, 3, 6, 7},
    {3, 7, 16, 20},
    {4, 5, 8, 9},
    {5, 6, 9, 10},
    {6, 7, 10, 11},
    {7, 11, 20, 24},
    {8, 9, 12, 13},
    {9, 10, 13, 14},
    {10, 11, 14, 15},
    {11, 24, 15, 28},
    {12, 13, 32, 33},
    {13, 14, 33, 34},
    {14, 15, 34, 35},
    {15, 28, 35, 48}
};
//animation7
static const uint8_t show7_1[4][29] = {
    {15, 28, 35, 48, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {10, 11, 14, 34, 38, 39, 52, 53, 49, 29, 25, 24, 11, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {5, 6, 7, 20, 21, 9, 13, 33, 37, 41, 42, 43, 56, 57, 58, 54, 50, 30, 26, 22, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {0, 1, 2, 3, 16, 17, 18, 19, 4, 8, 12, 32, 36, 40, 44, 45, 46, 47, 60, 61, 62, 63, 23, 27, 31, 51, 55, 59, 63}
};
static const uint8_t show7_2[9][24] = {
    {0, 5, 10, 15, 19, 22, 25, 28, 35, 38, 41, 44, 48, 53, 58, 63, 100, 100, 100, 100, 100, 100, 100, 100},
    {4, 9, 14, 35, 48, 39, 42, 45, 28, 49, 54, 59, 15, 24, 21, 18, 100, 100, 100, 100, 100, 100, 100, 100},
    {3, 16, 6, 7, 10, 11, 12, 32, 33, 37, 34, 38, 47, 60, 56, 57, 52, 53, 31, 51, 26, 30, 25, 29},
    {3, 16, 6, 7, 10, 12, 32, 33, 37, 38, 47, 60, 56, 57, 53, 31, 51, 30, 25, 26, 15, 28, 35, 48},
    {2, 3, 5, 6, 11, 24, 32, 36, 37, 41, 14, 34, 60, 61, 57, 58, 52, 39, 27, 31, 22, 26, 29, 49},
    {0, 1, 5, 6, 10, 19, 23, 22, 26, 25, 40, 44, 37, 41, 38, 62, 63, 57, 58, 53, 100, 100, 100, 100},
    {0, 1, 6, 40, 44, 37, 57, 62, 63, 19, 23, 26, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {0, 1, 40, 44, 62, 63, 19, 23, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {0, 44, 63, 19, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}
};

void MyOpenI2C( unsigned char sync_mode, unsigned char slew );
void MyRestartI2C( void );
void MyStartI2C();
void MyCloseI2C();
void MyIdleI2C();
signed char MyWriteI2C( unsigned char data_out );
unsigned char MyReadI2C( void );
signed char MyputsI2C( unsigned char *wrptr );
signed char MygetsI2C( unsigned char *rdptr, unsigned char length );
void MyNotAckI2C( void );
void MyStopI2C();
void ISR_Init();
void write_display();
void write(char data_out)
{
    signed char data;
    int status;
    data=SSPBUF;
     do{
        status=MyWriteI2C(data_out);   //write address of slave
        if(status == -1){   //collision happen
            data=SSPBUF;
            SSPCON1bits.WCOL = 0;
        }
    }while(status!=0);   //wait until success communicate
}
void clr_led(uint8_t i);
void set_led(uint8_t i);
int is_led(uint8_t x);
bool is_key_pressed(uint8_t k);
bool was_key_pressed(uint8_t k);
bool read_switch();

void animation0(uint8_t m, uint8_t n);
void animation1(uint8_t m, uint8_t n);
void animation2(uint8_t m, uint8_t n, uint8_t h);
void animation3(uint8_t m);
void animation4(uint8_t m, uint8_t h);
void animation5(uint8_t m, uint8_t n, uint8_t h);
void animation6(uint8_t m, uint8_t n);
void animation7(uint8_t h);

void main(void) {
    
    unsigned char sync_mode=0, slew=0,data,status,data_buffer[8];
    int i,j;
    matrices[0].address= 0x70<<1;
    matrices[1].address= 0x71<<1;
    matrices[2].address= 0x72<<1;
    matrices[3].address= 0x73<<1;
    //close if it are operate earlier
    MyCloseI2C();   
    //initalize i2c module
    sync_mode = MASTER;
    slew = SLEW_OFF;
    MyOpenI2C(sync_mode,slew);
    //set baud rate
    OSCCONbits.IRCF=6;
    SSPADD=9;
    
    __delay_ms(1);
    for(i=0;i<4;i++)
    {
        //set osc
        MyIdleI2C();
        MyStartI2C();
        MyIdleI2C();     
        write(matrices[i].address|0x00);
        MyIdleI2C();
        write(0x21);
        MyIdleI2C();
        MyStopI2C();

        //set blink out
        MyIdleI2C();
        MyStartI2C();
        MyIdleI2C();      
        write(matrices[i].address| 0x00);
        MyIdleI2C();
        write(0b10000001);
        MyIdleI2C();
        MyStopI2C(); 
    }
    
    uint8_t column = 0, rightbutton = 0, m = 0, n = 0, h = 0;
    
    for(i=0;i<8;i++)       
        for(j=0;j<8;j++)        
            set_led(Buttons[i][j]);             
    write_display();
    __delay_ms(50);
    for(i=0;i<64;i++)    
        clr_led(i);    
    write_display();
    
   while(1)
    {
        __delay_ms(30);
        if(read_switch())
        {
            for (h = 0; h < 64; h++) {
                if (is_key_pressed(h)&&(!was_key_pressed(h))) {
                    for (i = 0; i < 8; i++) { //right most button
                        if (h == Buttons[i][7]) {
                            rightbutton = (h - 19) / 4;//0.1.2.3.8.9.10.11
                        }
                    }
                    //decide button in which column
                    for (i = 0; i < 8; i++) {
                        for (j = 0; j < 8; j++) {
                            if (h == Buttons[i][j]) {
                                column = Buttons[0][j];//0.1.2.3.16.17.18.19
                            }
                        }
                    }

                    //push right most button, clear all led
                    if (column == 19) {
                        for (i = 0; i < 64; i++)
                            clr_led(i);
                        write_display();
                    }

                    if (rightbutton == 0 && column != 19) { //animation mode 1
                        for (i = 0; i < 14; i++) {
                            if (h == show0_1[i]) {
                                m = 1;
                                n = i;
                                break;
                            } else if (h == show0_1[39 - i]) {
                                m = 2;
                                n = i;
                                break;
                            } else if (h == show0_2[i]) {
                                m = 3;
                                n = i;
                                break;
                            } else if (h == show0_2[39 - i]) {
                                m = 4;
                                n = i;
                                break;
                            }
                        }
                        for (i = 0; i < 2; i++) {
                            for (j = 0; j < 8; j++) {
                                if (h == show0_3[i][j]) {
                                    m = 5;
                                    n = i;
                                    break;
                                }
                            }
                        }
                        animation0(m, n);
                    } else if (rightbutton == 1 && column != 19) {
                        //Serial.println("rightbutton is 1 ");
                        m = h / 16; //button is in which matrix
                        n = h % 16;
                        animation1(m, n);
                    } else if (rightbutton == 2 && column != 19) {
                        m = h / 16; //button is in which matrix
                        n = h % 16;
                        animation2(m, n, h);
                    } else if (rightbutton == 3 && column != 19) {
                        for (i = 0; i < 4; i++)
                            for (j = 0; j < 26; j++)
                                if(circle[i][j]!=100){
                                    if (h == circle[i][j]){
                                        m = i;
                                        goto A;
                                    }
                                }
                        A:
                        animation3(m);
                    } else if (rightbutton == 8 && column != 19) {
                        m = h / 16;
                        animation4(m, h);
                    } else if (rightbutton == 9 && column != 19) {

                        m = h / 16; //button is in which matrix
                        n = h % 16;
                        animation5(m, n, h);

                    } else if (rightbutton == 10 && column != 19) {
                        for (i = 0; i < 8; i++) {
                            for (j = 0; j < 8; j++) {
                                if (h == Buttons[i][j]) {
                                    m = i;
                                    n = j;
                                }
                            }
                        }
                        animation6(m, n);
                    } else if (rightbutton == 11 && column != 19) {
                        animation7(h);
                    }
                }
            }
            write_display();
        }
    }    
        
    while(1);   //end of program
    return;
}

void animation0(uint8_t m, uint8_t n) {
    uint8_t i = 0, j = 0;
    if (m == 1) {
        for (i = n; i < 40 - n; i++) {
            set_led(show0_1[i]);
            write_display();
            __delay_ms(40);
            clr_led(show0_1[i]);
            write_display();
        }
    } else if (m == 2) {
        for (i = 39 - n; i >= n; i--) {
            set_led(show0_1[i]);
            write_display();
            __delay_ms(40);
            clr_led(show0_1[i]);
            write_display();
        }
    } else if (m == 3) {
        for (i = n; i < 40 - n; i++) {
            set_led(show0_2[i]);
            write_display();
            __delay_ms(40);
            clr_led(show0_2[i]);
            write_display();
        }
    } else if (m == 4) {
        for (i = 39 - n; i >= n; i--) {
            set_led(show0_2[i]);
            write_display();
            __delay_ms(40);
            clr_led(show0_2[i]);
            write_display();
        }
    } else if (m == 5) {
        for (i = n; i < 6; i++) {
            for (j = 0; j < 20; j++) {
                if (show0_3[i][j] != 100)
                    set_led(show0_3[i][j]);
            }
            write_display();
            __delay_ms(50);
            for (j = 0; j < 20; j++) {
                if (show0_3[i][j] != 100)
                    clr_led(show0_3[i][j]);
            }
            write_display();
        }
    }
}

void animation1(uint8_t m, uint8_t n) {
    uint8_t i = 0, j = 0, x = 0;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            if (arrow[n][i][j] != 100) {
                x = arrow[n][i][j] + 16 * m;
                set_led(x);
            }
        }
        write_display();
        __delay_ms(50);
        for (j = 0; j < 4; j++) {
            if (arrow[n][i][j] != 100) {
                x = arrow[n][i][j] + 16 * m;
                clr_led(x);
            }
        }
        write_display();
    }
}

void animation2(uint8_t m, uint8_t n, uint8_t h) {
    int i = 0, j = 0, x = 0;
    int row = 0, col = n % 4;
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            if (h == Buttons[i][j]) {
                row = Buttons[i][0];
                break;
            }
        }
    }

    if (m == 0) {
        for (i = col; i < 8; i++) {
            for (j = 0; j < 3; j++)
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    set_led(x);
                }
            write_display();
            __delay_ms(30);
            for (j = 0; j < 3; j++)
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    clr_led(x);
                }
            write_display();
        }
    } else if (m == 1) {
        for (i = col + 2; i >= 0; i--) {
            for (j = 0; j < 3; j++) {
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    set_led(x);
                }
            }
            write_display();
            __delay_ms(40);
            for (j = 0; j < 3; j++)
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    clr_led(x);
                }
            write_display();
        }
    } else if (m == 2) {
        for (i = col; i < 7; i++) {
            for (j = 0; j < 3; j++)
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    set_led(x);
                }
            write_display();
            __delay_ms(40);
            for (j = 0; j < 3; j++)
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    clr_led(x);
                }
            write_display();
        }
    } else if (m == 3) {
        for (i = col + 2; i >= 0; i--) {
            for (j = 0; j < 3; j++)
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    set_led(x);
                }
            write_display();
            __delay_ms(40);
            for (j = 0; j < 3; j++)
                if (horizental_flash[i][j] != 100) {
                    x = horizental_flash[i][j] + row;
                    clr_led(x);
                }
            write_display();
        }
    }
}

void animation3(uint8_t m) {
    int i = 0, j = 0;
    if (m == 0) {
        for (i = 0; i < 10; i++) {
            for (j = 0; j < 8; j++) {
                if (show3_1[i][j] != 100)
                    set_led(show3_1[i][j]);
            }
            write_display();

            __delay_ms(30);
            for (j = 0; j < 8; j++) {
                if (show3_1[i][j] != 100)
                    clr_led(show3_1[i][j]);
            }
            write_display();
        }
    } else if (m == 1) {
        for (i = 0; i < 7; i++) {
            for (j = 0; j < 6; j++) {
                if (show3_2[i][j] != 100)
                    set_led(show3_2[i][j]);
            }
            write_display();
            __delay_ms(30);
            for (j = 0; j < 6; j++) {
                if (show3_2[i][j] != 100)
                    clr_led(show3_2[i][j]);
            }
            write_display();
        }
    } else if (m == 2) {
        for (i = 0; i < 5; i++) {
            for (j = 0; j < 4; j++) {
                if (show3_3[i][j] != 100)
                    set_led(show3_3[i][j]);
            }
            write_display();
            __delay_ms(30);
            for (j = 0; j < 4; j++) {
                if (show3_3[i][j] != 100)
                    clr_led(show3_3[i][j]);
            }
            write_display();
        }
    } else if (m == 3) {

        for (i = 3; i >= 0; i--) {
            for (j = 0; j < 26; j++) {
                if (circle[i][j] != 100)
                    set_led(circle[i][j]);
            }
            write_display();
            __delay_ms(30);
            for (j = 0; j < 26; j++) {
                if (circle[i][j] != 100)
                    clr_led(circle[i][j]);
            }
            write_display();
        }
        //return
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 26; j++) {
                if (circle[i][j] != 100)
                    set_led(circle[i][j]);
            }
            write_display();
            __delay_ms(40);
            for (j = 0; j < 26; j++) {
                if (circle[i][j] != 100)
                    clr_led(circle[i][j]);
            }
            write_display();
        }
    }
}

void animation4(uint8_t m, uint8_t h) {
    uint8_t i = 0, j = 0;
    set_led(h);
    write_display();
    if (m == 0) {
        for (i = 0; i < 8; i++) {
            for (j = 0; j < 8; j++)
                if (show4_1[i][j] != 100)
                    set_led(show4_1[i][j]);
            write_display();
            __delay_ms(40);

            for (j = 0; j < 8; j++)
                if (show4_1[i][j] != 100)
                    clr_led(show4_1[i][j]);
            write_display();
        }
    } else if (m == 1) {
        for (i = 0; i < 12; i++) {
            for (j = 0; j < 8; j++)
                if (show4_2[i][j] != 100)
                    set_led(show4_2[i][j]);
            write_display();
            __delay_ms(30);

            for (j = 0; j < 8; j++)
                if (show4_2[i][j] != 100)
                    clr_led(show4_2[i][j]);
            write_display();
        }
    } else if (m == 2) {
        for (i = 0; i < 12; i++) {
            for (j = 0; j < 8; j++)
                if (show4_2[i][j] != 100)
                    set_led(show4_3[i][j]);
            write_display();
            __delay_ms(30);

            for (j = 0; j < 8; j++)
                if (show4_2[i][j] != 100)
                    clr_led(show4_3[i][j]);
            write_display();
        }

    } else if (m == 3) {
        for (i = 0; i < 8; i++) {
            for (j = 0; j < 8; j++)
                if (show4_3[i][j] != 100)
                    set_led(show4_4[i][j]);
            write_display();
            __delay_ms(40);

            for (j = 0; j < 8; j++)
                if (show4_3[i][j] != 100)
                    clr_led(show4_4[i][j]);
            write_display();
        }
    }
    clr_led(h);
    write_display();
}

uint8_t temp[64] = {0};

void animation5(uint8_t m, uint8_t n, uint8_t h) {
    uint8_t i = 0, j = 0, x = 0;

    if (m == 0) {
        for (i = 0; i < 16; i++) {
            if (show5[i][0] == n) {
                for (j = 0; j < 4; j++) {
                    x = show5[i][j];
                    if (temp[x] == 0) {
                        set_led(x);
                        temp[x] = 1;
                    } else {
                        clr_led(x);
                        temp[x] = 0;
                    }
                }
                write_display();
                break;
            }
        }
    } else if (m == 1) {
        for (i = 0; i < 16; i++) {
            if (show5[i][0] == n) {
                for (j = 0; j < 4; j++) {
                    x = show5[i][j] + 16;
                    if (temp[x] == 0) {
                        set_led(x);
                        temp[x] = 1;
                    } else {
                        clr_led(x);
                        temp[x] = 0;
                    }
                }
                write_display();
                break;
            }
        }
    } else if (m == 2) {
        for (i = 0; i < 16; i++) {
            if (show5[i][0] == n) {
                for (j = 0; j < 4; j++) {
                    x = show5[i][j] + 32;
                    if (temp[x] == 0) {
                        set_led(x);
                        temp[x] = 1;
                    } else {
                        clr_led(x);
                        temp[x] = 0;
                    }
                }
                write_display();
                break;
            }
        }
    } else if (m == 3) {
        for (i = 0; i < 16; i++) {
            if (show5[i][0] == n) {
                for (j = 0; j < 4; j++) {
                    x = show5[i][j] + 48;
                    if (temp[x] == 0) {
                        set_led(x);
                        temp[x] = 1;
                    } else {
                        clr_led(x);
                        temp[x] = 0;
                    }
                }
                write_display();
                break;
            }
        }
    }
}

void animation6(uint8_t m, uint8_t n) {
    int a = m - 1, b = m + 1, c = n - 1, d = n + 1;

    set_led(Buttons[m][n]);
    write_display();
    __delay_ms(40);
    clr_led(Buttons[m][n]);
    write_display();

    for (uint8_t x = 0; x < 8; x++) {
        if (a >= 0)
            set_led(Buttons[a][n]);
        if (b < 8)
            set_led(Buttons[b][n]);
        if (c >= 0)
            set_led(Buttons[m][c]);
        if (d < 8)
            set_led(Buttons[m][d]);

        write_display();
        __delay_ms(40);
        clr_led(Buttons[a][n]);
        clr_led(Buttons[b][n]);
        clr_led(Buttons[m][c]);
        clr_led(Buttons[m][d]);
        write_display();
        a--;
        b++;
        c--;
        d++;
    }
}
int count = 0;

void animation7(uint8_t h) {
    int i = 0, j = 0;
    if (h == 15 || h == 28 || h == 35 || h == 48 || h == 10 || h == 11 || h == 24 || h == 25 || h == 14 || h == 34 || h == 38 || h == 39 || h == 52 || h == 53 || h == 29 || h == 49) {
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 29; j++) {
                if (show7_1[i][j] != 100)
                    set_led(show7_1[i][j]);
            }
            write_display();
            __delay_ms(40);
            for (j = 0; j < 29; j++)
                clr_led(show7_1[i][j]);
            write_display();
        }
    } else {

        if (count == 0) {
            for (i = 0; i < 9; i++) {
                for (j = 0; j < 24; j++) {
                    if (show7_2[i][j] != 100)
                        set_led(show7_2[i][j]);
                }
                write_display();
                __delay_ms(50);
                for (j = 0; j < 24; j++)
                    if (show7_2[i][j] != 100)
                        clr_led(show7_2[i][j]);
                write_display();
            }
            count = 1;

        } else if (count == 1) {
            for (i = 8; i >= 0; i--) {
                for (j = 0; j < 24; j++) {
                    if (show7_2[i][j] != 100)
                        set_led(show7_2[i][j]);
                }
                write_display();
                __delay_ms(50);
                for (j = 0; j < 24; j++)
                    if (show7_2[i][j] != 100)
                        clr_led(show7_2[i][j]);
                write_display();
            }
            count = 0;
        }
    }
}

void write_display()
{
    int i,j;
    for(i=0;i<4;i++)
    {
        MyIdleI2C();
        MyStartI2C();
        MyIdleI2C();      
        write(matrices[i].address | 0x00);
        MyIdleI2C();
        write(0x00);
        MyIdleI2C();
        //write display
        for(j=0;j<8;j++)
        {
            write(matrices[i].display_buffer[j]&0xff);
            write(matrices[i].display_buffer[j]>>8);
        }
        MyStopI2C();
    }
    
}

void clr_led(uint8_t i)
{
    int matrix=i>>4;
    int led=i&0b1111;
    matrices[matrix].display_buffer[ledLUT[led]>>4]&=~(1<<(ledLUT[led]&0x0f));   
}
void set_led(uint8_t i)
{
    int matrix=i>>4;
    int led=i&0b1111;
    matrices[matrix].display_buffer[ledLUT[led]>>4]|=(1<<(ledLUT[led]&0x0f));
}
int is_led(uint8_t x)
{
    int matrix=x>>4;
    int led=x&0b1111;
    return ((matrices[matrix].display_buffer[ledLUT[led] >> 4] & (1<<(ledLUT[led] & 0x0F))) > 0);
}
bool is_key_pressed(uint8_t k)
{
    int matrix=k>>4;
    int led=k&0b1111;
    return (matrices[matrix].keys[buttonLUT[led]>>4]&(1<<(buttonLUT[led]&0x0f)));
}
bool was_key_pressed(uint8_t k)
{
    int matrix=k>>4;
    int led=k&0b1111;
    return (matrices[matrix].last_keys[buttonLUT[led]>>4]&(1<<(buttonLUT[led]&0x0f)));
}
bool read_switch()
{
    int i,j;
    for(j=0;j<4;j++)
    {
        memcpy(matrices[j].last_keys,matrices[j].keys,sizeof(matrices[j].keys));
        //write command
        MyIdleI2C();
        MyStartI2C();
        MyIdleI2C();      
        write(matrices[j].address | 0x00);
        MyIdleI2C();   
        write(0x40);
        MyIdleI2C();
        MyStopI2C();
       //read key data
        MyIdleI2C();
        MyStartI2C();
        MyIdleI2C();
        write(matrices[j].address | 0x01);
        MyIdleI2C();
        while(MygetsI2C(matrices[j].keys,6));
        MyNotAckI2C();        
        MyStopI2C();

        for(i=0;i<6;i++)
        {
           if(matrices[j].keys[i]!=matrices[j].last_keys[i])
           {
               return true;
           }
       }
       
    }
    return false;
}
void MyStartI2C()
{
  SSPCON2bits.SEN = 1;            // initiate bus start condition
}
void MyStopI2C()
{
  SSPCON2bits.PEN = 1;            // initiate bus start condition
}
void MyCloseI2C()
{
  SSPCON1 &= 0xDF;                // disable synchronous serial port
}
void MyIdleI2C()
{
  while ( ( SSPCON2 & 0x1F ) || ( SSPSTATbits.R_W ) )
     continue;
}
signed char MyWriteI2C( unsigned char data_out )
{
  SSPBUF = data_out;           // write single byte to SSPBUF
  if ( SSPCON1bits.WCOL )      // test if write collision occurred
   return ( -1 );              // if WCOL bit is set return negative #
  else
  {
	if( ((SSPCON1&0x0F)!=0x08) && ((SSPCON1&0x0F)!=0x0B) )	//Slave mode only
	{
	      SSPCON1bits.CKP = 1;        // release clock line 
	      while ( !PIR1bits.SSPIF );  // wait until ninth clock pulse received

	      if ( ( !SSPSTATbits.R_W ) && ( !SSPSTATbits.BF ) )// if R/W=0 and BF=0, NOT ACK was received
	      {
	        return ( -2 );           //return NACK
	      }
		  else
		  {
			return ( 0 );				//return ACK
		  }	
	}
	else if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
	{ 
	    while( SSPSTATbits.BF );   // wait until write cycle is complete   
	    MyIdleI2C();                 // ensure module is idle
	    if ( SSPCON2bits.ACKSTAT ) // test for ACK condition received
	    	 return ( -2 );			// return NACK
		else return ( 0 );              //return ACK
	}
	
  }
}
signed char MyputsI2C( unsigned char *wrptr )
{
	signed char temp;
  while ( *wrptr )                // transmit data until null character 
  {
    if ( SSPCON1bits.SSPM3 )      // if Master transmitter then execute the following
    {
	  temp = MyWriteI2C ( *wrptr );
	  if (temp ) return ( temp );            // return with write collision error
    
//      if ( putcI2C ( *wrptr ) )   // write 1 byte
//      {
//        return ( -3 );            // return with write collision error
//      }
//      IdleI2C();                  // test for idle condition
//      if ( SSPCON2bits.ACKSTAT )  // test received ack bit state
//      {
//        return ( -2 );            // bus device responded with  NOT ACK
//      }                           // terminate putsI2C() function
    }

    else                          // else Slave transmitter
    {
      PIR1bits.SSPIF = 0;         // reset SSPIF bit
      SSPBUF = *wrptr;            // load SSPBUF with new data
      SSPCON1bits.CKP = 1;        // release clock line 
      while ( !PIR1bits.SSPIF );  // wait until ninth clock pulse received

      if ( ( SSPCON1bits.CKP ) && ( !SSPSTATbits.BF ) )// if R/W=0 and BF=0, NOT ACK was received
      {
        return ( -2 );            // terminate PutsI2C() function
      }
    }

  wrptr ++;                       // increment pointer

  }                               // continue data writes until null character

  return ( 0 );
}
void MyOpenI2C( unsigned char sync_mode, unsigned char slew )
{
  SSPSTAT &= 0x3F;                // power on state 
  SSPCON1 = 0x00;                 // power on state
  SSPCON2 = 0x00;                 // power on state
  SSPCON1 |= sync_mode;           // select serial mode 
  SSPSTAT |= slew;                // slew rate on/off 

  TRISCbits.TRISC3 = 1;
  TRISCbits.TRISC4 = 1;
  
  SSPCON1 |= SSPENB;              // enable synchronous serial port 

}
void MyRestartI2C( void )
{
  SSPCON2bits.RSEN = 1;           // initiate bus restart condition
}
signed char MygetsI2C( unsigned char *rdptr, unsigned char length )
{
    while ( length-- )           // perform getcI2C() for 'length' number of bytes
    {
      *rdptr++ = MyReadI2C();       // save byte received
      while ( SSPCON2bits.RCEN ); // check that receive sequence is over    

      if ( PIR2bits.BCLIF )       // test for bus collision
      {
        return ( -1 );            // return with Bus Collision error 
      }

	if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
	{	
      if ( length )               // test if 'length' bytes have been read
      {
        SSPCON2bits.ACKDT = 0;    // set acknowledge bit state for ACK        
        SSPCON2bits.ACKEN = 1;    // initiate bus acknowledge sequence
        while ( SSPCON2bits.ACKEN ); // wait until ACK sequence is over 
      } 
	} 
	  
    }
    return ( 0 );                 // last byte received so don't send ACK      
}
void MyNotAckI2C( void )
{
  SSPCON2bits.ACKDT = 1;          // set acknowledge bit for not ACK
  SSPCON2bits.ACKEN = 1;          // initiate bus acknowledge sequence
}
unsigned char MyReadI2C( void )
{
    if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
    SSPCON2bits.RCEN = 1;           // enable master for 1 byte reception
    while ( !SSPSTATbits.BF );      // wait until byte received  
    return ( SSPBUF );              // return with read byte 
}
void ISR_Init()
{
    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;              
    IPR1bits.SSPIP = 1;             
        
    RCONbits.IPEN = 1;      //enable Interrupt Priority mode
    INTCONbits.GIEH = 1;    //enable high priority interrupt
    INTCONbits.GIEL = 1;     //enable low priority interrupt            
    return ;
}
