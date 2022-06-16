//==============================================================================
//
//  CRYSTALFONTZ 
//
//  This code drives the CFAL12856A0-0151-B display
//  https://www.crystalfontz.com/product/cfal12856a00151b
//
//  The controller is a Solomon Systech SSD1309
//    https://www.crystalfontz.com/controllers/SolomonSystech/SSD1309/
//
//  Seeeduino v4.2, an open-source 3.3v capable Arduino clone.
//    https://www.seeedstudio.com/Seeeduino-V4.2-p-2517.html
//    https://github.com/SeeedDocument/SeeeduinoV4/raw/master/resources/Seeeduino_v4.2_sch.pdf
//
//==============================================================================
//This is free and unencumbered software released into the public domain.
//
//Anyone is free to copy, modify, publish, use, compile, sell, or
//distribute this software, either in source code form or as a compiled
//binary, for any purpose, commercial or non-commercial, and by any
//means.
//
//In jurisdictions that recognize copyright laws, the author or authors
//of this software dedicate any and all copyright interest in the
//software to the public domain. We make this dedication for the benefit
//of the public at large and to the detriment of our heirs and
//successors. We intend this dedication to be an overt act of
//relinquishment in perpetuity of all present and future rights to this
//software under copyright law.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
//OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//OTHER DEALINGS IN THE SOFTWARE.
//
//For more information, please refer to <http://unlicense.org/>
//==============================================================================
// LCD & USD control lines
//   ARD      | Port  |Display tail|  Function - 8080 Parallel   |  Function - SPI                            |	Function - I2C                  
//------------+-------+------------+-----------------------------+--------------------------------------------+
//  N/A	      |       | 23         |  POWER 12.5V                |  POWER 12V                                 |	POWER 12V  
//  3.3V      |       | 5          |  POWER 3.3V                 |  POWER 3.3V                                |	POWER 3.3V 
//  GND	      |       | 1-3, 24    |  GROUND                     |  GROUND                                    |	GROUND                
// -----------+-------+------------+-----------------------------+--------------------------------------------+
//  N/A       | N/A   | 6          |  BS1 - 3.3V                 |  BS1 - GND                                 |	BS1 - 3.3V
//  N/A       | N/A   | 7          |  BS2 - 3.3V                 |  BS2 - GND                                 |	BS2 - GND
// -----------+-------+------------+-----------------------------+--------------------------------------------+
//  A0        | PORTC | 10         |  Data/Command        (DC)   |  Data/Command (pull high for 3-wire) (DC)  |	SA0
//  A1        | PORTC | 11         |  Write               (WR)   |  N/A pull high                             |	N/A pull high    
//  A2        | PORTC | 12         |  Read                (RD)   |  N/A pull high                             |	N/A pull high     
//  D8        | PORTB | 8          |  Chip Enable Signal  (CS)   |  Chip Enable Signal                  (CS)  |	Chip Enable Signal (CS)
//  D9        | PORTB | 9          |  Reset            (RESET)   |  Reset                            (RESET)  |	Reset         (RESET)
// -----------+-------+------------+-----------------------------+--------------------------------------------+
// PARALLEL ONLY
// -----------+-------+------------+-----------------------------+--------------------------------------------+
//  D0        | PORTD | 13         |  LCD_D10 (DB0)              |                                            |
//  D1        | PORTD | 14         |  LCD_D11 (DB1)              |                                            |
//  D2        | PORTD | 15         |  LCD_D12 (DB2)              |                                            |
//  D3        | PORTD | 16         |  LCD_D13 (DB3)              |                                            |
//  D4        | PORTD | 17         |  LCD_D14 (DB4)              |                                            |
//  D5        | PORTD | 18         |  LCD_D15 (DB5)              |                                            |
//  D6        | PORTD | 19         |  LCD_D16 (DB6)              |                                            |
//  D7        | PORTD | 20         |  LCD_D17 (DB7)              |                                            |
// -----------+-------+------------+-----------------------------+--------------------------------------------+
// SPI ONLY
// -----------+-------+------------+-----------------------------+--------------------------------------------+
//  D13       | PORTD | 13         |                             |  SCLK                                      
//  D11       | PORTD | 14         |                             |  SDIN                                      
//  D2        | PORTD | 15         |                             |  No Connection                             
//  D3        | PORTD | 16         |                             |  N/A pull high                             
//  D4        | PORTD | 17         |                             |  N/A pull high                             
//  D5        | PORTD | 18         |                             |  N/A pull high                             
//  D6        | PORTD | 19         |                             |  N/A pull high                             
//  D7        | PORTD | 20         |                             |  N/A pull high                             
// -----------+-------+------------+-----------------------------+--------------------------------------------+
// I2C ONLY
// -----------+-------+------------+-----------------------------+--------------------------------------------+
//  A5        | PORTC | 13         |                             |                                            |  SCL                                      
//  A4        | PORTC | 14         |                             |                                            |  SDA                                          
//  A4        | PORTC | 15         |                             |                                            |  SDA                             
// -----------+-------+------------+-----------------------------+--------------------------------------------+
// Consult the datasheet for more interface options
//==============================================================================
//
// There are additional components that should be connected and you need an
// external 12.5v VOLED supply.
// Please consult the datasheet for information about these components.
//
// Alternatively, you can use a CFA10105 denonstration board which contains all
// the components and has a build-in VOLED supply.
//
//==============================================================================
//  BS0,BS1 interface settings:
//  
//      Interface         | BS1 | BS2 
//  ----------------------+-----+-----
//    I2C                 |  1  |  0  
//    4-wire SPI          |  0  |  0  
//    8-bit 6800 Parallel |  0  |  1  
//    8-bit 8080 Parallel |  1  |  1  
//
//  This code is demonstrated using 8080 Parallel, I2C, or 4-wire SPI
//  The CFA10105 breakout board is default SPI. To load code for the breakout
//  uncomment the define for SPI below and comment out the define for I2C.
//==============================================================================
//  Select the interface
#define SPI_4_WIRE
//#define PAR_8080	  
//#define I2C
//------------------------------------------------------------------------------
//Allow the serial debugging monitor, but only if SPI (pin conflict with parallel)
#define SER_DEBUG
#ifdef PAR_8080
  #undef SER_DEBUG
#endif // PAR_8080
//==============================================================================
#define CLR_CS     (PORTB &= ~(0x01)) //pin #8  - Chip Enable Signal
#define SET_CS     (PORTB |=  (0x01)) //pin #8  - Chip Enable Signal
#define CLR_RESET  (PORTB &= ~(0x02)) //pin #12 - Reset
#define SET_RESET  (PORTB |=  (0x02)) //pin #12 - Reset
#define CLR_DC     (PORTC &= ~(0x01)) //pin #9  - Data/Instruction
#define SET_DC     (PORTC |=  (0x01)) //pin #9  - Data/Instruction
#define CLR_WR	   (PORTC &= ~(0x02)) //pin #10 - Write
#define SET_WR	   (PORTC |=  (0x02)) //pin #10 - Write
#define CLR_RD	   (PORTC &= ~(0x04)) //pin #11 - Read
#define SET_RD	   (PORTC |=  (0x04)) //pin #11 - Read
#define CLR_DBG	   (PORTC &= ~(0x08)) //pin #12 - Debug
#define SET_DBG	   (PORTC |=  (0x08)) //pin #12 - Debug

#include <avr/io.h>
#ifdef SPI_4_WIRE
  #include <SPI.h>
#endif
#ifdef I2C
  #include <Wire.h>
#endif
#include "CFAL12856A0_0151_B_Splash.h"
//================================================================================
#define MAX_BRIGHT (0x8F)
//================================================================================
#ifdef SPI_4_WIRE
//================================================================================
void writeCommand(uint8_t command)
  {
  // Select the LCD's command register
  CLR_DC;
  // Select the LCD controller
  CLR_CS;

  //Send the command via SPI:
  SPI.transfer(command);
  //deselect the controller
  SET_CS;
  }
//================================================================================
void writeData(uint8_t data)
  {
  //Select the LCD's data register
  SET_DC;
  //Select the LCD controller
  CLR_CS;
  //Send the command via SPI:
  SPI.transfer(data);

  // Deselect the LCD controller
  SET_CS;
  }
#endif
//================================================================================

#ifdef I2C
//================================================================================
void writeCommand(uint8_t command)
{
	Wire.beginTransmission(0x3C); //send start & Slave address
	Wire.write(0x00);		          //Control Byte - Command
	Wire.write(command);	        //payload
	Wire.endTransmission();

}
//============================================================================
void writeData(uint8_t data)
{
	Wire.beginTransmission(0x3C);	//send start & Slave address
	Wire.write(0xC0);		          //Control Byte - Data (non-continued)
	Wire.write(data);          //payload
	Wire.endTransmission();
}
#endif
//================================================================================

#ifdef PAR_8080
void writeCommand(uint8_t command)
  {
  //select the LCD's command register
  CLR_DC;
  //select the LCD controller
  CLR_CS;
  //send the data via parallel
  PORTD = command;
  //clear the write register
  CLR_WR;
  //set the write register
  SET_WR;
  //deselct the LCD controller
  SET_CS;
  }
//================================================================================
void writeData(uint8_t data)
  {
  //select the LCD's data register
  SET_DC;
  //select the LCD controller
  CLR_CS;
  //send the data via parallel
  PORTD = data;
  //clear the write register
  CLR_WR;
  //set the write register
  SET_WR;
  //deselct the LCD controller
  SET_CS;
  }
//================================================================================
#endif

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Instruction Setting
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Set_Start_Column(uint8_t d)
  {
  writeCommand(0x00 + d % 16);		// Set Lower Column Start Address for Page Addressing Mode
            //   Default => 0x00
  writeCommand(0x10 + d / 16);		// Set Higher Column Start Address for Page Addressing Mode
            //   Default => 0x10
  }
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Set_Column_Address(uint8_t a, uint8_t b)
  {
  writeCommand(0x21);			// Set Column Address
  writeCommand(a);			//   Default => 0x00 (Column Start Address)
  writeCommand(b);			//   Default => 0x7F (Column End Address)
  }
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Set_Page_Address(uint8_t a, uint8_t b)
  {
  writeCommand(0x22);			// Set Page Address
  writeCommand(a);			//   Default => 0x00 (Page Start Address)
  writeCommand(b);			//   Default => 0x07 (Page End Address)
  }
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Set_Start_Page(uint8_t d)
  {
  writeCommand(0xB0 | d);			// Set Page Start Address for Page Addressing Mode
            //   Default => 0xB0 (0x00)
  }
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fill_RAM(uint8_t Data)
  {
  uint8_t
    i;
  uint8_t
    j;

  for (i = 0; i < 8; i++)
    {
    Set_Start_Page(i);
    Set_Start_Column(0x00);

    for (j = 0; j < 128; j++)
      {
      writeData(Data);
      }
    }
  }
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fill_RAM_CheckerBoard(void)
  {
  uint8_t
    page;
  uint8_t
    column;

  for(page = 0; page < 8; page++)
    {
    Set_Start_Page(page);
    Set_Start_Column(0x00);

    for (column= 0; column < 128; column++)
      {
      if(0 == (column&0x01))
        {
        writeData(0x55);
        }
      else
        {
        writeData(0xAA);
        }
      }
    }
  }
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define XLevelL    0x00
#define XLevelH    0x10
#define XLevel     ((XLevelH&0x0F)*16+XLevelL)
#define HRES       128
#define VRES       56
#define Brightness 0xBF

void OLED_Init()
  {
  //The CFA10105 has a power-on reset circuit, 
  //you can use the following code if you are using GPIO for reset
  CLR_RESET;
  delay(1);
  SET_RESET;
  delay(120);

  writeCommand(0xFD);	// Set Command Lock
  writeCommand(0X12);	//   Default => 0x12
                      //     0x12 => Driver IC interface is unlocked from entering command.
                      //     0x16 => All Commands are locked except 0xFD.

  writeCommand(0XAE);	// Set Display On/Off
                      //   Default => 0xAE
                      //     0xAE => Display Off
                      //     0xAF => Display On

                            
  writeCommand(0xD5);	// Set Display Clock Divide Ratio / Oscillator Frequency
//  writeCommand(0XA0);	// Set Clock as 116 Frames/Sec
  writeCommand(0X30);  // Set Clock as 116 Frames/Sec
                      //   Default => 0x70
                      //     D[3:0] => Display Clock Divider
                      //     D[7:4] => Oscillator Frequency

  writeCommand(0xA8);	// Set Multiplex Ratio
  writeCommand(0X37); //   Default => 0x3F (1/56 Duty)

  writeCommand(0xD3);	// Set Display Offset
  writeCommand(0X08);	//   Default => 0x00

  writeCommand(0x40); // Set Mapping RAM Display Start Line (0x00~0x3F)
                      //   Default => 0x40 (0x00)

  //writeCommand(0xD8);	// Set Low Power Display Mode (0x04/0x05)
  //writeCommand(0x05);	//   Default => 0x04 (Normal Power Mode)

  writeCommand(0xA1); // Set SEG/Column Mapping (0xA0/0xA1)
                      //   Default => 0xA0
                      //     0xA0 => Column Address 0 Mapped to SEG0
                      //     0xA1 => Column Address 0 Mapped to SEG127

  writeCommand(0xC8); // Set COM/Row Scan Direction (0xC0/0xC8)
                      //   Default => 0xC0
                      //     0xC0 => Scan from COM0 to 63
                      //     0xC8 => Scan from COM63 to 0

  writeCommand(0xDA); // Set COM Pins Hardware Configuration
  writeCommand(0x12); //   Default => 0x12
                      //     Alternative COM Pin Configuration
                      //     Disable COM Left/Right Re-Map

  writeCommand(0x81); // Set SEG Output Current
  writeCommand(0x8F); // Set Contrast Control for Bank 0

  writeCommand(0xD9); // Set Pre-Charge as 2 Clocks & Discharge as 5 Clocks
  writeCommand(0x25); //   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
                      //     D[3:0] => Phase 1 Period in 1~15 Display Clocks
                      //     D[7:4] => Phase 2 Period in 1~15 Display Clocks
  
  writeCommand(0xDB); // Set VCOMH Deselect Level
  writeCommand(0x34); //   Default => 0x34 (0.78*VCC)

  writeCommand(0xA4); // Set Entire Display On / Off
                      //   Default => 0xA4
                      //     0xA4 => Normal Display
                      //     0xA5 => Entire Display On

  writeCommand(0xA6); // Set Inverse Display On/Off
                      //   Default => 0xA6
                      //     0xA6 => Normal Display
                      //     0xA7 => Inverse Display On

  Fill_RAM(0x00);			// Clear Screen

  writeCommand(0XAF); // Display On (0xAE/0xAF)
  }
//================================================================================
// showSplash() takes an image out of flash and puts it on the screen. In this case,
// the image stored in flash is the splash screen
//================================================================================
void showImage(const uint8_t image[7][128])
  {
  //The logo fits in the first 7 pages (7x8=56)
  for (uint8_t y = 0; y < 7; y++)
    {
    // Set the starting page and column
    Set_Start_Page(y);
    Set_Start_Column(0x00);
    for (uint8_t x = 0; x < 128; x++)
      {
      writeData(pgm_read_byte(&image[y][x]));
      }
    }
  //Clear the last page so stray pixels do not show in getter area.
  Set_Start_Page(7);
  Set_Start_Column(0x00);
  for (uint8_t x = 0; x < 128; x++)
    {
    writeData(0x00);
    }
  }
//================================================================================
void setup()
  {
  //Set up port directions
  DDRD = 0xff;
  DDRC = 0xff;
  DDRB = 0x03;
  //Idle the lines in a reasonable state
  PORTD = 0xff;
  SET_RD;
  SET_WR;
  SET_CS;

#ifdef SER_DEBUG
  Serial.begin(9600);
  Serial.println();
  Serial.println("Serial Initialized");
#endif // SER_DEBUG

#ifdef SPI_4_WIRE
  //SPI begin transactions takes ~2.5us
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
#endif
			  
#ifdef I2C
  //
  //Get the I2C going
  Wire.begin();
  //Thanks to Limor / Adafruit for the hint ;-)   
  TWBR = 12; // upgrade to 400KHz!
#endif
 
  //OLED_Init takes ~120ms
  OLED_Init();
#ifdef SER_DEBUG
  Serial.println("Init done");
#endif // SER_DEBUG
  }
//================================================================================
void loop()
  {
#ifdef SER_DEBUG
  Serial.println("top of loop");
#endif // SER_DEBUG

  // Put up our splash screen
  showImage(Splash);
  delay(2000);
  
  //Demonstrate dimming
  writeCommand(0x81); // Set SEG Output Current
  writeCommand(0); // Set Contrast Control for Bank 0
  delay(1000);

  Fill_RAM(0xff);
  delay(1000);

  //Back to full brightness
  writeCommand(0x81); // Set SEG Output Current
  writeCommand(MAX_BRIGHT); // Set Contrast Control for Bank 0
  delay(500);

  //Put up some demo screens
  Fill_RAM_CheckerBoard();
  delay(1000);
  showImage(Eagle);
  delay(2000);
  Fill_RAM(0x00);
  delay(500);

  //Do a quick animation
  showImage(Tank_Animation_01);
  delay(70);
  showImage(Tank_Animation_02);
  delay(70);
  showImage(Tank_Animation_03);
  delay(70);
  showImage(Tank_Animation_04);
  delay(70);
  showImage(Tank_Animation_05);
  delay(70);
  showImage(Tank_Animation_06);
  delay(70);
  showImage(Tank_Animation_07);
  delay(70);
  showImage(Tank_Animation_08);
  delay(70);
  showImage(Tank_Animation_09);
  delay(70);
  showImage(Tank_Animation_10);
  delay(80);
  showImage(Tank_Animation_11);
  delay(90);
  showImage(Tank_Animation_12);
  delay(110);
  showImage(Tank_Animation_13);
  delay(130);
  showImage(Tank_Animation_14);
  delay(160);
  showImage(Tank_Animation_15);
  delay(200);
  showImage(Tank_Animation_16);
  delay(800);

  //Put up the "aiming" mock-up
  showImage(Aiming);
  delay(3000);

  //Finished
  Fill_RAM_CheckerBoard();
  delay(1000);

  //showImage(SuperTank);
  //delay(1000);

//  Fill_RAM(0x00);
//  delay(1500);
//  Fill_RAM(0xff);
//  delay(1500);
//  Fill_RAM(0x0f);
//  delay(1500);
//  showImage(Splash);
//  delay(3000);
  }
//================================================================================
