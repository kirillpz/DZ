#include <wiringPi.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

#define I2C_SLAVE	  0x0703            
#define I2C_SMBUS   0x0720           

#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

#define I2C_SMBUS_BYTE_DATA	    2 
#define I2C_SMBUS_WORD_DATA	    3

#define I2C_SMBUS_BLOCK_MAX	32        //максимальный размер блока

#define BMP180_Address 0x77           //статический адрес датчика

//Operating Modes
#define BMP180_ULTRALOWPOWER    0 
#define BMP180_STANDARD         1
#define BMP180_HIGHRES          2
#define BMP180_ULTRAHIGHRES     3

//BMP185 Registers
#define BMP180_CAL_AC1          0xAA  //Калибровочный регистр(16 bits)
#define BMP180_CAL_AC2          0xAC  //Калибровочный регистр(16 bits)
#define BMP180_CAL_AC3          0xAE  //Калибровочный регистр(16 bits)
#define BMP180_CAL_AC4          0xB0  //Калибровочный регистр(16 bits)
#define BMP180_CAL_AC5          0xB2  //Калибровочный регистр(16 bits)
#define BMP180_CAL_AC6          0xB4  //Калибровочный регистр(16 bits)
#define BMP180_CAL_B1           0xB6  //Калибровочный регистр(16 bits)
#define BMP180_CAL_B2           0xB8  //Калибровочный регистр(16 bits)
#define BMP180_CAL_MB           0xBA  //Калибровочный регистр(16 bits)
#define BMP180_CAL_MC           0xBC  //Калибровочный регистр(16 bits)
#define BMP180_CAL_MD           0xBE  //Калибровочный регистр(16 bits)
#define BMP180_CONTROL          0xF4  //Регистр управления измерениями (8 bits)
#define BMP180_TEMPDATA         0xF6  //Регистр хранящий измеренные данные
#define BMP180_PRESSUREDATA     0xF6  //Регистр хранящий старщий байт измеренных данных

//Команды
#define BMP180_READTEMPCMD      0x2E  //Считать температуру
#define BMP180_READPRESSURECMD  0x34  //Считать давление с базоваой точностью

#define OSS BMP180_STANDARD           //Биты точности

short AC1,AC2,AC3,B1,B2,MB,MC,MD;     //Объявлеине калировочных коэффицентов
unsigned short AC4,AC5,AC6;
int fd;                               //Объявлеине переменной номера устройства


//I2C function code 

// создание блока данных
union i2c_smbus_data
{
  uint8_t  byte;
  uint16_t word;
  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2];	
};
// создание передаваемой структуры
struct i2c_smbus_ioctl_data
{
  char read_write;
  uint8_t command;
  int size;
  union i2c_smbus_data *data;                
};

//функция формирования структуры и передачи
static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args;

  args.read_write = rw;
  args.command    = command;
  args.size       = size;
  args.data       = data;
  return ioctl (fd, I2C_SMBUS, &args);
}


//  Считывание 8-битного значения из регистратора на устройстве
int I2CRead (int fd, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data))
    return -1;
  else
    return data.byte & 0xFF;
}


// Запишите в данный регистр 8-или 16-битное значение
int I2CWrite (int fd, int reg, int value)
{
  union i2c_smbus_data data;

  data.byte = value;
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data) ;
}

// Инициализация интерфейса 
int I2CSetupInterface (const char *device, int devId)
{
  int fd;

  if ((fd = open (device, O_RDWR)) < 0)     // Создание файла устройства
    return wiringPiFailure (WPI_ALMOST, "Unable to open I2C device: %s\n", strerror (errno));

  if (ioctl (fd, I2C_SLAVE, devId) < 0)     // Конфигурирование адрес ведомого устройства
    return wiringPiFailure (WPI_ALMOST, "Unable to select I2C device: %s\n", strerror (errno));

  return fd;
}


// Проверка платы и инициализация интерфейса
int I2CSetup (const int devId)
{
  int rev;
  const char *device;

  rev = piGpioLayout ();                   // Проверка версии Raspberry Pi   

  if (rev == 1)
    device = "/dev/i2c-0";
  else
    device = "/dev/i2c-1";

  return I2CSetupInterface (device, devId);
}

// BMP180 code
//Считывание 1 байта 
char I2C_readByte(int reg)
{
    return (char)I2CRead(fd,reg);
}
//Считывание 2 байтов 
unsigned short I2C_readU16(int reg)
{
    int MSB,LSB;
    MSB = I2C_readByte(reg);
    LSB = I2C_readByte(reg + 1);
    int value = (MSB << 8) +LSB;
    return (unsigned short)value;
}
//Считывание 2 байтов с преобразованием в short
short I2C_readS16(int reg)
{
    int result;
    result = I2C_readU16(reg);
    if (result > 32767)result -= 65536;
    return (short)result;
}

//Запись байта 
void I2C_writeByte(int reg,int val)
{
    I2CWrite(fd,reg,val);
}

// Считываение калибровочных данных
void load_calibration()
{
    AC1 = I2C_readS16(BMP180_CAL_AC1);
    AC2 = I2C_readS16(BMP180_CAL_AC2);
    AC3 = I2C_readS16(BMP180_CAL_AC3);
    AC4 = I2C_readU16(BMP180_CAL_AC4);
    AC5 = I2C_readU16(BMP180_CAL_AC5);
    AC6 = I2C_readU16(BMP180_CAL_AC6);
    B1  = I2C_readS16(BMP180_CAL_B1);
    B2  = I2C_readS16(BMP180_CAL_B2);
    MB  = I2C_readS16(BMP180_CAL_MB);
    MC  = I2C_readS16(BMP180_CAL_MC);
    MD  = I2C_readS16(BMP180_CAL_MD);
}
// Считываение "Сырых" данных температуры
int read_raw_temp()
{
    int raw;
    I2C_writeByte(BMP180_CONTROL,BMP180_READTEMPCMD);
    delay(5);  //5ms;
    raw = I2C_readByte(BMP180_TEMPDATA) << 8;
    raw += I2C_readByte(BMP180_TEMPDATA+1);
    return raw;

}
// Считываение "Сырых" данных давления
int read_raw_pressure()
{
    int MSB,LSB,XLSB,raw;
    I2C_writeByte(BMP180_CONTROL,BMP180_READPRESSURECMD +(OSS << 6));
    switch(OSS)
    {
        case BMP180_ULTRALOWPOWER:
            delay(5);break;
        case BMP180_HIGHRES:
            delay(14);break;
        case BMP180_ULTRAHIGHRES:
            delay(26);break;
        default :
            delay(8);
    }
    MSB  = I2C_readByte(BMP180_PRESSUREDATA);
    LSB  = I2C_readByte(BMP180_PRESSUREDATA + 1);
    XLSB = I2C_readByte(BMP180_PRESSUREDATA + 2);
    raw = ((MSB << 16) + (LSB << 8) + XLSB) >> (8 - OSS);
    return raw;
}
// Считываение данных температуры и обработка
float read_temperature()
{
    float T;
    int UT,X1,X2,B5;
    UT = read_raw_temp();
    X1 = ((UT - AC6)*AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    T = ((B5 + 8) >> 4) /10.0;
    return T;
}
// Считываение данных давления и обработка
int read_pressure()
{
    int P;
    int UT,UP,X1,X2,X3,B3,B5,B6;
    unsigned int B4;
    int B7;
    UT = read_raw_temp();
    UP = read_raw_pressure();

    X1 = ((UT - AC6)*AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;

    //Pressure Calculations
    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6) >> 12) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (X3 + 32768)) >> 15;
    B7 = (UP - B3) * (50000 >> OSS);
    if (B7 < 0x80000000){P = (B7 * 2) / B4;}
    else {P = (B7 / B4) * 2;}
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + 3791) >> 4);
    return P;

}
// Рассчет высоты над уровнем моря
float read_altitude()
{
    float pressure,altitude;
    float sealevel_pa = 101325.0;
    pressure = (float)read_pressure();
    altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa,(1.0/5.255)));
    return altitude;
}
// Рассчет давление на уровне моря
float read_sealevel_pressure()
{
    float altitude_m = 0.0;
    float pressure,p0;
    pressure =(float)read_pressure();
    p0 = pressure / pow(1.0 - altitude_m/44330.0,5.255);
    return p0;
}

// main function
int main(int argc,char **argv)
{
    int old_time = 0, interval = 500;

    if(wiringPiSetup() < 0)
        return 1;

    fd = I2CSetup(BMP180_Address); //Инициализация  протокола
    load_calibration();            //Считывание калибровочных коэффициентов

    while(1)
    {
        if ((millis() - old_time) > interval)        //считывание данных каждые 0,5 с.
        {
            printf("\n********************\n");
            printf("Temperature : %.2f C\n", read_temperature());
            printf("Pressure :    %.2f Pa\n", read_pressure()/100.0);
            printf("Altitude :    %.2f h\n", read_altitude());
            old_time = millis();
        }

    }
    return 0;
}
