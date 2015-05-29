#ifndef __I2C_H
#define __I2C_H

//needed for event sets
#include <ctl.h>

//I2C events
enum {I2C_EV_COMPLETE=1<<0,I2C_EV_ERR_NACK=1<<1};
//I2C modes
enum {I2C_IDLE=0,I2C_TX=1,I2C_RX,I2C_TXRX,I2C_RXTX};
//I2C return values
enum{I2C_ERR_NACK=-1,I2C_ERR_TIMEOUT=-2,I2C_ERR_LEN=-3,I2C_ERR_BUSY_TIMEOUT=-4};
//All I2C events
#define I2C_EV_ALL      (I2C_EV_COMPLETE|I2C_EV_ERR_NACK)

//Internal I2C structure to keep track of things
typedef struct{
  //struct for Rx stuff
  struct {
    //pointer for recived data
    unsigned char *ptr;
    //length and index
    unsigned short len,idx;
  }rx;
  //Struct for Tx stuff
  struct {
    //data pointer
    const unsigned char *ptr;
    //length and index
    unsigned short len,idx;
  }tx;
  //I2C mode 
  unsigned short mode;
  //Event set
  CTL_EVENT_SET_t events;
}I2C_STAT;

extern I2C_STAT I2C_stat;

//functions

//Recive I2C data
short i2c_rx(unsigned short addr,unsigned char *dat,unsigned short len);
//Transmit I2C data
short i2c_tx(unsigned short addr,const unsigned char *dat,unsigned short len);
//recive then transmit I2C data
short i2c_rxtx(unsigned short addr,unsigned char *rx,unsigned short rxLen,const unsigned char *tx,unsigned short txLen);
//Transmit then recieve I2C data
short i2c_txrx(unsigned short addr,const unsigned char *tx,unsigned short txLen,unsigned char *rx,unsigned short rxLen);
//Setup I2C interface
void initI2C(unsigned int port,unsigned int sda,unsigned int scl);
//bit-bang I2C bus to try to force a slave to let go
void I2C_reset(void);
//return error strings for error code
const char * I2C_error_str(int error);

#endif
  