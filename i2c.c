#include <msp430.h>
#include <ctl.h>
#include "i2c.h"

//mutex for I2C resource
static CTL_MUTEX_t I2C_mutex;

//release the I2C bus
static release_I2C_bus(void){
  ctl_mutex_unlock(&I2C_mutex);
}

//reserve the I2C bus so no one else can use it
static unsigned I2C_lock(void){
  int i;
  //try to capture mutex
  if(0==ctl_mutex_lock(&I2C_mutex,CTL_TIMEOUT_DELAY,10)){
     return I2C_ERR_BUSY_TIMEOUT;
  }
  //wait for bus to be free
  for(i=0;i<10 && UCB1STAT&UCBBUSY;i++){
    //wait a bit
    ctl_timeout_wait(ctl_get_current_time()+3);
  }
  if(UCB1STAT&UCBBUSY){
    //release mutex
    release_I2C_bus();
    //bus is still busy, return error
    return I2C_ERR_BUSY_TIMEOUT;
  } 
  return 0;
}


//transmit len bytes pointed to by dat to address addr over i2c
short i2c_tx(unsigned short addr,const unsigned char *dat,unsigned short len){
  unsigned int e;
  int i;
  //check for zero length
  if(len==0){
    return I2C_ERR_LEN;
  }
  //wait for the bus to become free
  if(I2C_lock()){
    //I2C bus is in use
    return I2C_ERR_BUSY_TIMEOUT;
  }
  //set slave address
  UCB1I2CSA=addr;
  //set transmit mode
  UCB1CTL1|=UCTR;
  //clear TX interrupt flag
  UC1IFG&=~UCB1TXIFG;
  //disable RX interrupt
  UC1IE&=~UCB1RXIE;
  //enable TX interrupt
  UC1IE|= UCB1TXIE;
  //set index
  I2C_stat.tx.idx=0;
  //clear I2C events
  ctl_events_set_clear(&I2C_stat.events,0,I2C_EV_ALL);
  //set mode
  I2C_stat.mode=I2C_TX;
  //set length
  I2C_stat.tx.len=len;
  //set data
  I2C_stat.tx.ptr=(unsigned char*)dat;
  //generate start condition
  UCB1CTL1|=UCTXSTT;
  //wait for transaction to complete
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&I2C_stat.events,I2C_EV_ALL,CTL_TIMEOUT_DELAY,5);
  //release I2C lock
  release_I2C_bus();
  //check I2C done, for error
  if(e==I2C_EV_COMPLETE){
    //return number of bytes sent
    return I2C_stat.tx.idx;
  }else{
    //return error
    if(e&I2C_EV_ERR_NACK){
      //NACK by slave
      return I2C_ERR_NACK;
    }else{
      //events wait timed out
      return I2C_ERR_TIMEOUT;
    }
  }
}

//recive rxlen bytes over I2C then transmit txLen bytes 
short i2c_rxtx(unsigned short addr,unsigned char *rx,unsigned short rxLen,const unsigned char *tx,unsigned short txLen){
  unsigned int e;
  int i;
  //check for zero length
  if(rxLen==0 || txLen==0){
    return I2C_ERR_LEN;
  }
  //wait for the bus to become free
  if(I2C_lock()){
    //I2C bus is in use
    return I2C_ERR_BUSY_TIMEOUT;
  }
  //set slave address
  UCB1I2CSA=addr;
   //set receive mode
  UCB1CTL1&=~UCTR;
  //clear RX interrupt flag
  UC1IFG&=~UCB1RXIFG;
  //disable TX interrupt
  UC1IE&=~UCB1TXIE;
  //enable RX interrupt
  UC1IE|= UCB1RXIE;
  //set index
  I2C_stat.rx.idx=0;
  I2C_stat.tx.idx=0;
  //clear I2C events
  ctl_events_set_clear(&I2C_stat.events,0,I2C_EV_ALL);
  //set mode
  I2C_stat.mode=I2C_RXTX;
  //set Rx length
  I2C_stat.rx.len=rxLen;
  //set Tx length
  I2C_stat.tx.len=txLen;
  //set Rx data
  I2C_stat.rx.ptr=rx;
  //set Tx data
  I2C_stat.tx.ptr=tx;
  //clear I2C events
  ctl_events_set_clear(&I2C_stat.events,0,I2C_EV_ALL);
  //generate start condition
  UCB1CTL1|=UCTXSTT;
  //one byte packets are special
  if(rxLen==1){
    //wait for address to be sent
    while(UCB1CTL1&UCTXSTT);
    //transmit repeated start
    UCB1CTL1|=UCTXSTT;
    //go to LPM0 for Tx      
    //wait for transaction to complete
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&I2C_stat.events,I2C_EV_ALL,CTL_TIMEOUT_DELAY,5);
  }else{
    //wait for transaction to complete
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&I2C_stat.events,I2C_EV_ALL,CTL_TIMEOUT_DELAY,5);
  }
  //release I2C lock
  release_I2C_bus();
  //check I2Cdone for error
  if(e==I2C_EV_COMPLETE){
    //return number of bytes
    return I2C_stat.tx.idx+I2C_stat.rx.idx;
  }else{
    //return error
    if(e&I2C_EV_ERR_NACK){
      //NACK by slave
      return I2C_ERR_NACK;
    }else{
      //events wait timed out
      return I2C_ERR_TIMEOUT;
    }
  }
}

// transmit txLen bytes then recive rxlen bytes over I2C 
short i2c_txrx(unsigned short addr,const unsigned char *tx,unsigned short txLen,unsigned char *rx,unsigned short rxLen){
  unsigned int e;
  int i;
  //check for zero length
  if(rxLen==0 || txLen==0){
    return I2C_ERR_LEN;
  }
  //wait for the bus to become free
  if(I2C_lock()){
    //I2C bus is in use
    return I2C_ERR_BUSY_TIMEOUT;
  }
  //set slave address
  UCB1I2CSA=addr;
   //set transmit mode
  UCB1CTL1|=UCTR;
  //clear TX interrupt flag
  UC1IFG&=~UCB1TXIFG;
  //disable RX interrupt
  UC1IE&=~UCB1RXIE;
  //enable TX interrupt
  UC1IE|= UCB1TXIE;
  //set index
  I2C_stat.rx.idx=0;
  I2C_stat.tx.idx=0;
  //clear I2C events
  ctl_events_set_clear(&I2C_stat.events,0,I2C_EV_ALL);
  //set mode
  I2C_stat.mode=I2C_TXRX;
  //set Rx length
  I2C_stat.rx.len=rxLen;
  //set Tx length
  I2C_stat.tx.len=txLen;
  //set Rx data
  I2C_stat.rx.ptr=rx;
  //set Tx data
  I2C_stat.tx.ptr=tx;
  //clear I2C events
  ctl_events_set_clear(&I2C_stat.events,0,I2C_EV_ALL);
  //generate start condition
  UCB1CTL1|=UCTXSTT;
  //go to low power mode while transaction completes
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&I2C_stat.events,I2C_EV_ALL,CTL_TIMEOUT_DELAY,5);
  //release I2C lock
  release_I2C_bus();
  //check I2Cdone for error
  if(e==I2C_EV_COMPLETE){
    //return number of bytes
    return I2C_stat.tx.idx+I2C_stat.rx.idx;
  }else{
    //return error
    if(e&I2C_EV_ERR_NACK){
      //NACK by slave
      return I2C_ERR_NACK;
    }else{
      //events wait timed out
      return I2C_ERR_TIMEOUT;
    }
  }
}

//receive len bytes pointed to by dat from address addr over i2c
short i2c_rx(unsigned short addr,unsigned char *dat,unsigned short len){
  unsigned int e;
  int i;
  //check for zero length
  if(len==0){
    return I2C_ERR_LEN;
  }
  //wait for the bus to become free
  if(I2C_lock()){
    //I2C bus is in use
    return I2C_ERR_BUSY_TIMEOUT;
  }
  //set slave address
  UCB1I2CSA=addr;
  //set receive mode
  UCB1CTL1&=~UCTR;
  //clear RX interrupt flag
  UC1IFG&=~UCB1RXIFG;
  //disable TX interrupt
  UC1IE&=~UCB1TXIE;
  //enable RX interrupt
  UC1IE|= UCB1RXIE;
  //set index
  I2C_stat.rx.idx=0;
  //clear I2C events
  ctl_events_set_clear(&I2C_stat.events,0,I2C_EV_ALL);
  //set mode
  I2C_stat.mode=I2C_RX;
  //set length
  I2C_stat.rx.len=len;
  //set data
  I2C_stat.rx.ptr=dat;
  //generate start condition
  UCB1CTL1|=UCTXSTT;
  //one byte packets are special
  if(len==1){
    //wait for address to be sent
    while(UCB1CTL1&UCTXSTT);
    //transmit stop
    UCB1CTL1|=UCTXSTP;
    //wait for transaction to complete
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&I2C_stat.events,I2C_EV_ALL,CTL_TIMEOUT_DELAY,10);
  }else{    
    //wait for transaction to complete
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&I2C_stat.events,I2C_EV_ALL,CTL_TIMEOUT_DELAY,10);
  }
  //release I2C lock
  release_I2C_bus();
  //check I2Cdone for error
  if(e==I2C_EV_COMPLETE){
    //return number of bytes
    return I2C_stat.rx.idx;
  }else{
    //return error
    if(e&I2C_EV_ERR_NACK){
      //NACK by slave
      return I2C_ERR_NACK;
    }else{
      //events wait timed out
      return I2C_ERR_TIMEOUT;
    }
  }
}


//generate a clock on the I2C bus
//clock frequency is about 10kHz
void I2C_clk(void){
  //pull clock line low
  P5DIR|=BIT2;
  //wait for 0.05ms
  __delay_cycles(800);
  //realese clock line
  P5DIR&=~BIT2;
  //wait for 0.05ms
  __delay_cycles(800);
}

//Force an I2C device to release the bus by generating clocks
//this is needed with some I2C devices if the SSP is reset mid transaction
void I2C_reset(void){
  int i;
  //clock and data pins as inputs
  P5DIR&=~(BIT1|BIT2);
  //clock and data pins as GPIO
  P5SEL&=~(BIT1|BIT2);
  //set out bits to zero
  P5OUT&=~(BIT1|BIT2);
  //check if SDA is stuck low
  if(!(P5IN&BIT1)){
    //generate 9 clocks for good measure
    for(i=0;i<9;i++){
      I2C_clk();
    }
    //pull SDA low 
    P5DIR|=BIT1;
    //wait for 0.05ms
    __delay_cycles(800);
    //pull SCL low
    P5DIR|=BIT2;
    //wait for 0.05ms
    __delay_cycles(800);
    //realese SCL
    P5DIR&=~BIT2;
    //wait for 0.05ms
    __delay_cycles(800);
    //realese SDA
    P5DIR&=~BIT1;
  }//if SDA is not stuck low then SCL must be which means we are SOL
  //clock and data pins as I2C function
  P5SEL|=BIT1|BIT2;
}

void initI2C(void){
  //init mutex
  ctl_mutex_init(&I2C_mutex);
  //put UCB1 into reset state
  UCB1CTL1=UCSWRST;
  //setup registers
  UCB1CTL0=UCMST|UCMODE_3|UCSYNC;
  UCB1CTL1|=UCSSEL_2;
  //set baud rate to 50kB/s off of 16MHz SMCLK
  UCB1BR0=0x40;
  UCB1BR1=0x01;
  //configure ports
  P5SEL|=BIT1|BIT2;
  //bring UCB1 out of reset state
  UCB1CTL1&=~UCSWRST;
  //check if bus is busy
  if(UCB1STAT&UCBBUSY){
    //bit bang the bus until no device has hold of it
    I2C_reset();
  }
  //enable not-acknowledge interrupt
  UCB1I2CIE|=UCNACKIE;
}
