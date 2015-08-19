
#include <msp430.h>
#include <i2c.h>

//I2C structure for data passing
I2C_STAT I2C_stat;

//I2C data ISR, called to transmit or receive I2C data
//UART TX ISR called to transmit UART data
void I2C_dat(void) __ctl_interrupt[USCI_B1_VECTOR]{
  switch(UCB1IV){
    case USCI_I2C_UCNACKIFG:
      //Not-Acknowledge received
#ifndef SCCB
      //set NACK error event
      ctl_events_set_clear(&I2C_stat.events,I2C_EV_ERR_NACK,0);
#endif
      //clear interrupt flag
      UCB1STAT&=~UCNACKIFG;
#ifndef SCCB
      //generate stop condition
      UCB1CTLW0|=UCTXSTP;
#endif
    return;
    case USCI_I2C_UCRXIFG0:
      //receive data
      I2C_stat.rx.ptr[I2C_stat.rx.idx]=UCB1RXBUF;
      //increment index
      I2C_stat.rx.idx++;
      //check if this is the 2nd to last byte
      if(I2C_stat.rx.len==(I2C_stat.rx.idx+1)){
        if(I2C_stat.mode==I2C_RXTX){
          //set transmit mode
          UCB1CTLW0|=UCTR;
          //generate repeated start condition
          UCB1CTLW0|=UCTXSTT;
        }else{
          //generate stop condition
          UCB1CTLW0|=UCTXSTP;
        }
        //one more interrupt to go
      }
      //check if this was the last byte
      if(I2C_stat.rx.idx>=I2C_stat.rx.len && I2C_stat.mode!=I2C_RXTX){
        //set complete event
        ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
      }
    break;
    case USCI_I2C_UCTXIFG0:
      //check if there are more bytes
      if(I2C_stat.tx.len>I2C_stat.tx.idx){
        //transmit data
        UCB1TXBUF=I2C_stat.tx.ptr[I2C_stat.tx.idx++];
      }else{
        if(I2C_stat.mode==I2C_TXRX){
          //set receive mode
          UCB1CTLW0&=~UCTR;
          //generate start condition
          UCB1CTLW0|=UCTXSTT;
          //one byte receive needs special treatment
          if(I2C_stat.rx.len==1){
            //set complete event
            ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
          }
        }else{
          //generate stop condition
          UCB1CTLW0|=UCTXSTP;
          //set complete event
          ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
        }
      }
    break;
  }  
}
