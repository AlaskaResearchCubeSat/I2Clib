
#include <msp430.h>
#include <i2c.h>

//I2C structure for data passing
I2C_STAT I2C_stat;

//I2C data ISR, called to transmit or receive I2C data
//UART TX ISR called to transmit UART data
void I2C_dat(void) __ctl_interrupt[USCIAB1TX_VECTOR]{
  unsigned char flags=UC1IFG&(UC1IE);
//===============[I2C data Tx/Rx handler]================
  //check if data was received
  if(flags&UCB1RXIFG){
    //receive data
    I2C_stat.rx.ptr[I2C_stat.rx.idx]=UCB1RXBUF;
    //increment index
    I2C_stat.rx.idx++;
    //check if this is the 2nd to last byte
    if(I2C_stat.rx.len==(I2C_stat.rx.idx+1)){
      if(I2C_stat.mode==I2C_RXTX){
        //set transmit mode
        UCB1CTL1|=UCTR;
        //generate repeated start condition
        UCB1CTL1|=UCTXSTT;
        //enable Tx interrupt
        UC1IE|= UCB1TXIE;
      }else{
        //generate stop condition
        UCB1CTL1|=UCTXSTP;
      }
      //one more interrupt to go
    }
    //check if this was the last byte
    if(I2C_stat.rx.idx>=I2C_stat.rx.len && I2C_stat.mode!=I2C_RXTX){
      //set complete event
      ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
    }
  }
  //check if data needs to be transmitted
  if(flags&UCB1TXIFG){
    //check if there are more bytes
    if(I2C_stat.tx.len>I2C_stat.tx.idx){
      //transmit data
      UCB1TXBUF=I2C_stat.tx.ptr[I2C_stat.tx.idx++];
    }else{
      if(I2C_stat.mode==I2C_TXRX){
        //set receive mode
        UCB1CTL1&=~UCTR;
        //generate start condition
        UCB1CTL1|=UCTXSTT;
        //clear interrupt flag
        UC1IFG&= ~UCB1TXIFG;
        //clear Tx enable
        UC1IE&= ~UCB1TXIE;
        //enable Rx interrupt
        UC1IE|= UCB1RXIE;
        //one byte receive needs special treatment
        if(I2C_stat.rx.len==1){
          //set complete event
          ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
        }
      }else{
        //generate stop condition
        UCB1CTL1|=UCTXSTP;
        //clear interrupt flag
        UC1IFG&= ~UCB1TXIFG;
        //set complete event
        ctl_events_set_clear(&I2C_stat.events,I2C_EV_COMPLETE,0);
      }
    }
  }
//==============================================================

}

//I2C state ISR called when ack is not receive
//also receive UART ISR
void I2C_state(void) __ctl_interrupt[USCIAB1RX_VECTOR]{
  unsigned char flags=UC1IFG&(UC1IE);
  unsigned char I2Cstate=UCB1STAT;
//=================[I2C Status Handler]=============================
  if(I2Cstate&UCNACKIFG){
    //Not-Acknowledge received
#ifndef SCCB
    //clear Tx interrupt flag see USCI25 in "MSP430F261x, MSP430F241x Device Erratasheet (Rev. J)"
    UC1IFG&= ~UCB1TXIFG;
    //set NACK error event
    ctl_events_set_clear(&I2C_stat.events,I2C_EV_ERR_NACK,0);
#endif
    //clear interrupt flag
    UCB1STAT&=~UCNACKIFG;
#ifndef SCCB
    //disable I2C Tx and Rx Interrupts
    UC1IE&=~(UCB1TXIE|UCB1RXIE);
    //generate stop condition
    UCB1CTL1|=UCTXSTP;
#endif
  }
  if(I2Cstate&UCSTPIFG){
    //Stop condition received, not used in master mode
  }
  if(I2Cstate&UCSTTIFG){
    //Start condition received, not used in master mode
  }
  if(I2Cstate&UCALIFG){
    //Arbitration Lost, not used in master mode
  }
//==================================================================
}
