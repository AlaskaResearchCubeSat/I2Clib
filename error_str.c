#include "i2c.h"


//return error strings for error code
const char *I2C_error_str(int error){
  //positive values indicate success
  if(error>=0){
    return "SUCCESS";
  }
  //check for error
  switch(error){
    case I2C_ERRR_NACK:
      return "ERROR NACK";
    case I2C_ERR_TIMEOUT:
      return "ERROR TIME OUT";
    case I2C_ERR_LEN:
      return "ERROR LEN";
    case I2C_ERR_BUSY_TIMEOUT:
      return "ERROR BUSY TIME OUT";
    //Error was not found
    default:
      return "UNKNOWN ERROR";
  }
}
