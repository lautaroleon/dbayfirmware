#include "AD5684.h"


DAC_AD5684::DAC_AD5684(uint8_t ex_numdac, 
                       PCA9557 *BoardSel, 
                       int CSPin){

  this->_BoardSel = BoardSel;
  _CSPin = CSPin;

 if (!_BoardSel->pinMode(_CSPin,OUTPUT)) {
      sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", _BoardSel->boardN);
      Serial.print(_err);
      //rv = -1;
  }

  if (!_BoardSel->digitalWrite(_CSPin, HIGH)) {
        sprintf(_err, "Error setting address pin for board %i HIGH\n", _BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
   }   
  num_dacs=ex_numdac;
  
  dacs_volts=(float *)malloc(sizeof(float)*ex_numdac*4);
  dacsref=(float *)malloc(sizeof(float)*ex_numdac);

  if(dacs_volts == NULL) {
  //Serial.print("malloc failed!\n");  
  exit(1); 
  }

  if(dacsref == NULL) {
  //Serial.print("mallocd failed!\n");   // could also call perror here
  exit(1);
  } 

  delay(100);

  /////DAISY CHAIN ENABLE///////
  data_buff[0]=0x00 | (DCEN<<4);
  data_buff[1]=0x00;
  data_buff[2]=0x01;
  
  /*Serial.print("DCEN: ");
  Serial.print(data_buff[0],HEX);
  Serial.print(data_buff[1],HEX);
  Serial.println(data_buff[3],HEX);
*/
  
  SPI.beginTransaction(SPISettings(SPI_BAUD_AD5684, MSBFIRST, SPI_MODE_AD5684));
  _BoardSel->digitalWrite(_CSPin, LOW);
    SPI.transfer(data_buff[0]);
    SPI.transfer(data_buff[1]);
    SPI.transfer(data_buff[2]);
  _BoardSel->digitalWrite(_CSPin, HIGH);
  SPI.endTransaction();

   /////POWER on///////
  data_buff[0]=0x00 | (PWR_UPDWN<<4);
  data_buff[1]=0x00;
  data_buff[2]=0x00;
  SPI.beginTransaction(SPISettings(SPI_BAUD_AD5684, MSBFIRST, SPI_MODE_AD5684));
  _BoardSel->digitalWrite(_CSPin, LOW);
    SPI.transfer(data_buff[0]);
    SPI.transfer(data_buff[1]);
    SPI.transfer(data_buff[2]);
  _BoardSel->digitalWrite(_CSPin, HIGH);
  SPI.endTransaction();

   /////Reference set up///////
 
  data_buff[0]=0x00 | (IN_REF<<4);
  data_buff[1]=0x00;
  data_buff[2]=0x00;
  SPI.beginTransaction(SPISettings(SPI_BAUD_AD5684, MSBFIRST, SPI_MODE_AD5684));
  _BoardSel->digitalWrite(_CSPin, LOW);
    SPI.transfer(data_buff[0]);
    SPI.transfer(data_buff[1]);
    SPI.transfer(data_buff[2]);
  _BoardSel->digitalWrite(_CSPin, HIGH);
  SPI.endTransaction();

  
}

DAC_AD5684::~DAC_AD5684(){
  
}

int DAC_AD5684::set_V(int dac, int outChan, double value){
 int volt_set = 0;
 if(dac<num_dacs+1 && dacsref!=NULL)volt_set = (int)(value*4095/dacsref[dac-1]);
 Serial.println(value);
 //Serial.println(dacsref[dac-1]);
Serial.println(volt_set,HEX);

 data_buff[0]= (0x01<<(outChan)) | (WRITE_UPDATED_DAC<<4);
 data_buff[1]=volt_set>>4;
 data_buff[2]=0xF0 & volt_set<<4;

 SPI.beginTransaction(SPISettings(SPI_BAUD_AD5684, MSBFIRST, SPI_MODE_AD5684));
 _BoardSel->digitalWrite(_CSPin, LOW);
 if(dac<num_dacs+1){
 
    Serial.print("vset "); Serial.print("\t");
    Serial.print(data_buff[0],HEX); Serial.print("\t");
    Serial.print(data_buff[1],HEX); Serial.print("\t");
    Serial.println(data_buff[2],HEX);

    SPI.transfer(data_buff[0]);
    SPI.transfer(data_buff[1]);
    SPI.transfer(data_buff[2]);
      
  /*  for(int i=1; i<dac;i++){
      
        no_op[0] = 0xFF; //this because transfer is destructive

      Serial.print("NOOP ");Serial.print("\t");
        Serial.print(no_op[0],HEX);Serial.print("\t");
        Serial.print(no_op[1],HEX);Serial.print("\t");
        Serial.println(no_op[2],HEX);
        SPI.transfer(no_op,3);
    }*/
  
 }
 //delay(1);
 _BoardSel->digitalWrite(_CSPin, HIGH);
 SPI.endTransaction();
 return 1;
 
}
 
int DAC_AD5684::set_ref(int dac, float ref){
  if(dac<num_dacs+1 && dac && dacsref!=NULL){
    dacsref[dac-1]=ref;
    /*Serial.print("ref ");
    Serial.print(dac);
    Serial.print("  value: ");
    Serial.println(ref);*/
  } 
 // else Serial.print("ref DAC error ");
return 0;
}
