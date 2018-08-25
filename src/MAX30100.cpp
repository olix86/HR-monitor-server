

#include "MAX30100.h"

// fill-in for arduino millis()
unsigned long millis(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts );
    return ( ts.tv_sec * 1000 + ts.tv_nsec / 1000000L );
}

MAX30100::MAX30100(
        Mode mode, 
        SamplingRate samplingRate, 
        LEDPulseWidth pulseWidth, 
        LEDCurrent IrLedCurrent,
        bool highResMode,
        bool debug)
{
  this->debug = debug;
  currentPulseDetectorState = PULSE_IDLE;

  setMode(  mode );

  //Check table 8 in datasheet on page 19. You can't just throw in sample rate and pulse width randomly. 100hz + 1600us is max for that resolution
  setSamplingRate( samplingRate );
  setLEDPulseWidth( pulseWidth );

  redLEDCurrent = (uint8_t)STARTING_RED_LED_CURRENT;
  lastREDLedCurrentCheck = 0;

  this->IrLedCurrent = IrLedCurrent;
  setLEDCurrents(redLEDCurrent, IrLedCurrent );
  setHighresModeEnabled(highResMode);


  dcFilterIR.w = 0;
  dcFilterIR.result = 0;

  dcFilterRed.w = 0;
  dcFilterRed.result = 0;


  lpbFilterIR.v[0] = 0;
  lpbFilterIR.v[1] = 0;
  lpbFilterIR.result = 0;

  meanDiffIR.index = 0;
  meanDiffIR.sum = 0;
  meanDiffIR.count = 0;


  valuesBPM[0] = 0;
  valuesBPMSum = 0;
  valuesBPMCount = 0;
  bpmIndex = 0;
  

  irACValueSqSum = 0;
  redACValueSqSum = 0;
  samplesRecorded = 0;
  pulsesDetected = 0;
  currentSaO2Value = 0;

  lastBeatThreshold = 0;

}

pulseoxymeter_t MAX30100::update()
{
  pulseoxymeter_t result = {
    /*bool pulseDetected*/ false,
    /*float heartBPM*/ 0.0,
    /*float irCardiogram*/ 0.0,
    /*float irDcValue*/ 0.0,
    /*float redDcValue*/ 0.0,
    /*float SaO2*/ currentSaO2Value,
    /*uint32_t lastBeatThreshold*/ 0,
    /*float dcFilteredIR*/ 0.0,
    /*float dcFilteredRed*/ 0.0
  };

  
  fifo_t rawData = readFIFO();  
  
  dcFilterIR = dcRemoval( (float)rawData.rawIR, dcFilterIR.w, ALPHA );
  dcFilterRed = dcRemoval( (float)rawData.rawRed, dcFilterRed.w, ALPHA );

  float meanDiffResIR = meanDiff( dcFilterIR.result, &meanDiffIR);
  lowPassButterworthFilter( meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR );

  irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
  redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
  samplesRecorded++;

  if( detectPulse( lpbFilterIR.result ) && samplesRecorded > 0 )
  {
    result.pulseDetected=true;
    pulsesDetected++;

    float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

    if( debug == true )
    {
      printf("RMS Ratio: ");
      printf("%u \n",ratioRMS);
    }

    //This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration	
    currentSaO2Value = 110.0 - 18.0 * ratioRMS;
    result.SaO2 = currentSaO2Value;
    
    if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
    {
      irACValueSqSum = 0;
      redACValueSqSum = 0;
      samplesRecorded = 0;
    }
  }

  balanceIntesities( dcFilterRed.w, dcFilterIR.w );


  result.heartBPM = currentBPM;
  result.irCardiogram = lpbFilterIR.result;
  result.irDcValue = dcFilterIR.w;
  result.redDcValue = dcFilterRed.w;
  result.lastBeatThreshold = lastBeatThreshold;
  result.dcFilteredIR = dcFilterIR.result;
  result.dcFilteredRed = dcFilterRed.result;  
  
  
  return result;
}

bool MAX30100::detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {

        if(debug == true) 
        {
          printf("Peak reached: ");
          printf("%u \n",sensor_value);
          printf(" ");
          printf("%u \n",prev_sensor_value);
        }

        uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;
        if(debug == true) 
          printf("%u \n",rawBPM);

        //This method sometimes glitches, it's better to go through whole moving average everytime
        //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
        //valuesBPMSum -= valuesBPM[bpmIndex];
        //valuesBPM[bpmIndex] = rawBPM;
        //valuesBPMSum += valuesBPM[bpmIndex];

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;
        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }

        if(debug == true) 
        {
          printf("CurrentMoving Avg: ");
          for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
          {
            printf("%u",(valuesBPM[i]));
            printf(" ");
          }
  
          printf("\n");
        }

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;
        if(debug == true) 
        {
          printf("AVg. BPM: ");
          printf("%u \n",(currentBPM));
        }


        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}

void MAX30100::balanceIntesities( float redLedDC, float IRLedDC )
{
  
  if( millis() - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS) 
  {
    //Serial.println( redLedDC - IRLedDC );
    if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA) 
    {
      redLEDCurrent++;
      setLEDCurrents( redLEDCurrent, IrLedCurrent );
      if(debug == true) 
        printf("RED LED Current + \n");
    } 
    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0) 
    {
      redLEDCurrent--;
      setLEDCurrents( redLEDCurrent, IrLedCurrent );
      if(debug == true) 
        printf("RED LED Current - \n");
    }

    lastREDLedCurrentCheck = millis();
  }
}




void MAX30100::writeRegister(byte address, byte val)
{
	int file_i2c;
	int length;
	unsigned char buffer[60] = {0};
	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return;
	}
	
	//int addr = MAX30100_DEVICE //0x5a;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, MAX30100_DEVICE) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return;
	}
	
	//----- WRITE BYTES -----
	buffer[0] = val;
	length = 1;			//<<< Number of bytes to write
	if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
	}
}

uint8_t MAX30100::readRegister(uint8_t address)
{
	int file_i2c;
	int length = 1;
	uint8_t buffer;
	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return -1;
	}
	
	//int addr = MAX30100_DEVICE //0x5a;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, MAX30100_DEVICE) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return -1;
	}
	
	
	//----- READ BYTES -----
	//length = 1;			//<<< Number of bytes to read
	if (read(file_i2c, &buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
	}
	else
	{
		printf("Data read: %u \n", buffer);
	}
	return buffer;
}

void MAX30100::readFrom(byte address, int length, byte _buff[])
{
	int file_i2c;
	
	//unsigned char buffer[60] = {1};
	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return;
	}
	
	//int addr = MAX30100_DEVICE //0x5a;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, MAX30100_DEVICE) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return;
	}
	
	
	//----- READ BYTES -----
	//length = num;			//<<< Number of bytes to read
	if (read(file_i2c, _buff, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
	}
	else
	{
//		buffer[59] = '\0';
		printf("readFrom: ");
		for(int i=0; i<length; i++)
		{
			printf("%u", _buff[i]);
		}
		printf("\n");
	}
}
void MAX30100::setMode(Mode mode)
{
  byte currentModeReg = readRegister( MAX30100_MODE_CONF );
  writeRegister( MAX30100_MODE_CONF, (currentModeReg & 0xF8) | mode );
}

void MAX30100::setHighresModeEnabled(bool enabled)
{
    uint8_t previous = readRegister(MAX30100_SPO2_CONF);
    if (enabled) {
        writeRegister(MAX30100_SPO2_CONF, previous | MAX30100_SPO2_HI_RES_EN);
    } else {
        writeRegister(MAX30100_SPO2_CONF, previous & ~MAX30100_SPO2_HI_RES_EN);
    }
}

void MAX30100::setSamplingRate(SamplingRate rate)
{
  byte currentSpO2Reg = readRegister( MAX30100_SPO2_CONF );
  writeRegister( MAX30100_SPO2_CONF, ( currentSpO2Reg & 0xE3 ) | (rate<<2) );
}

void MAX30100::setLEDPulseWidth(LEDPulseWidth pw)
{
  byte currentSpO2Reg = readRegister( MAX30100_SPO2_CONF );
  writeRegister( MAX30100_SPO2_CONF, ( currentSpO2Reg & 0xFC ) | pw );
}

void MAX30100::setLEDCurrents( byte redLedCurrent, byte IRLedCurrent )
{
  writeRegister( MAX30100_LED_CONF, (redLedCurrent << 4) | IRLedCurrent );
}

float MAX30100::readTemperature()
{
  byte currentModeReg = readRegister( MAX30100_MODE_CONF );
  writeRegister( MAX30100_MODE_CONF, currentModeReg | MAX30100_MODE_TEMP_EN );

  sleep(0.1); //This can be changed to a while loop, there is an interrupt flag for when temperature has been read.

  int8_t temp = (int8_t)readRegister( MAX30100_TEMP_INT );
  float tempFraction = (float)readRegister( MAX30100_TEMP_FRACTION ) * 0.0625;

  return (float)temp + tempFraction;
}

fifo_t MAX30100::readFIFO()
{
  fifo_t result;

  byte buffer[4];
  readFrom( MAX30100_FIFO_DATA, 4, buffer );
  result.rawIR = (buffer[0] << 8) | buffer[1];
  result.rawRed = (buffer[2] << 8) | buffer[3];

  return result;
}

dcFilter_t MAX30100::dcRemoval(float x, float prev_w, float alpha)
{
  dcFilter_t filtered;
  filtered.w = x + alpha * prev_w;
  filtered.result = filtered.w - prev_w;

  return filtered;
}

void MAX30100::lowPassButterworthFilter( float x, butterworthFilter_t * filterResult )
{  
  filterResult->v[0] = filterResult->v[1];

  //Fs = 100Hz and Fc = 10Hz
  filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);

  //Fs = 100Hz and Fc = 4Hz
  //filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter 

  filterResult->result = filterResult->v[0] + filterResult->v[1];
}

float MAX30100::meanDiff(float M, meanDiffFilter_t* filterValues)
{
  float avg = 0;

  filterValues->sum -= filterValues->values[filterValues->index];
  filterValues->values[filterValues->index] = M;
  filterValues->sum += filterValues->values[filterValues->index];

  filterValues->index++;
  filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

  if(filterValues->count < MEAN_FILTER_SIZE)
    filterValues->count++;

  avg = filterValues->sum / filterValues->count;
  return avg - M;
}

void MAX30100::printRegisters()
{
	//HEX removed
  printf("%u \n",readRegister(MAX30100_INT_STATUS));
  printf("%u \n",readRegister(MAX30100_INT_ENABLE));
  printf("%u \n",readRegister(MAX30100_FIFO_WRITE));
  printf("%u \n",readRegister(MAX30100_FIFO_OVERFLOW_COUNTER));
  printf("%u \n",readRegister(MAX30100_FIFO_READ));
  printf("%u \n",readRegister(MAX30100_FIFO_DATA));
  printf("%u \n",readRegister(MAX30100_MODE_CONF));
  printf("%u \n",readRegister(MAX30100_SPO2_CONF));
  printf("%u \n",readRegister(MAX30100_LED_CONF));
  printf("%u \n",readRegister(MAX30100_TEMP_INT));
  printf("%u \n",readRegister(MAX30100_TEMP_FRACTION));
  printf("%u \n",readRegister(MAX30100_REV_ID));
  printf("%u \n",readRegister(MAX30100_PART_ID));
}

