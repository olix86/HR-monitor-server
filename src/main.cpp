#include "main.h" 
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
enum States {wait, freefall, impact, rest};

void setupAccelero() {
	// initialize device
	printf("Initializing I2C devices...\n");
	accelgyro.initialize();
	
	// verify connection
	printf("Testing device connections...\n");
	printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
}

void readAccelero() {
	// read raw accel/gyro measurements from device
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	
	// these methods (and a few others) are also available
	//accelgyro.getAcceleration(&ax, &ay, &az);
	//accelgyro.getRotation(&gx, &gy, &gz);
	
	// display accel/gyro x/y/z values
	double scaling_factor = 16384;
	//printf("a/g: %6hd %6hd %6hd   %6hd %6hd %6hd\n",(double)ax,(double)ay,(double)az,gx,gy,gz);
	printf("a/g: %f %f %f   %6hd %6hd %6hd\n",(double)ax/scaling_factor,(double)ay/scaling_factor,(double)az/scaling_factor,gx,gy,gz);
}

States fsm()
{
	static float g=9.81;
	static float T = 0.01;
	static float maxFallTime = 1.0;
	static float maxImpactTime = 1.0;
	static float maxRestTime = 1.0;
	static States state = wait;
	
	static float waitImpact = 0;
	static float waitRest = 0;
	static float restCounter = 0;
	double a[3];
	static double norm = 0;
	// read raw accel/gyro measurements from device
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	
	// these methods (and a few others) are also available
	//accelgyro.getAcceleration(&ax, &ay, &az);
	//accelgyro.getRotation(&gx, &gy, &gz);
	
	// display accel/gyro x/y/z values
	const double scaling_factor = 16384;
	a[0] = (double)ax/ scaling_factor;
	a[1] = (double)ay/ scaling_factor;
	a[2] = (double)az/ scaling_factor;
	
	//printf("a/g: %6hd %6hd %6hd   %6hd %6hd %6hd\n",(double)ax,(double)ay,(double)az,gx,gy,gz);
	printf("a/g: %f %f %f   %6hd %6hd %6hd\n",(double)ax/scaling_factor,(double)ay/scaling_factor,(double)az/scaling_factor,gx,gy,gz);
	printf("%d \n", state);
	
	
	
	norm = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	if (state == wait)
	{	if(norm <(0.5*g))
		{
			state = freefall;
		}
		else
		{
			state = wait;
		}
	}
	else if(state == freefall)
	{	
		if(norm >(3.0*g))
		{
			state = impact;
			waitImpact = 0;
		}
		else if(waitImpact > maxFallTime/T)
		{
			state = wait;
			waitImpact = 0;
		}
		else:
		{
			waitImpact = waitImpact + 1;
			state = freefall;
		}
	}
	else if(state == impact)
	{
		if(norm < (1.5*g))
		{
			state = rest;
			waitRest = 0;
		}
		else if(waitRest > maxImpactTime/T)
		{
			state = wait;
			waitRest = 0;
		}
		else
		{
			waitRest = waitRest + 1;
			state = impact;
		}
	}
	//State rest
	else
	{
		if(restCounter > maxRestTime/T)
		{
			state = wait;
			restCounter = 0;
		}
		else
		{
			state = rest;
			restCounter = restCounter + 1;
		}
	}
	
}




int main(){
	
	MAX30100* pulseOxymeter;
	
	//pulseOxymeter = new MAX30100( DEFAULT_OPERATING_MODE, DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH,DEFAULT_IR_LED_CURRENT, true, true );
	
	pulseOxymeter = new MAX30100( MAX30100_MODE_HR_ONLY, DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH, MAX30100_LED_CURRENT_24MA, true, false );
	//pulseOxymeter = new MAX30100();
	
	printf("begin \n");
	
	setupAccelero();
	
	//You have to call update with frequency at least 37Hz. But the closer you call it to 100Hz the better, the filter will work.
	//int i = 5;
	while(1)
	{
		pulseoxymeter_t result = pulseOxymeter->update();
		//printf("update \n");
		
		FILE *f = fopen("HB.txt", "w");
		if (f == NULL)
		{
			printf("Error opening file!\n");
			exit(1);
		}
		
		if( result.pulseDetected == true )
		{
			//printf("BEAT \n");
			printf( "BPM: " );
			printf("%f \n", result.heartBPM );
			/*printf( " | " );
			 *			
			 *			printf( "SaO2: " );
			 *			printf("%f", result.SaO2 ); */
			//printf( "% \n" );
			
			fprintf(f,"%f\n", result.heartBPM);
		}
		else
		{
			//printf("no pulse detected \n");  
			fprintf(f,"0\n");
			
		}
		
		fclose(f);
		
		//readAccelero();
		fsm();
		//takes microseconds as arg
		usleep(10*1000);
		//i--;
	}
	//pulseOxymeter->printRegisters();
	
	printf("end \n");
	return 0;	
}

