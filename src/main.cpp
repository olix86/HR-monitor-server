#include "MAX30100.h"

int main(){
	
	MAX30100* pulseOxymeter;
	
	//pulseOxymeter = new MAX30100( DEFAULT_OPERATING_MODE, DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH,DEFAULT_IR_LED_CURRENT, true, true );
	
	pulseOxymeter = new MAX30100( MAX30100_MODE_HR_ONLY, DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH, MAX30100_LED_CURRENT_24MA, true, false );
	//pulseOxymeter = new MAX30100();
	
	printf("begin \n");
	//You have to call update with frequency at least 37Hz. But the closer you call it to 100Hz the better, the filter will work.
	//int i = 5;
	while(1)
	{
		pulseoxymeter_t result = pulseOxymeter->update();
		//printf("update \n");
		
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
			
			
			FILE *f = fopen("HB.txt", "w");
			if (f == NULL)
			{
				printf("Error opening file!\n");
				exit(1);
			}
			fprintf(f,%f\n", result.heartBPM);
			fclose(f);
			
		}
		else
		{
			//printf("no pulse detected \n");  
			
		}
		//takes microseconds
		usleep(10*1000);
		//i--;
	}
	//pulseOxymeter->printRegisters();
	
	printf("end \n");
	return 0;	
}
