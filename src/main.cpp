#include "MAX30100.h"




int main(){
	
	MAX30100* pulseOxymeter;
	
	//pulseOxymeter = new MAX30100( DEFAULT_OPERATING_MODE, DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH, DEFAULT_IR_LED_CURRENT, true, true );
	pulseOxymeter = new MAX30100();
	
	printf("begin \n");
	//You have to call update with frequency at least 37Hz. But the closer you call it to 100Hz the better, the filter will work.
	int i = 5;
	while(i > 0)
	{
		pulseoxymeter_t result = pulseOxymeter->update();
		ol
		//printf("update \n");
		
		if( result.pulseDetected == true )
		{
			printf("BEAT \n");
			
			printf( "BPM: " );
			printf("%u", result.heartBPM );
			printf( " | " );
			
			printf( "SaO2: " );
			printf("%u", result.SaO2 );
			printf( "% \n" );
		}
		else
		{
			printf("no pulse detected \n");  
			
			
			
		}
		sleep(0.01)
		i--;
	}
	
	printf("end \n");
	return 0;	
}
