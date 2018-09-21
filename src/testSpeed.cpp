#include "RobotAPI.h"
#include "Utils.h"
           
int main(int argc, char* argv[])
{   
	//Se conseguiu conectar com a API
    if (APIInitConection())
    {
        printf("\rConexão efetuada.\n");
       
       	//Começa a simulação
        if(APIStartSimulation())
        {
        	printf("\rSimulação iniciada.\n");

        	float leftSpeed = 15.0;
        	float rightSpeed = 15.0;
        	float stepSpeed = 0.5;
        	bool startTest = false;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	printf("\rSpace to begin; Q to quit. Speed [Left | Right]: [%.2f | %.2f]\n", leftSpeed, rightSpeed);

		    	if(APIGetKey() == 'i')
		    		leftSpeed += stepSpeed;
		    	else if(APIGetKey() == 'k')
		    		leftSpeed -= stepSpeed;
		    	else if(APIGetKey() == 'o')
		    		rightSpeed += stepSpeed;
		    	else if(APIGetKey() == 'l')
		    		rightSpeed -= stepSpeed;
		    	else if(APIGetKey() == ' ')
		    		startTest = true;

		    	if(startTest)
		    	{
		    		float deltaS = 0.0;
		    		float beginTime = APIGetSimulationTimeInSecs();

		    		while(APISimulationIsRunning() && (APIGetSimulationTimeInSecs() - beginTime < 2.0))
		    		{
		    			deltaS += APIReadOdometers();
		    			APISetRobotSpeed(leftSpeed, rightSpeed);
		    			APIWait();
		    		}

		    		APIStopRobot();
		    		startTest = false;

		    		printf("\n\rΔS: %f\n\n", deltaS);
		    		APIWaitMsecs(3000);
		    	}

		    	APIWait();
		    }
		    //---------------------------------------------------------
		   
		   	printf("\rFim da simulação.\n\r");
		   
		    //Para o robô e desconecta
		    APIFinishSimulation();
        } 
        
        else
        	printf("\rNão foi possível iniciar a simulação.\n\r");
    } 
    
    else
        printf("\rNão foi possível conectar.\n\r");
   
    return 0;
}