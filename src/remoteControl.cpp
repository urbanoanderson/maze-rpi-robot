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

        	float leftSpeed = 0.0;
        	float rightSpeed = 0.0;
        	float motionSpeed = 80.0;
        	float speedIncrement = 1.0;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	//Controle de movimentos
		    	if(APIGetKey() == 'o')
		    	{
		    		leftSpeed = motionSpeed;
		    		rightSpeed = motionSpeed;
		    	}

		    	else if(APIGetKey() == 'l')
		    	{
		    		leftSpeed = -motionSpeed;
		    		rightSpeed = -motionSpeed;
		    	}

		    	else if(APIGetKey() == 'k')
		    	{
		    		leftSpeed = -motionSpeed;
		    		rightSpeed = motionSpeed;
		    	}

		    	else if(APIGetKey() == ';')
		    	{
		    		leftSpeed = motionSpeed;
		    		rightSpeed = -motionSpeed;
		    	}

		    	//Parar o robô
		    	else if(APIGetKey() == ' ')
		    	{
		    		leftSpeed = 0.0;
		    		rightSpeed = 0.0;
		    	}

		    	//Aumentar / Reduzir velocidade
		    	else if(APIGetKey() == 'u')
		    		motionSpeed += speedIncrement;

		    	else if(APIGetKey() == 'j')
		    		motionSpeed -= speedIncrement;

		    	//Tirar Foto
		    	else if(APIGetKey() == 'p')
		    	{
		    		printf("\rTaking picture...\n");
		    		APISavePicture(APIReadCamera());
		    	}

		    	printf("\rSpeed: %.1f | Sonars:[%.2f, %.2f, %.2f]\n", motionSpeed, APIReadSonarLeft(), APIReadSonarFront(), APIReadSonarRight());

		    	//APISetRobotSpeed(leftSpeed, rightSpeed);
		    	APISetMotorPower(leftSpeed, rightSpeed);

		        //Espera um tempo antes da próxima atualização
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