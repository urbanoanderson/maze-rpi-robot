#include "RobotAPI.h"
#include "Robot.h"

#define INIT_FILENAME "ini/robot.ini"
           
int main(int argc, char* argv[])
{   
	//Se conseguiu conectar com a API
    if (APIInitConection())
    {           
        //Inicializa o Robô
        Robot monstrinho(INIT_FILENAME);
       
       	//Começa a simulação
        if(APIStartSimulation())
        {
        	printf("\rSimulação iniciada.\n");
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning() && !monstrinho.FinishedWork())
		    {    		    
                //Atualiza o robô
                monstrinho.Update();
		    	
                //Printa o log do robô
                monstrinho.Log();
	 
                //Espera um tempo antes da próxima atualização
                APIWait();
		    }
		    //---------------------------------------------------------
		   
		   	printf("\rFim da simulação.\n\r");
		    APIFinishSimulation();
        } 
        
        else
        	printf("\rNão foi possível iniciar a simulação.\n\r");
    } 
    
    else
        printf("\rNão foi possível conectar.\n\r");
   
    return 0;
}