#include "RobotAPI.h"
#include "Utils.h"

void PushCommand(std::vector< std::vector<float> >* commands, float leftSpeed, float rightSpeed, float commandTime)
{
	std::vector<float> command;
	command.push_back(leftSpeed);
	command.push_back(rightSpeed);
	command.push_back(commandTime);
	commands->push_back(command);
}

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
        	bool stopped = true;
        	std::vector< std::vector<float> > commands;
        	float begin_time = APIGetSimulationTimeInSecs();
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	//Controle de movimentos
		    	if(APIGetKey() == 'o')
		    	{
		    		if(!stopped)
		    			PushCommand(&commands, leftSpeed, rightSpeed, APIGetSimulationTimeInSecs()-begin_time);
		    		leftSpeed = 11.0;
		    		rightSpeed = 11.0;
		    		stopped = false;
		    		begin_time = APIGetSimulationTimeInSecs();
		    	}

		    	else if(APIGetKey() == 'l')
		    	{
		    		if(!stopped)
		    			PushCommand(&commands, leftSpeed, rightSpeed, APIGetSimulationTimeInSecs()-begin_time);
		    		leftSpeed = -11.0;
		    		rightSpeed = -11.0;
		    		stopped = false;
		    		begin_time = APIGetSimulationTimeInSecs();
		    	}

		    	else if(APIGetKey() == 'k')
		    	{
		    		if(!stopped)
		    			PushCommand(&commands, leftSpeed, rightSpeed, APIGetSimulationTimeInSecs()-begin_time);
		    		leftSpeed = -11.0;
		    		rightSpeed = 11.0;
		    		stopped = false;
		    		begin_time = APIGetSimulationTimeInSecs();
		    	}

		    	else if(APIGetKey() == ';')
		    	{
		    		if(!stopped)
		    			PushCommand(&commands, leftSpeed, rightSpeed, APIGetSimulationTimeInSecs()-begin_time);
		    		leftSpeed = 11.0;
		    		rightSpeed = -11.0;
		    		stopped = false;
		    		begin_time = APIGetSimulationTimeInSecs();
		    	}

		    	//Parar o robô
		    	else if(APIGetKey() == ' ')
		    	{
		    		if(!stopped)
		    			PushCommand(&commands, leftSpeed, rightSpeed, APIGetSimulationTimeInSecs()-begin_time);
		    		leftSpeed = 0.0;
		    		rightSpeed = 0.0;
		    		stopped = true;
		    		APIStopRobot();
		    	}

		    	APISetRobotSpeed(leftSpeed, rightSpeed);

		        //Espera um tempo antes da próxima atualização
		        APIWait();
		    }
		    //---------------------------------------------------------
		   
		   	printf("\rFim da simulação.\n\r");
		   
		   	//Grava arquivo com os commandos
		   	printf("\rRecording commands...\n");
		   	FILE* f_log = fopen("ini/commands.ini", "w");
		   	for(int i = 0; i < (int) commands.size(); i++)
		   		fprintf(f_log, "%f %f %f\n", commands[i][0], commands[i][1], commands[i][2]);
		   	fprintf(f_log, "%.1f %.1f %.1f\n", 0.0, 0.0, -1.0);
		   	fclose(f_log);

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