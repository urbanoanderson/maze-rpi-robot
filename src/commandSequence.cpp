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

        	//Lê lista de commnados do arquivo
        	FILE* f = fopen("ini/commands.ini", "r");
        	std::vector< std::vector<float> > commandList;
        	float spdL, spdR, tim;
        	while(fscanf(f, "%f %f %f", &spdL, &spdR, &tim) == 3)
        	{
        		std::vector<float> command;
        		command.push_back(spdL);
        		command.push_back(spdR);
        		command.push_back(tim);
        		commandList.push_back(command);
        		printf("\r\tLoad Command %f %f %f\n", spdL, spdR, tim);
        	}
        	fclose(f);

        	int current_command = 0;
        	float beginTime = APIGetSimulationTimeInSecs();
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning() && commandList[current_command][2] > 0)
		    {    
		    	APISetRobotSpeed(commandList[current_command][0], commandList[current_command][1]);

		    	if(APIGetSimulationTimeInSecs() - beginTime >= commandList[current_command][2])
		    	{	
		    		printf("\rChanging to command: %f %f %f\n", commandList[current_command][0], commandList[current_command][1], commandList[current_command][2]);
		    		current_command++;
                    APISetRobotSpeed(commandList[current_command][0], commandList[current_command][1]);
		    		beginTime = APIGetSimulationTimeInSecs();
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