#include "RobotAPI.h"
           
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
        	
            cv::Mat image(1, 1, CV_32FC1);

        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
                image = APIReadCamera();

                if(APIGetKey() == 'p')
                {
                    printf("\rSaving picture...\n");
                    APISavePicture(image);
                }

		        //Espera um tempo antes da próxima atualização
		        APIWait();
		    }
		    //---------------------------------------------------------
		   
		   	printf("\rFim da simulação.\n\r");
		   
		    //Para o robô e desconecta;
		    APIFinishSimulation();
        } 
        
        else
        	printf("\rNão foi possível iniciar a simulação.\n\r");
    } 
    
    else
        printf("\rNão foi possível conectar.\n\r");
   
    return 0;
}