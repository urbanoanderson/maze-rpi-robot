#include "Utils.h"
#include "RobotAPI.h"
#include "ColorDetector.h"

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
        	
        	//Inicializa o detector de objetos
			ColorDetector colorDetector;
            cv::Mat frame(1, 1, CV_32FC3);
            std::string objectName;

        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	//Captura Imagem
                frame = APIReadCamera();

                //Classifica
				objectName = colorDetector.Detect(frame);

				//Printa objeto encontrado na tela
				printf("\rObject: %s\n", objectName.c_str());
				APIWaitMsecs(1000);

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