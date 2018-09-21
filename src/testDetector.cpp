#include "RobotAPI.h"
#include "ObjectDetector.h"
           
//Arquivos utilizados pelo programa
#define FILE_DICTIONARY "ini/dictionary.yml"
#define FILE_DATABASE "ini/objects.ini"
#define FILE_PARAMS "ini/detector.ini"
#define FILE_SVM "ini/svm.ini" 

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
			ObjectDetector objectDetector;
			objectDetector.LoadParams(FILE_PARAMS);
			objectDetector.LoadObjects(FILE_DATABASE);

			if(objectDetector.LoadDictionary(FILE_DICTIONARY))
			{
				objectDetector.LoadSVM(FILE_SVM);
			}
			
			else
			{
				objectDetector.Train();
				objectDetector.SaveDictionary(FILE_DICTIONARY);
				objectDetector.SaveSVM(FILE_SVM);
			}

            cv::Mat frame(1, 1, CV_32FC1);
            char objectName[100];

        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	//Captura Imagem
                frame = APIReadCamera();

                //Classifica
				objectDetector.Detect(frame, objectName);

				//Printa objeto encontrado na tela
				printf("\rObject: %s\n", objectName);

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