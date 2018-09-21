#include "ColorDetector.h"
#include "RobotAPI.h"
#include "Utils.h"

enum STATES { WALKING = 1, STOPPING = 2, DETECTING = 3, TURNING_A = 4, TURNING_B = 5, EMERGENCY_REAR = 6 };
           
void UpdateSonarReadings(float* sonarReadings)
{
	static MeasureBuffer bufLeft(3);
	static MeasureBuffer bufFront(3);
	static MeasureBuffer bufRight(3);

	float measure_left = APIReadSonarLeft();
	if(measure_left > 0.0) 
		bufLeft.PushMeasure(measure_left);
	
	float measure_front = APIReadSonarFront();
	if(measure_front > 0.0) 
		bufFront.PushMeasure(measure_front);

	float measure_right = APIReadSonarRight();
	if(measure_right > 0.0) 
		bufRight.PushMeasure(measure_right);

	sonarReadings[LEFT] = bufLeft.GetMean();
	sonarReadings[FRONT] = bufFront.GetMean();
	sonarReadings[RIGHT] = bufRight.GetMean();
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

        	//Inicializa detector de objetos
        	ColorDetector objectDetector;
            std::string objectName;

        	int state = WALKING;
        	float sonarReadings[3] = { -1.0, -1.0, -1.0 };
        	float leftSpeed = 0.0;
        	float rightSpeed = 0.0;
        	float deltaS = 0.0;
        	float stampDeltaSRear = 0.0;

        	float baseSpeed = 35.0;  

        	float WALL_THRESHOLD_FAR = 0.06;
        	float WALL_THRESHOLD_CLOSE = 0.05;
        	float WALL_THRESHOLD_FRONT = 0.08;
        	float DELTA_S_THRESHOLD = 0.025;

        	int substate = 0;
        	int pic_counter = 0;
        	float timeStamp = APIGetSimulationTimeInSecs();
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {   
		    	//Atualiza sonares
		    	UpdateSonarReadings(sonarReadings); 

		    	//Atualiza odometria
		    	deltaS += APIReadOdometers();

		    	//Máquina de estados
		    	if(state == WALKING)
		    	{
		    		leftSpeed = baseSpeed;
		    		rightSpeed = baseSpeed;

		    		//Odometria para checar se bateu
		    		if(APIGetSimulationTimeInSecs() - timeStamp < 2.0)
		    		{
		    			stampDeltaSRear = deltaS - stampDeltaSRear;
		    		}
		    		else
		    		{
		    			if(stampDeltaSRear < DELTA_S_THRESHOLD){
		    				substate = 0;
		    				state = EMERGENCY_REAR;
		    			}

		    			stampDeltaSRear = deltaS;	
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}

		    		if(sonarReadings[RIGHT] > WALL_THRESHOLD_FAR
		    		&& sonarReadings[RIGHT] < WALL_THRESHOLD_FAR + 0.04)
		    		{
		    			leftSpeed = baseSpeed;
		    			rightSpeed = baseSpeed*0.60;
		    		}

		    		if(sonarReadings[RIGHT] < WALL_THRESHOLD_CLOSE
		    		&& sonarReadings[RIGHT] > WALL_THRESHOLD_CLOSE - 0.04)
		    		{
		    			leftSpeed = baseSpeed*0.60;
		    			rightSpeed = baseSpeed;
		    		}

		    		//Hora de tirar foto
		    		if(sonarReadings[FRONT] >= 0.18 && sonarReadings[FRONT] <= 0.20)
		    		{
		    			state = DETECTING;
		    			substate = 0.0;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}

		    		//Curva tipo A
		    		if(sonarReadings[FRONT] <= 0.10)
		    		{
		    			state = TURNING_A;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}

		    		//Curva tipo B
		    		if(sonarReadings[FRONT] > 0.30 && sonarReadings[RIGHT] > 0.30
		    		&& sonarReadings[FRONT] < 2.20 && sonarReadings[RIGHT] < 2.20)
		    		{
		    			state = TURNING_B;
		    			substate = 0;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	else if(state == DETECTING)
		    	{
		    		//Virar para o objeto
		    		if(substate == 0)
		    		{
			    		leftSpeed = 35.0;
			    		rightSpeed = -35.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp >= 0.5)
			    		{
			    			substate = 1;
			    			timeStamp = APIGetSimulationTimeInSecs();
			    		}
					}

		    		//Parar
		    		else if(substate == 1)
		    		{
			    		leftSpeed = 0.0;
			    		rightSpeed = 0.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp >= 0.4)
			    		{
			    			substate = 2;
			    			timeStamp = APIGetSimulationTimeInSecs();
			    		}
					}

					//Detectar objeto
					else if(substate == 2)
					{
						printf("\n\rDetecting object...\n");
	                	cv::Mat frame = APIReadCamera(15, 10);
	                	objectName = objectDetector.Detect(frame);
   						char pic_path[50];
    					sprintf(pic_path, "img/pics/%d_%s.jpg", pic_counter, objectName.c_str());
    					pic_counter++;
						printf("\rObject: %s\n\n", objectName.c_str());
						APISavePicture(frame, pic_path);
						APIWaitMsecs(1000);
						timeStamp = APIGetSimulationTimeInSecs();
						substate = 3;
					}

					//Virar de volta
					else if(substate == 3)
		    		{
			    		leftSpeed = -35.0;
			    		rightSpeed = 35.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp >= 0.4)
			    		{
			    			substate = 4;
			    			timeStamp = APIGetSimulationTimeInSecs();
			    		}
					}

					//Ir um pouco para a frente
					else if(substate == 4)
		    		{
			    		leftSpeed = baseSpeed;
			    		rightSpeed = baseSpeed;

			    		if(APIGetSimulationTimeInSecs() - timeStamp >= 1.0)
			    		{
			    			state = WALKING;
			    			stampDeltaSRear = deltaS;
							timeStamp = APIGetSimulationTimeInSecs();
			    			substate = 0;
			    		}
					}
		    	}

		    	//Curva para a esquerda após detectar objeto
		    	else if(state == TURNING_A)
		    	{
		    		leftSpeed = -baseSpeed*0.60;
		    		rightSpeed = baseSpeed;

		    		if(APIGetSimulationTimeInSecs() - timeStamp > 2.0)
		    		{
		    			state = WALKING;
		    			stampDeltaSRear = deltaS;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	//Curva para a direita
		    	else if(state == TURNING_B)
		    	{
		    		leftSpeed = 70.0;
		    		rightSpeed = 35.0;

		    		if(APIGetSimulationTimeInSecs() - timeStamp > 1.7)
		    		{
		    			substate = 1;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    			state = WALKING;
		    			stampDeltaSRear = deltaS;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	else if(state == EMERGENCY_REAR)
		    	{
		    		//Ré
		    		if(substate == 0)
		    		{
		    			leftSpeed = -60.0;
			    		rightSpeed = -60.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp > 0.7)
			    		{
			    			substate = 1;
			    			timeStamp = APIGetSimulationTimeInSecs();
			    		}
		    		}

		    		//Correção
		    		else if(substate == 1)
		    		{
		    			leftSpeed = 30.0;
			    		rightSpeed = 55.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp > 0.7)
			    		{
			    			stampDeltaSRear = deltaS;
			    			state = WALKING;
			    			timeStamp = APIGetSimulationTimeInSecs();
			    		}
		    		}
		    	}

		    	//APISetRobotSpeed(leftSpeed, rightSpeed);
		    	APISetMotorPower(leftSpeed, rightSpeed);

		    	//Printa Informações
		    	printf("\rState: %d  |  Substate: %d  |  DeltaS: %f  |  Sonars:[%.2f, %.2f, %.2f]\n", state, substate, deltaS, sonarReadings[0], sonarReadings[1], sonarReadings[2]);

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