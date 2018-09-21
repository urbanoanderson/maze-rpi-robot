#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include <unistd.h>

#include "RobotAPI.h"
#include "EnvMap.h"
#include "Utils.h"
#include "ObjectDetector.h"

class Robot
{
	private:
		//Mapa do ambiente
		EnvMap envmap;

		//Detector de Objetos
		ObjectDetector objectDetector;	

		//Caminho do robô
		std::vector< std::vector<float> > path;	//Vetor de objetivos do robô
		float goal[3];							//Objetivo atual
		int current_goal;						//Índice do objetivo atual no path
		bool reached_goal;						//Se chegou ao objetivo atual
		bool loop_path;							//Se é pra repetir o caminho quando terminar
		bool finishedTask;						//Se cumpriu todos os objetivos
		int num_voltas;							//Quantidade de voltas que o robô deu no percurso

		//Posição do robô
		Matrix realpos;							//posição verdadeira de referencia da API do VREP
		Matrix pos;								//posição estimada pelo robô

		//Leituras dos sonares
		enum SONAR_ID { LEFT = 0, FRONT = 1, RIGHT = 2 };
		float sonarReading[3];			//Vetor que contem a leitura dos sonares
		float sensorLeftPos[3];			//Posicionamento do sensor esquerdo em relação ao robô
		float sensorFrontPos[3];		//Posicionamento do sensor frontal em relação ao robô
		float sensorRightPos[3];		//Posicionamento do sensor direito em relação ao robô
		float SENSOR_DEVIATION;			//Desvio padrão da medida de distancia fornecida pelo sonar
		float SENSOR_OPENING_ANGLE;		//Abertura em radianos do cone do sonar (metade da abertura)

		//Constantes do controle de movimento
		float MOTION_CONTROL_K_RHO;
		float MOTION_CONTROL_K_ALPHA;
		float MOTION_CONTROL_K_BETA;
		float MOTION_CONTROL_MIN_V;	
		float MOTION_CONTROL_MAX_V;
		float MOTION_CONTROL_DISTXY_THRESHOLD;
		float MOTION_CONTROL_DISTTHETA_THRESHOLD;
		
		//Variáveis da odometria
		float ODOMETRY_KL;
		float ODOMETRY_KR;
		float ACUMULATED_DISTANCE_THRESHOLD;
		float acumulatedDistance;

		//Variáveis do Filtro de Kalman
		Matrix sigmapos;					//Matriz de covariância da posição estimada (modelo de incerteza)
		Matrix R;							//Matriz de covariância do sensor
		float posdeviation[3];				//Desvio padrão da posição do robô
		float DEVIATION_THRESHOLD_X;		//Limiar em X para chamar atualização de percepção
		float DEVIATION_THRESHOLD_Y;		//Limiar em Y para chamar atualização de percepção
		float DEVIATION_THRESHOLD_THETA;	//Limiar em THETA para chamar atualização de percepção
		float PERCETION_UPDATE_STEP_X;
		float PERCETION_UPDATE_STEP_Y;
		float PERCETION_UPDATE_STEP_THETA;
		float PERCETION_UPDATE_STEP_SONAR;

		//Funções auxiliares
		void UpdateSonarReadings(); 		//Atualiza a leitura dos sonares do robô e armazena em sonarReading[]
		void ManageObjectives();			//Gerencia os objetivos do robô
		void MotionControl();		 		//Faz o robô andar em direção ao objetivo atual
		void ActionUpdate();				//Atualiza pos e sigmapos após movimento do robô usando a odometria
		bool PerceptionUpdateCondition();	//Decide quando fazer atualização de percepção	
		void PerceptionUpdate();			//Melhora a estimativa de posição do robô com os sensores e o mapa
		Matrix EstimateXz(); 				//Faz uma estimativa da posição do robô com base nos sensores
		float RobotToSensorPointDist(float x, float y, float theta, float Sx, float Sy, float Stheta, float sensorDist);
	
	public:
		//Funções principais de interface do robô
		Robot(const char* INIT_FILENAME);
		void Init(const char* INIT_FILENAME);
		bool LoadPath(const char* PATH_FILENAME);
		bool FinishedWork();
		void Log();
		void Update();
};

#endif
