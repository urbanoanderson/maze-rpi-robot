#include "Robot.h"

Robot::Robot(const char* INIT_FILENAME)
{
	pos.ResizeAndNulify(3, 1);
	realpos.ResizeAndNulify(3, 1);
	sigmapos.ResizeAndNulify(3, 3);
	posdeviation[0] = 0.0; posdeviation[1] = 0.0; posdeviation[2] = 0.0;
	acumulatedDistance = 0.0;
	current_goal = -1;
	reached_goal = true;
	finishedTask = false;
	num_voltas = 0;

	this->Init(INIT_FILENAME);
}

void Robot::Init(const char* INIT_FILENAME)
{
	char aux[100];

	FILE* file_init = fopen(INIT_FILENAME, "r");

	if(file_init != NULL)
	{
		//Mapa do ambiente
		fscanf(file_init, "%*[^:] %*c %[^\n]", aux);
		this->envmap.LoadFromFile(aux);
		this->envmap.Print();

		//Path
		fscanf(file_init, "%*[^:] %*c %[^\n]", aux);
		this->LoadPath(aux);

		//Object Detector
		fscanf(file_init, "%*[^:] %*c %[^\n]", aux);
		objectDetector.LoadParams(aux);
		fscanf(file_init, "%*[^:] %*c %[^\n]", aux);
		objectDetector.LoadObjects(aux);
		fscanf(file_init, "%*[^:] %*c %[^\n]", aux);
		objectDetector.LoadDictionary(aux);
		fscanf(file_init, "%*[^:] %*c %[^\n]", aux);
		objectDetector.LoadSVM(aux);		

		printf("\rLoading Robot Params...\n");

		//Sonar params
		fscanf(file_init, "%*[^:] %*c %f %f %f", &sensorLeftPos[0], &sensorLeftPos[1], &sensorLeftPos[2]);
		sensorLeftPos[2] = sensorLeftPos[2]*PI_DIV_180;
		fscanf(file_init, "%*[^:] %*c %f %f %f", &sensorFrontPos[0], &sensorFrontPos[1], &sensorFrontPos[2]);
		sensorFrontPos[2] = sensorFrontPos[2]*PI_DIV_180;
		fscanf(file_init, "%*[^:] %*c %f %f %f", &sensorRightPos[0], &sensorRightPos[1], &sensorRightPos[2]);
		sensorRightPos[2] = sensorRightPos[2]*PI_DIV_180;
		fscanf(file_init, "%*[^:] %*c %f", &SENSOR_DEVIATION);
		fscanf(file_init, "%*[^:] %*c %f", &SENSOR_OPENING_ANGLE);
		SENSOR_OPENING_ANGLE = to_rad(SENSOR_OPENING_ANGLE/2.0);
		printf("\r\tSONAR_LEFT_RELATIVE_POS: %fm %fm %f°\n", sensorLeftPos[0], sensorLeftPos[1], to_deg(sensorLeftPos[2]));
		printf("\r\tSONAR_FRONT_RELATIVE_POS: %fm %fm %f°\n", sensorFrontPos[0], sensorFrontPos[1], to_deg(sensorFrontPos[2]));
		printf("\r\tSONAR_RIGHT_RELATIVE_POS: %fm %fm %f°\n", sensorRightPos[0], sensorRightPos[1], to_deg(sensorRightPos[2]));
		printf("\r\tSONAR_DEVIATION: %fm\n", SENSOR_DEVIATION);
		printf("\r\tSONAR_OPENING_ANGLE: %f°\n", to_deg(SENSOR_OPENING_ANGLE));

		//Motion control params
		fscanf(file_init, "%*[^:] %*c %f", &MOTION_CONTROL_K_RHO);
		fscanf(file_init, "%*[^:] %*c %f", &MOTION_CONTROL_K_ALPHA);
		fscanf(file_init, "%*[^:] %*c %f", &MOTION_CONTROL_K_BETA);
		fscanf(file_init, "%*[^:] %*c %f", &MOTION_CONTROL_MIN_V);
		fscanf(file_init, "%*[^:] %*c %f", &MOTION_CONTROL_MAX_V);
		fscanf(file_init, "%*[^:] %*c %f", &MOTION_CONTROL_DISTXY_THRESHOLD);
		fscanf(file_init, "%*[^:] %*c %f", &MOTION_CONTROL_DISTTHETA_THRESHOLD);
		printf("\r\tMOTION_CONTROL_K_RHO: %f\n", MOTION_CONTROL_K_RHO);
		printf("\r\tMOTION_CONTROL_K_ALPHA: %f\n", MOTION_CONTROL_K_ALPHA);
		printf("\r\tMOTION_CONTROL_K_BETA: %f\n", MOTION_CONTROL_K_BETA);
		printf("\r\tMOTION_CONTROL_MIN_V: %f\n", MOTION_CONTROL_MIN_V);
		printf("\r\tMOTION_CONTROL_MAX_V: %f\n", MOTION_CONTROL_MAX_V);
		printf("\r\tMOTION_CONTROL_DISTXY_THRESHOLD: %f\n", MOTION_CONTROL_DISTXY_THRESHOLD);
		printf("\r\tMOTION_CONTROL_DISTTHETA_THRESHOLD: %f\n", MOTION_CONTROL_DISTTHETA_THRESHOLD);

		//Odometry
		fscanf(file_init, "%*[^:] %*c %f", &ODOMETRY_KL);
		fscanf(file_init, "%*[^:] %*c %f", &ODOMETRY_KR);
		printf("\r\tODOMETRY_KL: %f\n", ODOMETRY_KL);
		printf("\r\tODOMETRY_KR: %f\n", ODOMETRY_KR);

		//Perception update condition
		fscanf(file_init, "%*[^:] %*c %f", &ACUMULATED_DISTANCE_THRESHOLD);
		fscanf(file_init, "%*[^:] %*c %f %f %f", &DEVIATION_THRESHOLD_X, &DEVIATION_THRESHOLD_Y, &DEVIATION_THRESHOLD_THETA);
		DEVIATION_THRESHOLD_THETA = to_rad(DEVIATION_THRESHOLD_THETA);
		printf("\r\tACUMULATED_DISTANCE_THRESHOLD: %fm\n", ACUMULATED_DISTANCE_THRESHOLD);
		printf("\r\tDEVIATION_THRESHOLDS: %fm %fm %f°\n", DEVIATION_THRESHOLD_X, DEVIATION_THRESHOLD_Y, to_deg(DEVIATION_THRESHOLD_THETA));

		//Perception Update Thresholds
		fscanf(file_init, "%*[^:] %*c %f", &PERCETION_UPDATE_STEP_X);
		fscanf(file_init, "%*[^:] %*c %f", &PERCETION_UPDATE_STEP_Y);
		fscanf(file_init, "%*[^:] %*c %f", &PERCETION_UPDATE_STEP_THETA);
		PERCETION_UPDATE_STEP_THETA *= PI_DIV_180;
		fscanf(file_init, "%*[^:] %*c %f", &PERCETION_UPDATE_STEP_SONAR);
		PERCETION_UPDATE_STEP_SONAR *= PI_DIV_180;
		printf("\r\tPERCETION_UPDATE_STEP_X: %fm\n", PERCETION_UPDATE_STEP_X);
		printf("\r\tPERCETION_UPDATE_STEP_Y: %fm\n", PERCETION_UPDATE_STEP_Y);
		printf("\r\tPERCETION_UPDATE_STEP_THETA: %f°\n", to_deg(PERCETION_UPDATE_STEP_THETA));
		printf("\r\tPERCETION_UPDATE_STEP_SONAR: %f°\n", to_deg(PERCETION_UPDATE_STEP_SONAR));

		//Kalman R matrix
		float matAux[3][3];
		fscanf(file_init, "%*[^:] %*c %f %f %f %f %f %f %f %f %f",
		 &matAux[0][0], &matAux[0][1], &matAux[0][2],
		 &matAux[1][0], &matAux[1][1], &matAux[1][2],
		 &matAux[2][0], &matAux[2][1], &matAux[2][2]);

		R.Resize(3, 3);
		R.mat[0][0] = matAux[0][0];
		R.mat[0][1] = matAux[0][1];
		R.mat[0][2] = matAux[0][2];
		R.mat[1][0] = matAux[1][0];
		R.mat[1][1] = matAux[1][1];
		R.mat[1][2] = matAux[1][2];
		R.mat[2][0] = matAux[2][0];
		R.mat[2][1] = matAux[2][1];
		R.mat[2][2] = to_rad(matAux[2][2]);

		printf("\r\tR Matrix\n");
		printf("\r\t\t%f %f %f\n", R.mat[0][0], R.mat[0][1], R.mat[0][2]);
		printf("\r\t\t%f %f %f\n", R.mat[1][0], R.mat[1][1], R.mat[1][2]);
		printf("\r\t\t%f %f %f\n", R.mat[2][0], R.mat[2][1], R.mat[2][2]);

		fclose(file_init);
	}

	else
		printf("\rError. Could not open ´%s´.\n", INIT_FILENAME);
}

bool Robot::LoadPath(const char* PATH_FILENAME)
{
	FILE* file_path = fopen(PATH_FILENAME, "r");

	if(file_path != NULL)
	{
		float x, y, theta;

		//Se o path é em loop
		int loop_aux = 0;
		fscanf(file_path, "%*[^:] %*c %d", &loop_aux);
		loop_path = (loop_aux > 0);

		//Posição inicial
		fscanf(file_path, "%*[^:] %*c %f %f %f", &x, &y, &theta);
		pos.mat[0][0] = x; pos.mat[1][0] = y; pos.mat[2][0] = to_pi_range(theta*PI_DIV_180);

		//Pontos do path
		while(!feof(file_path))
		{
			fscanf(file_path, "%*[^:] %*c %f %f %f", &x, &y, &theta);
			std::vector<float> path_point;
			path_point.push_back(x);
			path_point.push_back(y);
			path_point.push_back(theta*PI_DIV_180);
			path.push_back(path_point);
		}

		fclose(file_path);
		return true;
	}

	else
	{
		printf("\rError. Could not open ´%s´.\n", PATH_FILENAME);
		return false;
	}
}

bool Robot::FinishedWork()
{
	return finishedTask;
}

void Robot::Log()
{
	printf("\r----------------------------------------------------\n");
	printf("\rPosição Real:        [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", realpos.mat[0][0], realpos.mat[1][0], realpos.mat[2][0]);
	printf("\rPosição Estimada:    [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", pos.mat[0][0], pos.mat[1][0], pos.mat[2][0]);
	printf("\rErro de Posição:     [%.4f, %.4f, %.4f°]\n", fabs(realpos.mat[0][0]-pos.mat[0][0]), fabs(realpos.mat[1][0]-pos.mat[1][0]), to_deg(angleDiff(realpos.mat[2][0], pos.mat[2][0])));
	printf("\rDesvio de Posição:   [%.4f, %.4f, %.4f°]\n", posdeviation[0], posdeviation[1], to_deg(posdeviation[2]));
	printf("\rVariância de Posição:[%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", sigmapos.mat[0][0], sigmapos.mat[1][1], sigmapos.mat[2][2]);
	printf("\rLeitura dos Sonares: [%f, %f, %f]  //[L, F, R]\n", sonarReading[LEFT], sonarReading[FRONT], sonarReading[RIGHT]);
	printf("\rNumero de voltas:    [%d]\n", num_voltas);
	float t_time = APIGetSimulationTimeInSecs();
	printf("\rTempo de simulação:  [%.2dm %.2ds]\n", (int) t_time/60,(int) t_time%60);
}

void Robot::Update()
{	
	//Gerencia os objetivos do robô
	ManageObjectives();

	//Se o objetivo for tirar uma foto e detectar objeto
	if(goal[0] >= 999.0)
	{
		APIStopRobot();
		printf("\n\rDetecting Object...\n");
		char objectName[100];
		cv::Mat frame(1, 1, CV_32FC1);
		frame = APIReadCamera();
		objectDetector.Detect(frame, objectName);
		printf("\rObject: %s\n\n", objectName);
		APIWaitMsecs(1000);
		reached_goal = true;
		return;
	}

	//Olha posição real do robô para comparação
	APIGetTrueRobotPosition(&realpos);

	//Faz a leitura dos sonares
	UpdateSonarReadings();

	//Passo de Atualização de Ação
	ActionUpdate();
		
	//Passo de Atualização de Percepção
	if(PerceptionUpdateCondition())
	{
		//APIStopRobot();
		PerceptionUpdate();
		acumulatedDistance = 0.0;
	}

	//Executa o controle de movimento
	MotionControl();
}

void Robot::UpdateSonarReadings()
{
	sonarReading[LEFT]  = APIReadSonarLeft();
	sonarReading[FRONT] = APIReadSonarFront();
	sonarReading[RIGHT] = APIReadSonarRight();
}

void Robot::ManageObjectives()
{
	if(reached_goal)
	{
		if(current_goal >= (int) path.size()-1)
		{
			//Se tiver loop volta pro goal inicial
			if(loop_path)
			{
				current_goal = -1;
				num_voltas++;
			}

			//Se não para o robô
			else
			{
				printf("\rUltimo objetivo alcançado.\n");
				APIStopRobot();
				finishedTask = true;
				return;
			}
		}
		
		current_goal++;	
		goal[0] = path[current_goal][0];
		goal[1] = path[current_goal][1];
		goal[2] = path[current_goal][2];
		reached_goal = false;
	}
}

void Robot::MotionControl()
{	
    float theta = pos.mat[2][0];
    float dtheta = smallestAngleDiff(goal[2], theta);
 
    float deltax = goal[0] - pos.mat[0][0];
    float deltay = goal[1] - pos.mat[1][0];
    float rho = sqrt(deltax*deltax + deltay*deltay);
 
    float atg = to_pi_range(atan2(deltay, deltax));
    float alpha = to_pi_range(smallestAngleDiff(atg, theta));
    float beta = to_pi_range(goal[2] - theta - alpha);
 
    float v = MOTION_CONTROL_K_RHO * rho;

    if(v < MOTION_CONTROL_MIN_V)
    	v = MOTION_CONTROL_MIN_V;
    else if(v > MOTION_CONTROL_MAX_V)
    	v = MOTION_CONTROL_MAX_V;

    float w = (MOTION_CONTROL_K_ALPHA * alpha + MOTION_CONTROL_K_BETA * beta);
 
 	float wR = v + WHEEL_L*w;
    float wL = v - WHEEL_L*w;

    float phiL = wL / WHEEL_R;
    float phiR = wR / WHEEL_R;

    if(rho < MOTION_CONTROL_DISTXY_THRESHOLD)
    {
		phiR = 2 * dtheta;
		phiL = -2 * dtheta;
		
		if(fabs(dtheta) >= MOTION_CONTROL_DISTTHETA_THRESHOLD)
		{
			phiR = 2 * dtheta;
			phiL = -2 * dtheta;
		}
		
		else
			reached_goal = true;
    }
    
    APISetRobotSpeed(phiL, phiR);
}

void Robot::ActionUpdate()
{
	//Leitura do odometro para saber as variações dPhiL e dPhiR
	float dPhiL, dPhiR;
	APIReadOdometers(&dPhiL, &dPhiR);
	
	//Cálculo de deltaTheta, deltaX e deltaY
	static float b = (2.0*WHEEL_L);
	float deltaSl = WHEEL_R * dPhiL;
	float deltaSr = WHEEL_R * dPhiR;
	float deltaS = (deltaSl + deltaSr) / 2.0;
	float deltaTheta = (deltaSr - deltaSl) / b;

	float argumento = to_pi_range(pos.mat[2][0] + (deltaTheta*0.5));
	float sinargumento = sin(argumento);
	float cosargumento = cos(argumento);

	float deltaX = deltaS * cosargumento;
	float deltaY = deltaS * sinargumento;

	//Atualiza distância acumulada desde a ultima atualização de percepção
	acumulatedDistance += deltaS;

	//Atualiza a média da posição	
	pos.mat[0][0] += deltaX;
	pos.mat[1][0] += deltaY;
	pos.mat[2][0] = to_pi_range(pos.mat[2][0] + deltaTheta);

	//Calcula o sigmaDelta
	Matrix sigmaDelta(2, 2);
	sigmaDelta.mat[0][0] = ODOMETRY_KR * fabs(deltaSr);
	sigmaDelta.mat[0][1] = 0.0;
	sigmaDelta.mat[1][0] = 0.0;
	sigmaDelta.mat[1][1] = ODOMETRY_KL * fabs(deltaSl);

	//Calcula Fp
	Matrix fp(3, 3);
	fp.mat[0][0] = 1;
	fp.mat[0][1] = 0;
	fp.mat[0][2] = -deltaY; //0.002;
	fp.mat[1][0] = 0;
	fp.mat[1][1] = 1;
	fp.mat[1][2] = deltaX; //0.002;
	fp.mat[2][0] = 0;
	fp.mat[2][1] = 0;
	fp.mat[2][2] = 1;

	//Calcula fDeltaRl
	Matrix fDeltaRl(3, 2);
	fDeltaRl.mat[0][0] = 0.5*cosargumento - deltaY/(2.0*b); //1
	fDeltaRl.mat[0][1] = 0.5*cosargumento + deltaY/(2.0*b); //0
	fDeltaRl.mat[1][0] = 0.5*sinargumento + deltaX/(2.0*b); //0
	fDeltaRl.mat[1][1] = 0.5*sinargumento - deltaX/(2.0*b); //1
	fDeltaRl.mat[2][0] = 1.0/b;								//0
	fDeltaRl.mat[2][1] = -1.0/b;							//0
	
	//Atualiza matriz de covariancias da posição estimada
	sigmapos = ((fp * sigmapos) * Transpose(fp)) + ((fDeltaRl * sigmaDelta) * Transpose(fDeltaRl));
	
	//Atualiza o desvio padrão da posição do robô	
	posdeviation[0] = sqrt(sigmapos.mat[0][0]);
	posdeviation[1] = sqrt(sigmapos.mat[1][1]);
	posdeviation[2] = sqrt(sigmapos.mat[2][2]);

	//-------------------------------------------
	//Teste com desvio perfeito
	posdeviation[0] = fabs(realpos.mat[0][0]-pos.mat[0][0]);
	posdeviation[1] = fabs(realpos.mat[1][0]-pos.mat[1][0]);
	posdeviation[2] = angleDiff(realpos.mat[2][0], pos.mat[2][0]);
}

bool Robot::PerceptionUpdateCondition()
{
	return
	(
		//Todos os sonares estão com leituras válidas
		(sonarReading[LEFT] >= 0 && sonarReading[FRONT] >= 0 && sonarReading[RIGHT] >= 0)
		
		&& 
		
		//O robô andou uma certa distancia ou o desvio em alguma variável passou de um limiar
		(
			acumulatedDistance > ACUMULATED_DISTANCE_THRESHOLD
			|| (posdeviation[0] >= DEVIATION_THRESHOLD_X || posdeviation[1] >= DEVIATION_THRESHOLD_Y || posdeviation[2] >= DEVIATION_THRESHOLD_THETA)
		)
	);
}

void Robot::PerceptionUpdate()
{
	//Calcula sigmav
	Matrix sigmav = sigmapos + R;

	//Calcula Kt
	Matrix K = sigmapos * Invert3x3(sigmav);

	//Calcula xz
	Matrix xz = EstimateXz();

	//Calcula Vt  
	Matrix v = xz - pos; 

	//Atualiza posição
	pos = pos + (K * v);
	
	//Atualiza modelo de incerteza de posição
	sigmapos = sigmapos - ((K * sigmav) * Transpose(K));
}

Matrix Robot::EstimateXz()
{	
	printf("\rCalculando XZ...\n");

	//Estimativa de posição
	static Matrix xz(3, 1);

	static float stepX = PERCETION_UPDATE_STEP_X;
	static float stepY = PERCETION_UPDATE_STEP_Y;
	static float stepTheta = PERCETION_UPDATE_STEP_THETA;
	static float stepDiffAngle = PERCETION_UPDATE_STEP_SONAR;

	float minX = pos.mat[0][0] - posdeviation[0];
	float maxX = pos.mat[0][0] + posdeviation[0];
	float minY = pos.mat[1][0] - posdeviation[1];
	float maxY = pos.mat[1][0] + posdeviation[1];
	float minTheta = pos.mat[2][0] - posdeviation[2];
	float maxTheta = pos.mat[2][0] + posdeviation[2];

	float maxCompat = 0.0;

	for(float x = minX; x <= maxX; x += stepX)
	{
		for(float y = minY; y <= maxY; y += stepY)
		{
			for(float theta = minTheta; theta <= maxTheta; theta += stepTheta)
			{
				float dL = INFINITE_DISTANCE;
				float dF = INFINITE_DISTANCE;
				float dR = INFINITE_DISTANCE;

				float diffAngleMin = sensorLeftPos[2] - SENSOR_OPENING_ANGLE;
				float diffAngleMax = sensorLeftPos[2] + SENSOR_OPENING_ANGLE;	
				for(float diffAngle = diffAngleMin; diffAngle <= diffAngleMax; diffAngle += stepDiffAngle)
					dL = fmin(dL, RobotToSensorPointDist(x, y, theta, sensorLeftPos[0], sensorLeftPos[1], diffAngle, sonarReading[LEFT]));

				diffAngleMin = sensorFrontPos[2] - SENSOR_OPENING_ANGLE;
				diffAngleMax = sensorFrontPos[2] + SENSOR_OPENING_ANGLE;	
				for(float diffAngle = diffAngleMin; diffAngle <= diffAngleMax; diffAngle += stepDiffAngle)
					dF = fmin(dF, RobotToSensorPointDist(x, y, theta, sensorFrontPos[0], sensorFrontPos[1], diffAngle, sonarReading[FRONT]));

				diffAngleMin = sensorRightPos[2] - SENSOR_OPENING_ANGLE;
				diffAngleMax = sensorRightPos[2] + SENSOR_OPENING_ANGLE;	
				for(float diffAngle = diffAngleMin; diffAngle <= diffAngleMax; diffAngle += stepDiffAngle)
					dR = fmin(dR, RobotToSensorPointDist(x, y, theta, sensorRightPos[0], sensorRightPos[1], diffAngle, sonarReading[RIGHT]));

				float compat = HansGaussian(dL, SENSOR_DEVIATION, stepX)*HansGaussian(dF, SENSOR_DEVIATION, stepX)*HansGaussian(dR, SENSOR_DEVIATION, stepX);	

				if(compat > maxCompat)
				{
					maxCompat = compat;
					xz.mat[0][0] = x;
					xz.mat[1][0] = y;
					xz.mat[2][0] = to_pi_range(theta);
				}	
			}
		}
	}

	printf("\rERRO DA ESTIMATIVA XZ: [%.2fm, %.2fm, %.2f°]\n", fabs(realpos.mat[0][0]-xz.mat[0][0]), fabs(realpos.mat[1][0]-xz.mat[1][0]), to_deg(angleDiff(realpos.mat[2][0], xz.mat[2][0])));
	
	return xz;
}

float Robot::RobotToSensorPointDist(float Rx, float Ry, float Rtheta, float Sx, float Sy, float Stheta, float sensorDist)
{
	float point[2];
	float pSR[2];

	//Sensor to robot
    pSR[0] = Sx + sensorDist * cos(Stheta);
    pSR[1] = Sy + sensorDist * sin(Stheta);
    
    //Robot to world
    float sintheta = sin(Rtheta);
    float costheta = cos(Rtheta);
    point[0] = Rx + (pSR[0] * costheta - pSR[1] * sintheta);
    point[1] = Ry + (pSR[0] * sintheta + pSR[1] * costheta);

	return envmap.DistanceToNearestWall(point[0], point[1]);
}