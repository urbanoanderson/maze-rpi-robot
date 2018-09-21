#include "RobotAPI.h"

KBAsync kb;
int kb_key;
bool stopped;
float WHEEL_R, WHEEL_L;
cv::VideoCapture cap(0);
TimeStamp simulationBeginTime;

#if USING_VREP == 1
    simxInt clientID;
    simxInt ddRobotHandle;
    simxInt leftMotorHandle;
    simxInt rightMotorHandle;
    simxInt sonarL;
    simxInt sonarR;
    simxInt sonarF;

#elif USING_VREP == 0
    Encoder encoderL, encoderR;
    Motor motorL, motorR;
    Sonar sonarL, sonarR, sonarF;
#endif

bool APIInitConection()
{
	if(!cap.isOpened())
	{
		printf("\rErro. Incapaz de iniciar captura de vídeo.\n");
        return false;
	}

	//Lê os valores do pid a partir de um arquivo
	float kpL = 0.05, kiL = 0.00, kdL = 0.0;
	float minPowerL = 5.0, spdSampRateL = 0.0;
	float kpR = 0.05, kiR = 0.00, kdR = 0.0;
	float minPowerR = 5.0, spdSampRateR = 0.0;

	FILE* f = fopen("ini/hardware.ini", "r");
	if(f != NULL)
	{
		//Constantes do PID do motor esquerdo
		fscanf(f, "%*[^:] %*c %f", &kpL);
		fscanf(f, "%*[^:] %*c %f", &kiL);
		fscanf(f, "%*[^:] %*c %f", &kdL);
		fscanf(f, "%*[^:] %*c %f", &spdSampRateL);
		fscanf(f, "%*[^:] %*c %f", &minPowerL);

		//Constantes do PID do motor direito
		fscanf(f, "%*[^:] %*c %f", &kpR);
		fscanf(f, "%*[^:] %*c %f", &kiR);
		fscanf(f, "%*[^:] %*c %f", &kdR);
		fscanf(f, "%*[^:] %*c %f", &spdSampRateR);
		fscanf(f, "%*[^:] %*c %f", &minPowerR);

		//Tamanhos
		fscanf(f, "%*[^:] %*c %f", &WHEEL_R);
		fscanf(f, "%*[^:] %*c %f", &WHEEL_L);
	}
	else
		printf("Erro ao ler arquivo `hardware.ini`\n");

    #if USING_VREP == 1
    	printf("\rIniciando conexão com: %s...\n", V_REP_IP_ADDRESS);
    	clientID = simxStart(V_REP_IP_ADDRESS, V_REP_PORT, true, true, 2000, 5);

    	if(clientID != -1)
    	{
    		//Pega referência para os componentes do robô 
    		simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "ProximitySensorL#", &sonarL, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "ProximitySensorR#", &sonarR, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "ProximitySensorF#", &sonarF, simx_opmode_oneshot_wait);

    		return true;
    	}

    #elif USING_VREP == 0
        if (PIN_MODE==PIN_BCM)
        {
            printf("\rPinos em modo BCM.\n");
            
            if (wiringPiSetupGpio()<0)
            {
                printf("\rErro. Incapaz de setar pinos GPIO.\n");
                return false;
            }
        } 

        else 
        {
            printf("\rPinos em modo wiringPi.\n");
            wiringPiSetup();
        }

        encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
        encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
        sonarL.setup(SONAR_LEFT_TRIGGER, SONAR_LEFT_ECHO);
        sonarR.setup(SONAR_RIGHT_TRIGGER, SONAR_RIGHT_ECHO);
        sonarF.setup(SONAR_FRONT_TRIGGER, SONAR_FRONT_ECHO);
        motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
        motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);

        motorL.pid.setParam(kpL, kiL, kdL);
        motorL.setSpdSamplingRate(spdSampRateL);
        motorL.setMinPower(minPowerL);
        motorR.pid.setParam(kpR, kiR, kdR);
        motorR.setSpdSamplingRate(spdSampRateR);
        motorR.setMinPower(minPowerR);

        return true;
    #endif

    return false;
}

bool APIStartSimulation()
{
    srand(time(NULL));
    simulationBeginTime = GetTimeMicroSecs();
    APIStopRobot();

    #if USING_VREP == 1   
	   return (simxStartSimulation(clientID, simx_opmode_oneshot_wait) != -1);
    #elif USING_VREP == 0
       return true;
    #endif
}

bool APISimulationIsRunning()
{
    kb_key = kb.getKey();

    if(kb_key == 'q')
        return false;

    #if USING_VREP == 1
    	if(simxGetConnectionId(clientID) == -1
    	|| simxGetLastCmdTime(clientID) == 0)
    	{
    		printf("\rParando a simulação...\n");
    		return false;
    	}

    	return true;

    #elif USING_VREP == 0 
        return true;
    #endif
}

void APIFinishSimulation()
{
    APIStopRobot();

    #if USING_VREP == 1
    	simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
    	simxFinish(clientID);
    #elif USING_VREP == 0
    #endif
}

void APIWait()
{
    #if USING_VREP == 1
        extApi_sleepMs(2);
    #elif USING_VREP == 0
        if(!stopped)
        {
            motorL.controlSpeed();
            motorR.controlSpeed();
        }
        //delayMicroseconds(msecs*1000);
    #endif
}

void APIWaitMsecs(int msecs)
{
    TimeStamp time_begin = GetTimeMsecs();
    while(GetTimeMsecs() - time_begin < msecs);
}

int APIGetKey()
{
    return kb_key;
}

float APIGetSimulationTimeInSecs()
{
    return (GetTimeMicroSecs() - simulationBeginTime) / 1000000.0;
}

void APIGetTrueRobotPosition(Matrix* realpos)
{
    #if USING_VREP == 1
    	static simxFloat real[3];
    	simxInt ret = simxGetObjectPosition(clientID, ddRobotHandle, -1, real, simx_opmode_oneshot_wait);
        if (ret > 0) {
            printf("\rErro ao ler posição do robô.\n");
            return;
        }
     
        static simxFloat orientation[3];
        ret = simxGetObjectOrientation(clientID, ddRobotHandle, -1, orientation, simx_opmode_oneshot_wait);
        if (ret > 0) {
            printf("\rErro ao ler orientação do robô.\n");
            return;
        }

    	realpos->mat[0][0] = real[0];
    	realpos->mat[1][0] = real[1];
        realpos->mat[2][0] = to_pi_range(orientation[2]);

    #elif USING_VREP == 0
        realpos->mat[0][0] = -1;
        realpos->mat[1][0] = -1;
        realpos->mat[2][0] = -1;

    #endif
}

void APIGetTrueRobotPosition(float* realpos)
{
    #if USING_VREP == 1
        static simxFloat real[3];
        simxInt ret = simxGetObjectPosition(clientID, ddRobotHandle, -1, real, simx_opmode_oneshot_wait);
        if (ret > 0) {
            printf("\rErro ao ler posição do robô.\n");
            return;
        }
     
        static simxFloat orientation[3];
        ret = simxGetObjectOrientation(clientID, ddRobotHandle, -1, orientation, simx_opmode_oneshot_wait);
        if (ret > 0) {
            printf("\rErro ao ler orientação do robô.\n");
            return;
        }

        realpos[0] = real[0];
        realpos[1] = real[1];
        realpos[2] = to_pi_range(orientation[2]);

    #elif USING_VREP == 0
        realpos[0] = -1;
        realpos[1] = -1;
        realpos[2] = -1;
    #endif
}

void APIReadOdometers(float* dPhiL, float* dPhiR)
{
    #if USING_VREP == 1
        //Old joint angle position
        static simxFloat lwprev = 0;
        static simxFloat rwprev = 0;
       
        //Current joint angle position
        simxFloat lwcur = 0;
        simxFloat rwcur = 0;
     
        simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_oneshot);
        simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_oneshot);
     
        *dPhiL = smallestAngleDiff(lwcur, lwprev);
        *dPhiR = smallestAngleDiff(rwcur, rwprev);
        lwprev = lwcur;
        rwprev = rwcur;

    #elif USING_VREP == 0
        *dPhiL = encoderL.getDeltaAngle();
        *dPhiR = encoderR.getDeltaAngle();
    #endif
}

float APIReadOdometers()
{
    float dPhiL, dPhiR;
    APIReadOdometers(&dPhiL, &dPhiR);

    float deltaSl = WHEEL_R * dPhiL;
    float deltaSr = WHEEL_R * dPhiR;

    return fabs((deltaSl + deltaSr) / 2.0);
}

void APISetRobotSpeed(float phiL, float phiR)
{
    if(phiL < -0.001 || phiL > 0.001 || phiR < -0.001 || phiR > 0.001)
            stopped = false;
    
    #if USING_VREP == 1
        simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot); 
    #elif USING_VREP == 0
        motorL.setTargetSpeed(phiR); //rodas da esquerda e direita sao trocadas
        motorR.setTargetSpeed(phiL);
    #endif 
}

void APISetMotorPower(float powL, float powR)
{
	if(powL < -0.001 || powL > 0.001 || powR < -0.001 || powR > 0.001)
        stopped = false;
    
    #if USING_VREP == 1
        simxSetJointTargetVelocity(clientID, leftMotorHandle, 6.0*(powL/100.0), simx_opmode_oneshot);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, 6.0*(powR/100.0), simx_opmode_oneshot); 
    #elif USING_VREP == 0
        motorL.setPower(powR); //rodas da esquerda e direita sao trocadas
        motorR.setPower(powL);
        motorL.writePower();
        motorR.writePower();
    #endif 
}

void APIStopRobot()
{
    stopped = true;
    APISetRobotSpeed(0.0, 0.0);

    #if USING_VREP == 1
    #elif USING_VREP == 0
        motorL.stop();
        motorR.stop();
    #endif
}

cv::Mat APIReadCamera()
{
	cv::Mat image(1, 1, CV_32FC3);
	cap >> image;
	return image;
}

cv::Mat APIReadCamera(int numPics, int delayMsecs)
{
	cv::Mat frame;
	for(int i = 0; i < numPics; i++)
	{
		frame = APIReadCamera();
	    APIWaitMsecs(delayMsecs);
	}

	return frame;
}

void APISavePicture(cv::Mat picture)
{
	static int pic_counter = 0;
	char pic_path[50];
	sprintf(pic_path, "img/pics/%d.jpg", pic_counter); 
	cv::imwrite(pic_path, picture);
	pic_counter++;
}

void APISavePicture(cv::Mat picture, char* filename)
{ 
    cv::imwrite(filename, picture);
}

float APIReadSonarLeft()
{
    #if USING_VREP == 1
        simxUChar detectionState;
        simxInt detectedObjectHandle;
        simxFloat detectedPoint[3];
        simxFloat detectedSurfaceNormalVector[3];
        
        simxReadProximitySensor(clientID, sonarL, &detectionState, detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_oneshot);
        if (detectionState != 0)
    		return detectedPoint[2];
        else
            return -1;

    #elif USING_VREP == 0
        return sonarL.measureDistance();
    #endif
}

float APIReadSonarFront()
{
    #if USING_VREP == 1
        simxUChar detectionState;
        simxInt detectedObjectHandle;
        simxFloat detectedPoint[3];
        simxFloat detectedSurfaceNormalVector[3];
        
        simxReadProximitySensor(clientID, sonarF, &detectionState, detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_oneshot);
        if (detectionState != 0)
            return detectedPoint[2];
        else
            return -1;

    #elif USING_VREP == 0
        breturn sonarF.measureDistance();
    #endif
}

float APIReadSonarRight()
{
    #if USING_VREP == 1
        simxUChar detectionState;
        simxInt detectedObjectHandle;
        simxFloat detectedPoint[3];
        simxFloat detectedSurfaceNormalVector[3];
        
        simxReadProximitySensor(clientID, sonarR, &detectionState, detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_oneshot);
        if (detectionState != 0)
            return detectedPoint[2];
        else
            return -1;

    #elif USING_VREP == 0
		return sonarR.measureDistance();
    #endif
}
