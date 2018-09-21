#include "Utils.h"

//Arquivos

bool file_exists(const char* fname)
{
    return (access(fname, F_OK) != -1);
}

//Obter sinal de um número

float fsignal(float v)
{
	if(v >= 0)
		return 1.0;
	return -1.0;
}

//Distâncias

float EuclidianDistance(float x1, float y1, float x2, float y2)
{
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

float SquareDistance(float x1, float y1, float x2, float y2)
{
    return ((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

//Números aleatórios

float RandBeetween0and1()
{
	return (rand()%1001) / 1000.0;
}

float RandSignal()
{
	if(rand()%2 == 0)
		return 1.0;
	return -1.0;
}

float RandomValue(float minValue, float maxValue)
{
	return fmax(minValue, (rand()%((int)(maxValue*1000))) / 1000.0);
}

//Tempo

TimeStamp GetTimeMicroSecs()
{
	struct timeval nowTimeVal;
	gettimeofday(&nowTimeVal, 0);
	return (1000000 * (TimeStamp)nowTimeVal.tv_sec + nowTimeVal.tv_usec);
}

TimeStamp GetTimeMsecs()
{
	struct timeval nowTimeVal;
	gettimeofday(&nowTimeVal, 0);
	return ((1000000 * (TimeStamp)nowTimeVal.tv_sec + nowTimeVal.tv_usec)) / 1000;
}

//Funções de Ângulos

float to_pi_range(float radians)
{
    radians = fmod(radians, PI_TIMES_2);
    if (radians < -PI)
        radians += PI_TIMES_2;
    else if (radians > PI)
        radians -= PI_TIMES_2;
 
    return radians;
}

float to_2pi_range(float radians)
{
    while(radians < 0.0)
		radians += PI_TIMES_2;
	
	if(radians > 2*PI)
		radians = fmod(radians, PI_TIMES_2);

    return radians;
}

float to_rad(float degrees)
{
	if(degrees < 0)
		degrees = 360 - degrees;
		
	return (degrees * (PI_DIV_180));
}

float to_deg(float radians)
{
    return radians * (180.0 / PI);
}

float to_pos_deg(float radians)
{
	while(radians < 0.0)
		radians += PI_TIMES_2;
	
	if(radians > PI_TIMES_2)
		radians = fmod(radians, PI_TIMES_2);
	
    return to_2pi_range(radians) * (180.0 / PI);
}
 
float smallestAngleDiff(float target, float source)
{
    float a;
    a = to_2pi_range(target) - to_2pi_range(source);
 
    if (a > PI)
        a = a - PI_TIMES_2;
    else if (a < -PI)
        a = a + PI_TIMES_2;
    
    return a;
}

float angleDiff(float a1, float a2)
{
	float dif = fmod(fabs(to_2pi_range(a1) - to_2pi_range(a2)), PI_TIMES_2);

	if (dif > PI)
    	dif = PI_TIMES_2 - dif;

	return dif;
}

//Distribuição de probabilidade

double NormalDistribution(double x)
{
	static float standardNormalCurve[31][10] = 
	{
		{0.5000, 0.5040, 0.5080, 0.5120, 0.5160, 0.5199, 0.5239, 0.5279, 0.5319, 0.5359}, 
		{0.5398, 0.5438, 0.5478, 0.5517, 0.5557, 0.5596, 0.5636, 0.5675, 0.5714, 0.5753}, 
		{0.5793, 0.5832, 0.5871, 0.5910, 0.5948, 0.5987, 0.6026, 0.6064, 0.6103, 0.6141},
		{0.6179, 0.6217, 0.6255, 0.6293, 0.6331, 0.6368, 0.6406, 0.6443, 0.6480, 0.6517}, 
		{0.6554, 0.6591, 0.6628, 0.6664, 0.6700, 0.6736, 0.6772, 0.6808, 0.6844, 0.6879},
		{0.6915, 0.6950, 0.6985, 0.7019, 0.7054, 0.7088, 0.7123, 0.7157, 0.7190, 0.7224}, 
		{0.7257, 0.7291, 0.7324, 0.7357, 0.7389, 0.7422, 0.7454, 0.7486, 0.7517, 0.7549}, 
		{0.7580, 0.7611, 0.7642, 0.7673, 0.7703, 0.7734, 0.7764, 0.7794, 0.7823, 0.7852}, 
		{0.7881, 0.7910, 0.7939, 0.7967, 0.7995, 0.8023, 0.8051, 0.8078, 0.8106, 0.8133}, 
		{0.8159, 0.8186, 0.8212, 0.8238, 0.8264, 0.8289, 0.8315, 0.8340, 0.8365, 0.8389}, 
		{0.8413, 0.8438, 0.8461, 0.8485, 0.8508, 0.8531, 0.8554, 0.8577, 0.8599, 0.8621}, 
		{0.8643, 0.8665, 0.8686, 0.8708, 0.8729, 0.8749, 0.8770, 0.8790, 0.8810, 0.8830}, 
		{0.8849, 0.8869, 0.8888, 0.8907, 0.8925, 0.8944, 0.8962, 0.8980, 0.8997, 0.9015}, 
		{0.9032, 0.9049, 0.9066, 0.9082, 0.9099, 0.9115, 0.9131, 0.9147, 0.9162, 0.9177}, 
		{0.9192, 0.9207, 0.9222, 0.9236, 0.9251, 0.9265, 0.9278, 0.9292, 0.9306, 0.9319}, 
		{0.9332, 0.9345, 0.9357, 0.9370, 0.9382, 0.9394, 0.9406, 0.9418, 0.9430, 0.9441}, 
		{0.9452, 0.9463, 0.9474, 0.9484, 0.9495, 0.9505, 0.9515, 0.9525, 0.9535, 0.9545}, 
		{0.9554, 0.9564, 0.9573, 0.9582, 0.9591, 0.9599, 0.9608, 0.9616, 0.9625, 0.9633}, 
		{0.9641, 0.9648, 0.9656, 0.9664, 0.9671, 0.9678, 0.9686, 0.9693, 0.9700, 0.9706}, 
		{0.9713, 0.9719, 0.9726, 0.9732, 0.9738, 0.9744, 0.9750, 0.9756, 0.9762, 0.9767}, 
		{0.9772, 0.9778, 0.9783, 0.9788, 0.9793, 0.9798, 0.9803, 0.9808, 0.9812, 0.9817}, 
		{0.9821, 0.9826, 0.9830, 0.9834, 0.9838, 0.9842, 0.9846, 0.9850, 0.9854, 0.9857}, 
		{0.9861, 0.9864, 0.9868, 0.9871, 0.9874, 0.9878, 0.9881, 0.9884, 0.9887, 0.9890}, 
		{0.9893, 0.9896, 0.9898, 0.9901, 0.9904, 0.9906, 0.9909, 0.9911, 0.9913, 0.9916}, 
		{0.9918, 0.9920, 0.9922, 0.9925, 0.9927, 0.9929, 0.9931, 0.9932, 0.9934, 0.9936}, 
		{0.9938, 0.9940, 0.9941, 0.9943, 0.9945, 0.9946, 0.9948, 0.9949, 0.9951, 0.9952}, 
		{0.9953, 0.9955, 0.9956, 0.9957, 0.9959, 0.9960, 0.9961, 0.9962, 0.9963, 0.9964}, 
		{0.9965, 0.9966, 0.9967, 0.9968, 0.9969, 0.9970, 0.9971, 0.9972, 0.9973, 0.9974}, 
		{0.9974, 0.9975, 0.9976, 0.9977, 0.9977, 0.9978, 0.9979, 0.9979, 0.9980, 0.9981}, 
		{0.9981, 0.9882, 0.9882, 0.9883, 0.9984, 0.9984, 0.9985, 0.9985, 0.9986, 0.9986}, 
		{0.9987, 0.9990, 0.9993, 0.9995, 0.9997, 0.9998, 0.9998, 0.9999, 0.9999, 1.0000}
	};

	int j = 0, k = 0, m = 0;

	if(x >= 0.0 && x <= 3.0999)
	{
		j = ((int)x%10);
		m = ((int)(x*10)%10);
		j = 10*j + m;
		k = (int)(x*100)%10;
			
		return standardNormalCurve[j][k];
	}
	
	else if(x >= -3.0999 && x <= 0.0)
	{
		x = -x;
		j = ((int)x%10);
		m = ((int)(x*10)%10);
		j = 10*j + m;
		k = (int)(x*100)%10;

		return (1.0-standardNormalCurve[j][k]);
	}
	
	else if(x >= 3.0999)
		return 1.0;
	
	return 0.0;
}

double NormalDistributionIntegrated(double x)
{
    static double a1 =  0.254829592;
    static double a2 = -0.284496736;
    static double a3 =  1.421413741;
    static double a4 = -1.453152027;
    static double a5 =  1.061405429;
    static double p  =  0.3275911;

    int sign = 1;
    
    if (x < 0)
        sign = -1;

    x = fabs(x) / 1.414213;//sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0 / (1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}

float GaussianCompatibility(float desiredMeasure, float realMeasure, float deviation)
{	
	if(realMeasure < 0.0)
		return 0.0;
	//printf("gaussian %f\n", (-fabs(realMeasure - desiredMeasure))/deviation);
	return 2*NormalDistribution((-fabs(realMeasure - desiredMeasure))/deviation);
}

float HansGaussian(float dist, float sigma, float step)
{
    float x = fabs(dist);
    float halfstep = step/2.0;    
    float zmax = (x+halfstep)/sigma;
    float zmin = (x-halfstep)/sigma;
    return NormalDistribution(zmax) - NormalDistribution(zmin);
}

//Matrizes

Matrix operator+(Matrix A, Matrix B)
{
    Matrix C;

    if(A.Rows() == B.Rows() && A.Cols() == B.Cols())
    {
    	C.Resize(A.Rows(), A.Cols());
    		
    	for(int i = 0; i < A.Rows(); i++)
		{
			for(int j = 0; j < A.Cols(); j++)
				C.mat[i][j] = A.mat[i][j] + B.mat[i][j];
		}
    }
    else
		printf("\rError on Matrix Sum. Matrices must have same dimensions.\n");

    return C;
}

Matrix operator-(Matrix A, Matrix B)
{
    Matrix C;

    if(A.Rows() == B.Rows() && A.Cols() == B.Cols())
    {
    	C.Resize(A.Rows(), A.Cols());
    		
    	for(int i = 0; i < A.Rows(); i++)
		{
			for(int j = 0; j < A.Cols(); j++)
				C.mat[i][j] = A.mat[i][j] - B.mat[i][j];
		}
    }
    else
		printf("\rError on Matrix Subtract. Matrices must have same dimensions.\n");

    return C;
}

Matrix operator*(Matrix A, Matrix B)
{
    Matrix C;

    if(A.Cols() == B.Rows())
    {
    	C.ResizeAndNulify(A.Rows(), B.Cols());
    		
    	for(int i = 0; i < C.Rows(); i++)
		{
			for(int j = 0; j < C.Cols(); j++)
			{
				for(int k = 0; k < A.Cols(); k++)
					C.mat[i][j] += A.mat[i][k] * B.mat[k][j];
			}
		}
    }
    else
		printf("\rError on Matrix Multiply. Cols(A) must be equal to Rows(B).\n");

    return C;
}

Matrix Transpose(Matrix A)
{
	Matrix t(A.Cols(), A.Rows());
			
	for(int i = 0; i < (int) t.mat.size(); i++)
	{ 
		for(int j = 0; j < (int) t.mat[i].size(); j++) 
			t.mat[i][j] = A.mat[j][i];
	}
			
	return t;
}

Matrix Invert3x3(Matrix A)
{
	Matrix t;
	
	if(A.Rows() == 3 && A.Cols() == 3)
	{
		float denominator = (A.mat[0][0]*(A.mat[1][1]*A.mat[2][2] - A.mat[1][2]*A.mat[2][1]) - A.mat[0][1]*(A.mat[1][0]*A.mat[2][2] - A.mat[1][2]*A.mat[2][0]) + A.mat[0][2]*(A.mat[1][0]*A.mat[2][1] - A.mat[1][1]*A.mat[2][0]));
		
		//Verifica se a matriz é inversível
		if(denominator == 0)
		{
			printf("\rError on Invert3x3(). Not invertible matrix.\n");
			return t;
		}

		t.Resize(3, 3);
		t.mat[0][0] = A.mat[1][1]*A.mat[2][2] - A.mat[1][2]*A.mat[2][1];
		t.mat[0][1] = A.mat[0][2]*A.mat[2][1] - A.mat[0][1]*A.mat[2][2];
		t.mat[0][2] = A.mat[0][1]*A.mat[1][2] - A.mat[0][2]*A.mat[1][1];
		t.mat[1][0] = A.mat[1][2]*A.mat[2][0] - A.mat[1][0]*A.mat[2][2];
		t.mat[1][1] = A.mat[0][0]*A.mat[2][2] - A.mat[0][2]*A.mat[2][0];
		t.mat[1][2] = A.mat[0][2]*A.mat[1][0] - A.mat[0][0]*A.mat[1][2];
		t.mat[2][0] = A.mat[1][0]*A.mat[2][1] - A.mat[1][1]*A.mat[2][0];
		t.mat[2][1] = A.mat[0][1]*A.mat[2][0] - A.mat[0][0]*A.mat[2][1];
		t.mat[2][2] = A.mat[0][0]*A.mat[1][1] - A.mat[0][1]*A.mat[1][0];

		float k = 1.0 / denominator;
		t = t*k;
	}
	else
		printf("\rError on Invert3x3(). Parameter is not a 3x3 matrix.\n");
			
	return t;
}
