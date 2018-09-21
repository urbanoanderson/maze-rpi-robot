#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <vector>
#include <list>
#include <unistd.h>
#include <sys/time.h>

#define PI 3.141593
#define PI_DIV_2 1.570796
#define PI_DIV_180 0.017453
#define PI_TIMES_2 6.283185

//Arquivos
bool file_exists(const char* fname);

//Obter sinal de um número
float fsignal(float v);

//Distâncias
float EuclidianDistance(float x1, float y1, float x2, float y2);
float SquareDistance(float x1, float y1, float x2, float y2);

//Números aleatórios
float RandBeetween0and1();
float RandSignal();
float RandomValue(float minValue, float maxValue);

//Tempo
typedef long long TimeStamp;
TimeStamp GetTimeMicroSecs();
TimeStamp GetTimeMsecs();

//Funções de Ângulos
float to_pi_range(float radians);
float to_2pi_range(float radians);
float to_rad(float degrees);
float to_deg(float radians);
float to_pos_deg(float radians);
float smallestAngleDiff(float target, float source);
float angleDiff(float a1, float a2);

//Distribuição de probabilidade
double NormalDistribution(double x);
double NormalDistributionIntegrated(double x);
float GaussianCompatibility(float desiredMeasure, float realMeasure, float deviation);
float HansGaussian(float dist, float sigma, float step);

//Buffer de medições
class MeasureBuffer
{
	private:
		int max_itens;
		std::list<float> buf;

	public:
		MeasureBuffer() { max_itens = 1; }
		MeasureBuffer(int max_itens) { this->max_itens = max_itens; }

		void SetBufferSize(int buf_size) { this->max_itens = buf_size; } 
		
		void PushMeasure(float measure)
		{
			buf.push_back(measure);

			if((int) buf.size() > max_itens)
				buf.pop_front();
		}

		float GetMean()
		{
			float mean = 0.0;

			for(std::list<float>::iterator iter = buf.begin(); iter != buf.end(); iter++)
        		mean += (*iter);

        	return mean / ((float) buf.size());
		}

		void Print()
		{
			printf("\r[ ");
			for(std::list<float>::iterator iter = buf.begin(); iter != buf.end(); iter++)
        		printf("%f ", (*iter));
			printf("]\n\r");
		}
}; 

//Matrizes
class Matrix
{
	public:
		std::vector<std::vector<float> > mat;
		
		//Cria uma matriz nula (0x0)
		Matrix() 
		{
		}
		
		//Cria uma Matriz MxN
		Matrix(int m, int n)
		{
			ResizeAndNulify(m, n);
		}
		
		int Rows()
		{
			return (int) mat.size();
		}
		
		int Cols()
		{
			if(this->Rows() > 0)
				return (int) mat[0].size();
			return 0;
		}
		
		void Resize(int m, int n)
		{
			mat.resize(m);
			
			for(int i = 0; i < (int) mat.size(); i++)
		        mat[i].resize(n);
		}
		
		void ResizeAndNulify(int m, int n)
		{
			Resize(m, n);
		
			for(int i = 0; i < (int) mat.size(); i++)
		    { 
		        for(int j = 0; j < (int) mat[i].size(); j++) 
		            mat[i][j] = 0.0;
		    }
		}
		
		void Print()
		{
			printf("\r[\n");
			for(int i = 0; i < (int) mat.size(); i++)
			{
				printf("\r");
				for(int j = 0; j < (int) mat[i].size(); j++)
					printf("%f ", mat[i][j]);
				printf("\n");
			}
			printf("\r]\n");
		}
		
		//Operador de atribuição
		void operator=(const Matrix& o)
    	{
		    mat.resize(o.mat.size());
		    
		    for(int i = 0; i < (int) mat.size(); i++)
		        mat[i].resize(o.mat[i].size());
		    
		    for(int i = 0; i < (int) mat.size(); i++)
		    { 
		        for(int j = 0; j < (int) mat[i].size(); j++) 
		            mat[i][j] = o.mat[i][j];
		    }
		}
		
		//Multiplicar por constante (a constante tem que ficar depois ex: M*5.0)
		Matrix& operator*(float k)
		{
			for(int i = 0; i < Rows(); i++)
			{ 
				for(int j = 0; j < Cols(); j++) 
		            mat[i][j] *= k;
			}
			
			return *this;
		}
};

Matrix operator+(Matrix A, Matrix B);
Matrix operator-(Matrix A, Matrix B);
Matrix operator*(Matrix A, Matrix B);
Matrix Transpose(Matrix A);
Matrix Invert3x3(Matrix A);

#endif
