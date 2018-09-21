/* 
 * File:   PIDController2.h
 * Author: lenovo02
 *
 * Created on 5 de Março de 2015, 18:42
 */

#ifndef PIDCONTROLLER2_H
#define	PIDCONTROLLER2_H

#include <limits>
#include <algorithm>
#include <iostream>

class Convert
{
	public:
	    float Ymin;
	    float Ymax;
	    float a;
	    float b;
	    enum {NO_CHANGES,OUTPUT_NOT_LIMITED,OUTPUT_LIMITED};
	    float lastY;
	    float smoothness;

	    Convert& fit(const float x0, const float y0, const float x1, const float y1, const int type = NO_CHANGES)
	    {
	        a = (y1 - y0) / (x1 - x0);
	        b = y0 - a*x0;

	        if (type == OUTPUT_LIMITED) {
	            Ymin = std::min(y0, y1);
	            Ymax = std::max(y0, y1);
	        } else
	            if (type == OUTPUT_NOT_LIMITED) {
	            Ymin = std::numeric_limits<float>::min();
	            Ymax = std::numeric_limits<float>::max();
	        }

	        return *this;
	    }
    
	    Convert& reset()
	    {
	        Ymin=std::numeric_limits<float>::min();
	        Ymax=std::numeric_limits<float>::max();
	        a=1.0;
	        b=0;
	        lastY=0;
	        smoothness=0;
			  return *this;
	    }
    
	    Convert()
	    {
	        reset();
	    }
    
	    Convert(const Convert& conv)
	    {
	        copy(conv);
	    }
    
	    inline Convert& copy(const Convert &conv)
	    {
	        Ymin=conv.Ymin;
	        Ymax=conv.Ymax;
	        a=conv.a;
	        b=conv.b;
	        lastY=conv.lastY;
	        smoothness=conv.smoothness;
	        return *this;
	    }
    
	    Convert& operator =(const Convert &conv)
	    {
	        copy(conv);
	        return *this;
	    }
    
	    inline float f(const float x)
	    {
	        float y=a*x+b;

	        y=y+smoothness*(lastY-y);
	        lastY=y;

	        if(y<Ymin) y=Ymin;
	        if(y>Ymax) y=Ymax;
	        return y;
	    }
};

class PIDController
{
	private:
	    float Kp; 		//Ganho proporcional
	    float Ki; 		//Ganho integral
	    float Kd; 		//Ganho derivativo
	    float Off; 		//Offset do controlador
	    float SP; 		//Valor desejado para a variável controlada
	    float PV; 		//Variável controlada
	    float ACT; 		// ação direto =1 ou inversa =-1
	    float m; 		//Valor do sinal de controle
	    const static int N = 30;
	    float e[N]; 	//Vetor de erros com o tamanho utilizado na integração
	    float hiOutput; //Maior valor da saída
	    float loOutput; //Menor valor da saída
	    float hiInput; 	//Maior valor da entrada
	    float loInput; 	//Menor valor da entrada
	
	public:
		PIDController();
		PIDController(const float Kp_, const float Ki_ = 0, const float Kd_ = 0, const float Off_ = 0, const float ACT_ = 1);
		PIDController(const PIDController& orig);
		PIDController& setParam(
			const float Kp_, const float Ki_ = 0,
			const float Kd_ = 0, const float Off_ = 0,
			const float ACT_ = 1, const float vi0 = 0);
		PIDController& setKp(float Kp_);
		PIDController& setKi(float Ki_);
		PIDController& setKd(float Kd_);
		
		inline PIDController& setpoint(const float SP_)
		{
			SP = SP_;

			if(SP > hiInput)
				SP = hiInput;
	        else if(SP < loInput)
	        	SP = loInput;

			return *this;
		};

		PIDController& setHiLoInput(const float hi, const float lo);
		PIDController& setHiLoOutput(const float hi, const float lo);

	    inline const float input(const float PV_)
	    {
	        PV = PV_;
	        int NN = N - 1; //Tamanho do vetor "e" -1
			static int b = 0;
	       
	        e[b] = (ACT)*(SP - PV);

	        //N = 10; //Tamanho do vetor "e"
	        float mi = 0;
	        for (int i = 0; i < N; i++)
				mi += e[i];

			int b1 = b-1;
			if (b1<0)
				b1 = NN;

        	m = Kp * e[b] + Ki*mi + Kd * (e[b] - e[b1]) + Off;

        	if (m > hiOutput) 
        		m = hiOutput;
        	else if (m < loOutput)
        		m = loOutput;

			b = (b+1)%N;
        	return m;
		}

	    inline const float output()
	    {
	        return m;
	    }

		inline const float err()
		{
	        return e[0];
	    }

	    PIDController& setLoInput(float loInput);
	    float getLoInput() const;
	    PIDController& setHiInput(float hiInput);
	    float getHiInput() const;
	    PIDController& setLoOutput(float loOutput);
	    float getLoOutput() const;
	    PIDController& setHiOutput(float hiOutput);
	    float getHiOutput() const;
	    void setACT(float ACT);
	    float getACT() const;
	    
	    float revertACT();
};

#endif

