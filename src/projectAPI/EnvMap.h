#ifndef ENVMAP_H_INCLUDED
#define ENVMAP_H_INCLUDED

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "Utils.h"

#define INFINITE_DISTANCE 999.9

class EnvMap
{
	private:
		class Segment
		{
			public:
				float x1, y1, x2, y2;
				Segment(float x1, float y1, float x2, float y2) 
				{
					this->x1 = x1; this->y1 = y1; 
					this->x2 = x2; this->y2 = y2;
				}
		};

		std::vector<Segment> segments;
	
		//Funções Auxiliares
		float DistancePointToSegment(float x, float y, Segment seg);
		float DistancePostureToSegment(float x, float y, float theta, Segment seg);
		
	public:
		EnvMap(); //Cria mapa vazio
		EnvMap(const char* map_filename); //Carrega um mapa de um arquivo
		void Print(); //Printa o mapa na tela
		void LoadFromFile(const char* map_filename);
		void AddWall(float x1, float y1, float x2, float y2);
		float MapDistance(float x, float y, float theta);
		float MapDistance2(float x, float y, float theta);
		float DistanceToNearestWall(float x, float y);
};

#endif
