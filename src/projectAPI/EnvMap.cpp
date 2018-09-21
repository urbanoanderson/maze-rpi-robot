#include "EnvMap.h"

EnvMap::EnvMap()
{	
}

EnvMap::EnvMap(const char* map_filename)
{
	LoadFromFile(map_filename);
}

void EnvMap::Print()
{
	printf("\rMapa do Ambiente:\n");
	for(int i = 0; i < (int) segments.size(); i++)
		printf("\r\tSegmento de (%.2f, %.2f) para (%.2f, %.2f)\n", segments[i].x1, segments[i].y1, segments[i].x2, segments[i].y2);
}

void EnvMap::LoadFromFile(const char* map_filename)
{
	FILE* f = fopen(map_filename, "r");

	if(f)
	{
		float x1, y1, x2, y2;
		while(fscanf(f, "%f %f %f %f", &x1, &y1, &x2, &y2) == 4)
			AddWall(x1, y1, x2, y2);
	}

	else
		printf("\rErro ao abrir arquivo de mapa.\n");
}

void EnvMap::AddWall(float x1, float y1, float x2, float y2)
{
	Segment seg(x1, y1, x2, y2);
	segments.push_back(seg);
}

float EnvMap::MapDistance(float x, float y, float theta)
{	
	//Verifica quais segmentos o vetor intersecta e guarda
	float min_dist = INFINITE_DISTANCE;
	for(int i = 0; i < (int) segments.size(); i++)
		min_dist = fmin(min_dist, DistancePostureToSegment(x, y, theta, segments[i]));
	
	return min_dist;
}

float EnvMap::MapDistance2(float x, float y, float theta)
{
	static float sensorOpening = 15.0*(PI/180.0); 
	return fmin(MapDistance(x, y, theta-sensorOpening), MapDistance(x, y, theta+sensorOpening));
}

float EnvMap::DistanceToNearestWall(float x, float y)
{
	float distance = INFINITE_DISTANCE;

	for(int i = 0; i < (int) segments.size(); i++)
		distance = fmin(distance, DistancePointToSegment(x, y, segments[i]));

	return sqrt(distance);
}

//--------------------------------------------------------------
//Funções Auxiliares
//--------------------------------------------------------------

float EnvMap::DistancePointToSegment(float x, float y, Segment seg)
{
	float dx = seg.x2 - seg.x1;
    float dy = seg.y2 - seg.y1;
    
    // It's a point not a line segment.
    if ((dx == 0) && (dy == 0))
    {
        dx = x - seg.x1;
        dy = y - seg.y1;
        return (dx * dx + dy * dy); //distância quadrática
    }

    // Calculate the t that minimizes the distance.
    float t = ((x - seg.x1) * dx + (y - seg.y1) * dy) / (dx * dx + dy * dy);

    // See if this represents one of the segment's
    // end points or a point in the middle.
    if (t < 0)
    {
        dx = x - seg.x1;
        dy = y - seg.y1;
    }
    else if (t > 1)
    {
        dx = x - seg.x2;
        dy = y - seg.y2;
    }
    else
    {
        dx = x - (seg.x1 + t*dx);
        dy = y - (seg.y1 + t*dy);
    }

    return (dx * dx + dy * dy); //distância quadratica
}

float EnvMap::DistancePostureToSegment(float x, float y, float theta, Segment seg)
{
	float angle = to_2pi_range(theta);

	//Encontrando a reta do vetor
	float a1 = tan(angle); //Possível crash em 90 graus (improvável)
	float b1 = y - a1*x;
	
	float increment = 10.0*fsignal(a1);
	
	if(angle > PI)
		increment *= -1;
	
	float p0_x = x;
	float p0_y = y;
	float p1_x = x + increment;
	float p1_y = a1*p1_x + b1;

	float p2_x = seg.x1;
	float p2_y = seg.y1;
	float p3_x = seg.x2+0.001;
	float p3_y = seg.y2;
	
	//-------------------------------------------
	//Colisão entre 2 segmentos de reta (p0, p1) e (p2, p3)

	float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
    	float xp = p0_x + (t * s1_x);
        float yp = p0_y + (t * s1_y);
    
    	return sqrt((x-xp)*(x-xp) + (y-yp)*(y-yp));
    }
        
    return INFINITE_DISTANCE;
}