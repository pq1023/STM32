#include "positioning.h"

Distances_T Distances;
float Yaw;
void Distance_State(void)
{
	Yaw = Angular_Handler.YAW;	
	
//	Distances.Real_Distance_X = -(Positioning_t.Distance_Right * cos((45 - Positioning_t.Angle)*3.14f/180.0f) -
//								(Positioning_t.Distance_Left) * sin((45 - Positioning_t.Angle)*3.14f/180.0f));
//	Distances.Real_Distance_Y = Positioning_t.Distance_Right * sin((45 - Positioning_t.Angle)*3.14f/180.0f) +
//								(Positioning_t.Distance_Left) * cos((45 + Positioning_t.Angle)*3.14f/180.0f);
	
	Distances.Real_Distance_X = Positioning_t.Distance_Left * cos((45)*3.14f/180.0f) -
								(Positioning_t.Distance_Right) * sin((45 )*3.14f/180.0f);
	Distances.Real_Distance_Y = Positioning_t.Distance_Left * sin((45 )*3.14f/180.0f) +
								(Positioning_t.Distance_Right) * cos((45 )*3.14f/180.0f);
}

