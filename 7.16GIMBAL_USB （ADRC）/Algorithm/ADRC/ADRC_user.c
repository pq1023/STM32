#include "ADRC_user.h"
#include "ADRC_core.h"
void ADRC_init(ADRC_t *ADRC_type , float *a)
{
    ADRC_type->r	= a[0];
    ADRC_type->h	= a[1];
	ADRC_type->h0	= a[2];

    ADRC_type->b        = a[3];
    ADRC_type->delta    = a[4];
    ADRC_type->belta01  = a[5];
    ADRC_type->belta02  = a[6];
    ADRC_type->belta03  = a[7];
	ADRC_type->z3_seperate = a[12];

    ADRC_type->alpha1 = a[8];
    ADRC_type->alpha2 = a[9];
    ADRC_type->belta1 = a[10];
    ADRC_type->belta2 = a[11];
	
		ADRC_type->KP = a[13];
		ADRC_type->KI = a[14];
		ADRC_type->KD = a[15];

    ADRC_type->set = 0.0f;
    ADRC_type->rel = 0.0f;
    ADRC_type->out = 0.0f;
    
}


