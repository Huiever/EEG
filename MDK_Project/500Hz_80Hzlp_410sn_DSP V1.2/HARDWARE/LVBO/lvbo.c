/******************************************************************************
* ����arm_biquad_cascade_df1_f32˵��
* ������������:
*    void arm_biquad_cascade_df1_f32(
*            const arm_biquad_casd_df1_inst_f32 * S,
*            float32_t * pSrc,
*            float32_t * pDst,
*           uint32_t blockSize)
* ��������:
*       [in]  *S         points to an instance of the floating-point Biquad cascade structure.   
*       [in]  *pSrc      points to the block of input data.   
*      [out] *pDst      points to the block of output data.   
*      [in]  blockSize  number of samples to process per call.   
*      return     none.  
******************************************************************************/
#include "sys.h"
#include "delay.h"
#include "iwdg.h"
#include "usart6.h"
#include "exti.h"
#include "timer.h"
#include "ads129x.h"
#include "ad5754r.h"
#include "math.h" 
#include "arm_math.h"  
#include "lvbo.h"


//#define TEST_LENGTH_SAMPLES 800 /* ��������*/
//float32_t testInput_f32_gaotong[LENGTH_SAMPLES]; /* ������*/
float32_t testOutput_ditong[410];
float32_t testOutput_ditong_1[410]; 

float32_t IIRStateF32_ditong[4*numStages2]; 

/* �б�ѩ���ͨ�˲���ϵ�� 80Hz*/ 
const float32_t IIRCoeffs32LP[5*numStages2] = {
	1.0f, 2.0f, 1.0f, 0.9440196979942016f, -0.88650266262502642f,
	1.0f, 2.0f, 1.0f, 1.0122046254995687f, -0.69129182533349154f,
	1.0f, 2.0f, 1.0f, 1.1703216710662463f, -0.5372834201030251f,
	1.0f, 2.0f, 1.0f, 1.2989804678340879f, -0.44654451401868306f,
};


/*
*********************************************************************************************************
* ������: arm_iir_f32_lp
* ����˵��: ���ú���arm_iir_f32_lpʵ�ֵ�ͨ�˲���
* �β�:��
* ����ֵ: ��
*********************************************************************************************************
*/
void arm_iir_f32_lp(void)
{
	uint32_t i;
	arm_biquad_casd_df1_inst_f32 S1;
	float32_t ScaleValue2;
//float32_t testOutput_ditong_1[800]; 

/* ��ʼ�� */
	arm_biquad_cascade_df1_init_f32(&S1, numStages2, (float32_t *)&IIRCoeffs32LP[0], (float32_t
	*)&IIRStateF32_ditong[0]);
/* IIR�˲���*/
	arm_biquad_cascade_df1_f32(&S1, z, testOutput_ditong_1, LENGTH_SAMPLES);
/*����ϵ��*/
	ScaleValue2 = 0.2356207411577062f * 0.16977179995848074f * 
	0.091740437259194701f * 0.036891011546148753f * 0.99426007395295668f;


/* ��ӡ�˲���Ľ�� */
	for (i=0;i<10;i++)
	{
		testOutput_ditong[i+400]=testOutput_ditong_1[i+400]*ScaleValue2;
	
	}
}

