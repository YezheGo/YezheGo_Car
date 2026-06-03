#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include <math.h>

static float PID_Clamp(float Value, float Min, float Max)
{
	if (Value > Max) {return Max;}
	if (Value < Min) {return Min;}
	return Value;
}

void PID_Init(PID_t *p)
{
	p->Target = 0;
	p->Actual = 0;
	p->Out = 0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
	p->TargetRamp = 0;
	p->ActualFilter = 0;
}

void PID_Update(PID_t *p)
{
	float Target;
	float Actual;
	float ErrorIntNext;
	float OutNoInt;
	float OutCandidate;
	
	if (p->TargetStepMax > 0)
	{
		float TargetDelta = p->Target - p->TargetRamp;
		if (TargetDelta > p->TargetStepMax)
		{
			p->TargetRamp += p->TargetStepMax;
		}
		else if (TargetDelta < -p->TargetStepMax)
		{
			p->TargetRamp -= p->TargetStepMax;
		}
		else
		{
			p->TargetRamp = p->Target;
		}
		Target = p->TargetRamp;
	}
	else
	{
		p->TargetRamp = p->Target;
		Target = p->Target;
	}
	
	if (p->ActualFilterAlpha > 0 && p->ActualFilterAlpha < 1)
	{
		p->ActualFilter += p->ActualFilterAlpha * (p->Actual - p->ActualFilter);
		Actual = p->ActualFilter;
	}
	else
	{
		p->ActualFilter = p->Actual;
		Actual = p->Actual;
	}
	
	p->Error1 = p->Error0;
	p->Error0 = Target - Actual;
	
	if (p->DeadBand > 0 && fabsf(p->Error0) < p->DeadBand)
	{
		p->Error0 = 0;
	}
	
	if (p->Ki != 0)
	{
		ErrorIntNext = p->ErrorInt + p->Error0;
		if (p->IntegralMax > p->IntegralMin)
		{
			ErrorIntNext = PID_Clamp(ErrorIntNext, p->IntegralMin, p->IntegralMax);
		}
	}
	else
	{
		ErrorIntNext = 0;
	}
	
	OutNoInt = p->Kp * p->Error0
		     + p->Kd * (p->Error0 - p->Error1);
	
	OutCandidate = OutNoInt + p->Ki * ErrorIntNext;
	
	if (p->Ki == 0)
	{
		p->ErrorInt = 0;
	}
	else if ((OutCandidate <= p->OutMax && OutCandidate >= p->OutMin)
		  || (OutCandidate > p->OutMax && p->Error0 < 0)
		  || (OutCandidate < p->OutMin && p->Error0 > 0))
	{
		p->ErrorInt = ErrorIntNext;
	}
	
	p->Out = OutNoInt + p->Ki * p->ErrorInt;
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}
