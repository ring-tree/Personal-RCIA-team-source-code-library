
#include<stdio.h>
#include<math.h>

/**
  * @ intro Zero crossing
  * @ parameter ExpectedAngle
  * @ parameter ActualAngle
  * @ return AfterProcessingAngle
  */
double main(double ExpectedAngle, double ActualAngle)
{
	double AbsoluteValue = fabs(ExpectedAngle - ActualAngle);
	double median;
	if (AbsoluteValue > 180.0f) {
		median = 360.0f - AbsoluteValue;
		(ActualAngle > ExpectedAngle) ? (ActualAngle = ExpectedAngle - median) : (ActualAngle = ExpectedAngle + median);		
	}

	return ActualAngle;
}
