
#include<stdio.h>
#include<math.h>

/**
 * @brief Adjusts the actual angle to match the expected angle based on the zero-crossing point.
 * 
 * This function takes in the expected angle and actual angle values, calculates the difference, 
 * and adjusts the actual angle to match the expected angle by considering the zero-crossing point.
 * 
 * @param ExpectedAngle The expected angle value.
 * @param ActualAngle The actual angle value to be adjusted.
 * 
 * @retval fint32_t The adjusted actual angle value.
 */
fint32_t ZeroCrossing(fint32_t ExpectedAngle, fint32_t ActualAngle) {
    const fint32_t MAX_ANGLE = 8192.0f;
    fint32_t DifferenceValue = ExpectedAngle - ActualAngle;
    if ( fabs(DifferenceValue) >= (MAX_ANGLE / 2.0f) )
        return ( (DifferenceValue >= 0) ? (DifferenceValue - MAX_ANGLE) : (DifferenceValue + MAX_ANGLE) );
    return DifferenceValue;
}
