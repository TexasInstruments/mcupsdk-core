/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   Computes the trigonometric sine value of the input angle using TMU.
 *
 * \param   [in] angleRad - input angle in radians within [0, 2PI]
 *
 * \return  Computed sine value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between 0 to 2PI.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_sin_pu(float angleRad);


/**
 * \brief   Computes the trigonometric cosine value of the input angle using TMU.
 *
 * \param   [in] angleRad - - input angle in radians within [0, 2PI]
 *
 * \return  Computed cosine value
 *
 *  \note   Usage Considerations:
 *          Valid input is limited to values between 0 to 2PI.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_cos_pu(float angleRad);


/**
 * \brief   Computes the trigonometric atan value of the input angle using TMU.
 *
 * \param   [in] angleRad - - input angle in radians
 *
 * \return  Computed atan value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between -1.0 to 1.0 per unit value.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_atan_pu(float angleRad);


/**
 * \brief   Computes the logarithmic value of the input using TMU.
 *
 * \param   [in] angleRad - - input value in float
 *
 * \return  Computed log value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between negative infinity to positive infinity.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_log_pu(float x);


/**
 * \brief   Computes the exponential value of the input using TMU.
 *
 * \param   [in] angleRad - - input value in float
 *
 * \return  Computed exponential value
 *
 * \note    Usage Considerations:
 *          No valid input range.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_iexp_pu(float x);


/**
 * \brief   Computes the trigonometric sine value of the input angle using TMU.
 *
 * \param   [in] angleRad - input angle in radians within [0, 2PI]
 *
 * \return  Computed sine value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between 0 to 2PI. The input is multiplied with 1/2PI to convert it to per unit value.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_sin(float angleRad);


/**
 * \brief   Computes the trigonometric cosine value of the input angle using TMU.
 *
 * \param   [in] angleRad - - input angle in radians within [0, 2PI]
 *
 * \return  Computed cosine value
 *
 *  \note   Usage Considerations:
 *          Valid input is limited to values between 0 to 2PI. The input is multiplied with 1/2PI to convert it to per unit value.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_cos(float angleRad);

/**
 * \brief   Computes the trigonometric atan value of the input angle using TMU.
 *
 * \param   [in] angleRad - - input angle in radians
 *
 * \return  Computed atan value
 *
 * \note    Usage Considerations:
 *          Valid input is limited to values between -1.0 to 1.0 per unit value. The computed output value is multiplied with 2PI to change it to radians.
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_atan(float angleRad);


/**
 * \brief   Computes the trigonometric atan2 value of the input values using TMU. Uses the quadratic built in TMU function to comput the ratio and quadrant value, which is then used to comput atan2 value of input.
 *
 * \param   [in] x - input value within the domain of arctangent
 * \param   [in] y - input value within the domain of arctangent
 *
 * \return  Computed atan2 value
 *
 * \note    Usage Considerations:
 *          No error checking is performed on input.
 */
extern float __attribute__((noinline, naked)) ti_tmu_atan2(float x, float y);

