

#include <bmirobot_hw/kalman_filter.h>

/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1; 
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 *   state - Klaman filter structure
 *   init_x - initial x state value   
 *   init_p - initial estimated error convariance
 * @outputs 
 * @retval  
 */
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 2e2;//10e-6;  /* predict noise convariance */
    state->r = 5e2;//10e-5;  /* measure error convariance */
}

/*
 * @brief   
 *   1 Dimension Kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 * @retval  
 *   Estimated result
 */
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    state->x(0,0) = init_x[0];
    state->x(1,0) = init_x[1];
    state->p(0,0) = init_p[0][0];
    state->p(0,1) = init_p[0][1];
    state->p(1,0) = init_p[1][0];
    state->p(1,1) = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A(0,0) = 1;
    state->A(0,1) = 0.1;
    state->A(1,0) = 0;
    state->A(1,1) = 1;
    //state->H       = {1,0};
    state->H(0,0)    = 1;
    state->H(1,0)    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q(0,0)    = 10e-7;
    state->q(1,0)    = 10e-7;
    state->r       << 10e-7, 10e-7, 10e-7, 10e-7;  /* estimated error convariance */
}

/*
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - Updated state value, Such as angle,velocity
 *   state->x[1] - Updated state value, Such as diffrence angle, acceleration
 *   state->p    - Updated estimated error convatiance matrix
 * @retval  
 *   Return value is equals to state->x[0], so maybe angle or velocity.
 */
float kalman2_filter(kalman2_state *state, float *z_measure)
{
    Eigen::Matrix<float, 2, 2> temp;
    Eigen::Matrix<float, 2, 2> temp1 ;
   // float temp = 0.0f;

    /* Step1: Predict */
	state->x= state->A * state->x;
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
	state->p = state->A * state->p * state->A.transpose() + state->q;

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
	temp = state->p * state->H.transpose();
//	temp1 = state->r + state->H * state->p * state->H.transpose();

//	state->gain = temp * temp1.inverse();
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    //temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    //state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    //state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    //state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p(0,0);
    //state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p(0,1);
    //state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p(1,0);
    //state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p(1,1);

    return state->x(0);
}

