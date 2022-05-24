 #ifndef _Kalman_h
 #define _Kalman_h

struct Kalman{
      /* Kalman filter variables */
      double Q_angle; // Process noise variance for the accelerometer
      double Q_bias; // Process noise variance for the gyro bias
      double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

      double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
      double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
      double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

      double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
      double K[2]; // Kalman gain - This is a 2x1 vector
      double y; // Angle difference
      double S; // Estimate error
  };


  void   Init(struct Kalman* klm){
      /* We will set the variables like so, these can also be tuned by the user */
      klm->Q_angle = 0.001;
      klm->Q_bias = 0.003;
      klm->R_measure = 0.03;

      klm->angle = 0; // Reset the angle
      klm->bias = 0; // Reset bias

      klm->P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en->wikipedia->org/wiki/Kalman_filter#Example_application->2C_technical
      klm->P[0][1] = 0;
      klm->P[1][0] = 0;
      klm->P[1][1] = 0;
  }


  // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
  double getAngle(struct Kalman * klm, double newAngle, double newRate, double dt) {

      /* Step 1 */
      klm->rate = newRate - klm->bias;
      klm->angle += dt * klm->rate;

      // Update estimation error covariance - Project the error covariance ahead
      /* Step 2 */
      klm->P[0][0] += dt * (dt*klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);
      klm->P[0][1] -= dt * klm->P[1][1];
      klm->P[1][0] -= dt * klm->P[1][1];
      klm->P[1][1] += klm->Q_bias * dt;

      // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
      // Calculate Kalman gain - Compute the Kalman gain
      /* Step 4 */
      klm->S = klm->P[0][0] + klm->R_measure;
      /* Step 5 */
      klm->K[0] = klm->P[0][0] / klm->S;
      klm->K[1] = klm->P[1][0] / klm->S;

      // Calculate angle and bias - Update estimate with measurement zk (newAngle)
      /* Step 3 */
      klm->y = newAngle - klm->angle;
      /* Step 6 */
      klm->angle += klm->K[0] * klm->y;
      klm->bias += klm->K[1] * klm->y;

      // Calculate estimation error covariance - Update the error covariance
      /* Step 7 */
      klm->P[0][0] -= klm->K[0] * klm->P[0][0];
      klm->P[0][1] -= klm->K[0] * klm->P[0][1];
      klm->P[1][0] -= klm->K[1] * klm->P[0][0];
      klm->P[1][1] -= klm->K[1] * klm->P[0][1];

      return klm->angle;
  }

  void setAngle(struct Kalman* klm, double newAngle) { klm->angle = newAngle; }
#endif
