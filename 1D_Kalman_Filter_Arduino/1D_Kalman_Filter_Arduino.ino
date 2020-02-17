/*
   ~~~~~Prerequisites~~~~~

   This code uses the 'Guassian' library, download it here: https://github.com/ivanseidel/Gaussian

   ~~~~~Details~~~~~~~~~~~~

   Author: Ben Money-Coomes
   Date: 26/4/19
   Purpose: Implement 1-Dimensional kalman filter
   References: Uses implementation as described at https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

   ~~~~~Version Control~~~

   1D_KF_v1.00 - basic 'fake' state, measurement and process model functions are working (KALMAN NOT IMPLEMENTED YET!)
   1D_KF_v1.01 - implementing kalman filter - it's working with a single measurement (from the fake gyroscope)
   1D_KF_v1.02 - incorporating a second measurement (i.e. from the magnetometer)
   1D_Kalman_Filter_Arduino - Creating Github Repository
   1D_KF_v1.03 - working on intermediate version where we seperate kalman filter out into a class
   1D_Kalman_Filter_Arduino_v1.01 - [Checkpoint] Kalman Filter implemented as seperate class
*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Library Includes.                                                              *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "univariate_kalman.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Definitions                                                                      *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define TIMESTEP 100
#define TIMESTEP_2 200

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Variables                                                                      *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//--------Setup for timers-------------
unsigned long general_timer;
unsigned long general_timer_2;
unsigned long general_timer_3;

//--------Master variable for Romi orientation----------
float romiOrientation = 0; //start at 0
float last_romiOrientation = 0;

//--------Gaussians for measurements----------
Gaussian measurementGyroscope = Gaussian(0, 10);
Gaussian measurementMagnetometer = Gaussian(0, 20); //centred around mean 0, with variance 20

//--------Initialise Kalman filter class----------
Kalman_Filter KF; //create kalman filter for tracking orientation

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Start of main program                                                          *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup() {
  Serial.begin(9600);
  randomSeed(1); //repeatable pseudo random seed
}

void loop() {
  //define variables---
  unsigned long elapsed_time_general_timer_3 = millis() - general_timer_3;
  //call function------
  update_romi_orientation_hidden();

  if ( elapsed_time_general_timer_3 > TIMESTEP_2 ) // print values
  {
    // update timestamp
    general_timer_3 = millis();

    //update 'fake' prediction and measurements (i.e. this part is simulated)
    gaussian_prediction(KF.xPosterior, last_romiOrientation, TIMESTEP_2, KF.processModel); //This is equivalent to getting a prediction from the Kinematics
    gaussian_noisy_measurement(romiOrientation, 10, measurementGyroscope) ;
    gaussian_noisy_measurement(romiOrientation, 20, measurementMagnetometer) ;

    //kalman filter using one measurement from the gyroscope
    KF.kalman_predict(KF.xPosterior, KF.processModel, KF.xPrior); //i.e. return mean, variance to xPrior from combination of xPosterior and process model gaussians
    KF.kalman_update(KF.xPrior, measurementGyroscope, KF.xPosterior); //i.e. return mean, variance to xPosterior from combination of xPrior and measurement gaussians

    //kalman filter using two measurements. One from the gyroscope, one from the magnetometer
    //    KF.kalman_predict(KF.xPosterior_2, KF.processModel, KF.xPrior); //i.e. return mean, variance to xPrior from combination of xPosterior and process model gaussians
    //    KF.kalman_update(KF.xPrior, measurementGyroscope, KF.xPosterior); //i.e. return mean, variance to xPosterior from combination of xPrior and measurement gaussians
    //    KF.kalman_update(KF.xPosterior, measurementMagnetometer, KF.xPosterior_2); //another update cycle

    // print values for plotting
    Serial.print(romiOrientation);
    //    Serial.print(",");
    //    Serial.print(measurementGyroscope.mean);
    //    Serial.print(",");
    //    Serial.print(measurementMagnetometer.mean);
    //    Serial.print(",");
    //    Serial.print(processModel.mean);
    Serial.print(",");
    Serial.print(KF.xPosterior.mean);
    Serial.print(",");
    Serial.println(KF.xPosterior_2.mean);

    //save previous value of angle to generate 'fake kinematic model'
    last_romiOrientation = romiOrientation;

  }//end of general_timer_3

} //end of 'loop()'



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  This is where the 'fake' system state is created (the orientation of romi)     *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~~~~UPDATE 'FAKE' ORIENTATION FUNCTION~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void update_romi_orientation_hidden() //note I call this 'hidden' because this is the true state of ROMI, normally not known to us
{
  static float targetOrientation;
  unsigned long elapsed_time_general_timer = millis() - general_timer;
  unsigned long elapsed_time_general_timer_2 = millis() - general_timer_2;

  if ( elapsed_time_general_timer > 5000 ) // set a target for the Romi periodically
  {
    // update timestamp
    general_timer = millis();

    targetOrientation = random(0, 360);
  }

  if ( elapsed_time_general_timer_2 > TIMESTEP ) // move towards new position
  {
    // update timestamp
    general_timer_2 = millis();

    float targetDifference = (fmod(((targetOrientation - romiOrientation) + 180 + 360), 360 ) - 180); //returns value between -180 -> 180 indicating direction of turn

    if (targetDifference >= 0) {
      if (targetDifference <= 2) {
        romiOrientation = targetOrientation;
      }
      else {
        romiOrientation += 2;
        romiOrientation = fmod((romiOrientation) + 360, 360); //fmod maps the value using floating modulus between 0 --> 360 degrees
      }
    }
    else {
      if (targetDifference >= -2) {
        romiOrientation = targetOrientation;
      }
      else {
        romiOrientation -= 2;
        romiOrientation = fmod((romiOrientation) + 360, 360);
      }
    }

  } //End of general_timer_2

} //End of function

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  This is where the 'fake' measurement and prediction data is created            *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~~~~TAKE 'FAKE' NOISY MEASURMEENT FUNCTION~~~~~~~~~~~~~~~~~~~~~~~~~~
void gaussian_noisy_measurement(float myMean, float myVariance, Gaussian & myMeasurement) {

  Gaussian myRandomMeasurementGaussian = Gaussian(myMean, myVariance); //create a gaussian to sample a random value from (to recreate noise)

  // This will generate a random value, normaly distribute around the mean
  float myRandomMeasurement = myRandomMeasurementGaussian.random();

  //map to interval 0 --> 360
  myRandomMeasurement = fmod((myRandomMeasurement) + 360, 360);

  //assign to the measurement gaussian passed to this function
  myMeasurement.mean = myRandomMeasurement;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~MAKE 'FAKE' NOISY PREDICTION FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~

void gaussian_prediction(Gaussian my_xPosterior, float myLastMean, int myTimestep, Gaussian & myProcessModel) {

  //equivalent to [float dx = (myMean - myLastMean)/ myTimestep;]
  float dx = (fmod(((my_xPosterior.mean - myLastMean) + 180 + 360), 360 ) - 180) / myTimestep; //returns value between -180 -> 180 indicating direction of turn

  Gaussian myPredictionGaussian = Gaussian(dx, 0.01); //create a gaussian to sample a random value from (to recreate noise)

  // This will generate a random value, normaly distribute around the mean (i.e. recreation of white noise)
  float myRandom_dx = myPredictionGaussian.random();

  //set process model parameters
  myProcessModel.mean = myRandom_dx * myTimestep;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
