#ifndef _univariate_kalman
#define _univariate_kalman_h
#include <Gaussian.h>
#include <math.h>

class Kalman_Filter
{
  public:

    //--------Gaussians for kalman filter
    Gaussian xPrior = Gaussian(); // centred around mean 0, with really large variance
    Gaussian xPosterior = Gaussian();
    Gaussian xPosterior_2 = Gaussian(); //Posterior after 2nd measurement update (NOTE: this is not needed if only one measurement is present!)
    Gaussian processModel = Gaussian(0,10); //centred around mean 0, with variance 10
    Gaussian measurementComplimentaryFilter = Gaussian(0,20);  //centred around mean 0, with variance 20

    //--------Function definitions for kalman filter
    void kalman_predict(Gaussian, Gaussian, Gaussian&);
    void kalman_update(Gaussian, Gaussian, Gaussian&);

  private:
    
};

/*Class constructor

Kalman_Filter::Kalman_Filter()
 {
  //pass
 }
*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  This is where we define the Kalman filter                                      *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~~~~KALMAN PREDICT FUNCTION~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Kalman_Filter:: kalman_predict(Gaussian posterior, Gaussian movement, Gaussian & xPrior)
{
  float my_x = posterior.mean; //mean of the posterior
  float P = posterior.variance; //variance of the posterior

  float dx = movement.mean; //mean of the movement
  float Q = movement.variance; //variance of the movement

  my_x = my_x + dx;
  P = P + Q;

  my_x = fmod((my_x) + 360, 360); //map into the interval 0--> 360

  //pass the values as the new prior here!
  xPrior.mean = my_x;
  xPrior.variance = P;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~KALMAN UPDATE FUNCTION~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Kalman_Filter:: kalman_update(Gaussian prior, Gaussian measurement, Gaussian & xPosterior)
{

  float my_x = prior.mean; //mean of the prior
  float P = prior.variance; //variance of the prior

  float z = measurement.mean; //mean of the measurement
  float R = measurement.variance; //variance of the measurement

  float y = z - my_x; //residual
  float K = P / (P + R); //Kalman gain

  my_x = my_x + K * y; //posterior mean
  P = (1 - K) * P; //posterior variance

  my_x = fmod((my_x) + 360, 360); //map into the interval 0--> 360

  //pass the values back into the posterior here!
  xPosterior.mean = my_x;
  xPosterior.variance = P;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#endif
