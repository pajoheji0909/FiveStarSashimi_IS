#include <project1/pid.h>
#include <cmath>
#include <iostream>

#define BOUND(a) ((a > M_PI) ? (a - 2*M_PI) : ((a < -M_PI) ? (a + 2*M_PI) : a))
PID::PID() : error(0), error_sum(0), error_diff(0),
			 Kp(0.35f), Ki(0), Kd(0.2f)
{

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */

}

float PID::get_control(point car_pose, point goal_pose)
{
    float ctrl;
    /* TO DO
     *
     * implement pid algorithm
     *
    */
	const float dt = 0.1f;

	float theta_g = atan2(goal_pose.y - car_pose.y, goal_pose.x - car_pose.x);
	float Wg = 0.95, Wd = 0.05;
	float distsq = std::pow(car_pose.x - goal_pose.x, 2)
								 + std::pow(car_pose.y - goal_pose.y, 2);
	if (distsq < 0.25) {
		Wg = 0.5; Wd = 0.5;
	}
	const float target = theta_g * Wg + goal_pose.th * Wd;

  std::cout << "target : " << target << std::endl;

	float newerror =  target - car_pose.th;
	BOUND(newerror);

	const float pterm = Kp * newerror;

	error_sum += newerror * dt;
	const float iterm = Ki * error_sum;

	error_diff = (newerror - error) / dt;
	const float dterm = Kd * error_diff;

	ctrl = pterm + iterm + dterm;
	BOUND(ctrl);

	error = newerror;

    return ctrl;
}
