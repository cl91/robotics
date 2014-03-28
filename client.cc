#include <libplayerc++/playerc++.h>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include "args.h"

using namespace std;
using namespace PlayerCc;

// Generate a random number uniformly on [0,1]
double rand_norm()
{
	return (double) rand() / RAND_MAX;
}

// Generate a random number uniformly on [0,d]
double rand_unif(double d)
{
	return rand_norm() * d;
}

// // Wander around to find people to approach
// // Basically just generates random forward_speed and turn_speed
// #define MAX_SPEED	1
// #define MAX_TURN	90	// degree/sec
// void wander(Position2dProxy &position)
// {
// 	double forward_speed = rand_unif(MAX_SPEED);
// 	double turn_speed = dtor(rand_unif(MAX_TURN));
// 	position.SetSpeed(forward_speed, turn_speed);
// }

// Avoid obstacles using sonar
// The idea is shamelessly borrowed from the player tutorial
// on CECIL, pp. 58
// #define AVOID_DISTANCE_THRESHOLD       	0.4
// #define AVOID_SPEED			0.1
// #define AVOID_TURN_SPEED		60	// deg/sec
// void avoid_obstacle(Position2dProxy &position, SonarProxy &sonar)
// {
// 	// left sonar is no.2
// 	// right is no.3
// 	// front and back is no.0 and no.1
// 	if (sonar[2] < AVOID_DISTANCE_THRESHOLD)
// 		position.SetSpeed(0, dtor(AVOID_TURN_SPEED));
// 	else if (sonar[3] < AVOID_DISTANCE_THRESHOLD)
// 		position.SetSpeed(0, -dtor(AVOID_TURN_SPEED));
// 	else if (sonar[0] < AVOID_DISTANCE_THRESHOLD)
// 		position.SetSpeed(-AVOID_SPEED, 0);
// 	else if (sonar[1] < AVOID_DISTANCE_THRESHOLD)
// 		position.SetSpeed(AVOID_SPEED, 0);
// }

// Laser Obstacle Avoid, shamelessly borrowed from the lab tutorial
void avoid_obstacle(Position2dProxy &position, LaserProxy &laser)
{
      double newspeed = 0;
      double newturnrate = 0;
      double min_right = laser.GetMinRight();
      double min_left = laser.GetMinLeft();
      double l = (1e5 * min_right)/500 - 100;
      double r = (1e5 * min_left)/500 - 100;
      if (l > 100)
	      l = 100;
      if (r > 100)
	      r = 100;

      newspeed = (r+l)/1e3;

      newturnrate = r - l;
      newturnrate = limit(newturnrate, -40.0, 40.0);
      newturnrate = dtor(newturnrate);

      // write commands to robot
      position.SetSpeed(newspeed, newturnrate);
}

#define MINIMAL_DISTANCE	0.4
#define MOVE_TO_LEG_SPEED	0.1
#define MOVE_TO_LEG_TURN_RATE	20	// deg/sec
bool move_to_leg(double angle, double distance,
		 Position2dProxy &position)
{
	double fwd_speed, turn_rate;
	bool reached_min_dist = false;

	if (distance < MINIMAL_DISTANCE) {
		fwd_speed = 0;
		reached_min_dist = true;
	} else
		fwd_speed = MOVE_TO_LEG_SPEED;

	if (angle < 0)
		turn_rate = -dtor(MOVE_TO_LEG_TURN_RATE);
	else
		turn_rate = dtor(MOVE_TO_LEG_TURN_RATE);

	position.SetSpeed(fwd_speed, turn_rate);
	return reached_min_dist;
}

// forward declaration
int detect_leg(const double (*)[2], int);
int main(int argc, char **argv)
{
	parse_args(argc, argv);

	try {
		PlayerClient robot(gHostname, gPort);
		Position2dProxy position(&robot, gIndex);
		LaserProxy laser(&robot, gIndex);
		SonarProxy sonar(&robot, gIndex);

		cout << robot << endl;

		position.SetMotorEnable(true);
		srand(time(NULL));
		sleep(2);
//		position.RequestGeom();
//		laser.RequestGeom();

		bool reached_min_dist = false;
		for (;;) {
			robot.Read();
			// if we can detect human presence, move to him
			int count = laser.GetCount();
			double (*data)[2] = new double[count][2];
			for (int i = 0; i < count; i++) {
				data[i][0] = laser.GetBearing(i);
				data[i][1] = laser[i];
			}
			int leg = detect_leg(data, count);
			if (leg != -1) {
				cerr << "Leg detected, bearing: " << data[leg][0]
				     << ", distance: " << data[leg][1] << endl;
				if (!reached_min_dist) {
					reached_min_dist =
						move_to_leg(data[leg][0], data[leg][1], position);
					sleep(1);
					continue;
				}
				position.SetSpeed(0.2, 0.1);
				sleep(2);
				position.SetSpeed(0, 0);
				continue;
			}
			// no legs detected, check for obstacles to avoid
			avoid_obstacle(position, laser);
			// no need to avoid obstacle, just wander around
			// wander(position);
			// wait for some time so that robot can move
			sleep(1);
		}
	} catch (PlayerError &e) {
		cerr << e << endl;
		return -1;
	}
	return 0;
}
