#include <libplayerc++/playerc++.h>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <unistd.h>
#include "args.h"

// forward declaration
int detect_leg(const double (*)[2], int);

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
#define ANGLE_THRESHOLD		dtor(5)
#define MOVE_TO_LEG_SPEED	0.05
#define MOVE_TO_LEG_TURN_RATE	dtor(5)	// deg/sec
void move_to_leg(double angle, double distance,
		 PlayerClient &robot,
		 Position2dProxy &position)
{
	double turn_rate = 0;
	double fwd_speed = MOVE_TO_LEG_SPEED;

	if (distance < MINIMAL_DISTANCE)
		return;

	if (fabs(angle) > ANGLE_THRESHOLD) {
		if (angle < 0)
			turn_rate = -MOVE_TO_LEG_TURN_RATE;
		else
			turn_rate = MOVE_TO_LEG_TURN_RATE;
	}
	position.SetSpeed(fwd_speed, turn_rate);
}

void read_laser_data(LaserProxy &laser, double (*&data)[2],
		     int count)
{
	if (!data)
		delete []data;
	data = new double[count][2];
	for (int i = 0; i < count; i++) {
		data[i][0] = laser.GetBearing(i);
		data[i][1] = laser[i];
	}
}

void dump_laser_data(const char *str, const double (*data)[2],
		     int count)
{
	for (int i = 0; i < count; i++) {
		cerr << data[i][0] << ' ' << data[i][1] << endl;
	}
}

double get_sonar_min(SonarProxy &sonar)
{
	double min = 100;
	for (int i = 0; i < 8; i++)
		if (sonar[i] < min)
			min = sonar[i];
	return min;
}

// Possible states for state machine
enum states {
	STATE_AVOID_OBSTACLE,
	STATE_MOVE_TO_LEG,
	STATE_OFFER_DRINKS,
	STATE_WAIT_FOR_SONAR,
	STATE_MOVE_AWAY,
};

#define SONAR_THRESHOLD		0.3
int main(int argc, char **argv)
{
	parse_args(argc, argv);

	try {
		PlayerClient robot(gHostname, gPort);
		Position2dProxy position(&robot, gIndex);
		LaserProxy laser(&robot, gIndex);
		SonarProxy sonar(&robot, gIndex);
		SpeechProxy speech(&robot, gIndex);
		cout << robot << endl;

		position.SetMotorEnable(true);
		srand(time(NULL));
		sleep(2);

		enum states state = STATE_AVOID_OBSTACLE;
		int count = 0;
		double (*data)[2] = NULL;
		int leg = -1;
		time_t start_time = 0; // time when robot is moving away
		time_t current_time = 0;

		while (count <= 0) {
			robot.Read();
			count = laser.GetCount();
		}
		while (true) {
			robot.Read();
			switch (state) {
			case STATE_AVOID_OBSTACLE:
				read_laser_data(laser, data, count);
				leg = detect_leg(data, count);
				if (leg != -1) {
					cout << "Leg detected, bearing: "
					     << data[leg][0]
					     << ", distance: "
					     << data[leg][1] << endl;
					cout << "Moving to target" << endl;
					state = STATE_MOVE_TO_LEG;
				} else {
					avoid_obstacle(position, laser);
					state = STATE_AVOID_OBSTACLE;
					sleep(1);
				}
				break;
			case STATE_MOVE_TO_LEG:
				move_to_leg(data[leg][0], data[leg][1],
					    robot, position);
				read_laser_data(laser, data, count);
				leg = detect_leg(data, count);
				if ((leg != -1) && (data[leg][1] < MINIMAL_DISTANCE)) {
					state = STATE_OFFER_DRINKS;
					cout << "Target reached" << endl;
				} else {
					state = STATE_MOVE_TO_LEG;
				}
				break;
			case STATE_OFFER_DRINKS:
				position.SetSpeed(0, 0);
				speech.Say("Take the drink!");
				sleep(1);
				state = STATE_WAIT_FOR_SONAR;
				break;
			case STATE_WAIT_FOR_SONAR:
				if (get_sonar_min(sonar) < SONAR_THRESHOLD) {
					speech.Say("The item is taken.");
					start_time = time(NULL);
					state = STATE_MOVE_AWAY;
				} else
					state = STATE_WAIT_FOR_SONAR;
				break;
			case STATE_MOVE_AWAY:
				position.SetSpeed(0.01, dtor(20));
				current_time = time(NULL);
				if ((current_time - start_time) < 9)
					state = STATE_MOVE_AWAY;
				else
					state = STATE_AVOID_OBSTACLE;
				break;
			default:
				cerr << "ERROR: unreachable case reached"
				     << endl;
				break;
			}
		}
	} catch (PlayerError &e) {
		cerr << e << endl;
		return -1;
	}
	return 0;
}
