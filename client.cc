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
      position.SetSpeed(newspeed/4, newturnrate/8);
}

#define MINIMAL_DISTANCE	0.4
#define MOVE_TO_LEG_SPEED	0.05
#define MOVE_TO_LEG_TURN_RATE	5	// deg/sec
void move_to_leg(double angle, double distance,
		 PlayerClient &robot,
		 Position2dProxy &position)
{
	double turn_rate = MOVE_TO_LEG_TURN_RATE;
	double fwd_speed = MOVE_TO_LEG_SPEED;
	double dt;		// time

	cout << "Moving to target" << endl;
	if (angle < 0)
		position.SetSpeed(0, -dtor(turn_rate));
	else
		position.SetSpeed(0, dtor(turn_rate));
	if (distance < MINIMAL_DISTANCE)
		return;
	dt = fabs(angle) / dtor(turn_rate);
	dt -= 0.1;		// robot.Read() blocks at 10Hz
	usleep(dt * 1000000);
	robot.Read();
	dt = (distance - MINIMAL_DISTANCE) / fwd_speed;
	dt -= 0.1;		// robot.Read() blocks at 10Hz
	position.SetSpeed(fwd_speed, 0);
	usleep(dt * 1000000);
	robot.Read();
	position.SetSpeed(0, 0);
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

// bool reached_target(PlayerClient &robot,
// 		    LaserProxy &laser,
// 		    double (*&data)[2], int count)
// {
// 	read_laser_data(laser, data, count);
// 	int leg = detect_leg(data, count);
// 	if (leg == -1)
// 		return true;
// 	double distance = data[leg][1];
// 	if (distance < MINIMAL_DISTANCE)
// 		return true;
// 	return false;
// }

void dump_laser_data(const char *str, const double (*data)[2],
		     int count)
{
	for (int i = 0; i < count; i++) {
		cerr << data[i][0] << ' ' << data[i][1] << endl;
	}
}

// Possible states for state machine
enum states {
	STATE_AVOID_OBSTACLE,
	STATE_MOVE_TO_LEG,
	STATE_REACHED_LEG,
	STATE_OFFER_DRINKS,
	STATE_WAIT_FOR_SONAR,
	STATE_MOVE_AWAY,
};

#define SONAR_THRESHOLD		0.2
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
//				if (reached_target(robot, laser, data, count)) {
				state = STATE_OFFER_DRINKS;
				cout << "Target reached" << endl;
//				}
//				else
//					state = STATE_MOVE_TO_LEG;
				break;
			case STATE_OFFER_DRINKS:
				speech.Say("Take the drink!");
				sleep(1);
				state = STATE_WAIT_FOR_SONAR;
				break;
			case STATE_WAIT_FOR_SONAR:
				if ((sonar[0] < SONAR_THRESHOLD) &&
				    (sonar[1] < SONAR_THRESHOLD))
					state = STATE_MOVE_AWAY;
				else
					state = STATE_WAIT_FOR_SONAR;
				break;
			case STATE_MOVE_AWAY:
				position.SetSpeed(-0.3, 0.8);
				sleep(1);
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
