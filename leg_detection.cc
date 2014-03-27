#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <gsl/gsl_fit.h>

using namespace std;

double get_average_distance(double (*data)[2], int count)
{
	double sum = 0;
	for (int i = 0; i < count; i++)
		sum += data[i][1];
	return sum / count;
}

#define DISTANCE_THREHOLD	2.0
bool is_far_way(double (*data)[2], int left, int right)
{
	int count = right - left;
	double avg = get_average_distance(data+left, count);
	if (avg > DISTANCE_THREHOLD)
		return true;
	return false;
}

int rand_unif(int n)
{
	return (int) ((double) n *
		      (double) rand() / (double) RAND_MAX);
}

// Fit the data (converted to cartisian coordinates) to a straight line
// Returns true if error is small enough
// Used for detecting if the segment of data represents wall or legs
bool is_line(double (*data)[2], int count)
{

	return false;
}

// data[i][0] --- angle
// data[i][1] --- distance

// all units are in meters
#define DIFF_DISTANCE_THREHOLD	(0.15)
int detect_leg(double (*data)[2], int count)
{
	int start = 1;
	int index = -1;
	double dist = -1;
	while (start < count) {
		int left, right;
		for (left = start; left < count; left++) {
			if (fabs(data[left][1] - data[left-1][1])
			    > DIFF_DISTANCE_THREHOLD)
				break;
		}
		if (left == count)		// not found
			break;
		for (right = left+1; right < count; right++) {
			if (fabs(data[right][1] - data[right-1][1])
			    > DIFF_DISTANCE_THREHOLD)
				break;
		}
		if (right == count)		// not found
			break;
		cout << '#' << left << ' ' << right << endl;
		if (((right - left) <= 2) // too few points
		    //		    || is_far_way(data, left, right) // too far
		    || is_line(data+left, right-left)) { // not a leg
			start = right;
		} else {	// found candidate
			double new_dist = get_average_distance(data+left,
							       right-left);
			if (dist < 0 || new_dist < dist) {
				dist = new_dist;
				index = (left+right) / 2;
			} else {
				start = right;
			}
		}
	}
	return index;
}

int main(int argc, char **argv)
{
	char *fname = "output_10";
	if (argc == 2 && argv[1] != NULL)
		fname = argv[1];
	ifstream fin(fname);
	int count = 0;
	double data[1024][2];
	string line;

	while (getline(fin, line)) {
		double angle, distance;
		istringstream ss(line);
		ss >> angle >> distance;
		data[count][0] = angle;
		data[count][1] = distance;
		++count;
	}

	int leg = -1;
	if ((leg = detect_leg(data, count)) != -1) {
		cout << "Leg detected, bearing: " << data[leg][0]
		     << ", distance: " << data[leg][1] << endl;
	}
	return 0;
}
