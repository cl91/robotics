#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <gsl/gsl_fit.h>

using namespace std;

// Compute the average distance to target from the laser data
double get_average_distance(const double (*data)[2], int count)
{
	double sum = 0;
	for (int i = 0; i < count; i++)
		sum += data[i][1];
	return sum / count;
}

// Used for ignoring targets that exceeds a distance threshold
#define DISTANCE_THRESHOLD	2.0
bool is_far_way(const double (*data)[2], int left, int right)
{
	int count = right - left;
	double avg = get_average_distance(data+left, count);
	if (avg > DISTANCE_THRESHOLD)
		return true;
	return false;
}

// Compute the maximal distances of laser data points to determine
// if the segment is a line. A line will not be detected as a leg
// in the leg detection algorithm
#define LINE_DETECTION_THRESHOLD	1
bool is_line(const double (*data)[2], int count)
{
	double r1 = data[0][1];
	double r2 = data[count-1][1];
	double theta = fabs(data[0][0]-data[count-1][0]);
	double d = sqrt(r1*r1+r2*r2 - 2*cos(theta)*r1*r2);
	if (d > LINE_DETECTION_THRESHOLD)
		return true;
	return false;
}

// Main leg detection algorithm. See the documentation for details
// data[i][0] --- angle
// data[i][1] --- distance
// all units are in meters
#define DIFF_DISTANCE_THRESHOLD	(0.15)
int detect_leg(const double (*data)[2], int count)
{
	int start = 1;
	int index = -1;
	double dist = -1;
	while (start < count) {
		int left, right;
		for (left = start; left < count; left++) {
			if (fabs(data[left][1] - data[left-1][1])
			    > DIFF_DISTANCE_THRESHOLD)
				break;
		}
		if (left == count)		// not found
			break;
		for (right = left+1; right < count; right++) {
			if (fabs(data[right][1] - data[right-1][1])
			    > DIFF_DISTANCE_THRESHOLD)
				break;
		}
		if (right == count)		// not found
			break;
		//		cout << '#' << left << ' ' << right << endl;
		if (((right - left) <= 2) // too few points
		    || is_far_way(data, left, right) // too far
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

// Test function for detect_leg()
// #if 0 comments it out
#if 0
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
		cerr << "Leg detected, bearing: " << data[leg][0]
		     << ", distance: " << data[leg][1] << endl;
	}
	return 0;
}
#endif
