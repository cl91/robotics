#include <libplayerc++/playerc++.h>
#include <cstdlib>
#include "args.h"

using namespace std;
using namespace PlayerCc;

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
		sleep(2);
//		position.RequestGeom();
//		laser.RequestGeom();

		int count = 0;
		for (;;) {
			robot.Read();
			position.SetSpeed(1.0, 0.5);
			int npoints = laser.GetCount();
			stringstream s;
			s << "output_" << count++;
			cout << s.str() << endl;
			FILE *f = fopen(s.str().c_str(), "w");
			for (int i = 0; i < npoints; i++) {
				fprintf(f, "%f %f\n", laser.GetBearing(i),
					laser[i]);
			}
			fclose(f);
		}
	} catch (PlayerError &e) {
		cerr << e << endl;
		return -1;
	}
	return 0;
}
