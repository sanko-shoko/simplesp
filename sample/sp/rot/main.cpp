#include "simplesp.h"

using namespace sp;

int main(){

	// rotation test
	const Vec3 euler = getVec(10.0, 20.0, 30.0) / 180.0 * SP_PI;

	// angle -> matrix
	{
		printf("------------------------------\n");
		printf("angle -> matrix\n");
		const Mat mat = getMatAngleX(euler.x);

		printf("angle %lf\n", euler.x);
		print(mat);
		printf("\n");
	}

	// euler -> matrix
	{
		printf("------------------------------\n");
		printf("euler -> matrix\n");

		const Mat mat = getMatEuler(euler);

		print(euler);
		print(mat);
		printf("\n");
	}

	// matrix -> euler
	{
		printf("------------------------------\n");
		printf("matrix -> euler\n");

		const Mat mat = getMatEuler(euler);
		const Vec3 vec = getEuler(mat);

		print(mat);
		print(vec);
		printf("\n");
	}

	// quaternion -> matrix
	{
		printf("------------------------------\n");
		printf("quaternion -> matrix\n");
		const Rot rot = getRotEuler(euler);
		const Mat mat = getMat(rot);

		print(rot);
		print(mat);
		printf("\n");
	}

	// matrix -> quaternion
	{
		printf("------------------------------\n");
		printf("matrix -> quaternion\n");

		const Mat mat = getMatEuler(euler);
		const Rot rot = getRot(mat);

		print(mat);
		print(rot);
		printf("\n");
	}

	return 0;
}