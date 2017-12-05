//#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main(){

	// test camera parameter
	const CamParam cam = getCamParam(640, 480);

	// detected marker pos[pixel], and world marker pos[mm]
	Mem1<Mem1<Vec2> > pixsList, objsList;

	// robot pose
	Mem1<Pose> hand2basePoses;

	// generate dataset
	{
		// hand to cam pose
		const Pose hand2camPose = getPose(getRotAngleZ(-0.1 * SP_PI), getVec(0.0, 100.0, 0.0));

		// base to mrk pose
		const Pose base2mrkPose = getPose(getRotAngleZ(+0.1 * SP_PI), getVec(0.0, 0.0, -600.0));

		printf("--------------------------------------------------------------------------------\n");
		printf("ground truth\n");
		printf("--------------------------------------------------------------------------------\n");
		printf("hand2camPose\n");
		print(hand2camPose);
		printf("base2mrkPose\n");
		print(base2mrkPose);
		printf("\n");

		Mem1<Pose> mrk2camPoses;

		// generate test pose
		{
			mrk2camPoses.push(getPose(getVec(+0.0, +0.0, 400)));
			mrk2camPoses.push(getPose(getVec(+0.0, +10.0, 400)) * getRotAngleX(+30 * SP_PI / 180.0));
			mrk2camPoses.push(getPose(getVec(+0.0, -10.0, 400)) * getRotAngleX(-30 * SP_PI / 180.0));
			mrk2camPoses.push(getPose(getVec(+10.0, +0.0, 400)) * getRotAngleY(+30 * SP_PI / 180.0));
			mrk2camPoses.push(getPose(getVec(-10.0, +0.0, 400)) * getRotAngleY(-30 * SP_PI / 180.0));
		}

		for (int i = 0; i < mrk2camPoses.size(); i++) {
			const Pose hand2basePose = invPose(base2mrkPose) * invPose(mrk2camPoses[i]) * hand2camPose;
			hand2basePoses.push(hand2basePose);
		}


		const DotMarkerParam mrk(5, 5, 30);
		const Mem2<Vec2> mrkMap = mrk.map * mrk.distance;

		for (int i = 0; i < mrk2camPoses.size(); i++) {
			Mem1<Vec2> pixs, objs;

			if (1) {
				// generate test points

				const double sigma = 0.1;
				for (int n = 0; n < mrkMap.size(); n++) {
					const Vec3 pos = mrk2camPoses[i] * getVec(mrkMap[n].x, mrkMap[n].y, 0.0);
					const Vec2 pix = mulCamD(cam, prjVec(pos)) + randVecGauss(sigma, sigma);

					pixs.push(pix);
					objs.push(mrkMap[n]);
				}
			}
			else {
				// generate test images

				Mem2<Byte> gry;
				renderMarker(gry, cam, mrk2camPoses[i], mrkMap);
				saveBMP(gry, strFormat("test%02d.bmp", i).c_str());

				DotMarker detector;
				detector.setCam(cam);
				detector.setMrk(mrk);
				detector.execute(gry);

				pixs = detector.getCrspPix();
				objs = detector.getCrspObj();
			}

			pixsList.push(pixs);
			objsList.push(objs);
		}
	}

	{
		Pose hand2camPose;
		Pose base2mrkPose;
		calibRobotCam(hand2camPose, base2mrkPose, hand2basePoses, cam, pixsList, objsList);

		printf("--------------------------------------------------------------------------------\n");
		printf("estimate\n");
		printf("--------------------------------------------------------------------------------\n");
		printf("hand2camPose\n");
		print(hand2camPose);
		printf("base2mrkPose\n");
		print(base2mrkPose);
		printf("\n");
	}

	return 0;
}