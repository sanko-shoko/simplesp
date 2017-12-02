#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main(){

	// test camera parameter
	const CamParam cam = getCamParam(640, 480);

	// detector
	DotMarker detector;
	{
		const DotMarkerParam mrk(5, 5, 30);
		detector.setCam(cam);
		detector.setMrk(mrk);
	}

	// base to mrk pose
	const Pose base2mrkPose = getPose(getRotAngleZ(+0.1 * SP_PI), getVec(0.0, 0.0, -600.0));

	// hand to cam pose
	const Pose hand2camPose = getPose(getRotAngleZ(-0.1 * SP_PI), getVec(0.0, 100.0, 0.0));


	// detected marker pos[pixel], and marker pos[mm]
	Mem1<Mem1<Vec2> > pixsList, objsList;

	// robot pose
	Mem1<Pose> hand2basePoses;

	// init dataset
	{
		// generate test pose
		Mem1<Pose> mrk2camPoses;
		{
			mrk2camPoses.push(getPose(getVec(+0.0, +0.0, 400)));
			mrk2camPoses.push(getPose(getVec(+0.0, +10.0, 400)) * getRotAngleX(+30 * SP_PI / 180.0));
			mrk2camPoses.push(getPose(getVec(+0.0, -10.0, 400)) * getRotAngleX(-30 * SP_PI / 180.0));
			mrk2camPoses.push(getPose(getVec(+10.0, +0.0, 400)) * getRotAngleY(+30 * SP_PI / 180.0));
			mrk2camPoses.push(getPose(getVec(-10.0, +0.0, 400)) * getRotAngleY(-30 * SP_PI / 180.0));
		}

		const Mem2<Vec2> mrkMap = detector.getMrk().map * detector.getMrk().distance;

		for (int i = 0; i < mrk2camPoses.size(); i++) {
			Mem1<Vec2> pixs, objs;

			if (1) {
				// generate test points

				const double sigma = 0.1;
				for (int n = 0; n < mrkMap.size(); n++) {
					const Vec3 pos = mrk2camPoses[i] * getVec(mrkMap[n].x, mrkMap[n].y, 0.0);
					const Vec2 pix = mulCam(cam, npxDist(cam, prjVec(pos))) + randVecGauss(sigma, sigma);

					pixs.push(pix);
					objs.push(mrkMap[n]);
				}
			}
			else {
				// generate test images

				Mem2<Byte> gry;
				renderMarker(gry, cam, mrk2camPoses[i], mrkMap);
				saveBMP(gry, strFormat("test%02d.bmp", i).c_str());

				detector.execute(gry);

				pixs = detector.getCrspPix();
				objs = detector.getCrspObj();
			}

			pixsList.push(pixs);
			objsList.push(objs);

			const Pose hand2basePose = invPose(base2mrkPose) * invPose(mrk2camPoses[i]) * hand2camPose;
			hand2basePoses.push(hand2basePose);
		}
	}

	Pose hand2camPoseEst;
	calibRobotCam(hand2camPoseEst, cam, hand2basePoses, pixsList, objsList);

	print(hand2camPoseEst);
	print(hand2camPose);


	return 0;
}