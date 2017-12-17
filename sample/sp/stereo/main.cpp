//#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

	Mem2<Col3> imgs[2];
	{
		SP_ASSERT(loadBMP(imgs[0], SP_DATA_DIR  "/image/shiba02.bmp"));
		SP_ASSERT(loadBMP(imgs[1], SP_DATA_DIR  "/image/shiba04.bmp"));

		saveBMP(imgs[0], "src0.bmp");
		saveBMP(imgs[1], "src1.bmp");
	}

	// get camera parameter
	CamParam cam0, cam1;
	{
		SP_ASSERT(loadText(cam0, SP_DATA_DIR  "/image/shiba.txt"));
		SP_ASSERT(loadText(cam1, SP_DATA_DIR  "/image/shiba.txt"));
	}

	// estimate camera pose
	Pose stereo;
	{
		SfM sfm;
		sfm.addData(imgs[0], cam0);
		sfm.addData(imgs[1], cam1);
		sfm.update(10);

		stereo = sfm.m_views[1].pose * invPose(sfm.m_views[0].pose);
		//print(stereo);
	}


	RectParam rects[2];
	Mem2<Vec2> tables[2];
	
	// make remap table
	{
		rectify(rects[0], rects[1], cam0, cam1, stereo);

		for (int i = 0; i < 2; i++) {
			makeRemapTable(tables[i], rects[i]);
		}
	}


	// pre filter
	if(0){
		for (int i = 0; i < 2; i++) {
			normalizeFilter<Col3, Byte>(imgs[i], imgs[i], 7);
		}
	}

	Mem2<Col3> rimgs[2];

	// remap
	{
		for (int i = 0; i < 2; i++) {
			remap<Col3, Byte>(rimgs[i], imgs[i], tables[i]);

			saveBMP(rimgs[i], strFormat("rect%d.bmp", i).c_str());
		}
	}

	const int maxDisp = 140;
	const int minDisp = 90;

	StereoBase estimator;
	estimator.setRange(maxDisp, minDisp);
	estimator.setCam(rects[0].cam, rects[1].cam);

	// matching
	{
		estimator.execute(rimgs[0], rimgs[1]);
	}

	// output
	{
		Mem2<Col3> imgs[2];
		cnvDispToImg(imgs[0], estimator.getDispMap(StereoBase::StereoL), maxDisp, minDisp);
		cnvDispToImg(imgs[1], estimator.getDispMap(StereoBase::StereoR), maxDisp, minDisp);
		
		saveBMP(imgs[0], "dispL.bmp");
		saveBMP(imgs[1], "dispR.bmp");

		const Mem2<Vec3> &depthMap = estimator.getDepthMap(StereoBase::StereoL);

		Mem1<Vec3> pnts;
		for (int i = 0; i < depthMap.size(); i++) {
			if (depthMap[i].z > 0.0) {
				pnts.push(depthMap[i]);
			}
		}
		savePLY(pnts, "test.ply");
	}

	return 0;
}