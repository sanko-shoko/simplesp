#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

int main(){
	
	// test camera parameter
	CamParam cam = getCamParam(640, 480);

	// test marker pose
	Mem1<Pose> poses;

	const Pose stereo = getPose(getVec(-1.0, 0.0, 0.0));

	// marker position
	const DotMarkerParam mrk(5, 5, 30);
	const Mem2<Vec2> mrkMap = mrk.map * mrk.distance;
	
	{
		cam.cx = 300;
		cam.cy = 250;
		cam.k1 = 0.1;
		cam.k2 = 0.1;
		cam.k3 = 0.1;
		cam.p1 = 0.1;
		cam.p2 = 0.1;

		printf("grand truth\n");
		print(cam);

		// generate test pose
		poses.push(getPose(getVec(+00.0, 0.0, 400)));
		poses.push(getPose(getVec(0.0, +10.0, 400)) * getRotAngleX(+30 * SP_PI / 180.0));
		poses.push(getPose(getVec(0.0, -10.0, 400)) * getRotAngleX(-30 * SP_PI / 180.0));
		poses.push(getPose(getVec(+10.0, 0.0, 400)) * getRotAngleY(+30 * SP_PI / 180.0));
		poses.push(getPose(getVec(-10.0, 0.0, 400)) * getRotAngleY(-30 * SP_PI / 180.0));
	}

	// test (generate test points)
	{
		printf("\n\n");
		printf("--------------------------------------------------------------------------------\n");
		printf("cam test (generate test points)\n");
		printf("--------------------------------------------------------------------------------\n");

		Mem1<Mem1<Vec2> > pixsList, objsList;

		// generate test points
		for (int i = 0; i < poses.size(); i++){
			Mem1<Vec2> pixs, objs;
			for (int n = 0; n < mrkMap.size(); n++){
				const Vec3 pos = poses[i] * getVec(mrkMap[n].x, mrkMap[n].y, 0.0);
				const Vec2 pix = mulCam(cam, npxDist(cam, prjVec(pos))) + randVecGauss(0.1, 0.1);

				pixs.push(pix);
				objs.push(mrkMap[n]);
			}
			pixsList.push(pixs);
			objsList.push(objs);
		}

		// simplesp calibration 
		{
			printf("\n\n");
			printf("simplesp\n");

			// calibration
			CamParam dst;
			const double rms = calibCam(dst, cam.dsize[0], cam.dsize[1], pixsList, objsList);
			printf("simplesp rms error: %g\n", rms);

			print(dst);
			saveText(dst, "cam0.txt");
		}

		// opencv calibration 
		{
			using namespace std;
			using namespace cv;

			printf("\n\n");
			printf("opencv\n");

			// Point2d is not available in cv::calibrateCamera
			vector<vector<Point2f> > cvpixsList;
			vector<vector<Point3f> > cvobjsList;

			for (int i = 0; i < pixsList.size(); i++){
				vector<Point2f> cvpixs;
				vector<Point3f> cvobjs;
				for (int j = 0; j < pixsList[i].size(); j++){
					cvpixs.push_back(Point2f(static_cast<float>(pixsList[i][j].x), static_cast<float>(pixsList[i][j].y)));
					cvobjs.push_back(Point3f(static_cast<float>(objsList[i][j].x), static_cast<float>(objsList[i][j].y), 0.f));
				}
				cvpixsList.push_back(cvpixs);
				cvobjsList.push_back(cvobjs);
			}

			const Size imgSize(cam.dsize[0], cam.dsize[1]);
			cv::Mat camMat, dist;
			vector<cv::Mat> rvecs, tvecs;

			const double rms = cv::calibrateCamera(cvobjsList, cvpixsList, Size(cam.dsize[0], cam.dsize[1]), camMat, dist, rvecs, tvecs);
			printf("opencv rms error: %g\n", rms);
			cout << "camMat = " << endl << camMat << endl;
			cout << "dist = " << endl << dist << endl;
		}
	}

	// cam test (generate test images)
	{
		printf("\n\n");
		printf("--------------------------------------------------------------------------------\n");
		printf("cam test (generate test images)\n");
		printf("--------------------------------------------------------------------------------\n");

		Mem1<Mem2<Byte> > imgList(poses.size());

		// generate test images
		for (int i = 0; i < poses.size(); i++){
			renderMarker(imgList[i], cam, poses[i], mrkMap);
			char str[256];
			sprintf(str, "test%02d.bmp", i);
			saveBMP(imgList[i], str);
		}

		// simplesp calibration 
		{
			printf("\n\n");
			printf("simplesp\n");

			DotMarker dotMarker;
			dotMarker.setMrk(mrk);

			Mem1<Mem1<Vec2> > pixsList, objsList;

			// detect points
			for (int i = 0; i < imgList.size(); i++){
				if (dotMarker.execute(imgList[i])){
					pixsList.push(dotMarker.getCrspPix());
					objsList.push(dotMarker.getCrspObj());
				}
			}

			// calibration
			CamParam dst;
			const double rms = calibCam(dst, cam.dsize[0], cam.dsize[1], pixsList, objsList);
			printf("simplesp rms error: %g\n", rms);

			print(dst);
			saveText(dst, "cam1.txt");
		}

		// opencv calibration 
		{
			using namespace std;
			using namespace cv;

			printf("\n\n");
			printf("opencv\n");

			vector<vector<Point2f> > cvpixsList;
			vector<vector<Point3f> > cvobjsList;

			const Size boardSize(mrk.map.dsize[0], mrk.map.dsize[1]);
			for (int i = 0; i < imgList.size(); i++){
				cv::Mat img(cam.dsize[1], cam.dsize[0], CV_8UC1);
				memcpy(img.ptr(), imgList[i].ptr, imgList[i].size());
				imwrite("test.png", img);
				vector<cv::Point2f> pointbuf;
				const bool found = findCirclesGrid(img, boardSize, pointbuf);
				if (found == false) continue;
				
				vector<Point2f> cvvpixs;
				vector<Point3f> cvvobjs;

				for (int n = 0; n < pointbuf.size(); n++){
					cvvpixs.push_back(pointbuf[n]);
					cvvobjs.push_back(cv::Point3f(static_cast<float>(mrkMap[n].x), static_cast<float>(mrkMap[n].y), 0.f));
				}
				cvpixsList.push_back(cvvpixs);
				cvobjsList.push_back(cvvobjs);
			}

			cv::Mat camMat, dist;
			vector<cv::Mat> rvecs, tvecs;

			const double rms = cv::calibrateCamera(cvobjsList, cvpixsList, Size(cam.dsize[0], cam.dsize[1]), camMat, dist, rvecs, tvecs);
			printf("opencv rms error: %g\n", rms);
			cout << "camMat = " << endl << camMat << endl;
			cout << "dist = " << endl << dist << endl;
		}
	}

	// stereo test (generate test points)
	{
		printf("\n\n");
		printf("--------------------------------------------------------------------------------\n");
		printf("stereo test (generate test points)\n");
		printf("--------------------------------------------------------------------------------\n");

		Mem1<Mem1<Vec2> > pixsList0, pixsList1, objsList0, objsList1;

		// generate test points
		for (int i = 0; i < poses.size(); i++){
			Mem1<Vec2> pixs0, pixs1, objs;
			for (int n = 0; n < mrkMap.size(); n++){
				const Vec3 pos = poses[i] * getVec(mrkMap[n].x, mrkMap[n].y, 0.0);
				const Vec2 pix0 = mulCam(cam, npxDist(cam, prjVec(pos)) + randVecGauss(0.1, 0.1));
				const Vec2 pix1 = mulCam(cam, npxDist(cam, prjVec(stereo * pos))) + randVecGauss(0.1, 0.1);

				pixs0.push(pix0);
				pixs1.push(pix1);
				objs.push(mrkMap[n]);
			}
			pixsList0.push(pixs0);
			pixsList1.push(pixs1);
			objsList0.push(objs);
			objsList1.push(objs);
		}

		// simplesp calibration 
		{
			printf("\n\n");
			printf("simplesp\n");

			// calibration
			Pose dst;
			const double rms = calibStereo(dst, cam, cam, pixsList0, pixsList1, objsList0, objsList1);
			printf("simplesp rms error: %g\n", rms);

			print(dst);
			saveText(dst, "pose.txt");
		}

		// opencv calibration 
		{
			using namespace std;
			using namespace cv;

			printf("\n\n");
			printf("opencv\n");

			// Point2d is not available in cv::calibrateCamera
			vector<vector<Point2f> > cvpixsList0;
			vector<vector<Point2f> > cvpixsList1;
			vector<vector<Point3f> > cvobjsList;

			for (int i = 0; i < pixsList0.size(); i++){
				vector<Point2f> cvpixs0;
				vector<Point2f> cvpixs1;
				vector<Point3f> cvobjs;
				for (int j = 0; j < pixsList0[i].size(); j++){
					cvpixs0.push_back(Point2f(static_cast<float>(pixsList0[i][j].x), static_cast<float>(pixsList0[i][j].y)));
					cvpixs1.push_back(Point2f(static_cast<float>(pixsList1[i][j].x), static_cast<float>(pixsList1[i][j].y)));
					cvobjs.push_back(Point3f(static_cast<float>(objsList0[i][j].x), static_cast<float>(objsList0[i][j].y), 0.f));
				}
				cvpixsList0.push_back(cvpixs0);
				cvpixsList1.push_back(cvpixs1);
				cvobjsList.push_back(cvobjs);
			}

			const Size imgSize(cam.dsize[0], cam.dsize[1]);
			cv::Mat camMat = cv::Mat::zeros(3, 3, CV_64FC1);
			cv::Mat dist = cv::Mat::zeros(1, 5, CV_64FC1);

			camMat.at<double>(0, 0) = cam.fx;
			camMat.at<double>(1, 1) = cam.fy;
			camMat.at<double>(0, 2) = cam.cx;
			camMat.at<double>(1, 2) = cam.cy;
			camMat.at<double>(2, 2) = 1.0;

			dist.at<double>(0) = cam.k1;
			dist.at<double>(1) = cam.k2;
			dist.at<double>(2) = cam.p1;
			dist.at<double>(3) = cam.p2;
			dist.at<double>(4) = cam.k3;
			cv::Mat R, T, E, F;

			const double rms = cv::stereoCalibrate(cvobjsList, cvpixsList0, cvpixsList1, camMat, dist, camMat, dist, imgSize, R, T, E, F);
			printf("opencv rms error: %g\n", rms);
			cout << "R " << R << endl;
			cout << "T " << T << endl;
		}
	}

	return 0;
}

