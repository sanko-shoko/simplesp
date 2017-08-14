#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class KalmanfilterGUI : public BaseWindow{

	int m_mode;

private:

	void help() {
		printf("'0' key : estimate position\n");
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init(){
		m_mode = 1;

		help();
	}

	virtual void action() {
		switch (m_keyAction) {
		case GLFW_KEY_0: m_mode = 0; break;
		case GLFW_KEY_1: m_mode = 1; break;
		}
	}

	virtual void display(){

		switch (m_mode) {
		case 0: func0(); break;
		case 1: func1(); break;

		}

	}

	void func0() {
		static int time = 0;
		static KalmanFilter kf;

		Mem2<Col3> img(320, 240);

		const double dt = 0.1;

		const double radius = 100.0;

		const double noize = 10.0;

		Mat Z(2, 1);
		{
			const double angle = time * dt * SP_PI / 180.0;
			const Vec2 cent = getVec(img.dsize[0] - 1, img.dsize[1] - 1) * 0.5;
			const Vec2 pos = getVec(::cos(angle), ::sin(angle)) * radius + cent + randVecGauss(1.0, 1.0) * noize;

			Z(0, 0) = pos.x;
			Z(1, 0) = pos.y;
		}

		
		if (time == 0) {
			Mat X = zeroMat(4, 1);

			// position & velocity
			X(0, 0) = Z(0, 0);
			X(1, 0) = Z(1, 0);
			X(2, 0) = 0.0;
			X(3, 0) = 0.0;


			Mat Q = zeroMat(4, 4);
			Q(0, 0) = square(dt * dt / 2.0);
			Q(1, 1) = square(dt * dt / 2.0);
			Q(2, 2) = dt * dt;
			Q(3, 3) = dt * dt;
			Q(0, 2) = Q(1, 0) = (dt * dt / 2.0) * dt;
			Q(1, 3) = Q(3, 1) = (dt * dt / 2.0) * dt;

			// acceleration
			Q *= 1.0;


			Mat P = Q;

			Mat R = zeroMat(2, 2);
			R(0, 0) = square(10.0);
			R(1, 1) = square(10.0);

			kf.init(X, P, Q, R);
		}
		else {
			Mat F = zeroMat(4, 4);
			F(0, 0) = 1.0;
			F(1, 1) = 1.0;
			F(2, 2) = 1.0;
			F(3, 3) = 1.0;
			F(0, 2) = dt;
			F(1, 3) = dt;

			Mat H = zeroMat(2, 4);
			H(0, 0) = 1.0;
			H(1, 1) = 1.0;

			kf.execute(Z, F, H);

		}
		const Mat &X = kf.getX();

		setElm(img, getCol(255, 255, 255));
		renderPoint(img, getVec(Z[0], Z[1]), getCol(0, 0, 0), 3);
		renderPoint(img, getVec(X[0], X[1]), getCol(0, 0, 255), 5);

		glLoadView2D(img.dsize[0], img.dsize[1], m_viewPos, m_viewScale);
		glRenderImage(img);

		//char path[256];
		//sprintf(path, "img%03d.bmp", time);
		//saveBMP(img, path);
		time++;
	}

	void func1() {
		static int time = 0;
		static KalmanFilter kf;

		CamParam cam = getCamParam(320, 240);
		Mem2<Col3> img0(cam.dsize);
		Mem2<Col3> img1(cam.dsize);

		Pose view = getPose(getVec(0.0, 0.0, 200.0));

		const double noize = 10.0;

		const double dt = 0.1;

		Mem1<Vec3> pnts;
		for (int z = -1; z <= 1; z++) {
			for (int y = -1; y <= 1; y++) {
				for (int x = -1; x <= 1; x++) {
					pnts.push(getVec(x, y, z) * 10.0);
				}
			}
		}

		// ground truth
		Pose move;
		{
			const double angle = time * dt * SP_PI / 180.0;
			const Vec3 pos = getRotAngle(getVec(0.0, 1.0, 0.0), angle) * getVec(0.0, 0.0, -1.0) * 80;
			move.trn = pos;
			move.trn.y = ::sin(angle * 4) * 5.0;
			//move.trn += randVecGauss(1.0, 1.0, 1.0) * 0.5;

			move.rot = getRotAngle(getVec(0.0, 1.0, 0.0), angle);
			//move.rot = move.rot * randRotGauss(0.05);

			move = invPose(move);
		}

		Mem1<Vec2> pixs;
		for (int i = 0; i < pnts.size(); i++) {
			const Vec2 pix = cam * prjVec(move * pnts[i]);
			pixs.push(pix);
		}

		static Pose pre;
		Pose pose;
		if (time == 0) {
			pose = move;

			const Vec3 trn = pose.trn;
			const Vec3 euler = getEuler(pose.rot);

			Mat X = zeroMat(6, 1);

			Mat Q = zeroMat(6, 6);
			Q(0, 0) = 1.0;
			Q(1, 1) = 1.0;
			Q(2, 2) = 1.0;
			Q(3, 3) = 1.0;
			Q(4, 4) = 1.0;
			Q(5, 5) = 1.0;

			Q *= 5.0;

			Mat P = Q;

			Mat R = eyeMat(2 * pixs.size(), 2 * pixs.size());
			R *= 10.0;

			kf.init(X, P, Q, R);
		}
		else {
			Mat T = kf.getX();
			{
				T(3, 0) = 0.0;
				T(4, 0) = 0.0;
				T(5, 0) = 0.0;
			}
			kf.setX(zeroMat(6, 1));

			Mat F = eyeMat(6, 6);

			Mat H(2 * pixs.size(), 6);
			Mat Z(2 * pixs.size(), 1);
			Mat Y(2 * pixs.size(), 1);
			Mem1<double> errs(pixs.size());

			for (int i = 0; i < pixs.size(); i++) {
				jacobPoseToPix(&H(i * 2, 0), zeroPose(), cam, pre * pnts[i]);

				const Vec2 z = pixs[i];
				const Vec2 y = cam * npxDist(cam, prjVec(pre * pnts[i]));

				Z(i * 2 + 0, 0) = z.x;
				Z(i * 2 + 1, 0) = z.y;
				Y(i * 2 + 0, 0) = y.x;
				Y(i * 2 + 1, 0) = y.y;
			}
			kf.execute(Z, F, H, Y);

			{
				const Mat &X = kf.getX();
				print(X);
				pose.trn = getVec(X(0, 0), X(1, 0), X(2, 0));
				pose.rot = getRotAngle(getVec(X(3, 0), X(4, 0), X(5, 0)));
			
				pose = updatePose(pre, X.ptr);
			}
			//pose = invPose(pose) * pre;
			//pose.rot = pose.rot * pre.rot;
			//pose.trn = pose.trn - pre.trn;
			//refinePose(pose, cam, pixs, pnts, 20);
		}
		pre = pose;

		setElm(img0, getCol(255, 255, 255));
		setElm(img1, getCol(255, 255, 255));

		renderCube(img0, cam, pose, 20.0, getCol(0, 0, 0), 1);
		renderPoint(img0, cam, pose, pnts, getCol(0, 0, 0), 2);

		renderCube(img1, cam, view, 20.0, getCol(0, 0, 0), 1);
		renderPoint(img1, cam, view, pnts, getCol(0, 0, 0), 2);
		renderCam(img1, cam, view * invPose(move), cam, 20, getCol(0, 0, 0), 1);

		glLoadView2D(img0.dsize[0], img0.dsize[1], m_viewPos - getVec(160, 0) * m_viewScale, m_viewScale);
		glRenderImage(img0);

		glLoadView2D(img1.dsize[0], img1.dsize[1], m_viewPos + getVec(160, 0) * m_viewScale, m_viewScale);
		glRenderImage(img1);

		time++;
		return;

	}
};

