#ifndef __DOTMARKER_H__
#define __DOTMARKER_H__

#define SP_USE_DEBUG 1

#include "simplesp.h"
#include "spex/sptw.h"
#include "spex/spcv.h"

using namespace sp;

class DotMarkerGUI : public TwWindow{

	// object model
	Mem1<Mesh> m_model;

	DotMarker m_dotMarker;

	Mem2<Col3> m_crntImg;

	// calib data
	Mem1<Mem1<Vec2> > m_pixPntsList;
	Mem1<Mem1<Vec2> > m_objPntsList;

	struct UIFlag{
		bool start;
		bool dispSkip;
		bool useDiminish;
		bool dispModel;

		bool dispMinGauss;
		bool dispMinLab;
		bool dispDetectPix;
		bool dispLinkLine;

		UIFlag(){
			memset(this, 0, sizeof(UIFlag));
		}
	}m_ui;

private:

	virtual void init(){

		//if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false){
		//	exit(0);
		//}

		m_ui.start = true;


		TwBar *bar = TwNewBar("TweakBar");

		TwAddButton(bar, "start", TwButtonCB(DotMarkerGUI, start), this, "");

		TwAddSeparator(bar, NULL, "");
		TwAddButton(bar, "addCrsp", TwButtonCB(DotMarkerGUI, addCrsp), this, "");
		TwAddVarRO(bar, "num", TW_TYPE_INT32, &m_pixPntsList.dsize[0], "");
		TwAddButton(bar, "calib", TwButtonCB(DotMarkerGUI, calib), this, "");

		TwAddSeparator(bar, NULL, "");
		TwAddVarRW(bar, "useDiminish", TW_TYPE_BOOL8, &m_ui.useDiminish, "");
		TwAddVarRW(bar, "dispModel", TW_TYPE_BOOL8, &m_ui.dispModel, "");

#if SP_USE_DEBUG
		TwAddVarRW(bar, "dispSkip", TW_TYPE_BOOL8, &m_ui.dispSkip, "");
		TwAddVarRW(bar, "dispMinGauss", TW_TYPE_BOOL8, &m_ui.dispMinGauss, "");
		TwAddVarRW(bar, "dispMinLab", TW_TYPE_BOOL8, &m_ui.dispMinLab, "");
		TwAddVarRW(bar, "dispDetectPix", TW_TYPE_BOOL8, &m_ui.dispDetectPix, "");
		TwAddVarRW(bar, "dispLinkLine", TW_TYPE_BOOL8, &m_ui.dispLinkLine, "");

		TwAddSeparator(bar, NULL, "");
		TwAddButton(bar, "calcOne", TwButtonCB(DotMarkerGUI, calcOne), this, "");
		TwAddButton(bar, "nextImg", TwButtonCB(DotMarkerGUI, nextImg), this, "");
		TwAddSeparator(bar, NULL, "");
		TwAddButton(bar, "loadImg", TwButtonCB(DotMarkerGUI, loadImg), this, "");
		TwAddButton(bar, "saveImg", TwButtonCB(DotMarkerGUI, saveImg), this, "");
#endif

	}

	void start(){
		m_ui.start ^= true;
	}

	void saveImg(){
		saveBMP(m_crntImg, "test.bmp");
	}

	void loadImg(){
		loadBMP(m_crntImg, "test.bmp");
	}

	// capture next image
	void nextImg();

	// estimate marker pose
	void calcOne();

	// add detected marker for calibration
	void addCrsp();

	// execute camera calibration
	void calib();

	virtual void display();
};

#endif