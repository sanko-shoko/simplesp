#ifndef __BITMARKER_H__
#define __BITMARKER_H__

#define SP_USE_DEBUG 1
#define SP_USE_CONSOLE 1

#include "simplesp.h"
#include "spex/sptw.h"
#include "spex/spcv.h"

using namespace sp;

class BitMarkerGUI : public TwWindow{

    BitMarker m_bitMarker;

    Mem2<Col3> m_crntImg;

    struct UIFlag{
        bool start;

        bool dispMinGauss;
        bool dispMinLab;
        bool dispContour;
        bool dispCorner;
        bool test;

        UIFlag(){
            memset(this, 0, sizeof(UIFlag));
        }
    }m_ui;

private:

    virtual void init(){
        static Mem1<BitMarkerParam> mrks(4 * 3);
        for (int i = 0; i < mrks.size(); i++){
            mrks[i].setImg(i);
        }
        m_bitMarker.setMrk(mrks);


        m_ui.start = true;

        TwBar *bar = TwNewBar("TweakBar");

        TwAddButton(bar, "start", TwButtonCB(BitMarkerGUI, start), this, "");

        TwAddSeparator(bar, NULL, "");

#if SP_USE_DEBUG
        TwAddVarRW(bar, "dispMinGauss", TW_TYPE_BOOL8, &m_ui.dispMinGauss, "");
        TwAddVarRW(bar, "dispMinLab", TW_TYPE_BOOL8, &m_ui.dispMinLab, "");
        TwAddVarRW(bar, "dispContour", TW_TYPE_BOOL8, &m_ui.dispContour, "");
        TwAddVarRW(bar, "dispCorner", TW_TYPE_BOOL8, &m_ui.dispCorner, "");
        TwAddVarRW(bar, "test", TW_TYPE_BOOL8, &m_ui.test, "");
#endif

    }

    void start(){
        m_ui.start ^= true;
    }

    // capture next image
    void nextImg();

    // estimate marker pose
    void calcOne();

    virtual void display();
};

#endif