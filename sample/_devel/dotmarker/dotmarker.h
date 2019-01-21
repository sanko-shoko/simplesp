#ifndef __DOTMARKER_H__
#define __DOTMARKER_H__

#define SP_USE_DEBUG 1
#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

using namespace sp;

class DotMarkerGUI : public BaseWindow{

    // object model
    Mem1<Mesh3> m_model;

    DotMarker m_dotMarker;

    Mem2<Col3> m_crntImg;

    Mem1<Mem2<Col3> > m_addImg;

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
        m_ui.start = true;
    }

    void saveImg(){
        saveBMP("test.bmp", m_crntImg);
    }

    void loadImg(){
        loadBMP("test.bmp", m_crntImg);
    }

    // capture next image
    void nextImg();

    // estimate marker pose
    void calcOne();

    virtual void keyFun(int key, int scancode, int action, int mods);
    virtual void display();
};

#endif