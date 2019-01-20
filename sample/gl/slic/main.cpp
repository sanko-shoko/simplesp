#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class SlicGUI : public BaseWindow {

    // image
    Mem2<Col3> m_img;

    Mem2<Col3> m_col;

    Mem2<Col3> *m_view;


    //
    Mem1<Mem1<Vec2> > m_vtxsList;

    //
    Mem2<int> m_map;

private:

    void help() {
        printf("'s' key : switch image\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        help();

        loadBMP(SP_DATA_DIR "/image/Lenna.bmp", m_img);
        m_view = &m_img;

        slic(m_map, m_img);

        cnvLabelToImg(m_col, m_map);

        m_vtxsList = getLabelContour(m_map, false, true);
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_S] == 1) {
            m_view = (m_view == &m_img) ? &m_col : &m_img;
        }
    }

    virtual void display() {

        // view 2D
        glLoadView2D(m_view->dsize, m_viewPos, m_viewScale);
        glTexImg(*m_view);


        for (int i = 0; i < m_vtxsList.size(); i++) {
            glLineWidth(3);
            glColor(getCol(0, 0, 0));
            glLine(m_vtxsList[i], true);
        }
    }


};



int main(){

    SlicGUI win;
    win.execute("slic", 800, 600);

    return 0;
}