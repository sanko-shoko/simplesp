#define SP_USE_DEBUG 1
#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

using namespace sp;

class FeatureGUI : public BaseWindow {

private:
    Mem1<Mem2<Col3>> m_imgs;

    Mem1<Mem1<Ftr>> m_ftrs;
    Mem1<int> m_matches;

    int m_mode;
    
    int m_ival;


private:

    void help() {
        printf("\n");
    }

    virtual void init() {
        help();

        m_mode = 0;
        m_ival = 0;

        m_imgs.resize(2);
        if(1){
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", m_imgs[0]));
            //SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/test.bmp", m_imgs[0]));

            m_imgs[1].resize(m_imgs[0].dsize);
            setElm(m_imgs[1], getCol3(127, 127, 127));

            double mat[3 * 3] = {
                +0.8000, -0.2000, +130.00,
                +0.2000, +0.8000, +50.000,
                +0.0002, +0.0002, +1.0000
            };
            const Mat hom(3, 3, mat);

            warp<Col3, Byte>(m_imgs[1], m_imgs[0], hom);
            merge(m_imgs[0], m_imgs[0], m_imgs[1]);
        }
        else {

            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba00.bmp", m_imgs[0]));
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", m_imgs[1]));

            merge(m_imgs[0], m_imgs[0], m_imgs[1]);
        }

        switch (m_mode) {
        case 0: initTest(); break;
        case 1: initSIFT(); break;
        }

        ImGui::GetIO().IniFilename = NULL;
    }

    virtual void drop(int num, const char **paths) {
        switch (m_mode) {
        case 0: dropTest(num, paths); break;
        case 1: dropSIFT(num, paths); break;
        }
    }

    virtual void display() {
        if (ImGui::Begin("gui", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {

            ImGui::InputInt("m_ival", &m_ival, 1, 10);

            ImGui::End();
        }
        switch (m_mode) {
        case 0: dispTest(); break;
        case 1: dispTest2(); break;
        }
    }

    void initTest() {
        {
            Mem2<Col3> imgs[2];
            if (1) {
                SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", imgs[0]));

                imgs[1].resize(imgs[0].dsize);
                setElm(imgs[1], getCol3(127, 127, 127));

                double mat[3 * 3] = {
                    +0.8000, -0.2000, +130.00,
                    +0.2000, +0.8000, +50.000,
                    +0.0002, +0.0002, +1.0000
                };
                const Mat hom(3, 3, mat);

                warp<Col3, Byte>(imgs[1], imgs[0], hom);
            }
            else {
                SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba00.bmp", imgs[0]));
                SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));
                //SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba02.bmp", imgs[0]));
                //SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));
            }

            Mem1<Ftr> ftrs[2];

            ftrs[0] = CFBlob::getFtrs(imgs[0]);
            ftrs[1] = CFBlob::getFtrs(imgs[1]);
            Mem1<int> matches;
            // matching
            matches = findMatch(ftrs[0], ftrs[1]);

            const Mem1<Vec2> pixs0 = getMatchPixs(ftrs[0], matches, true);
            const Mem1<Vec2> pixs1 = getMatchPixs(ftrs[1], matches, false);
            // print info
            printf("ftrs[0]: %d\n", ftrs[0].size());
            printf("ftrs[1]: %d\n", ftrs[1].size());

            printf("match [0->1]: cnt %d, eval %.2lf\n", getMatchCnt(matches), getMatchEval(matches));

            Mat hom;
            calcHMatRANSAC(hom, pixs1, pixs0);
        }
        
        m_ftrs.clear();

        static CFBlob test;
        test.execute(m_imgs[0]);
        if (test.getFtrs() != NULL) {
            m_ftrs.push(*test.getFtrs());
        }

      
        m_viewScale = 0.7;
        SP_LOGGER_PRINT(NULL);
    }
    void dropTest(int num, const char **paths) {

    }
    void dispTest() {
        const Mem2<Col3> &img = m_imgs[0];
        if (img.size() == 0) return;
        
        {
            glLoadView2D(img.dsize, m_viewPos, m_viewScale);
            glTexImg(img);
        }
        {
            const int s = m_ival;

            char str[256];
            sprintf(str, "pyimg%d", s);

            const Mem2<Byte> *pyimg = SP_HOLDER_GET(str, Mem2<Byte>);
            const Mem1<CFBlob::MyFtr> *myftrs1 = SP_HOLDER_GET("myftrs1", Mem1<CFBlob::MyFtr>);
            const Mem1<CFBlob::MyFtr> *myftrs2 = SP_HOLDER_GET("myftrs2", Mem1<CFBlob::MyFtr>);
            const Mem1<CFBlob::MyFtr> *myftrs3 = SP_HOLDER_GET("myftrs3", Mem1<CFBlob::MyFtr>);
            if (pyimg != NULL) {
                double scale = sp::pow(2, m_ival);
                glLoadView2D(pyimg->dsize, m_viewPos, m_viewScale * scale);
                glTexImg(*pyimg);
            }
            if (myftrs1 != NULL) {

                glBegin(GL_LINES);
                glColor(0);

                for (int i = 0; i < myftrs1->size(); i++) {
                    if ((*myftrs1)[i].pyid + 1 != m_ival) continue;
                    glCircle((*myftrs1)[i].pix/2, (*myftrs1)[i].scl);
                }

                glEnd();
            }
           if (myftrs1 != NULL) {

                glBegin(GL_LINES);
                glColor(0);

                for (int i = 0; i < myftrs1->size(); i++) {
                    if ((*myftrs1)[i].pyid != m_ival) continue;
                    glCircle((*myftrs1)[i].pix, (*myftrs1)[i].scl);
                }

                glEnd();
            }
            if (myftrs2 != NULL) {

                glBegin(GL_LINES);
                glColor(1);

                for (int i = 0; i < myftrs2->size(); i++) {
                    if ((*myftrs2)[i].pyid != m_ival) continue;
                    glCircle((*myftrs2)[i].pix, (*myftrs2)[i].scl);
                }

                glEnd();
            }
            if (m_ival < 0 && myftrs3 != NULL) {
                glBegin(GL_LINES);
                glColor(2);

                for (int i = 0; i < myftrs3->size(); i++) {
                    //if ((*myftrs3)[i].stat < 0) continue;
                    glCircle((*myftrs3)[i].pix, (*myftrs3)[i].scl);
                }

                glEnd();
            }
        }
       

    }
    void dispTest2() {
        const Mem2<Col3> &img = m_imgs[0];
        if (img.size() == 0) return;
        const int dsize[2] = { img.dsize[0] * 2, img.dsize[1] };

        {
            glLoadView2D(dsize, m_viewPos, m_viewScale);
            glTexImg(m_imgs[0]);
            glLoadView2D(dsize, m_viewPos + getVec2(img.dsize[0], 0.0) * m_viewScale, m_viewScale);
            glTexImg(m_imgs[1]);
        }
        for(int p = 0; p < 2; p++){
            glLoadView2D(dsize, m_viewPos + getVec2(img.dsize[0] * p, 0.0) * m_viewScale, m_viewScale);
            glBegin(GL_LINES);

            glColor(1);
            for (int i = 0; i < m_ftrs[p].size(); i++) {
                glCircle(m_ftrs[p][i].pix, m_ftrs[p][i].scl);
            }
            glEnd();
        }

    }
    void initSIFT() {
        printf("drag & drop images\n");

        m_ftrs.clear();
        m_ftrs.push(SIFT::getFtrs(m_imgs[0]));
        m_ftrs.push(SIFT::getFtrs(m_imgs[1]));
    }

    void dropSIFT(int num, const char **paths) {
        m_imgs.clear();
        m_ftrs.clear();

        for (int i = 0; i < 2; i++) {
            Mem2<Col3> img;
            SP_ASSERT(cvLoadImg(paths[i], img));

            const double scale = (600 * 2.0) / (img.dsize[0] + img.dsize[1]);
            if (scale < 1.0) {
                rescale<Col3, Byte>(img, img, scale, scale);
            }

            m_imgs.push(img);
        }

        if (m_imgs.size() >= 2) {
            m_ftrs.push(SIFT::getFtrs(m_imgs[0]));
            m_ftrs.push(SIFT::getFtrs(m_imgs[1]));
            m_matches = findMatch(m_ftrs[0], m_ftrs[1]);
        }
    }
    void dispSIFT(){
        if (m_matches.size() > 0) {

            const Vec2 offset = getVec2(m_imgs[0].dsize[0], 0.0);
            const CamParam cam = getCamParam(m_imgs[0].dsize[0] + m_imgs[1].dsize[0], m_imgs[0].dsize[1]);

            {
                glLoadView2D(cam, m_viewPos, m_viewScale);
                glTexImg(m_imgs[0]);

                glLoadView2D(cam, m_viewPos + offset * m_viewScale, m_viewScale);
                glTexImg(m_imgs[1]);
            }

            {
                glLoadView2D(cam, m_viewPos, m_viewScale);

                for (int f = 0, c = 0; f < m_ftrs[0].size(); f++) {
                    const int g = m_matches[f];
                    if (g < 0) continue;

                    glBegin(GL_LINES);
                    glColor(getCol3(c));
                    glCircle(m_ftrs[0][f].pix, m_ftrs[0][f].scl);
                    glEnd();

                    glBegin(GL_LINES);
                    glColor(getCol3(c));
                    glCircle(m_ftrs[1][g].pix + offset, m_ftrs[1][g].scl);
                    glEnd();

                    glBegin(GL_LINES);
                    glColor(getCol3(c));
                    glVertex(m_ftrs[0][f].pix);
                    glVertex(m_ftrs[1][g].pix + offset);
                    glEnd();

                    c++;
                }
            }
        }
        if (m_imgs.size() > 0) {
            glLoadView2D(m_imgs[0].dsize, m_viewPos, m_viewScale);
            glTexImg(m_imgs[0]);

            for (int f = 0; f < m_ftrs[0].size(); f++) {
                glBegin(GL_LINES);
                glColor(getCol3(f));
                glCircle(m_ftrs[0][f].pix, m_ftrs[0][f].scl);
                glEnd();
            }
        }
    }

    //void feature(const Ftr &ftr) {
    //    glBegin(GL_LINES);
    //    glCircle(ftr.pix, m_ftrs[0][f].scl);
    //    glEnd();
    //}
};

int main() {

    FeatureGUI win;
    win.execute("feature", 800, 600);

    return 0;
}