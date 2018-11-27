#define SP_USE_DEBUG 1
#define SP_USE_GLEW 1
#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

#include "testfeature.h"

using namespace sp;

class FeatureGUI : public BaseWindow {

private:
    Mem1<Mem2<Col3>> m_imgs;

    Mem1<Mem1<Feature>> m_fts;
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

        {
            m_imgs.resize(2);
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", m_imgs[0]));
            //SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/test.bmp", m_imgs[0]));

            m_imgs[1].resize(m_imgs[0].dsize);
            setElm(m_imgs[1], getCol(127, 127, 127));

            double mat[3 * 3] = {
                +0.8000, -0.2000, +130.00,
                +0.2000, +0.8000, +50.000,
                +0.0002, +0.0002, +1.0000
            };
            const Mat hom(3, 3, mat);

            warp<Col3, Byte>(m_imgs[1], m_imgs[0], hom);
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
        m_fts.clear();

        static FastBlob test;
        test.execute(m_imgs[0]);
        if (test.getFeatures() != NULL) {
            m_fts.push(*test.getFeatures());
        }

        //test.execute(m_imgs[1]);
        //m_fts.push(*test.getFeatures());
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
            const Mem2<Byte> *pyimg = SP_HOLDER_GET(strFormat("pyimg%d", s).c_str(), Mem2<Byte>);
            const Mem1<Feature> *cpnts = SP_HOLDER_GET(strFormat("cpnts%d", s).c_str(), Mem1<Feature>);
            const Mem1<Feature> *ref = SP_HOLDER_GET(strFormat("refine%d", s).c_str(), Mem1<Feature>);
            const Mem1<Feature> *ref2 = SP_HOLDER_GET(strFormat("refine2%d", s).c_str(), Mem1<Feature>);
            const Mem1<Feature> *output = SP_HOLDER_GET("output", Mem1<Feature>);
            if (pyimg != NULL) {
                double scale = sp::pow(2, m_ival);
                glLoadView2D(pyimg->dsize, m_viewPos, m_viewScale * scale);
                glTexImg(*pyimg);
            }
            //if (cpnts != NULL) {

            //    glBegin(GL_LINES);
            //    glColor(0);

            //    for (int i = 0; i < cpnts->size(); i++) {
            //        glCircle((*cpnts)[i].pix, 2.0);
            //    }

            //    glEnd();
            //}
            if (ref != NULL) {

                glBegin(GL_LINES);
                glColor(0);

                for (int i = 0; i < ref->size(); i++) {
                    glCircle((*ref)[i].pix, (*ref)[i].scl);
                }

                glEnd();
            }
            if (ref2 != NULL) {

                glBegin(GL_LINES);
                glColor(1);

                for (int i = 0; i < ref2->size(); i++) {
                    glCircle((*ref2)[i].pix, (*ref2)[i].scl);
                }

                glEnd();
            }
            if (m_ival < 0 && output != NULL) {
                glBegin(GL_LINES);
                glColor(2);

                for (int i = 0; i < output->size(); i++) {
                    glCircle((*output)[i].pix, (*output)[i].scl);
                }

                glEnd();
            }
        }
        return;
        {
            const Mem1<FastBlob::MyFeature> *keys = SP_HOLDER_GET("keys", Mem1<FastBlob::MyFeature>);
            const Mem1<FastBlob::MyFeature> *refs = SP_HOLDER_GET("refs", Mem1<FastBlob::MyFeature>);


            const Mem2<Byte> *pimg = SP_HOLDER_GET(strFormat("test%d", m_ival).c_str(), Mem2<Byte>);
            if (pimg) {
                double scale = sp::pow(2, m_ival);
                glLoadView2D(pimg->dsize, m_viewPos, m_viewScale * scale);
                glTexImg(*pimg);

                if (keys != NULL) {
                    glBegin(GL_LINES);
                    glColor(0);

                    for (int i = 0; i < keys->size(); i++) {
                        if ((*keys)[i].octave == m_ival) {
                            //glCircle((*keys)[i].pix / scale, (*keys)[i].scl / scale);
                            glCircle((*keys)[i].pix, (*keys)[i].scl);
                        }
                    }
                    glColor(1);

                    for (int i = 0; i < refs->size(); i++) {
                        if ((*refs)[i].octave == m_ival) {
                            //glCircle((*refs)[i].pix / scale, (*refs)[i].scl / scale);
                            glCircle((*refs)[i].pix, (*refs)[i].scl);
                        }
                    }
                    glEnd();
                }
            }
            else {
                glLoadView2D(img.dsize, m_viewPos, m_viewScale);

                if (keys != NULL) {
                    glBegin(GL_LINES);
                    double scale = sp::pow(2, m_ival + 1);

                    glColor(0);
                    for (int i = 0; i < keys->size(); i++) {
                        int oct = (*keys)[i].octave;
                        const double ss = ::pow(2, oct);
                        glCircle((*keys)[i].pix * ss, (*keys)[i].scl * ss);
                    }
                    glColor(1);
                    for (int i = 0; i < refs->size(); i++) {
                        int oct = (*refs)[i].octave;
                        const double ss = ::pow(2, oct);
                        glCircle((*refs)[i].pix * ss, (*refs)[i].scl * ss);
                    }
                    glEnd();
                }
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
            glLoadView2D(dsize, m_viewPos + getVec(img.dsize[0], 0.0)*m_viewScale, m_viewScale);
            glTexImg(m_imgs[1]);
        }
        for(int p = 0; p < 2; p++){
            glLoadView2D(dsize, m_viewPos + getVec(img.dsize[0] * p, 0.0)*m_viewScale, m_viewScale);
            glBegin(GL_LINES);

            glColor(1);
            for (int i = 0; i < m_fts[p].size(); i++) {
                glCircle(m_fts[p][i].pix, m_fts[p][i].scl);
            }
            glEnd();
        }

    }
    void initSIFT() {
        printf("drag & drop images\n");

        m_fts.clear();
        m_fts.push(SIFT::getFeatures(m_imgs[0]));
        m_fts.push(SIFT::getFeatures(m_imgs[1]));
    }

    void dropSIFT(int num, const char **paths) {
        m_imgs.clear();
        m_fts.clear();

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
            m_fts.push(SIFT::getFeatures(m_imgs[0]));
            m_fts.push(SIFT::getFeatures(m_imgs[1]));
            m_matches = findMatch(m_fts[0], m_fts[1]);
        }
    }
    void dispSIFT(){
        if (m_matches.size() > 0) {

            const Vec2 offset = getVec(m_imgs[0].dsize[0], 0.0);
            const CamParam cam = getCamParam(m_imgs[0].dsize[0] + m_imgs[1].dsize[0], m_imgs[0].dsize[1]);

            {
                glLoadView2D(cam, m_viewPos, m_viewScale);
                glTexImg(m_imgs[0]);

                glLoadView2D(cam, m_viewPos + offset * m_viewScale, m_viewScale);
                glTexImg(m_imgs[1]);
            }

            {
                glLoadView2D(cam, m_viewPos, m_viewScale);

                for (int f = 0, c = 0; f < m_fts[0].size(); f++) {
                    const int g = m_matches[f];
                    if (g < 0) continue;

                    glBegin(GL_LINES);
                    glColor(getCol(c));
                    glCircle(m_fts[0][f].pix, m_fts[0][f].scl);
                    glEnd();

                    glBegin(GL_LINES);
                    glColor(getCol(c));
                    glCircle(m_fts[1][g].pix + offset, m_fts[1][g].scl);
                    glEnd();

                    glBegin(GL_LINES);
                    glColor(getCol(c));
                    glVertex(m_fts[0][f].pix);
                    glVertex(m_fts[1][g].pix + offset);
                    glEnd();

                    c++;
                }
            }
        }
        if (m_imgs.size() > 0) {
            glLoadView2D(m_imgs[0].dsize, m_viewPos, m_viewScale);
            glTexImg(m_imgs[0]);

            for (int f = 0; f < m_fts[0].size(); f++) {
                glBegin(GL_LINES);
                glColor(getCol(f));
                glCircle(m_fts[0][f].pix, m_fts[0][f].scl);
                glEnd();
            }
        }
    }

    //void feature(const Feature &ft) {
    //    glBegin(GL_LINES);
    //    glCircle(ft.pix, m_fts[0][f].scl);
    //    glEnd();
    //}
};

int main() {

    FeatureGUI win;
    win.execute("feature", 800, 600);

    return 0;
}