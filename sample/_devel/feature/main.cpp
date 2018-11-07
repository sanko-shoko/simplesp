#define SP_USE_DEBUG 1
#define SP_USE_GLEW 1
#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

using namespace sp;

class FeatureGUI : public BaseWindow {

private:
    Mem1<Mem2<Col3>> m_imgs;

    Mem1<Feature> m_fts[2];
    Mem1<int> m_matches;

private:

    void help() {
        //printf("drag & drop images\n");
        printf("\n");
    }

    virtual void init() {
        help();
    }

    virtual void drop(int num, const char **paths) {
        dropSIFT(num, paths);
    }

    virtual void display() {
        dispSIFT();
    }


    void dropSIFT(int num, const char **paths) {
        m_imgs.clear();

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
            m_fts[0] = SIFT::getFeatures(m_imgs[0]);
            m_fts[1] = SIFT::getFeatures(m_imgs[1]);
            m_matches = findMatch(m_fts[0], m_fts[1]);
        }
    }
    void dispSIFT(){
        if (m_matches.size() < 2) return;
        
        const Vec2 offset = getVec(m_imgs[0].dsize[0], 0.0);
        const CamParam cam = getCamParam(m_imgs[0].dsize[0] + m_imgs[1].dsize[0], m_imgs[0].dsize[1]);

        {
            glLoadView2D(cam, m_viewPos, m_viewScale);
            glRenderImg(m_imgs[0]);

            glLoadView2D(cam, m_viewPos + offset * m_viewScale, m_viewScale);
            glRenderImg(m_imgs[1]);
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

};

int main() {

    FeatureGUI win;
    win.execute("feature", 800, 600);

    return 0;
}