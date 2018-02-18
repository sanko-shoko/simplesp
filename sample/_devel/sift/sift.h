#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

#include <cstdlib>

using namespace sp;

class SIFTGUI : public BaseWindow {

    Mem1<Mem2<Col3> > m_imgs;
    SIFT m_sift0;
    SIFT m_sift1;
    Mem1<int> m_matches;

    Mat m_hom;

private:

    void help() {
        printf("drag & drop images\n");
        printf("\n");
    }

    virtual void init() {
        help();
    }

    virtual void drop(int num, const char **paths) {
        m_imgs.clear();

        // add img and cam
        for (int i = 0; i < num; i++) {
            Mem2<Col3> img;
            SP_ASSERT(cvLoadImg(img, paths[i]));

            Mem2<Col3> tmp;
            const double scale = (600 * 2.0) / (img.dsize[0] + img.dsize[1]);
            if (scale < 1.0) {
                rescale<Col3, Byte>(tmp, img, scale, scale);
            }
            else {
                tmp = img;
            }

            m_imgs.push(tmp);
        }
        if(m_imgs.size() >= 2){

            // sift execute
            m_sift0.execute(m_imgs[0]);
            m_sift1.execute(m_imgs[1]);

            const Mem1<Feature> &fts0 = m_sift0.getFeatrue();
            const Mem1<Feature> &fts1 = m_sift1.getFeatrue();
            m_matches = findMatch(fts0, fts1);

            Mem1<Vec2> pixs0, pixs1;

            const Mem1<int> matches = findMatch(fts0, fts1);
            for (int i = 0; i < matches.size(); i++) {
                const int j = matches[i];
                if (j < 0) continue;

                pixs0.push(fts0[i].pix);
                pixs1.push(fts1[j].pix);
            }

            calcHMatRANSAC(m_hom, pixs1, pixs0);
            print(m_hom);
        }
    }

    virtual void display() {

        if (m_imgs.size() >= 2) {
            const Vec2 offset = getVec(m_imgs[0].dsize[0], 0.0);
            const CamParam cam = getCamParam(m_imgs[0].dsize[0] + m_imgs[1].dsize[0], m_imgs[0].dsize[1]);
        
            {
                glLoadView2D(cam, m_viewPos, m_viewScale);
                glRenderImage(m_imgs[0]);

                glLoadView2D(cam, m_viewPos + offset * m_viewScale, m_viewScale);
                glRenderImage(m_imgs[1]);
            }

            const Mem1<Feature> &fts0 = m_sift0.getFeatrue();
            const Mem1<Feature> &fts1 = m_sift1.getFeatrue();
        
            {
                glLoadView2D(cam, m_viewPos, m_viewScale);


                //glBegin(GL_LINES);
                //glColor(getCol(100, 255, 100));
    
                //// render feature
                //for (int i = 0; i < fts0.size(); i++) {
                //    glCircle(fts0[i].pix, fts0[i].scale);
                //}
                //for (int i = 0; i < fts1.size(); i++) {
                //    glCircle(fts1[i].pix + offset, fts1[i].scale);
                //}
                //glEnd();
            }
            // matching
            {
                glLoadView2D(cam, m_viewPos, m_viewScale);
                glBegin(GL_LINES);
                glColor(getCol(50, 200, 200));

                for (int i = 0; i < m_matches.size(); i++) {
                    const int j = m_matches[i];
                    if (j < 0) continue;

                    glVertex(fts0[i].pix);
                    glVertex(fts1[j].pix + offset);
                }
                glEnd();
            }

            {
                glLoadView2D(cam, m_viewPos, m_viewScale);
                glBegin(GL_LINES);
                glColor(getCol(100, 200, 100));
                const int w = m_imgs[0].dsize[0];
                const int h = m_imgs[1].dsize[1];
                if (m_hom.size() > 0) {
                    const Vec2 pix[4] = { getVec(0.0, 0.0), getVec(w, 0.0), getVec(w, h), getVec(0.0, h) };
                    for (int i = 0; i < 4; i++) {
                        const Vec2 p0 = pix[i] - getVec(0.5, 0.5);
                        const Vec2 p1 = pix[(i + 1) % 4] - getVec(0.5, 0.5);
                        glVertex(m_hom * p0 + getVec(w, 0));
                        glVertex(m_hom * p1 + getVec(w, 0));
                    }
                }
                glEnd();
            }

            //    const int w = img0.dsize[0];
            //    const int h = img0.dsize[1];

            //    renderLine(m_img, pixs0, pixs1 + getVec(w, 0), getCol(50, 200, 200), 1);

            //    Mat hom;
            //    if (calcHMatRANSAC(hom, pixs1, pixs0) == true) {
            //        const Vec2 pix[4] = { getVec(0.0, 0.0), getVec(w, 0.0), getVec(w, h), getVec(0.0, h) };
            //        for (int i = 0; i < 4; i++) {
            //            const Vec2 p0 = pix[i] - getVec(0.5, 0.5);
            //            const Vec2 p1 = pix[(i + 1) % 4] - getVec(0.5, 0.5);
            //            renderLine(m_img, hom * p0 + getVec(w, 0), hom * p1 + getVec(w, 0), getCol(100, 200, 100), 2);
            //        }
            //    }

            //    saveBMP(m_img, "match.bmp");
            //    print(hom);
            //}
        }

    }

};
