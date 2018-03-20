#ifndef __GTUTIL__
#define __GTUTIL__

#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

#include "tinyfiledialogs.h"

using namespace sp;

//--------------------------------------------------------------------------------
// data class
//--------------------------------------------------------------------------------

class GT {

public:

    // label id
    int label;

    // rect
    Rect rect;
    
    // contour
    Mem1<Vec2> contour;

public:

    GT() {
    }

    GT(const GT &gt) {
        *this = gt;
    }

    GT& operator = (const GT &gt) {
        label = gt.label;
        rect = gt.rect;
        contour = gt.contour;
        return *this;
    }

    void init(const Rect rect) {
        this->label = -1;
        this->rect = rect;
        this->contour.clear();
    }
};

class DataBase {

public:
    // work directory
    string wkDir;

    // image directory
    string imDir;

    // image names
    Mem1<string> imNames;

    // edit flags
    Mem1<bool> flags;

    // gt names;
    Mem1<string> gtNames;

    // gt rects list
    Mem1<MemP<GT>> gtsList;

public:

    DataBase() {
        wkDir = getTimeStamp();
    }

    bool isValid() {
        return (imNames.size() > 0) ? true : false;
    }

    bool open_imDir() {

        const char *path = tinyfd_selectFolderDialog("open image dir", getCrntDir().c_str());
        //const char *path = SP_DATA_DIR "/image";
        if (path == NULL) return false;

        imDir = path;
        imNames = getFileList(path, "bmp, BMP, png, PNG, jpeg, JPEG, jpg, JPG");
        
        gtsList.resize(imNames.size());

        flags.resize(imNames.size());
        flags.zero();

        if (imNames.size() > 0) {
            for (int i = 0; i < imNames.size(); i++) {
                printf("%06d %s\n", i, imNames[i].c_str());
            }
        }
        else {
            printf("no image in the directory");
        }

        return isValid();
    }

    bool open_wkDir() {
        static string base = getCrntDir();

        const char *path = tinyfd_selectFolderDialog("open work dir", base.c_str());
        if (path == NULL) return false;
        
        base = path;

        wkDir = path;

        gtNames.clear();

        {
            const string path = wkDir + "\\labels.csv";
                
            File file;
            if (file.open(path.c_str(), "r") == false) return false;

            file.scanf("index,name,\n");

            char str[SP_STRMAX];
            while (file.gets(str) == true) {
                const Mem1<string> split = strSplit(str, ",");
                if (split.size() < 2) break;

                gtNames.push(strTrim(split[1].c_str(), " "));
            }
        }

        {
            for (int i = 0; i < imNames.size(); i++) {

                const string path = (wkDir + "\\rect\\" + imNames[i] + ".csv");

                MemP<GT> &gts = gtsList[i];
                gts.clear();

                File file;
                if (file.open(path.c_str(), "r") == false) continue;

                file.scanf("index,label,x,y,width,height,\n");

                char str[SP_STRMAX];
                while (file.gets(str) == true) {
                    const Mem1<string> split = strSplit(str, ",");
                    if (split.size() != 7) break;

                    int buf;
                    GT &gt = *gts.malloc();
                    gt.rect.dim = 2;
                    sscanf(str, "%d,%d,%d,%d,%d,%d,\n", &buf, &gt.label, &gt.rect.dbase[0], &gt.rect.dbase[1], &gt.rect.dsize[0], &gt.rect.dsize[1]);
                }
            }
        }

        {
            for (int i = 0; i < imNames.size(); i++) {

                const string path = (wkDir + "\\cont\\" + imNames[i] + ".csv");

                MemP<GT> &gts = gtsList[i];

                File file;
                if (file.open(path.c_str(), "r") == false) continue;

                file.scanf("index,label,x,y,\n");

                char str[SP_STRMAX];

                while (file.gets(str) == true) {
                    const Mem1<string> split = strSplit(str, ",");
                    if (split.size() != 5) break;

                    int index, label;
                    Vec2 pos;

                    sscanf(str, "%d,%d,%lf,%lf,\n", &index, &label, &pos.x, &pos.y);
                    
                    if (index < gts.size()) {
                        GT &gt = gts[index];
                        gt.contour.push(pos);
                    }
                }
            }
        }
        return true;
    }

    void updateLabel(const int id, const int val) {
        for (int i = 0; i < gtsList.size(); i++) {
            MemP<GT> &gts = gtsList[i];

            for (int j = 0; j < gts.size(); j++) {
                GT &gt = gts[j];
                if (gt.label == id && val < 0) {
                    gt.label = -1;
                    continue;
                }
                if (gt.label >= id) {
                    gt.label += val;
                }
            }
        }

    }

    void save() {

        {
            mkdir(wkDir.c_str());

            const string path = wkDir + "\\labels.csv";
            
            File file;
            SP_ASSERT(file.open(path.c_str(), "w"));

            file.printf("index,name,\n");

            for (int i = 0; i < gtNames.size(); i++) {
                file.printf("%d,%s,\n", i, gtNames[i].c_str());
            }
        }


        {
            mkdir((wkDir + "\\rect").c_str());
            for (int i = 0; i < gtsList.size(); i++) {

                const string path = wkDir + "\\rect\\" + imNames[i] + ".csv";

                MemP<GT> &gts = gtsList[i];

                if (gts.size() == 0) {
                    remove(path.c_str());
                    continue;
                }

                File file;
                SP_ASSERT(file.open(path.c_str(), "w"));

                file.printf("index,label,x,y,width,height,\n");

                for (int j = 0; j < gts.size(); j++) {
                    const GT &gt = gts[j];
                    file.printf("%d,%d,%d,%d,%d,%d,\n", j, gt.label, gt.rect.dbase[0], gt.rect.dbase[1], gt.rect.dsize[0], gt.rect.dsize[1]);
                }
            }
        }

        {
            mkdir((wkDir + "\\cont").c_str());
            for (int i = 0; i < gtsList.size(); i++) {

                const string path = wkDir + "\\cont\\" + imNames[i] + ".csv";

                MemP<GT> &gts = gtsList[i];

                if (gts.size() == 0) {
                    remove(path.c_str());
                    continue;
                }

                File file;
                SP_ASSERT(file.open(path.c_str(), "w"));

                file.printf("index,label,x,y,\n");

                for (int j = 0; j < gts.size(); j++) {
                    const GT &gt = gts[j];
                    for (int k = 0; k < gt.contour.size(); k++) {
                        file.printf("%d,%d,%lf,%lf,\n", j, gt.label, gt.contour[k].x, gt.contour[k].y);
                    }
                }
            }
        }
    }

};


//--------------------------------------------------------------------------------
// render
//--------------------------------------------------------------------------------

class Render{

#define RENDER_BASE getCol( 80, 180, 160)
#define RENDER_HIGH getCol(220, 240, 220)
#define RENDER_GRAY getCol(180, 180, 180)
#define RENDER_NEAR getCol(160, 160, 250)

public:
 
    static void point(const Vec2 &a, const Col3 &col, const float size) {
        for (int s = 0; s < 2; s++) {
            glPointSize(s == 0 ? size : size - 2.0f);

            glBegin(GL_POINTS);
            glColor(s == 0 ? col / 3.0 : col);
            glVertex(a);
            glEnd();
        }
    }
    static void point(const Mem1<Vec2> &pnts, const Col3 &col, const float size) {
        for (int i = 0; i < pnts.size(); i++) {
            point(pnts[i], col, size);
        }
    }

    static void line(const Mem1<Vec2> &vtxs, const Col3 &col, const float size, const bool loop = false) {

        for (int s = 0; s < 2; s++) {
            glLineWidth(s == 0 ? size : size - 2.0f);
            glBegin(GL_LINES);
            glColor(s == 0 ? col / 3.0 : col);
            glLine(vtxs, loop);
            glEnd();
        }
    };

    static void fill(const Rect &rect, const Col3 &col, const float size) {
        const Mem1<Vec2> vtxs = getVtx2(rect);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glBegin(GL_TRIANGLE_FAN);

        glColor(extCol(col, 220));
        glVertex(vtxs[0]);
        glVertex(vtxs[1]);
        glVertex(vtxs[2]);
        glVertex(vtxs[3]);
        glEnd();

        glDisable(GL_BLEND);
    }

    static void fill(const Mem1<Mesh2> &meshes, const Col3 &col, const float size) {
        glEnable(GL_BLEND);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBegin(GL_TRIANGLES);

        glColor(extCol(col, 220));
        for (int i = 0; i < meshes.size(); i++) {
            glMesh(meshes[i]);
        }
        glEnd();

        glDisable(GL_BLEND);
    }
};

#endif