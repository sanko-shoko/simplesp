//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_TOOL_H__
#define __SP_TOOL_H__

#include "spapp/spgeom/spcalib.h"
#include "spapp/spgeomex/spdotmarker.h"
#include "spapp/spdata/spfile.h"

namespace sp {

    class CalibTool {
    private:

        //--------------------------------------------------------------------------------
        // input
        //--------------------------------------------------------------------------------

        // add images
        Mem1<Byte> m_imgs;

        // detected points
        Mem1<Mem1<Vec2> > m_pixs, m_objs;

        //--------------------------------------------------------------------------------
        // output 
        //--------------------------------------------------------------------------------

        // success flag
        bool m_est;

        // estimated parameter
        CamParam m_cam;

    public:

        CalibTool() {
            clear();
        }

        void clear() {
            m_pixs.clear();
            m_objs.clear();

            m_est = false;
            m_cam = getCamParam(0, 0);
        }

        //--------------------------------------------------------------------------------
        // input 
        //--------------------------------------------------------------------------------
        
        bool addImg(const DotMarkerParam &mrk, const void *img, const int dsize0, const int dsize1, const int ch) {
            Mem2<Byte> gry;
            cnvPtrToImg(gry, img, dsize0, dsize1, ch);
            return addImg(mrk, gry);
        }
        
        bool addImg(const DotMarkerParam &mrk, const Mem2<Col3> &img) {
            Mem2<Byte> gry;
            cnvImg(gry, img);
            return addImg(mrk, gry);
        }

        bool addImg(const DotMarkerParam &mrk, const Mem2<Byte> &img) {

            if (m_imgs.size() == 0 || m_cam.dsize[0] == 0 || m_cam.dsize[1] == 0) {
                m_cam.dsize[0] = img.dsize[0];
                m_cam.dsize[1] = img.dsize[1];
            }

            DotMarker dotMarker;

            dotMarker.setMrk(mrk);
            dotMarker.execute(img);

            if (dotMarker.getPose() == NULL) return false;
            
            SP_PRINTF("add image id %d \n", m_pixs.size());

            m_pixs.push(*dotMarker.getCrspPixs());
            m_objs.push(*dotMarker.getCrspObjs());
            m_imgs.push(img);

            return true;
        }

        //--------------------------------------------------------------------------------
        // execute 
        //--------------------------------------------------------------------------------

        bool execute() {
            const int mini = 3;

            if (m_imgs.size() < mini) {
                SP_PRINTF("minimal image num >= %d\n", mini);
                return false;
            }

            const SP_REAL rms = calibCam(m_cam, m_cam.dsize[0], m_cam.dsize[1], m_pixs, m_objs);
            if (rms >= 0.0) {
                m_est = true;
                SP_PRINTF("calibration was successful : rms %lf \n", rms);
                print(m_cam);
            }
            else {
                m_est = false;
                SP_PRINTF("calibration was failed \n");
            }
            return m_est;
        }

        //--------------------------------------------------------------------------------
        // output 
        //--------------------------------------------------------------------------------

        int size() {
            return m_imgs.size();
        }

        CamParam* getCam() {
            return (m_est == true) ? &m_cam : NULL;
        }

        void save(char *path) {
            if (m_est == true) {
                saveText(path, m_cam);
                print(m_cam);
            }
            else {
                SP_PRINTF("no valid data \n")
            }
        }
    };

}

#endif
