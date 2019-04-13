//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RSUTIL_H__
#define __SP_RSUTIL_H__

#include "simplesp.h"
#include "librealsense/rs.hpp"


namespace sp{
    using namespace rs;

    //--------------------------------------------------------------------------------
    // real sense capture class
    //--------------------------------------------------------------------------------

    class RealSense{
        
        context m_ctx;
        device *m_dev;

        bool m_cenable;
        bool m_denable;

        bool m_start;

        CamParam m_ccam;
        CamParam m_dcam;

        Pose m_cpose;
        Pose m_dpose;

        Mem2<Col3> m_color;
        Mem2<SP_REAL> m_depth;

    public:
        RealSense(){
            m_dev = NULL;

            m_cenable = false;
            m_denable = false;
            m_start = false;
        }

        ~RealSense(){
            stop();
        }

        void init(const int id = 0){
            SP_ASSERT(id < m_ctx.get_device_count());

            SP_ASSERT(m_dev = m_ctx.get_device(id));
        }

        void enableColor(){
            SP_ASSERT(m_dev);

            m_cenable = true;

            m_dev->enable_stream(stream::color, 640, 480, format::rgb8, 30);
            const rs_intrinsics &cin = m_dev->get_stream_intrinsics(stream::color);
            m_ccam = getCamParam(cin.width, cin.height, cin.fx, cin.fy, cin.ppx, cin.ppy);

            m_color.resize(m_ccam.dsize);
            m_color.zero();

            m_cpose = zeroPose();

        }

        void enableDepth(){
            SP_ASSERT(m_dev);

            m_denable = true;
            m_dev->enable_stream(stream::depth, 640, 480, format::z16, 30);
            const rs_intrinsics &din = m_dev->get_stream_intrinsics(stream::depth);
            m_dcam = getCamParam(din.width, din.height, din.fx, din.fy, din.ppx, din.ppy);

            m_depth.resize(m_dcam.dsize);
            m_depth.zero();

            const rs_extrinsics &dex = m_dev->get_extrinsics(stream::color, stream::depth);

            SP_REAL rot[3 * 3];
            for (int r = 0; r < 3; r++){
                for (int c = 0; c < 3; c++){
                    rot[r * 3 + c] = dex.rotation[c * 3 + r];
                }
            }
            m_dpose.rot = getRot(rot, 3, 3);
            m_dpose.trn = getVec3(dex.translation[0], dex.translation[1], dex.translation[2]) * 1000.0;
        }

        void start(){
            SP_ASSERT(m_dev);

            if (m_start == false){
                m_dev->start();
                m_start = true;
            }
        }

        void stop(){
            SP_ASSERT(m_dev);
            
            if (m_start == true){
                m_dev->stop();
                m_start = false;
            }
        }

        const Mem2<Col3>* getColor(){
            return (m_cenable == true) ? &m_color : NULL;
        }

        const CamParam* getColorCam(){
            return (m_cenable == true) ? &m_ccam : NULL;
        }

        const Mem2<SP_REAL>* getDepth(){
            return (m_denable == true) ? &m_depth : NULL;
        }

        const CamParam* getDepthCam(){
            return (m_denable == true) ? &m_dcam : NULL;
        }

        void capture(){
            SP_ASSERT(m_dev);

            start();
            m_dev->wait_for_frames();

            // capture color
            if (m_cenable){
                const uint8_t *cframe = reinterpret_cast<const uint8_t*>(m_dev->get_frame_data(stream::color));

                for (int i = 0; i < m_color.size(); i++){
                    m_color[i].r = *cframe++;
                    m_color[i].g = *cframe++;
                    m_color[i].b = *cframe++;
                }
            }

            // capture depth
            if (m_denable){
                const uint16_t *dframe = reinterpret_cast<const uint16_t*>(m_dev->get_frame_data(stream::depth));

                // -> mm scale
                const SP_REAL scale = m_dev->get_depth_scale() * 1000.0;
                
                for (int i = 0; i < m_depth.size(); i++){
                    m_depth[i] = (*dframe++) * scale;
                }
            }
        }

        bool testLoad(const char *dir, const int id){

            char name[256];

            sprintf(name, "%s/color%03d.bmp", dir, id);
            const bool cret = loadBMP(name, m_color);
            
            sprintf(name, "%s/depth%03d.bin", dir, id);
            const bool dret = loadMem(name, m_depth);

            return (cret || dret) ? true : false;
        }
    };

}
#endif