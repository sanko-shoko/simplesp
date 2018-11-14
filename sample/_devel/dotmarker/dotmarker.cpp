#include "dotMarker.h"

// capture next image
void DotMarkerGUI::nextImg(){
    // usb camera
    static cv::VideoCapture cap;
    cvCaptureImg(m_crntImg, cap);
}

// estimate marker pose
void DotMarkerGUI::calcOne(){

    m_dotMarker.execute(m_crntImg);

    if (m_ui.useDiminish && m_dotMarker.getHom()){
        diminishDotMarker(m_crntImg, m_dotMarker.getMrk(), *m_dotMarker.getHom());
    }

    Mem2<Vec2> table;
    makeRemapTable(table, m_dotMarker.getCam());

    remap<Col3, Byte>(m_crntImg, m_crntImg, table);
}

void DotMarkerGUI::display(){

    if (ImGui::Begin("gui", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {

        if (ImGui::Button(m_ui.start ? "stop" : "start")) {
            m_ui.start ^= true;
        }
        ImGui::Checkbox("useDiminish", &m_ui.useDiminish);
        ImGui::Checkbox("dispModel", &m_ui.dispModel);
        ImGui::Checkbox("dispSkip", &m_ui.dispSkip);
        ImGui::Checkbox("dispMinGauss", &m_ui.dispMinGauss);
        ImGui::Checkbox("dispMinLab", &m_ui.dispMinLab);
        ImGui::Checkbox("dispDetectPix", &m_ui.dispDetectPix);
        ImGui::Checkbox("dispLinkLine", &m_ui.dispLinkLine);

        if (ImGui::Button("loadImg")) {
            loadImg();
        }
        if (ImGui::Button("saveImg")) {
            saveImg();
        }

        ImGui::End();
    }

    if (m_ui.start){
        nextImg();
        calcOne();
    }

    {
        glLoadView2D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);
        glTexImg(m_crntImg);
    }

    if (m_ui.dispSkip == true) return;
    if (m_dotMarker.getPose() != NULL){
        glLoadView3D(m_dotMarker.getCam(), m_viewPos, m_viewScale);

        if (m_ui.dispModel == false && m_ui.useDiminish == false){

            glLoadMatrix(*m_dotMarker.getPose());

            glLineWidth(4.0f);
            glBegin(GL_LINES);
            glAxis(100.0);
            glEnd();

            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glColor3d(0.0, 0.4, 0.8);
            glGrid(100.0, 2);
            glEnd();
        }

        if (m_ui.dispModel == true){

            // 90 + 5 degree (cam----> / board)
            const Pose AnglePose = getPose(getRotAngleX((90 + 5) * SP_PI / 180.0));

            {
                const Pose grandPose = getPose(getVec(0.0, 0.0, -100.0));
                glLoadMatrix(*m_dotMarker.getPose() * AnglePose * grandPose);
                glLineWidth(4.0f);
                glColor3d(0.0, 0.4, 0.8);

                glBegin(GL_LINES);
                glGrid(100.0, 15);
                glEnd();
            }

            {
                glLoadMatrix(zeroPose());
                const GLfloat lightPos[4] = { 0.f, 0.f, -1000.f, 0.f };
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

                const GLfloat color[] = { 0.4f, 0.5f, 0.5f, 1.0f };
                glMaterialfv(GL_FRONT, GL_DIFFUSE, color);

                const Pose modelPose = getPose(getRotAngleX(90 * SP_PI / 180.0));
                const Pose offset = getPose(getVec(0.0, 0.0, -30.0));
                glLoadMatrix(*m_dotMarker.getPose() * AnglePose * (offset * modelPose));

                for (int i = 0; i < m_model.size(); i++){
                    glBegin(GL_TRIANGLES);
                    glNormal(getMeshNrm(m_model[i]));
                    glMesh(m_model[i]);
                    glEnd();
                }
            }

        }
    }

#if SP_USE_DEBUG

    if (m_ui.dispMinGauss){
        const sp::Mem2<unsigned char> *minImg = SP_HOLDER_GET("minImg", const sp::Mem2<Byte>);
        if (minImg){
            const double scale = static_cast<double>(m_crntImg.dsize[0]) / minImg->dsize[0];
            glLoadView2D(getCamParam(minImg->dsize), m_viewPos, scale * m_viewScale);
            glTexImg(*minImg);
        }
    }
    if (m_ui.dispMinLab){
        const sp::Mem2<int> *labelMap = SP_HOLDER_GET("labelMap", const sp::Mem2<int>);
        if (labelMap){
            glPointSize(static_cast<float>(2 * m_viewScale));
            const double scale = static_cast<double>(m_crntImg.dsize[0]) / labelMap->dsize[0];
            glLoadView2D(getCamParam(labelMap->dsize), m_viewPos, scale * m_viewScale);

            glBegin(GL_POINTS);
            for (int v = 0; v < labelMap->dsize[1]; v++){
                for (int u = 0; u < labelMap->dsize[0]; u++){
                    if ((*labelMap)(u, v) < 0) continue;
                    glColor((*labelMap)(u, v));
                    glVertex(getVec(u, v));
                }
            }
            glEnd();
        }
    }

    if (m_ui.dispDetectPix){
        const sp::Mem1<Vec2> *pixs = SP_HOLDER_GET("pixs", const sp::Mem1<Vec2>);
        if (pixs){
            glLoadView2D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);
            glColor3ub(0, 255, 0);
            glPointSize(static_cast<float>(5 * m_viewScale));
            glBegin(GL_POINTS);
            for (int i = 0; i < pixs->size(); i++){
                glVertex((*pixs)[i]);
            }
            glEnd();
        }
    }

    if (m_ui.dispLinkLine){
        const sp::Mem1<Mem1<Vec2> > *links = SP_HOLDER_GET("links", const Mem1<Mem1<Vec2> >);
        if (links){
            glLoadView2D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);
            glColor3ub(0, 255, 0);
            glLineWidth(static_cast<float>(1 * m_viewScale));

            for (int i = 0; i < links->size(); i++){
                glBegin(GL_LINE_LOOP);
                for (int j = 0; j < (*links)[i].size(); j++){
                    glVertex((*links)[i][j]);
                }
                glEnd();
            }
        }
    }

#endif

}