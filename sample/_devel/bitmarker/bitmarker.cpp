#include "bitmarker.h"


// capture next image
void BitMarkerGUI::nextImg(){
	// usb camera
	static cv::VideoCapture cap;
	cvCaptureImg(m_crntImg, cap);
}
// estimate marker pose
void BitMarkerGUI::calcOne(){

	m_bitMarker.execute(m_crntImg);
}


void BitMarkerGUI::display(){
	if (m_ui.start){
		nextImg();
		calcOne();
	}

	{
		glLoadView2D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);
		glRenderImage(m_crntImg);
	}

	{
		glLoadView3D(m_bitMarker.getCam(), m_viewPos, m_viewScale);

		if (1 || m_bitMarker.getBase() == NULL){
			for (int i = 0; i < m_bitMarker.getMrk().size(); i++){
				if (m_bitMarker.getPose(i) == NULL) continue;

				glLoadMatrix(*m_bitMarker.getPose(i));
				glLineWidth(4.0f);

				glBegin(GL_LINES);
				glAxis(30.0);
				glEnd();
			}

		}
		else{
			glLoadMatrix(*m_bitMarker.getBase());
			glLineWidth(4.0f);

			glBegin(GL_LINES);
			glAxis(60.0);
			glEnd();
		}
	}

#if SP_USE_DEBUG

	if (m_ui.dispMinGauss){
		const sp::Mem2<unsigned char> *minImg = (const sp::Mem2<unsigned char> *)SP_HOLDER_GET("minImg", m_bitMarker);
		if (minImg){
			const double scale = static_cast<double>(m_crntImg.dsize[0]) / minImg->dsize[0];
			glLoadView2D(getCamParam(minImg->dsize), m_viewPos, scale * m_viewScale);
			glRenderImage(*minImg);
		}
	}
	if (m_ui.dispMinLab){
		const sp::Mem2<int> *labelMap = (const sp::Mem2<int>*)SP_HOLDER_GET("labelMap", m_bitMarker);
		if (labelMap){
			glPointSize(static_cast<float>(1.2 * m_viewScale));
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
	if (m_ui.dispContour){
		const sp::Mem1<sp::Mem1<sp::Vec2> > *contours = (const sp::Mem1<sp::Mem1<sp::Vec2> >*)SP_HOLDER_GET("contours", m_bitMarker);
		if (contours){
			glColor(sp::getCol(0, 255, 0));
			glPointSize(static_cast<float>(2 * m_viewScale));

			glLoadView2D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);

			glBegin(GL_POINTS);
			for (int i = 0; i < contours->size(); i++){
				for (int j = 0; j < (*contours)[i].size(); j++){
					glVertex((*contours)[i][j]);
				}
			}
			glEnd();
		}
	}

	if (m_ui.dispCorner) {
		const sp::Mem1<sp::Mem1<sp::Vec2> > *corners = (const sp::Mem1<sp::Mem1<sp::Vec2> >*)SP_HOLDER_GET("corners", m_bitMarker);
		if (corners) {
			glColor(sp::getCol(0, 255, 0));
			glPointSize(static_cast<float>(2 * m_viewScale));

			glLoadView2D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);

			glBegin(GL_POINTS);
			for (int i = 0; i < corners->size(); i++){
				for (int j = 0; j < (*corners)[i].size(); j++){
					glVertex((*corners)[i][j]);
				}
			}
			glEnd();
		}
	}

	if (m_ui.test) {
		const sp::Mem1<sp::Mem1<sp::Vec2> > *test = (const sp::Mem1<sp::Mem1<sp::Vec2> >*)SP_HOLDER_GET("test", m_bitMarker);
		if (test) {
			glColor(sp::getCol(0, 255, 0));
			glPointSize(static_cast<float>(2 * m_viewScale));

			glLoadView2D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);

			glBegin(GL_POINTS);
			for (int i = 0; i < test->size(); i++){
				for (int j = 0; j < (*test)[i].size(); i++){
					glVertex((*test)[i][j]);
				}
			}
			glEnd();
		}
	}
#endif

}