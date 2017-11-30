#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

// [saba library] benikabocha  
// web page: http://qiita.com/benikabocha/items/ae9d48e314f51746f453
// github: https://github.com/benikabocha/saba

#include "saba.h"

using namespace sp;

class MMDGUI : public BaseWindow{

	// model
	SABA m_model;

	SABATimer m_timer;

	// model scale
	float m_scale;

	// marker detector
	DotMarker m_dotMarker;

	// diminish flag
	bool m_diminish;

private:

	virtual void init(){

		m_dotMarker.setMrk(DotMarkerParam(5, 5, 30.0));
		m_diminish = false;

		m_scale = 10.f;

		printf("'d' key : diminish marker\n");
		printf("'r' key : reset animation\n");
		printf("'ESC' key : exit\n");
	}
	virtual void drop(int num, const char **paths) {

		for (int i = 0; i < num; i++) {
			if (cmpFileExt(paths[i], "pmx")) {
				m_model.loadPMX(paths[i]);
			}
			if (cmpFileExt(paths[i], "pmd")) {
				m_model.loadPMD(paths[i]);
			}
			if (cmpFileExt(paths[i], "vmd")) {
				m_model.loadVMD(paths[i]);
				m_timer.start();
			}
		}
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {

		if (m_keyAction[GLFW_KEY_D]) {
			m_diminish ^= true;
		}
		if (m_keyAction[GLFW_KEY_R]) {
			m_timer.reset();
		}

		if (m_keyAction[GLFW_KEY_S]) {
			if (m_timer.status == true) {
				m_timer.stop();
			}
			else {
				m_timer.start();
			}
		}
		if (m_keyAction[GLFW_KEY_P]) {
			m_timer.addTime(+0.1f);
		}
		if (m_keyAction[GLFW_KEY_M]) {
			m_timer.addTime(-0.1f);
		}

		// calibration
		{
			static Mem1<Mem1<Vec2> > pixsList, objsList;

			if (m_keyAction == GLFW_KEY_A && m_dotMarker.getPose() != NULL) {
				printf("add detected points (i = %d)\n", pixsList.size());
				pixsList.push(m_dotMarker.getCrspPix());
				objsList.push(m_dotMarker.getCrspObj());

			}

			if (m_keyAction[GLFW_KEY_C]) {
				CamParam cam;
				const double rms = calibCam(cam, m_dotMarker.getCam().dsize[0], m_dotMarker.getCam().dsize[1], pixsList, objsList);
				if (rms >= 0.0) {
					printf("rms %lf\n", rms);
					saveText(cam, "cam.txt");
					m_dotMarker.setCam(cam);
				}
			}
		}
	}

	virtual void display(){
		Mem2<Col3> img;

		// capture image
		{
			static cv::VideoCapture cap;
			if (cvCaptureImg(img, cap) == false) return;
		}

		// detect dot marker
		m_dotMarker.execute(img);

		if (m_diminish == true) {
			m_dotMarker.diminish(img);
		}

		{
			glLoadView2D(m_dotMarker.getCam(), m_viewPos, m_viewScale);
			glRenderImage(img);
		}

		if (m_dotMarker.getPose() != NULL && m_model.getModel() != NULL) {

			// cam           /
			//       model  / board (5 deg)
			//             /  

			const Pose boardToCamPose = *m_dotMarker.getPose();
			const Pose worldToBoardPose = getPose(getRotAngleX(5 * SP_PI / 180.0));
			const Pose worldToModelPose = getPose(getRotAngleX(180 * SP_PI / 180.0), getVec(0.0, 120.0, -30.0));
		
			const Pose modelToCamPose = boardToCamPose * worldToBoardPose * invPose(worldToModelPose);

			// view 3D
			glLoadView3D(m_dotMarker.getCam(), m_viewPos, m_viewScale);

			// light
			{
				glLoadMatrix(zeroPose());

				glEnable(GL_LIGHTING);
				glEnable(GL_LIGHT0);

				GLfloat lightPos0[4] = { 0.f, 0.f, -1500.f, 1.f };
				glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
			}


			m_model.updateAnimation(m_timer.getTime());

			// model
			{
				glLoadMatrix(modelToCamPose);

				const glm::vec3* positions = m_model.getModel()->GetUpdatePositions();
				const glm::vec3* normals = m_model.getModel()->GetUpdateNormals();
				const glm::vec2* uvs = m_model.getModel()->GetUpdateUVs();

				const MMDSubMesh* subMeshes = m_model.getModel()->GetSubMeshes();
				const MMDMaterial* materials = m_model.getModel()->GetMaterials();

				const vector<size_t> indices = m_model.getIndice();

				glEnable(GL_TEXTURE_2D);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glShadeModel(GL_SMOOTH);

				for (size_t i = 0; i < m_model.getModel()->GetSubMeshCount(); i++) {

					// material
					const MMDMaterial &mat = materials[subMeshes[i].m_materialID];
					const float ambient[4] = { mat.m_ambient.x, mat.m_ambient.y, mat.m_ambient.z, mat.m_alpha };
					const float diffuse[4] = { mat.m_diffuse.x, mat.m_diffuse.y, mat.m_diffuse.z, mat.m_alpha };
					const float specular[4] = { mat.m_specular.x, mat.m_specular.y, mat.m_specular.z, mat.m_alpha };
					glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
					glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
					glMaterialfv(GL_FRONT, GL_SPECULAR, specular);

					// texture
					glBindTexture(GL_TEXTURE_2D, m_model.getTexture(subMeshes[i].m_materialID));
					
					// vertex
					int cnt = subMeshes[i].m_beginIndex;
					for (size_t j = 0; j < subMeshes[i].m_vertexCount; j += 3) {
						glBegin(GL_TRIANGLES);
						for (size_t k = 0; k < 3; k++) {
							const size_t vi = indices[cnt++];

							const glm::vec3 p = m_scale * positions[vi];
							const glm::vec3 n = normals[vi];
							const glm::vec2 uv = uvs[vi];
							
							glTexCoord2f(uv.x, 1+uv.y);
							glNormal3d(n.x, n.y, n.z);
							glVertex3f(p.x, p.y, p.z);
						}
						glEnd();
					}
				}
				glBindTexture(GL_TEXTURE_2D, 0);
				glDisable(GL_TEXTURE_2D);
				glDisable(GL_BLEND);
			}
		}
	}


};
