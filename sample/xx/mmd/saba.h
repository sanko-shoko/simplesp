//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SABA_H__
#define __SABA_H__

// [saba library] benikabocha  
// web page: http://qiita.com/benikabocha/items/ae9d48e314f51746f453
// github: https://github.com/benikabocha/saba

#include <Saba/Base/UnicodeUtil.h>
#include <Saba/Base/Path.h>
#include <Saba/Model/MMD/MMDModel.h>
#include <Saba/Model/MMD/PMDModel.h>
#include <Saba/Model/MMD/PMXModel.h>
#include <Saba/Model/MMD/VMDFile.h>
#include <Saba/Model/MMD/VMDAnimation.h>

#include <vector>
#include <string>
#include <locale>
#include <codecvt>
#include <time.h>

#include "spex/splocale.h"
#include "GLFW/glfw3.h"
#include "opencv2/opencv.hpp"

namespace sp {
	using namespace std;
	using namespace chrono;
	using namespace saba;

	class SABATimer {
		float time;
		float prev;

	public:
		SABATimer() {
			reset();
		}
		void reset() {
			time = 0.f;
			prev = 0.f;
			status = false;
		}

		bool status;

		void start() {
			prev = clock_s();
			status = true;
		}

		void stop() {
			status = false;
		}

		float getDelta() {
			return clock_s() - prev;
		}

		float getTime() {
			if (status == true) {
				time += getDelta();
			}
			prev = clock_s();
			return time;
		}

		void addTime(const float t) {
			time += t;
		}

	private:
		float clock_s() {
			return static_cast<float>(clock()) / CLOCKS_PER_SEC;
		}
	};

	class SABA {
	private:
		shared_ptr<MMDModel> m_mmd;

		shared_ptr<VMDAnimation> m_vmd;

		vector<GLuint> m_texs;

	private:

		vector<cv::Mat> m_imgs;

		void loadTex(const shared_ptr<MMDModel> mmd) {
			const size_t size = mmd->GetMaterialCount();
			const MMDMaterial* materials = mmd->GetMaterials();

			m_imgs.resize(size);
			m_texs.resize(size);
			for (size_t i = 0; i < m_imgs.size(); i++) {
				string path;
				cnvUTF8ToMB(path, materials[i].m_texture);
				m_imgs[i] = cv::imread(path.c_str());
				if (m_imgs[i].size().width == 0 || m_imgs[i].size().height == 0) continue;

				glGenTextures(1, &m_texs[i]);
				glBindTexture(GL_TEXTURE_2D, m_texs[i]);

				glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_imgs[i].size().width, m_imgs[i].size().height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_imgs[i].ptr());
			}
			glBindTexture(GL_TEXTURE_2D, 0);
		}

		void deleteTex() {
			for (size_t i = 0; i < m_imgs.size(); i++) {
				if (m_imgs[i].size().width == 0 || m_imgs[i].size().height == 0) continue;
				glDeleteTextures(1, &m_texs[i]);
			}
		}

	public:

		SABA() {
			m_mmd = NULL;
			m_vmd = NULL;
		}

		~SABA() {
			deleteTex();
		}

		const shared_ptr<MMDModel> getModel() const {
			return m_mmd;
		}

		GLuint getTexture(size_t i) const {
			return m_texs[i];
		}

		void loadPMX(const string modelPath, const string dataPath = "") {
			shared_ptr<PMXModel> pmx = make_shared<PMXModel>();
			pmx->Load(modelPath, dataPath);
			m_mmd = move(pmx);

			loadTex(m_mmd);

			m_vmd = make_shared<VMDAnimation>();
			m_vmd->Create(m_mmd);

			m_mmd->InitializeAnimation();
			m_vmd->SyncPhysics(0.f);

		}

		void loadPMD(const string modelPath, const string dataPath = "") {
			shared_ptr<PMDModel> pmd = make_shared<PMDModel>();
			pmd->Load(modelPath, dataPath);
			m_mmd = move(pmd);

			loadTex(m_mmd);

			m_vmd = make_shared<VMDAnimation>();
			m_vmd->Create(m_mmd);

			m_mmd->InitializeAnimation();
			m_vmd->SyncPhysics(0.f);

		}

		void loadVMD(const string vmdPath) {

			m_vmd = make_shared<VMDAnimation>();
			m_vmd->Create(m_mmd);

			m_mmd->InitializeAnimation();
			m_vmd->SyncPhysics(0.f);

			VMDFile vmdFile;
			ReadVMDFile(&vmdFile, vmdPath.c_str());

			m_vmd->Add(vmdFile);

			m_mmd->InitializeAnimation();
			m_vmd->SyncPhysics(0.f);

		}

		void updateAnimation(const float time) {
			// update animation

			m_mmd->BeginAnimation();
			m_vmd->Evaluate(time * 30.f);
			m_mmd->UpdateAnimation();
			m_mmd->EndAnimation();

			m_mmd->UpdatePhysics(1.0f / 6.0f);

			m_mmd->Update();
		}

		vector<size_t> getIndice() {
			vector<size_t> indices(m_mmd->GetIndexCount());

			switch (m_mmd->GetIndexElementSize()) {
			case 1:
			{
				uint8_t* ptr = (uint8_t*)m_mmd->GetIndices();
				for (int i = 0; i < indices.size(); i++) {
					indices[i] = ptr[i];
				}
				break;
			}
			case 2:
			{
				uint16_t* ptr = (uint16_t*)m_mmd->GetIndices();
				for (int i = 0; i < indices.size(); i++) {
					indices[i] = ptr[i];
				}
				break;
			}
			case 4:
			{
				uint32_t* ptr = (uint32_t*)m_mmd->GetIndices();
				for (int i = 0; i < indices.size(); i++) {
					indices[i] = ptr[i];
				}
				break;
			}
			}
			return indices;
		}

	};

}

#endif