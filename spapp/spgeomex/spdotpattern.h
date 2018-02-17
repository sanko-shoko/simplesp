//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_DOTPATTERN_H__
#define __SP_DOTPATTERN_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spimg/sprender.h"

namespace sp{

	SP_CPUFUNC void splitMarkerAndPattern(Mem<Byte> &mrkImg, Mem<Byte> &ptnImg, const Mem<Byte> &src) {
		
		SP_ASSERT(isValid(2, src));

		Mem2<Byte> base(src.dsize);
		{
			const double scale = 100.0 / (src.dsize[0] + src.dsize[1]);

			Mem2<Byte> minImg;
			rescale(minImg, src, scale, scale);

			medianFilter(minImg, minImg, 3);

			rescale(base, minImg);
		}

		mrkImg.resize(2, base.dsize);
		ptnImg.resize(2, base.dsize);

		for (int i = 0; i < base.size(); i++) {
			mrkImg[i] = (src[i] < base[i]) ? src[i] : base[i];
			ptnImg[i] = (src[i] > base[i]) ? src[i] - base[i] : 0;
		}
	}

	SP_CPUFUNC void splitMarkerAndPattern(Mem<Col3> &mrkImg, Mem<Col3> &ptnImg, const Mem<Col3> &src) {

		Mem2<Byte> gry;
		cnvImg(gry, src);

		Mem2<Byte> mrkGry, ptnGry;
		splitMarkerAndPattern(mrkGry, ptnGry, gry);

		cnvImg(mrkImg, mrkGry);
		cnvImg(ptnImg, ptnGry);
	}

	//--------------------------------------------------------------------------------
	// dot pattern design parameter
	//--------------------------------------------------------------------------------

	class DotPatternParam {
	public:
		Mem2<Byte> img;
		Mem2<Vec2> map;
		Mem2<Vec2> prjs;
		Mem2<Byte> bits;

		int distance;

		DotPatternParam(const int dsize0 = 640, const int dsize1 = 480, const int distance = 40) {
			setParam(dsize0, dsize1, distance);
		}

		DotPatternParam(const DotPatternParam &mrk) {
			*this = mrk;
		}

		DotPatternParam& operator = (const DotPatternParam &ptn) {
			img = ptn.img;
			map = ptn.map;
			prjs = ptn.prjs;
			bits = ptn.bits;
			distance = ptn.distance;

			return *this;
		}

		void setParam(const int dsize0, const int dsize1, const int distance) {

			img.resize(dsize0, dsize1);
			img.zero();

			map = grid((dsize0 - distance / 2) / distance, (dsize1 - distance / 2) / distance);

			prjs.resize(map.dsize);
			bits.resize(map.dsize);

			srand(0);

			const int radius = 2;
			const int val = SP_BYTEMAX >> 1;

			for (int y = 0; y < map.dsize[1]; y++) {
				for (int x = 0; x < map.dsize[0]; x++) {
					const int u = (x + 1) * distance;
					const int v = (y + 1) * distance;

					const Vec2 prj = getVec(u, v);
					const Byte bit = rand() % 2;

					renderPoint<Byte>(img, prj, val, (bit == 0) ? radius : 2 * radius);

					prjs(x, y) = prj;
					bits(x, y) = bit;
				}
			}
		}	

	};


	//--------------------------------------------------------------------------------
	// dot pattern pose estimator
	//--------------------------------------------------------------------------------

	class DotPattern {
	private:
		// input parameter
		DotPatternParam m_ptn;

		// crsp pnts
		Mem1<Vec2> m_cpixs, m_cprjs;

		// prj to pix homography
		Mat m_hom;

	public:
		SP_LOGGER_INSTANCE;
		SP_HOLDER_INSTANCE;

		DotPattern() {
		}

		//--------------------------------------------------------------------------------
		// input parameter
		//--------------------------------------------------------------------------------

		void setPtn(const DotPatternParam &ptn) {
			m_ptn = ptn;
		}

		const DotPatternParam& getPtn() const {
			return m_ptn;
		}

		//--------------------------------------------------------------------------------
		// output parameter
		//--------------------------------------------------------------------------------

		const Mem1<Vec2>& getCrspPix() const {
			return m_cpixs;
		}

		const Mem1<Vec2>& getCrspPrj() const {
			return m_cprjs;
		}

		const Mat& getCrspHom() const {
			return m_hom;
		}

		//--------------------------------------------------------------------------------
		// execute pose estimation
		//--------------------------------------------------------------------------------

		bool execute(const Mem2<Col3> &src) {
			Mem2<Byte> gry;
			cnvImg(gry, src);
			return _execute(gry);
		}

		bool execute(const Mem2<Byte> &src) {
			return _execute(src);
		}

	private:

		bool _execute(const Mem2<Byte> &src) {
			SP_LOGGER_SET("-execute");

			// clear data
			{
				m_cpixs.clear();
				m_cprjs.clear();
			}

			try {
				if (src.size() == 0) throw "image size";

				// detect blob
				Mem1<Vec2> pixs;
				Mem1<double> scales;
				detect(pixs, scales, src);

				estimate(src, pixs, scales);
			}
			catch (const char *str) {
				SP_PRINTD("DotPattern.execute [%s]\n", str);

				return false;
			}

			return true;
		}

	private:

		//--------------------------------------------------------------------------------
		// execute main flow
		//--------------------------------------------------------------------------------

		void detect(Mem1<Vec2> &pixs, Mem1<double> &scales, const Mem2<Byte> &img) {
			Mem2<int> labelMap;
			{
				SP_LOGGER_SET("labeling");

				Mem2<Byte> bin;
				binalizeAdapt(bin, img);

				labeling(labelMap, bin);

				SP_HOLDER_SET("labelMap", labelMap);
			}

			// get label center (and refine center)
			{
				SP_LOGGER_SET("getBlob");

				getBlob(pixs, scales, labelMap);

				SP_HOLDER_SET("pixs", pixs);
			}
		}

		void estimate(const Mem2<Byte> &img, const Mem1<Vec2> &pixs, const Mem1<double> &scales) {

			KdTree<double> kdtree(2);
			{
				SP_LOGGER_SET("kdtree");
				for (int i = 0; i < pixs.size(); i++) {
					kdtree.addData(&pixs[i]);
				}
			}

			double eval = 0.0;
			Mat hom;
			{
				SP_LOGGER_SET("recog");

				Mem1<Mem1<Vec2> > links;
				getLink(links, pixs, kdtree);
				SP_HOLDER_SET("links", links);

				if (links.size() == 0) {
					throw "links";
				}

				eval = recog(hom, m_ptn, links, pixs, scales, kdtree);
			}

			{
				SP_LOGGER_SET("eval");

				if (eval < 0.3 * m_ptn.map.dsize[0] * m_ptn.map.dsize[1]) {
					throw "eval";
				}
			}

			{
				Mem1<Vec2> map;
				getFineCrsp(m_cpixs, map, hom, m_ptn.map, pixs, kdtree);

				for (int i = 0; i < map.size(); i++) {
					const Vec2 prj = m_ptn.prjs(round(map[i].x), round(map[i].y));
					m_cprjs.push(prj);
				}

				calcHMat(m_hom, m_cpixs, m_cprjs);
			}
		}

		//--------------------------------------------------------------------------------
		// modules
		//--------------------------------------------------------------------------------

		void getBlob(Mem1<Vec2> &pixs, Mem1<double> &scales, const Mem2<int> &labelMap) {

			const Mem1<Rect> rects = getLabelRect(labelMap);

			for (int i = 0; i < rects.size(); i++) {
				const Rect rect = adjustRect(rects[i], 1);

				// check outside area
				if (isInRect(getRect2(labelMap.dsize), rect) == false) continue;

				Vec2 sum = getVec(0, 0);
				int cnt = 0;
				for (int y = 0; y < rect.dsize[1]; y++) {
					for (int x = 0; x < rect.dsize[0]; x++) {
						if (labelMap(x + rect.dbase[0], y + rect.dbase[1]) > 0) continue;

						sum += getVec(x, y);
						cnt++;
					}
				}
				if (cnt > 0) {
					pixs.push(sum / cnt + getVec(rect.dbase[0], rect.dbase[1]));
					scales.push(cnt);
				}
			}
		}


		double recog(Mat &hom, const DotPatternParam &ptn, const Mem1<Mem1<Vec2> > &links, const Mem1<Vec2> &pixs, const Mem1<double> &scales, const KdTree<double> &kdtree) {

			const Mem2<Vec2> ext = grid(2 * (ptn.map.dsize[0] - 1), 2 * (ptn.map.dsize[1] - 1));
			const Mem2<Vec2> unit0 = grid(2, 2) + meanVec(ext) - getVec(0.5, 0.5);
			const Mem2<Vec2> unit1 = grid(4, 4) + meanVec(ext) - getVec(1.5, 1.5);
			const Mem2<Vec2> unit2 = grid(6, 6) + meanVec(ext) - getVec(2.5, 2.5);

			const int maxn = 10;
			const double step = maxVal(static_cast<double>(links.size()) / maxn, 1.0);

			Mat H = eyeMat(3, 3);

			double maxv = 0.0;
			for (int n = 0; n < maxn; n++) {
				const int i = round(n * step);
				if (i >= links.size()) break;

				Mat h;
				if (calcHMat(h, links[i], unit0) == false) continue;
				if (refineHMat(h, unit1, pixs, kdtree) == false) continue;
				if (refineHMat(h, unit2, pixs, kdtree) == false) continue;

				if (refineHMat(h, ext, pixs, kdtree) == false) continue;

				Mem2<double> evalMap;
				getEvalMap(evalMap, h, ext, pixs, kdtree);

				Vec2 v;
				const double eval = sumVal(evalMap);
				if (eval > maxv) {
					H = h;
					maxv = eval;
				}
			}

			for (int i = 0; i < 3; i++) {
				if (refineHMat(H, ext, pixs, kdtree) == false) return 0.0;
			}

			Mem2<double> codeMap;
			getCodeMap(codeMap, H, ext, pixs, scales, kdtree);

			Vec2 peak;
			searchPeak(peak, ptn, codeMap);

			Mat offset = eyeMat(3, 3);
			offset(0, 2) = peak.x;
			offset(1, 2) = peak.y;

			hom = H * offset;

			return maxv;
		}

		void getLink(Mem1<Mem1<Vec2> > &links, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree) {
			const double MIN_ASPECT = 0.3;
			const double MIN_COS = cos(SP_PI * (90.0 + 30.0) / 180.0);

			Mem1<double> norms;
			for (int p = 0; p < pixs.size(); p++) {
				const Mem1<int> indexes = kdtree.search(&pixs[p], 100.0);
				double minv = SP_INFINITY;
				for (int i = 0; i < indexes.size(); i++) {
					const double norm = normVec(pixs[p] - pixs[indexes[i]]);
					if (norm > 0 && norm < minv) {
						minv = norm;
					}
				}
				if (indexes.size() > 0) {
					norms.push(minv);
				}
			}
			const double range = medianVal(norms);

			for (int p0 = 0; p0 < pixs.size(); p0++) {
				Mem1<int> index = kdtree.search(&pixs[p0], 2.0 * range);

				for (int n1 = 0; n1 < index.size(); n1++) {
					const int p1 = index[n1];
					if (p1 == p0 || pixs[p1].y < pixs[p0].y) continue;

					for (int n2 = n1 + 1; n2 < index.size(); n2++) {
						const int p2 = index[n2];
						if (p2 == p0 || pixs[p2].y < pixs[p0].y) continue;

						// select 3 pixs
						// p0, p1, p2 ( pixs[p1].y, pixs[p2].y >= pixs[p0].y )
						// s0, s1, s2 ( |pixs[s2] -  pixs[s1]| >= others )

						int s0, s1, s2;
						{
							const double lng01 = normVec(pixs[p1] - pixs[p0]);
							const double lng12 = normVec(pixs[p2] - pixs[p1]);
							const double lng20 = normVec(pixs[p0] - pixs[p2]);

							if (lng12 >= maxVal(lng01, lng20)) {
								s0 = p0, s1 = p1, s2 = p2;
							}
							if (lng20 >= maxVal(lng12, lng01)) {
								s0 = p1, s1 = p2, s2 = p0;
							}
							if (lng01 >= maxVal(lng20, lng12)) {
								s0 = p2, s1 = p0, s2 = p1;
							}
						}

						// check 3 pixs relation
						const Vec2 A = pixs[s1] - pixs[s0];
						const Vec2 B = pixs[s2] - pixs[s0];
						{
							const double lngA = normVec(A);
							const double lngB = normVec(B);
							const double cosAB = dotVec(A, B) / (lngA * lngB);

							if (minVal(lngA, lngB) / maxVal(lngA, lngB) < MIN_ASPECT) continue;
							if (cosAB > 0.0 || cosAB < MIN_COS) continue;
						}

						// check outlier in triangle and search s3
						int s3 = -1;
						{
							const double margin = 0.2;

							double inv[2 * 2];
							double mat[2 * 2] = { A.x, B.x, A.y, B.y };
							if (invMat22(inv, mat) == false) continue;

							bool check = true;
							for (int k = 0; k < index.size(); k++) {
								const int pk = index[k];
								if (pk == p0 || pk == p1 || pk == p2) continue;

								const Vec2 e = mulMat(inv, 2, 2, pixs[pk] - pixs[s0]);
								if (minVal(e.x, e.y) < -margin || maxVal(e.x, e.y) > 1.0 + margin) continue;

								if (s3 < 0 && minVal(e.x, e.y) > 1.0 - margin) {
									s3 = pk;
								}
								else {
									check = false;
									break;
								}
							}

							if (s3 < 0 || check == false) continue;
						}

						// align pixs
						Vec2 v[4] = { pixs[s0], pixs[s1], pixs[s2], pixs[s3] };
						{
							const Vec2 direct = getVec(1.0, 0.0);

							const Vec2 xdirect = direct;
							const Vec2 ydirect = getVec(-direct.y, direct.x);

							for (int i = 0; i < 4; i++) {
								for (int j = i + 1; j < 4; j++) {
									if (dotVec(ydirect, v[i]) > dotVec(ydirect, v[j])) {
										swap(v[i], v[j]);
									}
								}
							}

							if (dotVec(xdirect, v[0]) > dotVec(xdirect, v[1])) swap(v[0], v[1]);
							if (dotVec(xdirect, v[2]) > dotVec(xdirect, v[3])) swap(v[2], v[3]);
						}

						links.push(Mem1<Vec2>(4, &v));
					}
				}
			}
		}

		void getEvalMap(Mem2<double> &evalMap, const Mat &hom, const Mem2<Vec2> &mrkMap, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree) {
			evalMap.resize(mrkMap.dsize);
			evalMap.zero();

			const Mat ihom = invMat(hom);

			for (int y = 0; y < mrkMap.dsize[1]; y++) {
				for (int x = 0; x < mrkMap.dsize[0]; x++) {
					const Vec2 obj = mrkMap(x, y);
					const Vec2 pix = hom * obj;

					const int id = kdtree.search(&pix);
					if (id < 0) continue;

					const double err = normVec(obj - ihom * pixs[id]);

					if (err < 0.1) {
						evalMap(x, y) = 1.0;
					}
				}
			}
		}

		double searchPeak(Vec2 &offset, const Mem2<double> &evalMap, const int dsize0, const int dsize1) {

			double maxEval = 0.0;
			for (int y = 0; y <= evalMap.dsize[1] - dsize1; y++) {
				for (int x = 0; x <= evalMap.dsize[0] - dsize0; x++) {
					double eval = 0.0;
					for (int b = 0; b < dsize1; b++) {
						for (int a = 0; a < dsize0; a++) {
							eval += evalMap(x + a, y + b);
						}
					}
					if (eval > maxEval) {
						maxEval = eval;
						offset = getVec(x, y);
					}
				}
			}
			return maxEval;
		}

		void getCodeMap(Mem2<double> &codeMap, const Mat &hom, const Mem2<Vec2> &mrkMap, const Mem1<Vec2> &pixs, const Mem1<double> &scales, const KdTree<double> &kdtree) {

			codeMap.resize(mrkMap.dsize);
			codeMap.zero();

			Mem2<double> scaleMap(mrkMap.dsize);
			scaleMap.zero();

			const Mat ihom = invMat(hom);

			for (int y = 0; y < mrkMap.dsize[1]; y++) {
				for (int x = 0; x < mrkMap.dsize[0]; x++) {
					const Vec2 obj = mrkMap(x, y);
					const Vec2 pix = hom * obj;

					const int id = kdtree.search(&pix);
					if (id < 0) continue;

					const double err = normVec(obj - ihom * pixs[id]);

					if (err < 0.1) {
						scaleMap(x, y) = scales[id];
					}
				}
			}
			
			Mem1<double> list;
			for (int i = 0; i < scaleMap.size(); i++) {
				if (scaleMap[i] > 0) {
					list.push(scaleMap[i]);
				}
			}
			const double median = medianVal(list);

			for (int i = 0; i < scaleMap.size(); i++) {
				if (scaleMap[i] > 0) {
					codeMap[i] = scaleMap[i] - median;
				}
			}

		}
	
		double searchPeak(Vec2 &peak, const DotPatternParam &ptn, const Mem2<double> &codeMap) {

			double maxEval = 0.0;
			for (int y = 0; y <= codeMap.dsize[1] - ptn.map.dsize[1]; y++) {
				for (int x = 0; x <= codeMap.dsize[0] - ptn.map.dsize[0]; x++) {
					double eval = 0.0;
					for (int b = 0; b < ptn.map.dsize[1]; b++) {
						for (int a = 0; a < ptn.map.dsize[0]; a++) {
							const double code = codeMap(x + a, y + b);
							if (code < 0.0 && ptn.bits(a, b) == 0) {
								eval++;
							}
							if (code < 0.0 && ptn.bits(a, b) == 0) {
								eval++;
							}
						}
					}
					if (eval > maxEval) {
						maxEval = eval;
						peak = getVec(x, y);
					}
				}
			}
			return maxEval;
		}

		void getFineCrsp(Mem1<Vec2> &cpixs, Mem1<Vec2> &cobjs, const Mat &hom, const Mem2<Vec2> &mrkMap, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree) {
			cpixs.clear();
			cobjs.clear();
			const Mat ihom = invMat(hom);

			for (int y = 0; y < mrkMap.dsize[1]; y++) {
				for (int x = 0; x < mrkMap.dsize[0]; x++) {
					const Vec2 obj = mrkMap(x, y);
					const Vec2 pix = hom * obj;

					const int id = kdtree.search(&pix);
					if (id < 0) continue;

					const double err = normVec(obj - ihom * pixs[id]);

					if (err < 0.1) {
						cpixs.push(pixs[id]);
						cobjs.push(obj);
					}
				}
			}
		}

		bool refineHMat(Mat &hom, const Mem2<Vec2> &mrkMap, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree) {
			Mem1<Vec2> cpixs, cobjs;
			getFineCrsp(cpixs, cobjs, hom, mrkMap, pixs, kdtree);

			return calcHMat(hom, cpixs, cobjs);
		}

	};

}

#endif