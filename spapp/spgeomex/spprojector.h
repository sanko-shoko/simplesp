//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_PROJECTOR_H__
#define __SP_PROJECTOR_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"

namespace sp{
	class StructuredLight {

	protected:
		int m_dsize[2];

	public:

		void setSize(const int dsize0, const int dsize1) {
			m_dsize[0] = dsize0;
			m_dsize[1] = dsize1;
		}

		virtual bool isValid() const {
			return (m_dsize[0] > 0 && m_dsize[1] > 0) ? true : false;
		}

		Mem2<Byte> getPlain(const Byte val) const {
			SP_ASSERT(isValid() == true);

			Mem2<Byte> img(m_dsize);
			setElm(img, val);

			return img;
		}

	};

	class GrayCode : public StructuredLight {

	public:
		GrayCode() {
			StructuredLight::setSize(0, 0);
		}

		GrayCode(const int dsize0, const int dsize1) {
			StructuredLight::setSize(dsize0, dsize1);
		}

		Mem1<Mem2<Byte> > encode(const int axis) const{
			SP_ASSERT(isValid() == true);

			Mem1<Mem2<Byte> > imgs;

			const int size = ceil(log(m_dsize[axis]) / log(2.0));

			for (int i = 0; i < size; i++) {
				Mem2<Byte> img(m_dsize);

				if (axis == 0) {
					for (int u = 0; u < m_dsize[0]; u++) {
						const Byte bit = getBits(size, u)[i];
						for (int v = 0; v < m_dsize[1]; v++) {
							img(u, v) = bit * SP_BYTEMAX;
						}
					}
				}
				else {
					for (int v = 0; v < m_dsize[1]; v++) {
						const Byte bit = getBits(size, v)[i];
						for (int u = 0; u < m_dsize[0]; u++) {
							img(u, v) = bit * SP_BYTEMAX;
						}
					}
				}
				imgs.push(img);
			}

			return imgs;
		}

		Mem2<int> decode(const int axis, const Mem1<Mem2<Byte> > &imgs, const Mem2<Byte> &wimg, const Mem2<Byte> &bimg, const int thresh = 10) const {
			SP_ASSERT(isValid() == true);

			Mem2<int> map(imgs[0].dsize);

			const int size = ceil(log(m_dsize[axis]) / log(2.0));
			Mem1<Byte> bits(size);

			for (int i = 0; i < map.size(); i++) {
				const Byte w = wimg[i];
				const Byte b = bimg[i];
				if (w - b > thresh) {
					for (int j = 0; j < size; j++) {
						bits[j] = (2 * imgs[j][i] > w + b) ? 1 : 0;
					}
					map[i] = getIndex(bits);
				}
				else {
					map[i] = -1;
				}
			}

			return map;
		}


	private:

		Mem1<Byte> getBits(const int size, const int index) const {
			Mem1<Byte> bits(size);

			for (int i = 0; i < size; i++) {
				const int t = round(pow(2, size - i));

				bits[i] = ((index + t / 2) / t) % 2;
			}
			return bits;
		}

		int getIndex(const Mem1<Byte> &bits) const {
			const int size = bits.size();

			int index = 0;

			Byte v = 0;
			for (int i = 0; i < size; i++) {
				const int t = round(pow(2, size - i));

				v = v ^ bits[i];
				index += v * (t / 2);
			}

			return index;
		}

	};

}
#endif