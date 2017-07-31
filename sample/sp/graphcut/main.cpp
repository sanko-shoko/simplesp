#include "simplesp.h"

using namespace sp;

int main(){

	//--------------------------------------------------------------------------------
	// graph cut
	//--------------------------------------------------------------------------------

	Mem2<Byte> img;
	SP_ASSERT(loadBMP(img, SP_DATA_DIR  "/image/Lenna.bmp"));

	binalizeAdapt(img, img);
	for (int i = 0; i < img.size(); i++){
		if (sp::rand() % 10 == 0){
			img[i] = 255 - img[i];
		}
	}
	saveBMP(img, "input.bmp");


	const int kappa = 80;

	GraphCut gc(img.size(), img.size() * 4);

	for (int v = 0; v < img.dsize[1]; v++){
		for (int u = 0; u < img.dsize[0]; u++){
			gc.setNode(acsid2(img.dsize, u, v), img(u, v),  255 - img(u, v));

			const int link[][2] = { { -1, 0 }, { 0, -1 }, { -1, -1 }, { +1, -1 } };
			for (int d = 0; d < 4; d++){
				const int x = u + link[d][0];
				const int y = v + link[d][1];

				if (isInRect2(img.dsize, x, y) == true){
					gc.setLink(acsid2(img.dsize, u, v), acsid2(img.dsize, x, y), kappa);
				}
			}
		}
	}

	gc.execute();

	for (int i = 0; i < img.size(); i++){
		img[i] = (gc.getLabel(i) > 0) ? 0 : 255;
	}
	saveBMP(img, "graphcut.bmp");

	return 0;
}

