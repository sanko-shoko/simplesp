#define SP_USE_DEBUG 1
#include "simplesp.h"

using namespace sp;

int main(){
	SP_LOGGER_INSTANCE;

	Mem2<Byte> src;
	{
		SP_ASSERT(loadBMP(src, SP_DATA_DIR  "/image/Lenna.bmp"));
	}

	Mem2<Byte> dst0;
	Mem2<Byte> dst1;

	// box filter
	{
		SP_LOGGER_SET("boxfilter");

		boxFilter(dst0, src, 21);
	}

	// box filter integral
	{
		SP_LOGGER_SET("boxfilter integral");
	
		Mem2<int> sum;
		integral(sum, src);
		boxFilterIntegral(dst1, sum, 21);
	}

	saveBMP(dst0, "test0.bmp");
	saveBMP(dst1, "test1.bmp");


	return 0;
}