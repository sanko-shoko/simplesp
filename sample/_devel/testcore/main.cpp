#include "simplesp.h"
using namespace sp;

int main(){ 
	SP_PRINTF("compile test\n");

	Mem2<int> rm(10, 10);
	for (int i = 0; i < rm.size(); i++) {
		rm[i] = sp::rand() % 5;
	}
	print(rm);

	Mem2<int> sum;
	integral(sum, rm);
	print(sum);
		
	Mem2<int> fil;
	boxFilterIntegral(fil, sum, 3);
	print(fil);
	return 0;
}