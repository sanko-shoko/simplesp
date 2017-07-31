#include "opencv2/opencv.hpp"
#include "librealsense/rs.hpp"

int main(){
	rs::context ctx;

	// set realsense id
	rs::device *dev = ctx.get_device(0);

	if (dev == NULL) {
		printf("could not find realsense\n");
		return 0;
	}

	// enable depth
	dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);

	dev->start();

	// -> mm scale
	const double scale = dev->get_depth_scale() * 1000.0;

	int key = 0;
	cv::Mat img(480, 640, CV_8UC1);

	// 27 = 'ESC' key
	while ((key = cv::waitKey(1)) != 27) {

		dev->wait_for_frames();

		const uint16_t *ptr = (const uint16_t*)(dev->get_frame_data(rs::stream::depth));

		// convert to image
		for (int i = 0; i < img.size().area(); i++) {
			const double depth = scale * ptr[i];
			img.data[i] = (unsigned char)((depth / 1500.0) * 255);
		}

		cv::imshow("realsense", img);
	}

	dev->stop();
	cv::destroyAllWindows();

	return 0;
}

