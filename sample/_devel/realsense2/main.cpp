#include "opencv2/opencv.hpp"
#include "librealsense2/rs.hpp"

int main(){
	//rs2::pipeline pipe;
	//rs2::pipeline_profile profile = pipe.start();

	//rs2::device dev = profile.get_device();

	//if (dev.query_sensors().size() == 0) {
	//	printf("could not find realsense\n");
	//	return 0;
	//}

	////rs2::sensor sensor = dev.query_sensors()[0];
	////rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>();
	////return 0;

	////// -> mm scale
	////const double scale = depth_sensor.get_depth_scale() * 1000.0;
	////printf("%lf\n", scale);
	//int key = 0;
	//cv::Mat img(480, 640, CV_8UC1);

	//// 27 = 'ESC' key
	//while ((key = cv::waitKey(1)) != 27) {
	//	printf("a");
	//	
	//	rs2::frameset data = pipe.wait_for_frames();

	//	printf("b");
	//	rs2::depth_frame depth = data.get_depth_frame();
	//	printf("c\n");

	//	//const uint16_t *ptr = (const uint16_t*)(depth.get_data());

	//	//// convert to image
	//	//for (int i = 0; i < img.size().area(); i++) {
	//	//	const double depth = scale * ptr[i];
	//	//	img.data[i] = (unsigned char)((depth / 1500.0) * 255);
	//	//}

	//	cv::imshow("realsense2", img);
	//}

	//pipe.stop();
	//cv::destroyAllWindows();

	return 0;
}

