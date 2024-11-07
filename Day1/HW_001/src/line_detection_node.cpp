#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

class LineDetectionNode : public rclcpp::Node {
public:
    LineDetectionNode() : Node("line_detection_node") {
        // 이미지 파일 경로 설정
        string image_path = "/home/dy/colcon_ws/src/line_detection/include/line_detection/image.png";

        // 이미지를 불러옵니다.
        Mat img = imread(image_path);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "이미지를 불러올 수 없습니다: %s", image_path.c_str());
            return;
        }

        // 원본 이미지를 HSV로 변환하여 노란색 필터링 (블러 미적용)
        Mat hsv_img, binary_img_no_blur;
        cvtColor(img, hsv_img, COLOR_BGR2HSV);
        Scalar lower_yellow(20, 170, 100);
        Scalar upper_yellow(30, 255, 255);
        inRange(hsv_img, lower_yellow, upper_yellow, binary_img_no_blur);

        // 가우시안 블러 적용
        Mat blurred_img;
        GaussianBlur(img, blurred_img, Size(5, 5), 0);

        // 가우시안 블러 적용 후 HSV로 변환 및 노란색 필터링
        Mat hsv_blurred_img, binary_img_blur;
        cvtColor(blurred_img, hsv_blurred_img, COLOR_BGR2HSV);
        inRange(hsv_blurred_img, lower_yellow, upper_yellow, binary_img_blur);

        // 결과 이미지를 각각 창으로 표시
        imshow("Original Image", img);                      // 원본 이미지
        imshow("Binary Image (No Blur)", binary_img_no_blur); // 블러 미적용 필터링 이미지
        imshow("Binary Image (Blur)", binary_img_blur); // 블러 적용 필터링 이미지
        waitKey(0);  // 키 입력이 있을 때까지 창을 유지합니다.
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
