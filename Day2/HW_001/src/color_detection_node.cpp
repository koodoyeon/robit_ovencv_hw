#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;
using namespace cv;

class ColorDetectionNode : public rclcpp::Node {
public:
    ColorDetectionNode() : Node("color_detection") {
        timer_ = this->create_wall_timer(1000ms, std::bind(&ColorDetectionNode::process_image, this));
    }

private:
    // Open 함수
    Mat openOperationCustom(const Mat &mask, int kernel_size) {
        Mat open_mask;
        Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
        erode(mask, open_mask, kernel);
        dilate(open_mask, open_mask, kernel);
        return open_mask;
    }

    // Close 함수 
    Mat closeOperationCustom(const Mat &mask, int kernel_size) {
        Mat close_mask;
        Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
        dilate(mask, close_mask, kernel);
        erode(close_mask, close_mask, kernel);
        return close_mask;
    }

    // OpenCV의 함수로 Open 연산 수행
    Mat openOperation(const Mat &mask, int kernel_size) {
        Mat open_mask;
        Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
        morphologyEx(mask, open_mask, MORPH_OPEN, kernel);
        return open_mask;
    }

    // OpenCV의 함수로 Close 연산 수행
    Mat closeOperation(const Mat &mask, int kernel_size) {
        Mat close_mask;
        Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
        morphologyEx(mask, close_mask, MORPH_CLOSE, kernel);
        return close_mask;
    }

    void process_image() {
        // 이미지 경로 설정 
        std::string image_path = "/home/dy/colcon_ws/src/color_detection/resources/images/image.jpg";
        RCLCPP_INFO_ONCE(this->get_logger(), "이미지 경로: %s", image_path.c_str());

        // 이미지 불러오기
        Mat img = imread(image_path);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "이미지를 불러올 수 없습니다.");
            return;
        }

        // HSV로 변환
        Mat hsv_img;
        cvtColor(img, hsv_img, COLOR_BGR2HSV);

        // 색상 검출을 위한 커널 크기 설정
        int kernel_size = 20;

        // 빨강색 범위 설정 및 마스크 생성
        Mat red_mask_lower, red_mask_upper;
        inRange(hsv_img, Scalar(0, 100, 100), Scalar(10, 255, 255), red_mask_lower);
        inRange(hsv_img, Scalar(160, 100, 100), Scalar(179, 255, 255), red_mask_upper);
        Mat red_mask = red_mask_lower | red_mask_upper;

        // 파랑색 범위 설정 및 마스크 생성
        Mat blue_mask;
        inRange(hsv_img, Scalar(100, 150, 0), Scalar(140, 255, 255), blue_mask);

        // 초록색 범위 설정 및 마스크 생성
        Mat green_mask;
        inRange(hsv_img, Scalar(35, 100, 100), Scalar(85, 255, 255), green_mask);

        // OpenCV 함수로 Open과 Close 수행
        Mat red_open = openOperation(red_mask, kernel_size);
        Mat red_close = closeOperation(red_open, kernel_size);
        Mat blue_open = openOperation(blue_mask, kernel_size);
        Mat blue_close = closeOperation(blue_open, kernel_size);
        Mat green_open = openOperation(green_mask, kernel_size);
        Mat green_close = closeOperation(green_open, kernel_size);

        // Open과 Close 수행
        Mat red_open_custom = openOperationCustom(red_mask, kernel_size);
        Mat red_close_custom = closeOperationCustom(red_open_custom, kernel_size);
        Mat blue_open_custom = openOperationCustom(blue_mask, kernel_size);
        Mat blue_close_custom = closeOperationCustom(blue_open_custom, kernel_size);
        Mat green_open_custom = openOperationCustom(green_mask, kernel_size);
        Mat green_close_custom = closeOperationCustom(green_open_custom, kernel_size);

        // 결과 출력
        imshow("Red Detection (OpenCV)", red_close);
        imshow("Red Detection (Custom)", red_close_custom);

        imshow("Blue Detection (OpenCV)", blue_close);
        imshow("Blue Detection (Custom)", blue_close_custom);

        imshow("Green Detection (OpenCV)", green_close);
        imshow("Green Detection (Custom)", green_close_custom);

        waitKey(1);
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
