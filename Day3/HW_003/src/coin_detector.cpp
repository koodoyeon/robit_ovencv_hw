#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

using namespace cv;
using namespace std;

class CoinDetector : public rclcpp::Node
{
public:
    CoinDetector() : Node("coin_detector")
    {
        detectAndLabelCoins();
    }

private:
    void detectAndLabelCoins()
    {
        // 이미지 경로 설정
        std::string image_path = "/home/dy/colcon_ws/src/coin_detection/include/coin_detection/image.jpg";
        RCLCPP_INFO(this->get_logger(), "Loading image from path: %s", image_path.c_str());

        // 이미지 로드
        Mat image = imread(image_path, IMREAD_GRAYSCALE);
        
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image at %s", image_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Image loaded successfully.");

        // 컬러 이미지로 변환하여 색상 라벨링 가능하게 설정
        Mat color_image;
        cvtColor(image, color_image, COLOR_GRAY2BGR);

        // 가우시안 블러 적용하여 노이즈 감소
        Mat blurred;
        GaussianBlur(image, blurred, Size(9, 9), 2);

        // 허프 원 변환을 이용해 원 검출
        vector<Vec3f> circles;
        HoughCircles(blurred, circles, HOUGH_GRADIENT, 1, blurred.rows / 8, 100, 30, 20, 80);

        int total_amount = 0;

        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);
            int radius = c[2];
            
            // 원 그리기
            circle(color_image, center, radius, Scalar(0, 255, 0), 2);
            circle(color_image, center, 2, Scalar(0, 255, 0), 3);

            // 반지름을 기준으로 금액 결정
            int coin_value = 0;
            if (radius > 40) {
                coin_value = 500; // 500원
            } else if (radius > 30) {
                coin_value = 100; // 100원
            } else if (radius > 20) {
                coin_value = 50;  // 50원
            } else {
                coin_value = 10;  // 10원
            }
            total_amount += coin_value;

            // 동전 값 라벨링
            putText(color_image, to_string(coin_value) + " Won", center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
        }

        // 터미널에 총 금액 출력
        RCLCPP_INFO(this->get_logger(), "총 금액 : %d 원", total_amount);

        // 결과 이미지 표시
        imshow("coin_cal", color_image);
        waitKey(0); 

    
        imwrite("/home/dy/colcon_ws/src/coin_detection/include/coin_detection/labeled_coins.jpg", color_image);
        RCLCPP_INFO(this->get_logger(), "Labeled image saved successfully.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoinDetector>());
    rclcpp::shutdown();
    return 0;
}
