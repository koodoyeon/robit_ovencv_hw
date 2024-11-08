#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std::chrono_literals;

class BallDetectionNode : public rclcpp::Node {
public:
    BallDetectionNode() : Node("ball_detection") {
        timer_ = this->create_wall_timer(1000ms, std::bind(&BallDetectionNode::process_image, this));
    }

private:
    void process_image() {
        std::string image_path = "/home/dy/colcon_ws/src/ball_detection/resources/images/image.jpg";
        Mat img = imread(image_path);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "이미지를 불러올 수 없습니다.");
            return;
        }

        // HSV 변환 후 각 색상별 마스크 생성
        Mat hsv_img, red_mask, green_mask, blue_mask;
        cvtColor(img, hsv_img, COLOR_BGR2HSV);

        inRange(hsv_img, Scalar(0, 150, 150), Scalar(10, 255, 255), red_mask);
        inRange(hsv_img, Scalar(35, 100, 100), Scalar(85, 255, 255), green_mask);
        inRange(hsv_img, Scalar(100, 150, 150), Scalar(130, 255, 255), blue_mask);

        // 레이블링 및 노이즈 제거를 위해 Open/Close 연산 적용
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel);
        morphologyEx(green_mask, green_mask, MORPH_CLOSE, kernel);
        morphologyEx(blue_mask, blue_mask, MORPH_CLOSE, kernel);

        // 각 색상별 객체 검출 및 표시
        detect_and_draw_labels(img, red_mask, Scalar(0, 0, 255));   // 빨간색 객체
        detect_and_draw_labels(img, green_mask, Scalar(0, 255, 0)); // 초록색 객체
        detect_and_draw_labels(img, blue_mask, Scalar(255, 0, 0));  // 파란색 객체

        // 결과 출력
        imshow("Labeled Balls", img);
        waitKey(1);
    }

    // 레이블링 후 각 객체의 중심에 도형 그리기
    void detect_and_draw_labels(Mat &img, Mat &mask, Scalar color) {
        Mat labels, stats, centroids;
        int num_objects = connectedComponentsWithStats(mask, labels, stats, centroids);

        for (int i = 1; i < num_objects; ++i) { // 0은 배경이므로 1부터 시작
            int x = stats.at<int>(i, CC_STAT_LEFT);
            int y = stats.at<int>(i, CC_STAT_TOP);
            int width = stats.at<int>(i, CC_STAT_WIDTH);
            int height = stats.at<int>(i, CC_STAT_HEIGHT);
            int area = stats.at<int>(i, CC_STAT_AREA);
            
            Point center(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
            if (area > 100) { // 작은 노이즈는 무시
                rectangle(img, Rect(x, y, width, height), color, 2); // 바운딩 박스 그리기
                circle(img, center, 5, color, -1); // 객체 중심에 원 그리기
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
