#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

class EdgeDetectionNode : public rclcpp::Node {
public:
    EdgeDetectionNode() : Node("edge_detection") {
        processImage();
    }

private:
    void processImage() {
        // 이미지 경로 설정
        std::string image_path = "/home/dy/colcon_ws/src/line_detection_v2/include/line_detection_v2/image.jpg";

        // 파일 존재 확인
        std::ifstream file(image_path);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "이미지 파일이 경로에 존재하지 않습니다: %s", image_path.c_str());
            return;
        }

        // 이미지 로드
        Mat img = imread(image_path, IMREAD_COLOR);

        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "이미지를 불러오는 데 실패했습니다. 경로: %s", image_path.c_str());
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "이미지가 성공적으로 로드되었습니다.");
        }

        // Canny 엣지 검출
        Mat edges;
        Canny(img, edges, 50, 150);

        // Canny 엣지 검출 결과 표시
        imshow("Canny Edges", edges);

        // 허프 선 변환
        vector<Vec2f> lines;
        HoughLines(edges, lines, 1, CV_PI / 180, 150);

        // 선 그리기 및 기울기 출력
        for (size_t i = 0; i < lines.size(); i++) {
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * a));
            Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * a));
            line(img, pt1, pt2, Scalar(0, 255, 0), 2, LINE_AA);
            
            // 기울기 계산 및 출력
            double slope = b / a;
            RCLCPP_INFO(this->get_logger(), "기울기: %.2f", slope);
        }

        //결과 출력
        imshow("Detected Lines", img);
        waitKey(0);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdgeDetectionNode>();
    rclcpp::spin_some(node);  
    rclcpp::shutdown();
    return 0;
}
