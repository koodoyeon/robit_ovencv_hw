#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

class ContourExtractionNode : public rclcpp::Node {
public:
    ContourExtractionNode() : Node("contour_extraction_node") {
        processImage();
    }

private:
    void processImage() {
        // 패키지 경로를 동적으로 가져와 이미지 파일 경로를 설정합니다.
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("contour_extraction");
        std::string image_path = package_share_directory + "/contour_extraction/image.png";

        RCLCPP_INFO(this->get_logger(), "Attempting to load image from path: %s", image_path.c_str());

        cv::Mat src = cv::imread(image_path, cv::IMREAD_GRAYSCALE);  // 그레이스케일로 로드
        if (src.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the image at path: %s", image_path.c_str());
            return;
        }

        // 컬러 이미지로 변환하여 원본에 덧입힐 수 있도록 준비
        cv::Mat color_src;
        cv::cvtColor(src, color_src, cv::COLOR_GRAY2BGR);

        // 각 컨투어 모드별 색상 정의
        std::map<std::string, cv::Scalar> colors = {
            {"RETR_EXTERNAL", cv::Scalar(0, 0, 255)}, // 빨간색
            {"RETR_LIST", cv::Scalar(0, 255, 0)},     // 녹색
            {"RETR_CCOMP", cv::Scalar(255, 0, 0)},    // 파란색
            {"RETR_TREE", cv::Scalar(255, 0, 255)}    // 자홍색
        };

        // 각 모드별로 컨투어를 원본에 덧입히기
        drawAndDisplayContours(src, color_src, cv::RETR_EXTERNAL, "RETR_EXTERNAL", colors["RETR_EXTERNAL"]);
        drawAndDisplayContours(src, color_src, cv::RETR_LIST, "RETR_LIST", colors["RETR_LIST"]);
        drawAndDisplayContours(src, color_src, cv::RETR_CCOMP, "RETR_CCOMP", colors["RETR_CCOMP"]);
        drawAndDisplayContours(src, color_src, cv::RETR_TREE, "RETR_TREE", colors["RETR_TREE"]);

        // 모든 창을 한 번에 표시
        cv::waitKey(0);
    }

    void drawAndDisplayContours(const cv::Mat& src, cv::Mat& color_src, int mode, const std::string& mode_name, cv::Scalar color) {
        cv::Mat drawing = color_src.clone();  // 컬러 이미지에 덧입히기
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        // 컨투어 찾기 (그레이스케일 이미지를 입력으로 사용)
        cv::findContours(src, contours, hierarchy, mode, cv::CHAIN_APPROX_SIMPLE);

        // 컨투어 그리기
        for (size_t i = 0; i < contours.size(); i++) {
            cv::drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
        }

        // 결과 표시
        cv::imshow(mode_name, drawing);

        // 계층 정보 출력
        printHierarchy(mode_name, hierarchy);
    }

    void printHierarchy(const std::string &mode_name, const std::vector<cv::Vec4i> &hierarchy) {
        RCLCPP_INFO(this->get_logger(), "%s 계층 정보:", mode_name.c_str());
        for (size_t i = 0; i < hierarchy.size(); i++) {
            RCLCPP_INFO(this->get_logger(),
                        "컨투어 번호 %ld: 다음 = %d, 이전 = %d, 자식 = %d, 부모 = %d",
                        i, hierarchy[i][0], hierarchy[i][1], hierarchy[i][2], hierarchy[i][3]);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContourExtractionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
