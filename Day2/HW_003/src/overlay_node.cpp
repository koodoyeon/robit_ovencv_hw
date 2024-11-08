#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std::chrono_literals;

class WatermarkOverlayNode : public rclcpp::Node {
public:
    WatermarkOverlayNode() : Node("watermark_overlay_node") {
        timer_ = this->create_wall_timer(1000ms, std::bind(&WatermarkOverlayNode::overlayImages, this));
    }

private:
    void overlayImages() {
        std::string background_path = "/home/dy/colcon_ws/src/watermark_overlay/resources/background.png";
        std::string logo_path = "/home/dy/colcon_ws/src/watermark_overlay/resources/rogo.png";

        // 이미지 불러오기
        Mat background = imread(background_path);
        Mat logo = imread(logo_path, IMREAD_UNCHANGED); // 알파 채널 포함

        if (background.empty() || logo.empty()) {
            RCLCPP_ERROR(this->get_logger(), "이미지를 불러올 수 없습니다.");
            return;
        }

        // 로고 이미지를 배경 이미지 위에 위치시키기 (여기서는 왼쪽 상단을 예시로 함)
        int x_offset = 10;
        int y_offset = 10;
        
        // 배경 이미지 크기와 맞지 않으면 알맞게 조정 (원본 비율 유지)
        if (logo.cols > background.cols || logo.rows > background.rows) {
            float scale = std::min(static_cast<float>(background.cols) / logo.cols, 
                                   static_cast<float>(background.rows) / logo.rows);
            resize(logo, logo, Size(), scale, scale, INTER_AREA);
        }

        // 알파 채널 분리
        std::vector<Mat> channels;
        split(logo, channels);

        Mat bgr_logo, alpha_channel;
        if (channels.size() == 4) {
            // BGR과 알파 채널 분리
            merge(std::vector<Mat>{channels[0], channels[1], channels[2]}, bgr_logo);
            alpha_channel = channels[3];

            // 알파 채널을 50% 투명도로 설정 (값을 0.5배)
            alpha_channel *= 0.5;
        } else {
            bgr_logo = logo;
            alpha_channel = Mat::ones(logo.size(), CV_8UC1) * 127; // 알파 채널이 없는 경우 50% 투명도 설정
        }

        // ROI 설정 및 위치 지정
        Mat roi = background(Rect(x_offset, y_offset, logo.cols, logo.rows));

        // 알파 채널을 사용하여 투명도 적용
        Mat alpha_f, alpha_inv;
        alpha_channel.convertTo(alpha_f, CV_32FC1, 1.0 / 255);  // 0~1 범위로 변환
        alpha_inv = 1.0 - alpha_f;

        std::vector<Mat> bgr_logo_channels;
        split(bgr_logo, bgr_logo_channels);

        std::vector<Mat> bgr_roi_channels;
        split(roi, bgr_roi_channels);

        for (int i = 0; i < 3; i++) {
            bgr_logo_channels[i].convertTo(bgr_logo_channels[i], CV_32FC1);
            bgr_roi_channels[i].convertTo(bgr_roi_channels[i], CV_32FC1);
            bgr_roi_channels[i] = bgr_roi_channels[i].mul(alpha_inv) + bgr_logo_channels[i].mul(alpha_f);
            bgr_roi_channels[i].convertTo(bgr_roi_channels[i], CV_8UC1);
        }

        merge(bgr_roi_channels, roi);

        // 결과 이미지 출력
        imshow("Watermarked Image", background);
        waitKey(1);
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WatermarkOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
