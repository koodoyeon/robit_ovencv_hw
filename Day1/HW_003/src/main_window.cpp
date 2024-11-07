#include "../include/hw_1/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  timer = new QTimer(this)
  connect(timer, &QTimer::timeout, this, &MainWindow::updateRobotArm);
  timer->start(16);

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::updateRobotArm()
{
    // QLabel 중앙에 origin 위치 설정
    int labelWidth = ui->imageDisplay->width();
    int labelHeight = ui->imageDisplay->height();
    cv::Point origin(labelWidth / 2, labelHeight / 2);

    // 각도 증가 속도와 길이 조절
    static double angle1 = 0.0;
    static double angle2 = 0.0;
    double length1 = 100.0;  // 길이 줄임
    double length2 = 70.0;   // 길이 줄임

    // 각도 증가값을 줄여 회전 속도 감소
    angle1 += 0.02;
    angle2 += 0.015;

    // OpenCV로 이미지 생성 및 로봇 팔 계산
    cv::Mat image = cv::Mat::zeros(labelHeight, labelWidth, CV_8UC3);  // QLabel 크기에 맞춰 이미지 생성
    cv::Point joint1(
        origin.x + length1 * cos(angle1),
        origin.y + length1 * sin(angle1)
    );
    cv::Point joint2(
        joint1.x + length2 * cos(angle1 + angle2),
        joint1.y + length2 * sin(angle1 + angle2)
    );

    // 로봇 팔 그리기 (흰색 선과 점)
    cv::Scalar whiteColor(255, 255, 255);
    cv::line(image, origin, joint1, whiteColor, 3); // 흰색 선
    cv::line(image, joint1, joint2, whiteColor, 3); // 흰색 선
    cv::circle(image, origin, 5, whiteColor, -1);   // 흰색 점
    cv::circle(image, joint1, 5, whiteColor, -1);   // 흰색 점
    cv::circle(image, joint2, 5, whiteColor, -1);   // 흰색 점

    // OpenCV 이미지를 QImage로 변환
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    QImage qImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);

    // QLabel에 표시
    ui->imageDisplay->setPixmap(QPixmap::fromImage(qImage));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

