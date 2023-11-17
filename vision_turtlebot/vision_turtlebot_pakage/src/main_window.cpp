/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/image_viewer_qt/main_window.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <QPixmap>
#include <QImage>
int flag = 0;

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace cv;
namespace image_viewer_qt
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  QObject::connect(&qnode, SIGNAL(Cam_SIGNAL()), this, SLOT(update()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow()
{
}
void MainWindow::update()
{
  int flag_line_remove_yellow = 0;
  int flag_line_remove_white = 0;
  cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
  Mat clone_mat = qnode.cam_img->clone();
  Mat clone_mat2 = qnode.cam_img->clone();

  cv::resize(clone_mat, clone_mat, cv::Size(432, 230));
  cv::resize(clone_mat2, clone_mat2, cv::Size(432, 230));

  cv::GaussianBlur(clone_mat2, clone_mat2, cv::Size(), 1);
  // cv::cvtColor(clone_mat2,img_HSV,cv::COLOR_BGR2HSV);

  cv::inRange(
      clone_mat2,
      cv::Scalar(ui.horizontalSlider->value(), ui.horizontalSlider_2->value(), ui.horizontalSlider_3->value()),
      cv::Scalar(ui.horizontalSlider_4->value(), ui.horizontalSlider_5->value(), ui.horizontalSlider_6->value()),
      img_HSV);
  cv::inRange(clone_mat2, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), img_HSV2);
  cv::inRange(clone_mat2, cv::Scalar(0, 190, 118), cv::Scalar(37, 234, 250), img_HSV3);

  cv::dilate(img_HSV, img_HSV, mask, cv::Point(-1, -1), 7);
  cv::dilate(img_HSV2, img_HSV2, mask, cv::Point(-1, -1), 4);
  cv::dilate(img_HSV3, img_HSV3, mask, cv::Point(-1, -1), 4);

  // white line
  cv::Canny(img_HSV2, line_img, 10, 300, 5);

  cv::Point points_white[4];

  points_white[0] = cv::Point(0, 220);
  points_white[1] = cv::Point(0, 0);
  points_white[2] = cv::Point(180, 0);
  points_white[3] = cv::Point(180, 220);
  line_img = region_of_interest_white(line_img, points_white);
  std::vector<cv::Vec4i> white_line;
  cv::HoughLinesP(line_img, white_line, 1, CV_PI / 180, 50, 30, 60);
  for (size_t i = 0; i < white_line.size(); i++)
  {
    Vec4i L = white_line[i];

    cv::line(clone_mat, cv::Point(L[0], L[1]), cv::Point(L[2], L[3]), cv::Scalar(255, 0, 0), 3, 8);
    // std::cout << "L[0]" << L[0] << std::endl;
    // std::cout << "L[1]" << L[1] << std::endl;
    // std::cout << "L[2]" << L[2] << std::endl;
    // std::cout << "L[3]" << L[3] << std::endl;
    if (i == 0)
    {
      white_start[0] = L[0];
      white_edge[0] = L[2];
    }
    else if (i == 1)
    {
      white_start[1] = L[0];
      white_edge[1] = L[2];
    }
  }

  hap_white_start = white_start[0] + white_start[1];
  hap_white_edge = white_edge[0] + white_edge[1];

  ui.white_start->setText(QString::number(hap_white_start));
  ui.white_edge->setText(QString::number(hap_white_edge));

  //  std::cout<<"flag"<<flag_line_remove<<std::endl;

  // yellow line
  cv::Canny(img_HSV3, line_img2, 10, 300, 5);

  cv::Point points_yellow[4];

  points_yellow[0] = cv::Point(250, 220);
  points_yellow[1] = cv::Point(250, 0);
  points_yellow[2] = cv::Point(430, 0);
  points_yellow[3] = cv::Point(430, 220);
  line_img2 = region_of_interest_yellow(line_img2, points_yellow);
  std::vector<cv::Vec4i> yellow_line;
  cv::HoughLinesP(line_img2, yellow_line, 1, CV_PI / 180, 50, 30, 60);
  for (size_t i = 0; i < yellow_line.size(); i++)
  {
    cv::Vec4i L2 = yellow_line[i];

    cv::line(clone_mat, cv::Point(L2[0], L2[1]), cv::Point(L2[2], L2[3]), cv::Scalar(0, 0, 255), 3, 8);

    if (i == 0)
    {
      yellow_start[0] = L2[0];
      yellow_edge[0] = L2[2];
    }
    else if (i == 1)
    {
      yellow_start[1] = L2[0];
      yellow_edge[1] = L2[2];
    }
  }
  hap_yellow_edge = -(yellow_start[0] - 410) - (yellow_start[1] - 410);
  hap_yellow_start = -(yellow_edge[0] - 450) - (yellow_edge[1] - 450);

  ui.yellow_start->setText(QString::number(hap_yellow_start));
  ui.yellow_edge->setText(QString::number(hap_yellow_edge));

  if (hap_yellow_edge > hap_white_edge)
  {
    if (hap_yellow_start > 150 && hap_yellow_start < 100)
    {
      qnode.direction = 0.3;
      qnode.publisher();
    }
    else if (hap_yellow_start < 200 && hap_yellow_start > 150)
    {
      qnode.direction = 0.4;
      qnode.publisher();
    }
    else if (hap_yellow_start < 230 && hap_yellow_start > 200)
    {
      qnode.direction = 0.5;
      qnode.publisher();
    }
    else if (hap_yellow_start < 316 && hap_yellow_start > 200)
    {
      qnode.direction = 0.8;
      qnode.publisher();
    }
    else
    {
      qnode.direction = 0;
      qnode.publisher();
    }
  }

  else if (hap_yellow_edge < hap_white_edge)
  {
      if (hap_yellow_edge > 95 && hap_yellow_edge < 118)
    {
      qnode.direction = -0.6;
      qnode.publisher();
    }
    else if (hap_yellow_edge < 138 && hap_yellow_edge > 118)
    {
      qnode.direction = -0.5;
      qnode.publisher();
    }
    else if (hap_yellow_edge < 158 && hap_yellow_edge > 138)
    {
      qnode.direction = -0.4;
      qnode.publisher();
    }
    else if (hap_yellow_edge < 178 && hap_yellow_edge > 158)
    {
      qnode.direction =-0.3;
      qnode.publisher();
    }
    else
    {
      qnode.direction = 0;
      qnode.publisher();
    }
  }
  else
  {
    // qnode.front = 0.3;
    // qnode.publisher();
  }

  // if (flag == 1)
  // {
  //   if (hap_white_edge > 50)
  //   {
  //     qnode.direction += 0.01;
  //     qnode.publisher();
  //   }
  //   else
  //   {
  //     qnode.direction = 0;
  //     qnode.publisher();
  //   }
  // }

  // if (flag_line_remove_white == 1 && flag_line_remove_yellow == 1)
  // {
  //   std::cout << "asd" << std::endl;
  //   qnode.front = 0.1;
  //   qnode.publisher();
  // }
  // else if (flag_line_remove_white == 0 && flag_line_remove_yellow == 0)
  // {
  //   qnode.direction = 0.02;
  //   qnode.publisher();
  // }
  // else if (flag_line_remove_white == 0 && flag_line_remove_yellow == 0)
  // {
  //   if (count == 0)
  //   {
  //     qnode.direction += 0.4;
  //     qnode.publisher();
  //     count++;
  //   }
  // }
  // else
  // {
  //   std::cout << "123" << std::endl;
  // }

  QImage img5((uchar*)img_mask2.data, img_mask2.cols, img_mask2.rows, QImage::Format_Indexed8);
  QPixmap pixmap5 = QPixmap::fromImage(img5);
  ui.label_5->setPixmap(pixmap5);

  QImage img((uchar*)clone_mat.data, clone_mat.cols, clone_mat.rows, QImage::Format_RGB888);
  img = img.rgbSwapped();
  QPixmap pixmap = QPixmap::fromImage(img);
  ui.label->setPixmap(pixmap);

  QImage img2((uchar*)img_HSV.data, img_HSV.cols, img_HSV.rows, QImage::Format_Indexed8);
  QPixmap pixmap2 = QPixmap::fromImage(img2);
  ui.label_2->setPixmap(pixmap2);

  QImage img3((uchar*)img_mask.data, img_mask.cols, img_mask.rows, QImage::Format_Indexed8);
  QPixmap pixmap3 = QPixmap::fromImage(img3);
  ui.label_3->setPixmap(pixmap3);

  QImage img4((uchar*)line_img.data, line_img.cols, line_img.rows, QImage::Format_Indexed8);
  QPixmap pixmap4 = QPixmap::fromImage(img4);
  ui.label_4->setPixmap(pixmap4);

  qnode.cam_img = NULL;
  qnode.isRecved = 0;
}  // namespace image_viewer_qt

cv::Mat MainWindow::region_of_interest_white(cv::Mat img_edges, cv::Point* points)
{
  img_mask = cv::Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);
  const cv::Point* ppt[1] = { points };
  int npt[] = { 4 };
  cv::fillPoly(img_mask, ppt, npt, 1, cv::Scalar(255, 255, 255), cv::LINE_8);
  cv::bitwise_and(img_edges, img_mask, img_masked);

  return img_masked;
}

cv::Mat MainWindow::region_of_interest_yellow(cv::Mat img_edges, cv::Point* points)
{
  img_mask2 = cv::Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);
  const cv::Point* ppt[1] = { points };
  int npt[] = { 4 };
  cv::fillPoly(img_mask2, ppt, npt, 1, cv::Scalar(255, 255, 255), cv::LINE_8);
  cv::bitwise_and(img_edges, img_mask2, img_masked2);

  return img_masked2;
}
void MainWindow::on_stop_clicked()
{
  qnode.front = 0;
  qnode.direction = 0;

  qnode.publisher();
}
void MainWindow::on_go_clicked()
{
  flag = 1;
  qnode.front += 0.1;

  qnode.publisher();
}
void MainWindow::on_back_clicked()
{
  qnode.front -= 0.1;

  qnode.publisher();
}
void MainWindow::on_right_clicked()
{
  qnode.direction -= 0.1;

  qnode.publisher();
}
void MainWindow::on_left_clicked()
{
  qnode.direction += 0.1;

  qnode.publisher();
}
/*****************************************************************************
** Functions
*****************************************************************************/

}  // namespace image_viewer_qt
