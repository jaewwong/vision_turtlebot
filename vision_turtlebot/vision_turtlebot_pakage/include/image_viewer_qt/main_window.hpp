/**
 * @file /include/image_viewer_qt/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef image_viewer_qt_MAIN_WINDOW_H
#define image_viewer_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <thread>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace image_viewer_qt
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();
  cv::Mat img_HSV;
  cv::Mat img_HSV2;
  cv::Mat img_HSV3;
  cv::Mat img_HSV4;
  cv::Mat region_of_interest_white(cv::Mat img_edges, cv::Point* points);
  cv::Mat region_of_interest_yellow(cv::Mat img_edges, cv::Point* points);
  cv::Mat img_masked;
  cv::Mat img_mask;
  cv::Mat img_masked2;
  cv::Mat img_mask2;
  cv::Mat line_img;
  cv::Mat line_img2;
  cv::Mat line_img3;
  cv::Mat line_img4;
  cv::Vec4i L;
  int flag_line_remove_yellow = 0;
  int flag_line_remove_white = 0;
  int count = 0;
  int white_start[4] = {
    0,
  };
  int white_edge[4] = {
    0,
  };
    int yellow_start[4] = {
    0,
  };
  int yellow_edge[4] = {
    0,
  };
  int hap_yellow_start = 0;
  int hap_yellow_edge = 0;

  int hap_white_start = 0;

  int hap_white_edge = 0;
  int flag = 0; 

public Q_SLOTS:
  void update();
  void on_go_clicked();
  void on_back_clicked();
  void on_right_clicked();
  void on_left_clicked();
  void on_stop_clicked();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace image_viewer_qt

#endif  // image_viewer_qt_MAIN_WINDOW_H
