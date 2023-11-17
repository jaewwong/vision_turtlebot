/**
 * @file /include/image_viewer_qt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef image_viewer_qt_QNODE_HPP_
#define image_viewer_qt_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_viewer_qt
{
  /*****************************************************************************
  ** Class
  *****************************************************************************/

  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();
    void publisher(); 
    void image_callback(const sensor_msgs::ImageConstPtr& msg_img); 
    bool isRecved; 
     cv::Mat* cam_img = nullptr; 
    ros::Subscriber cam_sub;
      float front = 0; 
      float direction = 0; 
   
    

  Q_SIGNALS:
    void rosShutdown();
    void Cam_SIGNAL(); 

  private:
    int init_argc;
    char** init_argv;
     
    

  };

}  // namespace s

#endif /* image_viewer_qt_QNODE_HPP_ */
