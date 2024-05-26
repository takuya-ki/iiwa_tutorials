#include "screen_publisher_rviz_plugin.hpp"

namespace rviz_plugins
{
  ScreenPublisher::ScreenPublisher():
    Display(), publishing_(false), it_(nh_)
  {
    publish_screen_property_ = new rviz::BoolProperty(
      "publish screen", false, "publish screen",
      this, SLOT(updatePublishScreen()));
    topic_name_property_ = new rviz::StringProperty(
      "topic name", "/rviz/rendered_image",
      "topic name", this, SLOT(updateTopicName()));
  }

  ScreenPublisher::~ScreenPublisher()
  {
    delete publish_screen_property_;
    delete topic_name_property_;
  }

  void ScreenPublisher::onInitialize()
  {
    updateTopicName();
    publish_screen_property_->setBool(false);
    context_->queueRender();
  }

  void ScreenPublisher::onEnable()
  {
    publish_screen_property_->setBool(false);
    context_->queueRender();
  }
  
  void ScreenPublisher::updateTopicName()
  {
    if (publishing_) {
      ROS_WARN("cannot change name while publishing");
      topic_name_property_->setStdString(topic_name_);
    }
    else {
      topic_name_ = topic_name_property_->getStdString();
    }
  }

  void ScreenPublisher::updatePublishScreen()
  {
    ROS_INFO("updatePublishScreen");
    if (publish_screen_property_->getBool()) {
      publishing_ = true;
      startPublisher();
    }
    else {
      publishing_ = false;
      stopPublisher();
    }
  }

  void ScreenPublisher::startPublisher()
  {
    ROS_INFO("start publisher");
    frame_counter_ = 0;
    image_pub_ = it_.advertise(topic_name_, 1);
  }
  
  void ScreenPublisher::stopPublisher()
  {
    ROS_INFO("stop publisher");
    frame_counter_ = 0;
    image_pub_.shutdown();
  }

  void ScreenPublisher::update(float wall_dt, float ros_dt)
  {
    if (publishing_) {
      rviz::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
      QPixmap screenshot
        = QGuiApplication::primaryScreen()->grabWindow(context_->getViewManager()->getRenderPanel()->winId());
#else
      QPixmap screenshot
        = QPixmap::grabWindow(context_->getViewManager()->getRenderPanel()->winId());
#endif
      QImage src = screenshot.toImage().convertToFormat(QImage::Format_RGB888);
      cv::Mat image(src.height(), src.width(), CV_8UC3, (uchar*)src.bits(), src.bytesPerLine());
      std_msgs::Header header;
      header.seq = frame_counter_;
      header.stamp = ros::Time::now();
      cv_bridge::CvImage image_msg;
      image_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
      image_pub_.publish(image_msg.toImageMsg());

      ++frame_counter_;
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ScreenPublisher, rviz::Display)
