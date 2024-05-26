#ifndef SCREEN_PUBLISHER_RVIZ_PLUGIN_HPP
#define SCREEN_PUBLISHER_RVIZ_PLUGIN_HPP

#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <QImage>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QScreen>
#include <QGuiApplication>
#endif
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace rviz_plugins
{
  class ScreenPublisher: public rviz::Display
  {
    Q_OBJECT
  public:
    typedef std::shared_ptr<ScreenPublisher> Ptr;
    ScreenPublisher();
    virtual ~ScreenPublisher();
  protected:
    virtual void onInitialize();
    virtual void onEnable();
    virtual void update(float wall_dt, float ros_dt);
    virtual void startPublisher();
    virtual void stopPublisher();

    rviz::StringProperty* topic_name_property_;
    rviz::BoolProperty* publish_screen_property_;
    std::string topic_name_;
    bool publishing_;
    int frame_counter_;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
  protected Q_SLOTS:
    void updateTopicName();
    void updatePublishScreen();
  private:
    
  };
}

#endif // SCREEN_PUBLISHER_RVIZ_PLUGIN_HPP