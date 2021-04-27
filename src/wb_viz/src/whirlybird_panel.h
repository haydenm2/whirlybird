/**
 * \file whirlybird_panel.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Rviz panel plugin for setting whirlybird setpoints
 */

#ifndef WHIRLYBIRD_PANEL_H
#define WHIRLYBIRD_PANEL_H

#include <QLineEdit>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>
#include <QComboBox>

#include <std_msgs/Float32.h>
#include <wb_msgs/WbStates.h>
#include <wb_msgs/WbReferenceStates.h>
#include <wb_msgs/Command.h>

#include <vector>
#include <string>

#include <iostream>

#ifndef Q_MOC_RUN
  #include <ros/ros.h>
  #include <rviz/panel.h>
#endif

class QLineEdit;
class QSlider;
class QSpinBox;
class QCheckBox;
class QString;

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

namespace wb_viz
{

const std::string kStateTopic ="/wb_states";
const std::string kReferenceTopic ="/wb_reference_states";
const std::string kCommandTopic ="/wb_command";

enum class LayoutType { states, reference_states, command };

class WhirlybirdPanel : public rviz::Panel
{
Q_OBJECT
public:
  WhirlybirdPanel(QWidget* parent);

  // Load parameters from the .rviz config file.
  // These parameters are used to initialize the panel 
  // and ROS topic
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  /* These functions are called when the corresponding slider or
     spin box values change. 
  */

  // Controls which topic is being published.
  void setTopic();


protected Q_SLOTS:

  void sliderOneCallback(int value);
  void sliderTwoCallback(int value);
  void sliderThreeCallback(int value);

  void spinboxOneCallback(double value);
  void spinboxTwoCallback(double value);
  void spinboxThreeCallback(double value);


  void sendSetpoints();

  // Used to enable or disable user  input and 
  // the ROS topic
  void setEnabled(bool enabled);

  // Used to update the layout to reflect the change in selected topic
  // 
  void updateLayout(const QString& layout_type);



protected:

  void init();

  void updateWidgets();

  QSlider* slider_1_;  // Used for pitch, pitch_r, and command F
  QSlider* slider_2_;  // Used for yaw, yaw_r, and command tau
  QSlider* slider_3_;  // Used for roll 

  QDoubleSpinBox* spinbox_1_;  // Used for pitch, pitch_r, and command F
  QDoubleSpinBox* spinbox_2_; // Used for yaw, yaw_r, and command tau
  QDoubleSpinBox* spinbox_3_; // Used for roll

  QLabel* label_1_; // Used for pitch, pitch_r, and command F
  QLabel* label_2_; // Used for yaw, yaw_r, and command tau
  QLabel* label_3_; // Used for roll

  // labels that change when the topic is changed
  QLabel* pitch_label_;
  QLabel* yaw_label_;
  QLabel* roll_label_;

  QString topic_;  

  QCheckBox* enabled_check_box_;
  QComboBox* layout_type_;


  // layouts that change
  QVBoxLayout* layout_;
  QGridLayout* setpoints_layout_;
  QGroupBox* setpoints_group_box_;

  ros::Publisher publisher_;

  ros::NodeHandle nh_;

  wb_msgs::WbStates wb_msg_;
  wb_msgs::WbReferenceStates wb_r_msg_;
  wb_msgs::Command wb_cmd_msg_;

  // Whirlybird Pararmeters
  float d_;       // Length of the wb rod, m
  float km_;      // Force to PWM conversion at equilibrium


  float ratio_;

  bool enabled_;
  bool layout_set_ = false;

  // Whirlybird states in degrees
  int pitch_setpoint_deg_ = 0;
  int yaw_setpoint_deg_ = 0;
  int roll_setpoint_deg_ = 0;

  // Whirlybird reference states in degrees
  int pitch_setpoint_deg_r_ = 0;
  int yaw_setpoint_deg_r_ = 0;

  // Commands
  double force_ = 0;
  double torque_ = 0;
  
  // Max and min range for pitch, yaw, roll
  int pitch_max_deg_;
  int yaw_max_deg_;
  int roll_max_deg_;

  int force_max_;
  int torque_max_;


};

} // namespace whirlybird_description

#endif // WHIRLYBIRD_PANEL_H
