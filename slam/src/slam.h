#ifndef MOBOTS_SLAM_H
#define MOBOTS_SLAM_H

/**
 * \class Slam
 * \brief ROS-Interface zum SLAM-System
 *
 * Implementiert einen Node und stellt somit die Schnittstelle der SLAM-Systems zum ROS-Framework dar.
 */
class Slam
{
public:
  /**
   * @brief Constructor
   * @param argc Pass this from main-function call
   * @param argv Pass this from main-function call
   */
  Slam(int& argc, char** argv);

  /**
   * \brief Enable this display
   * @param force If false, does not re-enable if this display is already enabled.  If true, it does.
   */
  void enable( bool force = false );

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
 
};

#endif
