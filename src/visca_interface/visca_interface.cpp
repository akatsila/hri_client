#include <stdexcept>
#include <ros/ros.h>

#include <visca_interface/visca_interface.h>

namespace visca_interface {

  ViscaInterface::ViscaInterface(std::string port)
  {
    if (!openInterface(port)) {
      throw std::runtime_error("ViscaInterface constructor fail");
    }
  }

  ViscaInterface::~ViscaInterface()
  {
    closeInterface();
  }

  bool ViscaInterface::setPanTilt(const visca_interface::PanTilt& pan_tilt, const visca_interface::PanTiltSpeed& pan_tilt_speed)
  {
    if (VISCA_set_pantilt_absolute_position(&iface_, &camera_, pan_tilt_speed.pan_speed, pan_tilt_speed.tilt_speed, pan_tilt.pan, pan_tilt.tilt) != VISCA_SUCCESS){
      ROS_WARN_STREAM("visca: set_pantilt_absolute_position failed");
      return false;
    }

    return true;
  }

  visca_interface::PanTilt ViscaInterface::getPanTilt()
  {
    visca_interface::PanTilt res;
    VISCA_get_pantilt_position(&iface_, &camera_, &res.pan, &res.tilt);
  }

  bool ViscaInterface::openInterface(std::string ttydev)
  {
    if (VISCA_open_serial(&iface_, ttydev.c_str())!=VISCA_SUCCESS) {
      ROS_ERROR_STREAM("visca: unable to open serial device " << ttydev);
      return false;
    }
  
    iface_.broadcast=0;
    int camera_num = 0;
    VISCA_set_address(&iface_, &camera_num);
    if(VISCA_set_address(&iface_, &camera_num)!=VISCA_SUCCESS) {
      ROS_ERROR_STREAM("visca: unable to set address");
      VISCA_close_serial(&iface_);
      return false;
    }
  
    camera_.address=1; 
  
    if(VISCA_clear(&iface_, &camera_)!=VISCA_SUCCESS) {
      ROS_ERROR_STREAM("visca: unable to clear interface");
      VISCA_close_serial(&iface_);
      return false;
    }
    if(VISCA_get_camera_info(&iface_, &camera_)!=VISCA_SUCCESS) {
      ROS_ERROR_STREAM("visca: unable to get camera infos");
      VISCA_close_serial(&iface_);
      return false;
    }
  
    return true;
  }

  void ViscaInterface::closeInterface()
  {
    // read the rest of the data: (should be empty)
    unsigned char packet[3000];
    uint32_t buffer_size = 3000;
  
    VISCA_usleep(2000);
  
    if (VISCA_unread_bytes(&iface_, packet, &buffer_size)!=VISCA_SUCCESS) {
      uint32_t i;
      ROS_ERROR_STREAM("ERROR: " << buffer_size << " bytes not processed");
      #if DEBUG
      for (i=0; i<buffer_size; ++i) {
        fprintf(stderr,"%2x ",packet[i]);
      }
      fprintf(stderr,"\n");
      #endif
    }
  
    VISCA_close_serial(&iface_);
  }

} //namespace visca_interface