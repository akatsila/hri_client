#include <ros/ros.h>

// libVISCA low-level C library
#include <visca/libvisca.h>  

namespace visca_interface {
  
  struct PanTilt {
    PanTilt(int16_t _pan=0, int16_t _tilt=0)
     : pan(_pan)
     , tilt(_tilt)
    {}
    int16_t pan;
    int16_t tilt;
  };
  struct PanTiltSpeed {
    PanTiltSpeed(int16_t _pan_speed=0, int16_t _tilt_speed=0)
     : pan_speed(_pan_speed)
     , tilt_speed(_tilt_speed)
    {}
    int16_t pan_speed;
    int16_t tilt_speed;
  };

  class ViscaInterface {

    public:
      ViscaInterface(std::string port);
      ~ViscaInterface();

      ViscaInterface(const ViscaInterface&) = delete;
      void operator=(const ViscaInterface&) = delete;

      bool setPanTilt(const PanTilt& pan_tilt, const PanTiltSpeed& pan_tilt_speed);
      PanTilt getPanTilt();

    private:
      bool openInterface(std::string ttydev);
      void closeInterface();

      // per needs of VISCA library
      VISCAInterface_t iface_;
      VISCACamera_t camera_;
  };

} //namespace visca_interface
