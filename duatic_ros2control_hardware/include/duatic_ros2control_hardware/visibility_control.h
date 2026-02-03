#ifndef DUATIC_ROS2CONTROL_HARDWARE__VISIBILITY_CONTROL_H_
#define DUATIC_ROS2CONTROL_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUATIC_ROS2CONTROL_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define DUATIC_ROS2CONTROL_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define DUATIC_ROS2CONTROL_HARDWARE_EXPORT __declspec(dllexport)
    #define DUATIC_ROS2CONTROL_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUATIC_ROS2CONTROL_HARDWARE_BUILDING_LIBRARY
    #define DUATIC_ROS2CONTROL_HARDWARE_PUBLIC DUATIC_ROS2CONTROL_HARDWARE_EXPORT
  #else
    #define DUATIC_ROS2CONTROL_HARDWARE_PUBLIC DUATIC_ROS2CONTROL_HARDWARE_IMPORT
  #endif
  #define DUATIC_ROS2CONTROL_HARDWARE_PUBLIC_TYPE DUATIC_ROS2CONTROL_HARDWARE_PUBLIC
  #define DUATIC_ROS2CONTROL_HARDWARE_LOCAL
#else
  #define DUATIC_ROS2CONTROL_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define DUATIC_ROS2CONTROL_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define DUATIC_ROS2CONTROL_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define DUATIC_ROS2CONTROL_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUATIC_ROS2CONTROL_HARDWARE_PUBLIC
    #define DUATIC_ROS2CONTROL_HARDWARE_LOCAL
  #endif
  #define DUATIC_ROS2CONTROL_HARDWARE_PUBLIC_TYPE
#endif

#endif  // DUATIC_ROS2CONTROL_HARDWARE__VISIBILITY_CONTROL_H_
