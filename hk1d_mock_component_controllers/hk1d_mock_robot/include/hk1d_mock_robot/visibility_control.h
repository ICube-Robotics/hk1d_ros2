#ifndef HK1D_MOCK_ROBOT__VISIBILITY_CONTROL_H_
#define HK1D_MOCK_ROBOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HK1D_MOCK_ROBOT_EXPORT __attribute__ ((dllexport))
    #define HK1D_MOCK_ROBOT_IMPORT __attribute__ ((dllimport))
  #else
    #define HK1D_MOCK_ROBOT_EXPORT __declspec(dllexport)
    #define HK1D_MOCK_ROBOT_IMPORT __declspec(dllimport)
  #endif
  #ifdef HK1D_MOCK_ROBOT_BUILDING_LIBRARY
    #define HK1D_MOCK_ROBOT_PUBLIC HK1D_MOCK_ROBOT_EXPORT
  #else
    #define HK1D_MOCK_ROBOT_PUBLIC HK1D_MOCK_ROBOT_IMPORT
  #endif
  #define HK1D_MOCK_ROBOT_PUBLIC_TYPE HK1D_MOCK_ROBOT_PUBLIC
  #define HK1D_MOCK_ROBOT_LOCAL
#else
  #define HK1D_MOCK_ROBOT_EXPORT __attribute__ ((visibility("default")))
  #define HK1D_MOCK_ROBOT_IMPORT
  #if __GNUC__ >= 4
    #define HK1D_MOCK_ROBOT_PUBLIC __attribute__ ((visibility("default")))
    #define HK1D_MOCK_ROBOT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HK1D_MOCK_ROBOT_PUBLIC
    #define HK1D_MOCK_ROBOT_LOCAL
  #endif
  #define HK1D_MOCK_ROBOT_PUBLIC_TYPE
#endif

#endif  // HK1D_MOCK_ROBOT__VISIBILITY_CONTROL_H_
