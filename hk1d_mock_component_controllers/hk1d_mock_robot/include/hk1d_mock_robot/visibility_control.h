// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
/// \authors: Thibault Poignonec

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
