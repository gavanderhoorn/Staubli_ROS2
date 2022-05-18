/*
 *  Copyright (c) 2022 Ivo Dekker ACRO Diepenbeek KULeuven

 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 */


#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define URDF_EXPORT __attribute__ ((dllexport))
    #define URDF_IMPORT __attribute__ ((dllimport))
  #else
    #define URDF_EXPORT __declspec(dllexport)
    #define URDF_IMPORT __declspec(dllimport)
  #endif
  #ifdef URDF_BUILDING_LIBRARY
    #define URDF_PUBLIC URDF_EXPORT
  #else
    #define URDF_PUBLIC URDF_IMPORT
  #endif
  #define URDF_PUBLIC_TYPE URDF_PUBLIC
  #define URDF_LOCAL
#else
  #define URDF_EXPORT __attribute__ ((visibility("default")))
  #define URDF_IMPORT
  #if __GNUC__ >= 4
    #define URDF_PUBLIC __attribute__ ((visibility("default")))
    #define URDF_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define URDF_PUBLIC
    #define URDF_LOCAL
  #endif
  #define URDF_PUBLIC_TYPE
#endif
