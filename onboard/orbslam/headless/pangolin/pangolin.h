// SPDX-License-Identifier: GPL-3.0-or-later
// HEADLESS BUILD of ORB-SLAM3: a stand-in for <pangolin/pangolin.h>.
//
// ORB-SLAM3's core headers include Pangolin only to NAME a few types (in
// MapDrawer's interface, and OpenGL's byte in Map.h); the drawing itself lives in Viewer.cc and
// MapDrawer.cc, which the headless build replaces with headless_stubs.cc.
// The aircraft has no display, and Pangolin drags in OpenGL, GLEW and a
// window system the Pi does not need.
#pragma once
#include <string>

// Map.h keeps a thumbnail buffer typed with OpenGL's byte; nothing draws it.
typedef unsigned char GLubyte;
typedef float GLfloat;

namespace pangolin {
struct OpenGlMatrix {
    double m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    void SetIdentity() { for (int i = 0; i < 16; ++i) m[i] = (i % 5 == 0) ? 1.0 : 0.0; }
};
inline void BindToContext(const std::string&) {}
}  // namespace pangolin
