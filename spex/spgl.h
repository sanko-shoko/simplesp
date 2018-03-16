//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SPGL_H__
#define __SPGL_H__

#ifndef SP_USE_GLEW
#define SP_USE_GLEW 0
#endif

#ifndef SP_USE_IMGUI
#define SP_USE_IMGUI 0
#endif

#if defined(_WIN32) && SP_USE_GLEW
#include "spex/spglew.h"
#endif

#if SP_USE_IMGUI
#include "spex/spimgui.h"
#endif

#include "spex/spglutil.h"
#include "spex/spglwin.h"

#endif