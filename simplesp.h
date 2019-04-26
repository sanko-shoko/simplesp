//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SIMPLESP_H__
#define __SIMPLESP_H__


#ifdef WIN32
#pragma warning(disable:4996)
#pragma warning(disable:4819)
#pragma warning(disable:4101)

#pragma warning(push)
#pragma warning(disable:4244)
#pragma warning(disable:4305)
#pragma warning(disable:4838)
#endif


#ifndef SP_ROOT_DIR
#define SP_ROOT_DIR "../../../.."
#endif

#ifndef SP_DATA_DIR
#define SP_DATA_DIR "../../../../data"
#endif


#include "spcore/spcore.h"
#include "spapp/spapp.h"

#ifdef WIN32
#pragma warning(pop)
#endif

#endif