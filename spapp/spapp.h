//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_APP_H__
#define __SP_APP_H__

// data
#include "spapp/spdata/spstr.h"
#include "spapp/spdata/spfile.h"
#include "spapp/spdata/spply.h"
#include "spapp/spdata/spbmp.h"
#include "spapp/spdata/spmodel.h"
#include "spapp/spdata/spdataset.h"

// algorithm
#include "spapp/spalgo/spkdtree.h"
#include "spapp/spalgo/spgraphcut.h"
#include "spapp/spalgo/spcluster.h"
#include "spapp/spalgo/sprandomforest.h"
#include "spapp/spalgo/spkalmanfilter.h"

// image
#include "spapp/spimg/spimage.h"
#include "spapp/spimg/spbinalization.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spimg/sprender.h"

#include "spapp/spimgex/spfeature.h"
#include "spapp/spimgex/spguidedfilter.h"
#include "spapp/spimgex/spharris.h"
#include "spapp/spimgex/splucaskanade.h"
#include "spapp/spimgex/sppatchmatch.h"
#include "spapp/spimgex/spsift.h"
#include "spapp/spimgex/spslic.h"
#include "spapp/spimgex/spcanny.h"
#include "spapp/spimgex/spfourier.h"

// geometry
#include "spapp/spgeom/spgeometry.h"
#include "spapp/spgeom/spdepth.h"
#include "spapp/spgeom/spcalibration.h"
#include "spapp/spgeom/spicp.h"

#include "spapp/spgeomex/spsfm.h"
#include "spapp/spgeomex/spfitting.h"
#include "spapp/spgeomex/spdotmarker.h"
#include "spapp/spgeomex/spbitmarker.h"
#include "spapp/spgeomex/spdotpattern.h"

#include "spapp/spgeomex/spkinectfusion.h"
//#include "spapp/spgeomex/sptrack3d.h"

// learning
#include "spapp/splearn/spneubase.h"
#include "spapp/splearn/spneulayer.h"
#include "spapp/splearn/spneumodel.h"

#endif