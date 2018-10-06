//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_APP_H__
#define __SP_APP_H__

// data
#include "spapp/spdata/spstr.h"
#include "spapp/spdata/spfile.h"
#include "spapp/spdata/spxml.h"
#include "spapp/spdata/spply.h"
#include "spapp/spdata/spbmp.h"
#include "spapp/spdata/spsvg.h"
#include "spapp/spdata/spmodel.h"
#include "spapp/spdata/spvoxel.h"
#include "spapp/spdata/spdataset.h"

// algorithm
#include "spapp/spalgo/spkdtree.h"
#include "spapp/spalgo/spgraphcut.h"
#include "spapp/spalgo/spcluster.h"
#include "spapp/spalgo/sprandomforest.h"
#include "spapp/spalgo/spkalmanfilter.h"
#include "spapp/spalgo/spbeliefpropagation.h"

// image
#include "spapp/spimg/spimage.h"
#include "spapp/spimg/spbinalize.h"
#include "spapp/spimg/splabeling.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spimg/sprender.h"

#include "spapp/spimgex/spfeature.h"
#include "spapp/spimgex/spguidedfilter.h"
#include "spapp/spimgex/spcontour.h"
#include "spapp/spimgex/spharris.h"
#include "spapp/spimgex/splucaskanade.h"
#include "spapp/spimgex/sppatchmatch.h"
#include "spapp/spimgex/spsift.h"
#include "spapp/spimgex/spslic.h"
#include "spapp/spimgex/spcanny.h"
#include "spapp/spimgex/spfourier.h"
#include "spapp/spimgex/spstereo.h"

// geometry
#include "spapp/spgeom/spgeometry.h"
#include "spapp/spgeom/spdepth.h"
#include "spapp/spgeom/spcalibration.h"
#include "spapp/spgeom/spicp.h"
#include "spapp/spgeom/spvertex.h"

#include "spapp/spgeomex/spsfm.h"
#include "spapp/spgeomex/sptrack.h"
#include "spapp/spgeomex/sptrackrf.h"
#include "spapp/spgeomex/spdotmarker.h"
#include "spapp/spgeomex/spbitmarker.h"
#include "spapp/spgeomex/spdotpattern.h"
#include "spapp/spgeomex/spprojector.h"

#include "spapp/spgeomex/spkinectfusion.h"

// learning
#include "spapp/splearn/spneubase.h"
#include "spapp/splearn/spneulayer.h"
#include "spapp/splearn/spneumodel.h"

// util
#include "spapp/spthread.h"

#endif