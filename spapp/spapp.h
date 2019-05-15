//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_APP_H__
#define __SP_APP_H__

// data
#include "spapp/spdata/spfile.h"
#include "spapp/spdata/spxml.h"
#include "spapp/spdata/spply.h"
#include "spapp/spdata/spstl.h"
#include "spapp/spdata/spbmp.h"
#include "spapp/spdata/spsvg.h"
#include "spapp/spdata/spmodel.h"
#include "spapp/spdata/spvoxel.h"
#include "spapp/spdata/spdataset.h"

// algorithm
#include "spapp/spalgo/spkdtree.h"
#include "spapp/spalgo/spgraphcut.h"
#include "spapp/spalgo/spcluster.h"
#include "spapp/spalgo/sprforest.h"
#include "spapp/spalgo/spkfilter.h"
#include "spapp/spalgo/spbprop.h"

// image
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spimg/spbin.h"
#include "spapp/spimg/splabel.h"
#include "spapp/spimg/sprender.h"

#include "spapp/spimgex/spfeature.h"
#include "spapp/spimgex/spcfblob.h"
#include "spapp/spimgex/spgfilter.h"
#include "spapp/spimgex/spcontour.h"
#include "spapp/spimgex/spcorner.h"
#include "spapp/spimgex/spoptflow.h"
#include "spapp/spimgex/sppmatch.h"
#include "spapp/spimgex/spsift.h"
#include "spapp/spimgex/spslic.h"
#include "spapp/spimgex/spcanny.h"
#include "spapp/spimgex/spfourier.h"
#include "spapp/spimgex/spstereo.h"

// geometry
#include "spapp/spgeom/spxmat.h"
#include "spapp/spgeom/spgeom.h"
#include "spapp/spgeom/spdepth.h"
#include "spapp/spgeom/spcalib.h"
#include "spapp/spgeom/spicp.h"
#include "spapp/spgeom/spvertex.h"

#include "spapp/spgeomex/spsfm.h"
#include "spapp/spgeomex/spslam.h"
#include "spapp/spgeomex/spfit.h"
#include "spapp/spgeomex/sptrackrf.h"
#include "spapp/spgeomex/spviewtrack.h"
#include "spapp/spgeomex/spdotmarker.h"
#include "spapp/spgeomex/spbitmarker.h"
#include "spapp/spgeomex/spdotpattern.h"
#include "spapp/spgeomex/spprojector.h"

#include "spapp/spgeomex/spkfusion.h"

// learning
#include "spapp/splearn/spneubase.h"
#include "spapp/splearn/spneulayer.h"
#include "spapp/splearn/spneumodel.h"

// util
#include "spapp/sptool.h"

#endif