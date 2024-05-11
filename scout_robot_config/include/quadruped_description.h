#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin(0.28882, 0.067155, 0.0515, 0.0, 0.0, 0.0);
base.lf.upper_leg.setOrigin(-0.01, 0.11015, -0.0, 0.0, 0.0, 0.0);
base.lf.lower_leg.setOrigin(-0.0077812, 0.0, -0.19985, 0.0, 0.0, 0.0);
     base.lf.foot.setOrigin(-0.011199171847237, 0.0, -0.209761807091393, 0.0, 0.0, 0.0);

      base.rf.hip.setOrigin(0.22257, -0.062845, 0.0515, 0.0, 0.0, 0.0);
base.rf.upper_leg.setOrigin(0.056252, -0.11015, -0.0, 0.0, 0.0, 0.0);
base.rf.lower_leg.setOrigin(0.0077812, 0.0, -0.19985, 0.0, 0.0, 0.0);
     base.rf.foot.setOrigin(-0.00498806894979309, 0.0, -0.210001324596927, 0.0, 0.0, 0.0);

      base.lh.hip.setOrigin(-0.330780215437656, 0.0671546202239636, 0.0514999999999987, 0.0, 0.0, 0.0);
base.lh.upper_leg.setOrigin(0.01, 0.11015, -0.0, 0.0, 0.0, 0.0);
base.lh.lower_leg.setOrigin(-0.0077812, 0.0, -0.19985, 0.0, 0.0, 0.0);
     base.lh.foot.setOrigin(-0.0111991718472406, 0.0, -0.20976180709139, 0.0, 0.0, 0.0);

      base.rh.hip.setOrigin(-0.33078, -0.062845, 0.0515, 0.0, 0.0, 0.0);
base.rh.upper_leg.setOrigin(0.01, -0.11015, -0.0, 0.0, 0.0, 0.0);
base.rh.lower_leg.setOrigin(0.0077812, 0.0, -0.19985, 0.0, 0.0, 0.0);
     base.rh.foot.setOrigin(-0.00498806894979753, 0.0, -0.210001324596924, 0.0, 0.0, 0.0);
        }
    }
}
#endif