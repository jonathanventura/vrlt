
#include <MultiView/multiview.h>

namespace vrlt
{
    int removeOutliers( Reconstruction *r, Node *node, double maxError = 16 );
    int removeOutliers( Reconstruction *r, Camera *camera, double maxError = 16 );
}
