#include "../common.h"

#include "../Util/Transform.h"

class Homographies {
public:
    static void compute(
        const vector<Point2f> & _p_src,
        const vector<Point2f> & _p_dst,
        const vector<Point2f> & _src,
        vector<Point2f>       & _dst,
        vector<Mat>          & _homographies);
};