#include "../common.h"

#include "../Utils/Transform.h"

class APAP_Stitching {
public:
    static void apap_project(const vector<Point2f> & _p_src,
        const vector<Point2f> & _p_dst,
        const vector<Point2f> & _src,
        vector<Point2f>       & _dst,
        vector<Mat>          & _homographies);
};