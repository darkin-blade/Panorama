#include "../common.h"

#include "../Util/Transform.h"

class Homographies {
public:
static void compute(
  const vector<Point2f> & _p_src,
  const vector<Point2f> & _p_dst,
  const vector<Point2f> & _src,
  vector<Point2f>       & _dst,
  vector<Mat>           & _homographies);

static void combine(
    const vector<Point2f> & _pts_1,// mdlt计算得到的网格点
    const vector<Point2f> & _pts_2,// 平移/旋转计算得到的网格点
    vector<Point2f>       & _result_pts);
};