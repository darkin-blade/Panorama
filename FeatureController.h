#if !defined(COMMON_H)
#include "common.h"
#endif

using namespace std;

using namespace cv;

class FeatureController {
public:
  double getDistance(
      const Mat & _descriptor1,
      const Mat & _descriptor2,
      const double _threshold);
  void detect(
      const Mat & _grey_img,// TODO
      vector<Point2f> & _feature_points,
      vector<Mat> & _feature_descriptors);
}
