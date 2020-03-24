#if !defined(COMMON_H)
#include "common.h"
#endif

using namespace std;

using namespace cv;

const int SIFT_DESCRIPTOR_DIM = 128;

class FeatureController {
public:
  double getDistance(
      const vector<Mat> & _descriptor1,
      const vector<Mat> & _descriptor2,
      const double _threshold);
  void detect(
      const Mat & _grey_img,// TODO
      vector<Point2f> & _feature_points,
      vector<vector<Mat> > & _feature_descriptors);
};
