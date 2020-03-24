#if !defined(COMMON_H)
#include "common.h"
#endif

#include "MultiImages.h"

using namespace std;

using namespace cv;
using namespace cv::detail;
using namespace cv::xfeatures2d;

class NISwGSP_Stitching {
public:
  NISwGSP_Stitching(MultiImages &multiImages);

  MultiImages *multiImages;
  Mat draw_matches();
  Mat get_matching_pts();
  vector<pair<int, int> > getFeaturePairs(const pair<int, int> &_match_pair);

  void sift_1(Mat img1, Mat img2);
  void sift_2(Mat img1, Mat img2);
  void show_img(const char *window_name, Mat img);
};
