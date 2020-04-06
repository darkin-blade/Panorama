#include "common.h"

#include "FeatureController.h"
#include "MultiImages.h"

class NISwGSP_Stitching {
public:
  NISwGSP_Stitching(MultiImages &multiImages);

  MultiImages *multiImages;
  Mat feature_match();
  Mat matching_match();

  void show_img(const char *window_name, Mat img);
};
