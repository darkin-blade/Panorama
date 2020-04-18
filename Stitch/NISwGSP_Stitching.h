#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/MultiImages.h"

class NISwGSP_Stitching {
public:
  NISwGSP_Stitching(MultiImages &multiImages);

  MultiImages *multiImages;
  Mat feature_match();
  Mat matching_match();
  Mat texture_mapping();

  void show_img(const char *window_name, Mat img);
};
