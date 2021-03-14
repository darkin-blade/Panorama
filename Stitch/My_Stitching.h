#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/MultiImages.h"

class My_Stitching {
public:
  My_Stitching(MultiImages & _multi_images);

  MultiImages *multi_images;

  Mat getMyResult();

  // DEBUG 用
  void debug();
};
