#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/MultiImages.h"
#include "../Mesh/MeshOptimization.h"

class My_Stitching : public MeshOptimization {
public:
  My_Stitching(MultiImages & _multi_images);

  Mat getMyResult();
  Mat getNISResult();
  Mat getAPAPResult();

  // DEBUG 用
  void drawFeatureMatch();
  void drawMatchingMatch();
  void drawResult(Mat _result);
};
