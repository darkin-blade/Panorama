#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/MultiImages.h"
#include "../Mesh/MeshOptimization.h"

class NISwGSP_Stitching : public MeshOptimization {
public:
  NISwGSP_Stitching(MultiImages & _multi_images);

  void featureMatch();
  void matchingMatch();
  Mat apapResult();
  void getMesh();
  Mat textureMapping();

  // DEBUG 用
  Mat drawFeatureMatch();
  Mat drawMatchingMatch();
};
