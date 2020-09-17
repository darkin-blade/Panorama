#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/MultiImages.h"
#include "../Mesh/MeshOptimization.h"

class NISwGSP_Stitching : public MeshOptimization {
public:
  NISwGSP_Stitching(MultiImages & _multi_images);

  Mat feature_match();
  Mat matching_match();
  void get_solution();
  Mat texture_mapping();
};
