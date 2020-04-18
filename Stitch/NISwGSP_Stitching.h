#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/MultiImages.h"
#include "../Mesh/MeshOptimization.h"
#include "../Stitch/APAP_Stitching.h"

class NISwGSP_Stitching : public MeshOptimization {
public:
  NISwGSP_Stitching(MultiImages & _multi_images);

  Mat feature_match();
  Mat matching_match();
  void get_solution();
  Mat texture_mapping();

  void show_img(const char *window_name, Mat img);
};
