#include "MeshOptimization.h"

MeshOptimization::MeshOptimization(MultiImages & _multi_images) {
  multiImages = & _multi_images;

  alignment_weight = 0;
  local_similarity_weight = 0;
  global_similarity_weight_beta = global_similarity_weight_gamma = 0;

  alignment_equation = make_pair(0, 0);
  local_similarity_equation = make_pair(0, 0);
  global_similarity_equation = make_pair(0, 0);
}

void MeshOptimization::reserveData(vector<Triplet<double> > & _triplets,
    vector<pair<int, double> > & _b_vector,
    const int _start_index) {
}
