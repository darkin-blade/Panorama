#if !defined(MeshOptimization_H)
#define MeshOptimization_H

#include "../Feature/MultiImages.h"
#include "../Util/Blending.h"

class MeshOptimization {
public:
  MeshOptimization(MultiImages & _multi_images);

  vector<vector<Point2f> > solution;// 最优解
  MultiImages *multiImages;
  
  double alignment_weight;
  double local_similarity_weight;
  double global_similarity_weight_beta, global_similarity_weight_gamma;
  
  pair<int, int> alignment_equation;
  pair<int, int> local_similarity_equation;
  pair<int, int> global_similarity_equation;

  int getEdgesCount();
  int getEdgeNeighborVerticesCount();
  void reserveData(vector<Triplet<double> > & _triplets,
                    vector<pair<int, double> > & _b_vector,
                    const int _start_index);
};

#endif