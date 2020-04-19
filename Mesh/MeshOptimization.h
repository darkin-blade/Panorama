#if !defined(MeshOptimization_H)
#define MeshOptimization_H

#include "../Feature/MultiImages.h"
#include "../Util/Blending.h"

class MeshOptimization {
public:
  MeshOptimization(MultiImages & _multi_images);

  vector<vector<Point2f> > solution;// 最优解
  MultiImages *multi_images;
  
  double alignment_weight;
  double local_similarity_weight;
  double global_similarity_weight_beta, global_similarity_weight_gamma;
  
  pair<int, int> alignment_equation;
  pair<int, int> local_similarity_equation;
  pair<int, int> global_similarity_equation;

  void reserveData(vector<Triplet<double> > & _triplets,
                   vector<pair<int, double> > & _b_vector,
                   const int _start_index);// TODO
  void prepareAlignmentTerm(vector<Triplet<double> > & _triplets);
  void prepareSimilarityTerm(vector<Triplet<double> > & _triplets,
                             vector<pair<int, double> > & _b_vector);
  void getImageMeshPoints(vector<Triplet<double> > & _triplets,
                          const vector<pair<int, double> > & _b_vector);

  int getEdgesCount();// TODO
  int getVerticesCount();// mesh point数目
  int getEdgeNeighborVerticesCount();// TODO
  int getAlignmentTermEquationsCount();// TODO
};

#endif