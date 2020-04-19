#include "MeshOptimization.h"

MeshOptimization::MeshOptimization(MultiImages & _multi_images) {
  multi_images = & _multi_images;

  alignment_weight = 0;
  local_similarity_weight = 0;
  global_similarity_weight_beta = global_similarity_weight_gamma = 0;

  alignment_equation = make_pair(0, 0);
  local_similarity_equation = make_pair(0, 0);
  global_similarity_equation = make_pair(0, 0);
}

int MeshOptimization::getEdgesCount() {
  int result = 0;
  for (int i = 0; i < multi_images->img_num; i ++) {
    result += multi_images->imgs[i]->getEdges().size();
  }
  return result;
}

int MeshOptimization::getEdgeNeighborVerticesCount() {
  int result = 0;
  for (int i = 0; i < multi_images->img_num; i ++) {
    const vector<Edge> & edges = multi_images->imgs[i]->getEdges();
    const vector<vector<int> > & v_neighbors = multi_images->imgs[i]->getVertexStructures();
    for (int j = 0; j < edges.size(); j ++) {
      for (int e = 0; e < EDGE_VERTEX_SIZE; e ++) {
        result += v_neighbors[edges[j].indices[e]].size();// TODO Indices
      }
    }
    result -= edges.size();
  }
  return result;
}

void MeshOptimization::reserveData(vector<Triplet<double> > & _triplets,
    vector<pair<int, double> > & _b_vector,
    const int _start_index) {
  int equation = _start_index;
  const bool alignment_term = alignment_weight;
  const bool local_similarity_term = local_similarity_weight;
  const bool global_similarity_term = (global_similarity_weight_beta || global_similarity_weight_gamma);
  int edge_count = (local_similarity_term || global_similarity_term) ? getEdgesCount() : 0;
  int similarity_equation_count = (edge_count) ? edge_count * DIMENSION_2D : 0;
  int edge_neighbor_vertices_count = (similarity_equation_count) ? getEdgeNeighborVerticesCount() : 0;

  alignment_equation.first = equation;
  alignment_equation.second = (alignment_term) ? getAlignmentTermEquationsCount() : 0;// 未出界匹配点对 * 2
  equation += alignment_equation.second;

  local_similarity_equation.first = equation;
  local_similarity_equation.second = (local_similarity_term) ? similarity_equation_count : 0;
  equation += local_similarity_equation.second;

  global_similarity_equation.first = equation;
  global_similarity_equation.second = (global_similarity_term) ? similarity_equation_count : 0;

  _triplets.reserve(alignment_equation.second * 8 +
      (local_similarity_term) * (edge_neighbor_vertices_count * 8 + edge_count * 4) +
      (global_similarity_term) * (edge_neighbor_vertices_count * 8) +
      _start_index);
  _b_vector.reserve((global_similarity_term) * edge_neighbor_vertices_count * 4 +
      _start_index);
}

void MeshOptimization::prepareAlignmentTerm(vector<Triplet<double> > & _triplets) {
  if (alignment_equation.second) {
    const int equation = alignment_equation.first;

    const vector<vector<InterpolateVertex> > & mesh_interpolate_vertex_of_matching_pts = multi_images->getInterpolateVerticesOfMatchingPoints();
    const vector<int> images_vertices_start_index = multi_images->getImagesVerticesStartIndex();

    int eq_count = 0;
    for (int i = 0; i < 1; i ++) {// TODO
      int m1 = 0, m2 = 1;// TODO
      const vector<vector<int> > polygons_indices_1 = multi_images->imgs[m1]->polygons_indices;
      const vector<vector<int> > polygons_indices_2 = multi_images->imgs[m2]->polygons_indices;

      for (int j = 0; j < multi_images->matching_pairs[m1][m2].size(); j ++) {
        const pair<int, int> D_Match = multi_images->matching_pairs[m1][m2][j];// TODO

        for (int dim = 0; dim < DIMENSION_2D; dim ++) {
          for (int k = 0; k < GRID_VERTEX_SIZE; k ++) {// m1
            _triplets.emplace_back(equation + eq_count + dim,
                images_vertices_start_index[m1] + dim + 
                DIMENSION_2D * (polygons_indices_1[mesh_interpolate_vertex_of_matching_pts[m1][D_Match.first].polygon][k]),// TODO
                alignment_weight * mesh_interpolate_vertex_of_matching_pts[m1][D_Match.first].weights[k]);// TODO
          }
          for (int k = 0; k < GRID_VERTEX_SIZE; k ++) {// m2
            _triplets.emplace_back(equation + eq_count + dim,
                images_vertices_start_index[m2] + dim + 
                DIMENSION_2D * (polygons_indices_2[mesh_interpolate_vertex_of_matching_pts[m2][D_Match.second].polygon][k]),// TODO
                -alignment_weight * mesh_interpolate_vertex_of_matching_pts[m2][D_Match.second].weights[k]);
          }
        }
        eq_count += DIMENSION_2D;
      }
    }
    assert(eq_count == alignment_equation.second);// 匹配点对的2倍
  }
}

void MeshOptimization::prepareSimilarityTerm(vector<Triplet<double> > & _triplets,
    vector<pair<int, double> > & _b_vector) {
  const bool local_similarity_term = local_similarity_equation.second;
  const bool global_similarity_term = global_similarity_equation.second;
  if (local_similarity_term || global_similarity_term) {
    const vector<int> images_vertices_start_index = multi_images->getImagesVerticesStartIndex();
    const vector<vector<double> > images_grid_space_matching_pts_weight = multi_images->getImagesGridSpaceMatchingPointsWeight(global_similarity_weight_gamma);
  } 
}

int MeshOptimization::getAlignmentTermEquationsCount() {
  int result = 0;
  int m1 = 0, m2 = 1;
  result += multi_images->matching_pairs[m1][m2].size();// TODO
  return result * DIMENSION_2D;
}