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
    result += multi_images->imgs[i]->edges.size();
  }
  return result;
}

int MeshOptimization::getEdgeNeighborVerticesCount() {
  int result = 0;
  for (int i = 0; i < multi_images->img_num; i ++) {
    const vector<Edge> & edges = multi_images->imgs[i]->edges;
    const vector<vector<int> > & v_neighbors = multi_images->imgs[i]->vertex_structures;
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
  }
}

int MeshOptimization::getAlignmentTermEquationsCount() {
  int result = 0;
  vector<pair<int, int> > images_match_graph_pair_list;
  images_match_graph_pair_list.emplace_back(make_pair(0, 1));
  for (int i = 0; i < images_match_graph_pair_list.size(); i ++) {
    pair<int, int> match_pair = images_match_graph_pair_list[i];
    int m1 = match_pair.first;
    int m2 = match_pair.second;
    result += multi_images->matching_pairs[m1][m2].size();// TODO
  }
  return result * DIMENSION_2D;
}