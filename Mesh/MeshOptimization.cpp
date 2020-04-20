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

int MeshOptimization::getVerticesCount() {
  int result = 0;
  for (int i = 0; i < multi_images->img_num; i ++) {
    result += multi_images->imgs[i]->getMeshPoints().size();
  }
  return result * DIMENSION_2D;
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

int MeshOptimization::getAlignmentTermEquationsCount() {
  int result = 0;
  int m1 = 0, m2 = 1;
  result += multi_images->keypoints_pairs[m1][m2].size();// TODO
  result += multi_images->keypoints_pairs[m2][m1].size();// TODO
  return result * DIMENSION_2D;
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

    int m1 = 0, m2 = 1;
    const vector<vector<int> > polygons_indices_1 = multi_images->imgs[m1]->getPolygonsIndices();
    const vector<vector<int> > polygons_indices_2 = multi_images->imgs[m2]->getPolygonsIndices();

    int n1, n2;
    for (int i = 0; i < 2; i ++) {// TODO
      if (i == 0) {// TODO
        n1 = 0, n2 = 1;
      } else {
        n1 = 1, n2 = 0;
      }

      for (int j = 0; j < multi_images->keypoints_pairs[n1][n2].size(); j ++) {
        const pair<int, int> D_Match = multi_images->keypoints_pairs[n1][n2][j];// TODO

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
    const vector<SimilarityElements> images_similarity_elements = multi_images->getImagesSimilarityElements();
    int eq_count = 0, eq_count_rotation = 0;
    for (int i = 0; i < multi_images->img_num; i ++) {
      const vector<Edge> edges = multi_images->imgs[i]->getEdges();
      const vector<Point2f> mesh_points = multi_images->imgs[i]->getMeshPoints();
      const vector<vector<int> > v_neighbors = multi_images->imgs[i]->getVertexStructures();
      const vector<vector<int> > e_neighbors = multi_images->imgs[i]->getEdgeStructures();

      const double similarity[DIMENSION_2D] = {
        images_similarity_elements[i].scale * cos(images_similarity_elements[i].theta),
        images_similarity_elements[i].scale * sin(images_similarity_elements[i].theta)
      };

      for (int j = 0; j < edges.size(); j ++) {
        const int ind_e1 = edges[j].indices[0];
        const int ind_e2 = edges[j].indices[1];
        const Point2f src = multi_images->imgs[i]->getMeshPoints()[ind_e1];
        const Point2f dst = multi_images->imgs[i]->getMeshPoints()[ind_e2];
        set<int> point_ind_set;
        for (int e = 0; e < EDGE_VERTEX_SIZE; e ++) {
          for (int v = 0; v < v_neighbors[edges[j].indices[e]].size(); v ++) {
            int v_index = v_neighbors[edges[j].indices[e]][v];
            if (v_index != ind_e1) {
              point_ind_set.insert(v_index);
            }
          }
        }
        Mat Et, E_Main(DIMENSION_2D, DIMENSION_2D, CV_64FC1), E((int)point_ind_set.size() * DIMENSION_2D, DIMENSION_2D, CV_64FC1);
        set<int>::const_iterator it = point_ind_set.begin();
        for (int p = 0; it != point_ind_set.end(); p ++, it ++) {
          Point2f e = mesh_points[*it] - src;
          E.at<double>(DIMENSION_2D * p    , 0) =  e.x;
          E.at<double>(DIMENSION_2D * p    , 1) =  e.y;
          E.at<double>(DIMENSION_2D * p + 1, 0) =  e.y;
          E.at<double>(DIMENSION_2D * p + 1, 1) = -e.x;
        }
        transpose(E, Et);// 转置
        Point2f e_main = dst - src;
        E_Main.at<double>(0, 0) =  e_main.x;
        E_Main.at<double>(0, 1) =  e_main.y;
        E_Main.at<double>(1, 0) =  e_main.y;
        E_Main.at<double>(1, 1) = -e_main.x;

        Mat G_W = (Et * E).inv(DECOMP_SVD) * Et;
        Mat L_W = - E_Main * G_W;

        double _global_similarity_weight = global_similarity_weight_beta;
        if (global_similarity_weight_gamma) {// TODO debug
          double sum_weight = 0;
          for (int p = 0; p < e_neighbors[j].size(); p ++) {
            sum_weight += images_grid_space_matching_pts_weight[i][e_neighbors[j][p]];
            cout << e_neighbors[j][p] << endl;
          }
          _global_similarity_weight = _global_similarity_weight + global_similarity_weight_gamma * (sum_weight / e_neighbors[j].size());
        }

        double _local_similarity_weight = 1;
        it = point_ind_set.begin();
        for (int p = 0; it != point_ind_set.end(); p ++, it ++) {
          for (int xy = 0; xy < DIMENSION_2D; xy ++) {
            for (int dim = 0; dim < DIMENSION_2D; dim ++) {
              if (local_similarity_term) {
                _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
                    images_vertices_start_index[i]  + DIMENSION_2D * (*it) + xy,
                    _local_similarity_weight *
                    local_similarity_weight * L_W.at<double>(dim, DIMENSION_2D * p + xy));
                _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
                    images_vertices_start_index[i]  + DIMENSION_2D * ind_e1 + xy,
                    _local_similarity_weight *
                    -local_similarity_weight * L_W.at<double>(dim, DIMENSION_2D * p + xy));
              }
              if (global_similarity_term) {
                _triplets.emplace_back(global_similarity_equation.first + eq_count + dim,
                    images_vertices_start_index[i]    + DIMENSION_2D * (*it) + xy,
                    _global_similarity_weight * G_W.at<double>(dim, DIMENSION_2D * p + xy));
                _triplets.emplace_back(global_similarity_equation.first + eq_count + dim,
                    images_vertices_start_index[i]    + DIMENSION_2D * ind_e1 + xy,
                    -_global_similarity_weight * G_W.at<double>(dim, DIMENSION_2D * p + xy));
                _b_vector.emplace_back(global_similarity_equation.first + eq_count + dim, _global_similarity_weight * similarity[dim]);
              }
            }
          }
        }
        if (local_similarity_term) {
          _triplets.emplace_back(local_similarity_equation.first + eq_count    , images_vertices_start_index[i] + DIMENSION_2D * ind_e2    ,
              _local_similarity_weight *  local_similarity_weight);
          _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, images_vertices_start_index[i] + DIMENSION_2D * ind_e2 + 1,
              _local_similarity_weight *  local_similarity_weight);
          _triplets.emplace_back(local_similarity_equation.first + eq_count    , images_vertices_start_index[i] + DIMENSION_2D * ind_e1    ,
              _local_similarity_weight * -local_similarity_weight);
          _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, images_vertices_start_index[i] + DIMENSION_2D * ind_e1 + 1,
              _local_similarity_weight * -local_similarity_weight);
        }
        eq_count += DIMENSION_2D;
        eq_count_rotation ++;
      }
    }
    assert(! local_similarity_term || eq_count ==  local_similarity_equation.second);
    assert(!global_similarity_term || eq_count == global_similarity_equation.second);
  }
}

void MeshOptimization::getImageMeshPoints(vector<Triplet<double> > & _triplets,
    const vector<pair<int, double> > & _b_vector) {
  LOG("%ld %ld", _triplets.size(), _b_vector.size());

  const int equations = global_similarity_equation.first + global_similarity_equation.second;

  LeastSquaresConjugateGradient<SparseMatrix<double> > lscg;
  SparseMatrix<double> A(equations, getVerticesCount());
  VectorXd b = VectorXd::Zero(equations), x;

  A.setFromTriplets(_triplets.begin(), _triplets.end());
  for (int i = 0; i < _b_vector.size(); i ++) {
    b[_b_vector[i].first] = _b_vector[i].second;
  }

  lscg.compute(A);
  x = lscg.solve(b);

  multi_images->image_mesh_points.resize(multi_images->img_num);
  for (int i = 0, x_index = 0; i < multi_images->img_num; i ++) {
    int count = (int)multi_images->imgs[i]->getMeshPoints().size() * DIMENSION_2D;
    multi_images->image_mesh_points[i].reserve(count);
    for (int j = 0; j < count; j += DIMENSION_2D) {
      multi_images->image_mesh_points[i].emplace_back(x[x_index + j    ],
                                                      x[x_index + j + 1]);
    }
    x_index += count;
  }
}