#include "MultiImages.h"

MultiImages::MultiImages() {
  img_num = 0;
}

void MultiImages::readImg(const char *img_path) {
  // 读取原始图片Mat
  Mat origin_img = imread(img_path);
  float original_img_size = origin_img.rows * origin_img.cols;

  // 缩放
  if (original_img_size > DOWN_SAMPLE_IMAGE_SIZE) {
    float scale = sqrt(DOWN_SAMPLE_IMAGE_SIZE / original_img_size);
    resize(origin_img, origin_img, Size(), scale, scale);
  }
  origin_data.emplace_back(origin_img);
  img_num ++;
  assert(origin_data.size() == img_num);
}

/***
  *
  * 计算特征匹配
  *
  **/

vector<pair<int, int> > MultiImages::getInitialFeaturePairs() {  
  const int nearest_size = 2;
  const bool ratio_test = true;

  int size_1 = imgs[0]->feature_points.size();
  int size_2 = imgs[1]->feature_points.size();
  const int feature_size[2] = { size_1, size_2 };
  const int pair_match[2] = { 0, 1 };

  const int another_feature_size = feature_size[1];
  const int nearest_k = min(nearest_size, another_feature_size);// 只可能为0, 1, 2
  const vector<vector<Mat> > &feature_descriptors_1 = imgs[pair_match[0]]->descriptors;
  const vector<vector<Mat> > &feature_descriptors_2 = imgs[pair_match[1]]->descriptors;

  // 对每个点计算最相近的特征点
  vector<FeatureDistance> feature_pairs_result;
  for (int f1 = 0; f1 < feature_size[0]; f1 ++) {
    set<FeatureDistance> feature_distance_set;// set 中每个元素都唯一, 降序
    feature_distance_set.insert(FeatureDistance(MAXFLOAT, 0, -1, -1));// 存放计算结果
    for (int f2 = 0; f2 < feature_size[1]; f2 ++) {
      const double dist = FeatureController::getDistance(feature_descriptors_1[f1],
          feature_descriptors_2[f2],
          feature_distance_set.begin()->distance);
      if (dist < feature_distance_set.begin()->distance) {// 如果比最大值小
        if (feature_distance_set.size() == nearest_k) {// 如果容器满了, 删掉最大值
          feature_distance_set.erase(feature_distance_set.begin());
        }
        feature_distance_set.insert(FeatureDistance(dist, 0, f1, f2));// 插入新的值, 这里f1和f2是两幅图片特征点的总索引
      }
    }

    set<FeatureDistance>::const_iterator it = feature_distance_set.begin();// feature_distance只可能有0, 1, 2个元素
    if (ratio_test) {// TODO
      const set<FeatureDistance>::const_iterator it2 = std::next(it, 1);
      if (nearest_k == nearest_size &&
          it2->distance * FEATURE_RATIO_TEST_THRESHOLD > it->distance) {// 后一个的1.5倍 > 当前, 则直接放弃当前descriptor
        continue;
      }
      it = it2;// 否则向后迭代一次
    }
    // 向pairs的尾部添加it及其之后的[it, end)的元素, 个数为0或1
    feature_pairs_result.insert(feature_pairs_result.end(), it, feature_distance_set.end());
  }

  // 计算平均值和标准差
  vector<double> distances;
  distances.reserve(feature_pairs_result.size());
  for (int i = 0; i < feature_pairs_result.size(); i ++) {
    distances.emplace_back(feature_pairs_result[i].distance);
  }
  double mean, std;
  Statistics::getMeanAndSTD(distances, mean, std);

  // 保存结果: 两幅图片的特征点总索引配对
  const double OUTLIER_THRESHOLD = (INLIER_TOLERANT_STD_DISTANCE * std) + mean;// 计算特征配对筛选条件
  vector<pair<int, int> > initial_indices;
  initial_indices.reserve(feature_pairs_result.size());
  for (int i = 0; i < feature_pairs_result.size(); i ++) {
    if (feature_pairs_result[i].distance < OUTLIER_THRESHOLD) {// 如果没有超出范围
      initial_indices.emplace_back(feature_pairs_result[i].feature_index[0], feature_pairs_result[i].feature_index[1]);
    }
  }
  LOG("initial pairs %ld", initial_indices.size());
  return initial_indices;
}

vector<pair<int, int> > MultiImages::getFeaturePairsBySequentialRANSAC(
    const vector<Point2f> & _X,
    const vector<Point2f> & _Y,
    const vector<pair<int, int> > & _initial_indices) {
  vector<char> final_mask(_initial_indices.size(), 0);// 存储最终的结果
  findHomography(_X, _Y, CV_RANSAC, GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST, final_mask, GLOBAL_MAX_ITERATION);

  vector<Point2f> tmp_X = _X, tmp_Y = _Y;

  vector<int> mask_indices(_initial_indices.size(), 0);
  for (int i = 0; i < mask_indices.size(); i ++) {
    mask_indices[i] = i;
  }

  // while (tmp_X.size() >= HOMOGRAPHY_MODEL_MIN_POINTS && // 4
  //     LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST < GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST) 
  while (true) {
    if (tmp_X.size() < HOMOGRAPHY_MODEL_MIN_POINTS) {
      break;
    }

    vector<Point2f> next_X, next_Y;
    vector<char> mask(tmp_X.size(), 0);
    findHomography(tmp_X, tmp_Y, CV_RANSAC, LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST, mask, LOCAL_MAX_ITERATION);

    int inliers_count = 0;
    for (int i = 0; i < mask.size(); i ++) {
      if (mask[i]) { inliers_count ++; }
    }

    if (inliers_count < LOCAL_HOMOGRAPHY_MIN_FEATURES_COUNT) {// 40
      break;
    }

    for (int i = 0, shift = -1; i < mask.size(); i ++) {
      if (mask[i]) {
        final_mask[mask_indices[i]] = 1;
      } else {
        next_X.emplace_back(tmp_X[i]);
        next_Y.emplace_back(tmp_Y[i]);
        shift ++;
        mask_indices[shift] = mask_indices[i];
      }
    }

    LOG("Local true Probabiltiy = %lf", next_X.size() / (float)tmp_X.size());

    tmp_X = next_X;
    tmp_Y = next_Y;
  }
  vector<pair<int, int> > result;
  for (int i = 0; i < final_mask.size(); i ++) {
    if (final_mask[i]) {
      result.emplace_back(_initial_indices[i]);
    }
  }

  LOG("Global true Probabiltiy = %lf", result.size() / (float)_initial_indices.size());

  return result;
}

void MultiImages::getFeaturePairs(int _m1, int _m2) {
  // 获取feature points下标的配对信息

  vector<pair<int, int> > initial_indices = getInitialFeaturePairs();

  // 将所有成功配对的特征点进行筛选
  const vector<Point2f> & m1_fpts = imgs[_m1]->feature_points;
  const vector<Point2f> & m2_fpts = imgs[_m2]->feature_points;
  vector<Point2f> X, Y;
  X.reserve(initial_indices.size());
  Y.reserve(initial_indices.size());
  for (int j = 0; j < initial_indices.size(); j ++) {
    const pair<int, int> it = initial_indices[j];
    X.emplace_back(m1_fpts[it.first ]);
    Y.emplace_back(m2_fpts[it.second]);
  }
  initial_pairs = initial_indices;
  feature_pairs = getFeaturePairsBySequentialRANSAC(X, Y, initial_indices);

  LOG("%d %d feature pairs %ld", _m1, _m2, feature_pairs.size());
}

/***
  *
  * 图像配准
  *
  **/

void MultiImages::getFeatureInfo() {
  // 每张图片检测特征点
  feature_points.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    feature_points[i].resize(img_num);

    Mat grey_img;
    cvtColor(imgs[i]->data, grey_img, CV_BGR2GRAY);
    FeatureController::detect(grey_img,
                              imgs[i]->feature_points,
                              imgs[i]->descriptors);
    LOG("[picture %d] feature points: %ld", i, imgs[i]->feature_points.size());
  }

  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;

    // 清空临时数据    
    initial_pairs.clear();
    feature_pairs.clear();

    // 特征点匹配
    getFeaturePairs();

    // 筛选所有图片的成功匹配的特征点
    const vector<Point2f> & m1_fpts = imgs[m1]->feature_points;
    const vector<Point2f> & m2_fpts = imgs[m2]->feature_points;
    for (int j = 0; j < feature_pairs.size(); j ++) {
      const pair<int, int> it = feature_pairs[j];
      feature_points[m1][m2].emplace_back(m1_fpts[it.first ]);
      feature_points[m2][m1].emplace_back(m2_fpts[it.second]);
    }
  }
}

void MultiImages::getMeshInfo() {
  // 初始化数据
  apap_pts.resize(img_num);
  matching_pts.resize(img_num);
  total_mask.resize(img_num);
  single_mask.resize(img_num);

  for (int i = 0; i < img_num; i ++) {
    apap_pts[i].resize(img_num);
    single_mask[i].resize(img_num);
    total_mask.resize(imgs[i]->vertices.size());// 初始化为false
        
    // 图像网格化
    vector<double> col_r, row_r;

    // double base = 0;
    // for (int i = 0; i <= 7; i ++) {
    //   col_r.emplace_back(base);
    //   base += (1 - base) / 2.5;
    // }
    // col_r.emplace_back(1);
    int mesh_size = 8;
    for (int i = 0; i <= mesh_size; i ++) {
      col_r.emplace_back((double)i / mesh_size);
      row_r.emplace_back((double)i / mesh_size);
    }
    imgs[i]->initVertices(col_r, row_r);
  }

  // apap匹配点
  for (int i = 0; i < img_pairs; i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;
    
    // 计算apap结果
    Homographies::compute(
      feature_points[m1][m2],
      feature_points[m2][m1],
      imgs[m1]->vertices,
      apap_pts[m1][m2],
      imgs[m1]->homographies
    );

    // 获取m1在m2上(未出界)的匹配点
    int pts_num = apap_pts[m1][m2].size();
    assert(pts_num == imgs[m1]->vertices.size());
    assert(single_mask[m1][m2].empty());
    single_mask[m1][m2].resize(pts_num);// 初始化为false

    for (int i = 0; i < pts_num; i ++) {
      Point2f tmp_point = apap_pts[m1][m2][i];
      // 此时以m2为参照, m1的坐标可能为负
      if ( tmp_point.x >= 0 && tmp_point.y >= 0
        && tmp_point.x <= imgs[m2]->data.cols && tmp_point.y <= imgs[m2]->data.rows) {
        // 没有出界
        single_mask[m1][m2][i] = true;
        total_mask[m1][i] = true;
      }
    }
  }
}

void MultiImages::similarityTransform(int _mode, vector<double> _angles) {
  // _angle必须是弧度
  // 初始化
  similarity_pts.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    similarity_pts[i].resize(img_num);
  }

  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;

    int equations = feature_points[m1][m2].size();// 数目与[m2][m1]相同
    // 式子是特征点数目的两倍

    if (_mode == 0) {
      // 只计算平移, 未知数为(x, y)

      MatrixXd A = MatrixXd::Zero(equations * 2, 2);
      VectorXd b = VectorXd::Zero(equations * 2);
      for (int i = 0; i < equations; i ++) {
        // x + 0 = x2 - (x1 cos - y1 sin)
        // 0 + y = y2 - (x1 sin + y1 cos)
        double x1 = feature_points[m1][m2][i].x;
        double y1 = feature_points[m1][m2][i].y;
        double x2 = feature_points[m2][m1][i].x;
        double y2 = feature_points[m2][m1][i].y;
        A(i*2 + 0, 0) = 1;
        b(i*2 + 0)    = x2 - (x1 * cos(rotate) - y1 * sin(rotate));
        A(i*2 + 1, 1) = 1;
        b(i*2 + 1)    = y2 - (x1 * sin(rotate) + y1 * cos(rotate));
      }
      VectorXd solution = A.colPivHouseholderQr().solve(b);
      rotate  = _angle;
      scale   = 1;
      shift_x = solution(0);
      shift_y = solution(1);

    } else if (_mode == 1) {
      // 计算平移和缩放, 未知数为(s, x, y)
      // s: 以图像(0, 0)为中心放缩图像s倍
    
      MatrixXd A = MatrixXd::Zero(equations * 2, 3);
      VectorXd b = VectorXd::Zero(equations * 2);
      for (int i = 0; i < equations; i ++) {
        // (x1 cos - y1 sin) * s + x + 0 = x2
        // (x1 sin + y1 cos) * s + 0 + y = y2
        double x1 = feature_points[m1][m2][i].x;
        double y1 = feature_points[m1][m2][i].y;
        double x2 = feature_points[m2][m1][i].x;
        double y2 = feature_points[m2][m1][i].y;
        A(i*2 + 0, 0) = x1 * cos(rotate) - y1 * sin(rotate);
        A(i*2 + 0, 1) = 1;
        b(i*2 + 0)    = x2;
        A(i*2 + 1, 0) = x1 * sin(rotate) + y1 * cos(rotate);
        A(i*2 + 1, 2) = 1;
        b(i*2 + 1)    = y2;
      }
      VectorXd solution = A.colPivHouseholderQr().solve(b);
      rotate  = _angle;
      scale   = solution(0);
      shift_x = solution(1);
      shift_y = solution(2);

    } else {
      // 计算平移和旋转, 并使用CV方式计算旋转, 未知数为(r, s, x, y)
      // r: 绕图像(0, 0)逆时针旋转r弧度
      // s: 以图像(0, 0)为中心放缩图像s倍
      /*
        旋转公式: (正常方向下逆时针)
        x' = x cos - y sin
        y' = x sin + y cos
        原方程:
        s(x1 cos - y1 sin) + x = x2
        s(x1 sin + y1 cos) + y = y2
        转换为:
        a x1 - b y1 + x = x2
        b x1 + a y1 + y = y2
        其中:
        a = s cos
        b = s sin
      */

      assert(0);// 该种方法不好用
      MatrixXd A = MatrixXd::Zero(equations * 2, 4);
      VectorXd b = VectorXd::Zero(equations * 2);
      for (int i = 0; i < equations; i ++) {
        // x1 p - y1 q + x + 0 = x2
        // y1 p + x1 q + 0 + y = y2
        A(i*2 + 0, 0) = feature_points[m1][m2][i].x;
        A(i*2 + 0, 1) = -feature_points[m1][m2][i].y;
        A(i*2 + 0, 2) = 1;
        b(i*2 + 0)    = feature_points[m2][m1][i].x;
        A(i*2 + 1, 0) = feature_points[m1][m2][i].y;
        A(i*2 + 1, 1) = feature_points[m1][m2][i].x;
        A(i*2 + 1, 3) = 1;
        b(i*2 + 1)    = feature_points[m2][m1][i].y;
      }
      VectorXd solution = A.colPivHouseholderQr().solve(b);
      double p = solution(0);
      double q = solution(1);
      scale   = sqrt(p*p + q*q);
      rotate  = acos(p / scale);
      if (q < 0) {
        rotate = 2 * M_PI - rotate;
      }
      shift_x = solution(1);
      shift_y = solution(2);
    }

    // 存储相似变换结果
    LOG("r=%lf, s=%lf, x=%lf, y=%lf", rotate, scale, shift_x, shift_y);
    for (int i = 0; i < imgs[0]->vertices.size(); i ++) {
      double x = imgs[0]->vertices[i].x;
      double y = imgs[0]->vertices[i].y;
      double new_x = (x * cos(rotate) - y * sin(rotate)) * scale + shift_x;
      double new_y = (x * sin(rotate) + y * cos(rotate)) * scale + shift_y;
      similarity_pts[m1][m2].emplace_back(new_x, new_y);
    }
  }
}

/***
  *
  * 网格优化
  *
  **/

void MultiImages::meshOptimization() {
  
  vector<Triplet<double> > triplets;
  vector<pair<int, double> > b_vector;

  pair_index.resize(img_num);
  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;

    pair_index[m1].emplace_back(m2);
  }

  for (int i = 0; i < img_num; i ++) {
    // 输入的图像顺序要按拼接的顺序输入
    for (int j = 0; j < pair_index[i].size(); j ++) {
      reserveData(i, j, triplets, b_vector);
      if (alignment_equation.second > 0) {
        prepareAlignmentTerm(i, j, triplets, b_vector);
      }
      if (local_similarity_equation.second > 0) {
        prepareLocalSimilarityTerm(i, triplets, b_vector);
      }
      if (global_similarity_equation.second  > 0) {
        prepareGlobalSimilarityTerm(i, triplets, b_vector);
      }
      if (sensor_equation.second > 0) {
        prepareSensorTerm(i, j, triplets, b_vector);
      }
      getSolution(triplets, b_vector);
    }
  }
}

void MultiImages::reserveData(
    int _m1,
    vector<Triplet<double> > & _triplets, 
    vector<pair<int, double> > & _b_vector) {
  total_eq = 0;

  alignment_equation.second = 0;
  local_similarity_equation.second = 0;
  global_similarity_equation.second = 0;
  sensor_equation.second = 0;

  // 相似项处理
  int edge_count = imgs[_m1]->edges.size();// 边的数目
  int edge_neighbor_vertices_count = 0;// 每条边的每个顶点的相邻顶点数目和(不包括自身)
  for (int i = 0; i < imgs[_m1]->edge_neighbors.size(); i ++) {
    edge_neighbor_vertices_count += imgs[_m1]->edge_neighbors[i].size();
  }
  local_similarity_equation.second = edge_count * 2;// 每条边对应两个顶点
  // global_similarity_equation.second = edge_count * 2;// 已经舍弃

  // 对齐项
  for (int i = 0; i < pair_index[_m1].size(); i ++) {
    int m1 = _m1;
    int m2 = pair_index[_m1][i];

    // 计算目标图像在参考图像上的未出界顶点数目和(可能与多张图片有重合)
    for (int j = 0; j < single_mask[m1][m2].size(); j ++) {
      if (single_mask[m1][m2]) {
        alignment_equation.second += 2;
      }
    }
  }

  // 传感器处理
  for (int i = 0; i < total_mask[_m1].size(); i ++) {
    // 计算未成功匹配的数目和
    if (!total_mask[_m1][i]) {
      sensor_equation.second += pair_index[_m1].size() * 2;// 顶点数 * 2 * 配对图像数目
    }
  }

  // 对齐项
  alignment_equation.first = 0;

  // 局部相似项
  local_similarity_equation.first = alignment_equation.first + alignment_equation.second;

  // 全局相似项
  global_similarity_equation.first = local_similarity_equation.first + local_similarity_equation.second;

  // 传感器定位
  sensor_equation.first = global_similarity_equation.first + global_similarity_equation.second;

  // triplet的大小并不对应equations的数目
  // triplet(row, col, val):
  // row: 第row个等式, 不会超过equations
  // col: 第col个matching points
  // val: 系数
  // dim: TODO, x或y
  int triplet_size = 
      alignment_equation.second * 4 // 对齐项: 每个匹配点需要grid的4个顶点
    + edge_neighbor_vertices_count * 4 + edge_count * 2 // 局部相似项: 等式 == edge的2个端点 + 2维 * 2维 * edge的邻接顶点
    + edge_neighbor_vertices_count * 4// 全局相似项: 等式 == 2维 * 2维 * edge的邻接顶点
    + sensor_equation.second;// TODO
  _triplets.reserve(triplet_size);

  // b向量不为零的数目
  int b_vector_size = alignment_equation.second + global_similarity_equation.second + sensor_equation.second;
  _b_vector.reserve(b_vector_size);
}

void MultiImages::prepareAlignmentTerm(
    int _m1,
    vector<Triplet<double> > & _triplets,
    vector<pair<int, double> > & _b_vector) {

  if (alignment_equation.second <= 0) {
    assert(eq_count == alignment_equation.second);
    return;
  }

  // polygon索引
  const vector<vector<int> > indices_1 = imgs[_m1]->polygons_indices;

  const int equation = alignment_equation.first;
  int eq_count = 0;

  for (int i = 0; i < pair_index[_m1].size(); i ++) {
    int m1 = _m1;
    int m2 = pair_index[_m1][i];

    // 记录顶点所在polygon索引, 以及在polygon中的分量
    vector<int>             index_1(single_mask[m1][m2].size());
    vector<vector<double> > weights_1(single_mask[m1][m2].size());

    for (int j = 0; j < single_mask[m1][m2].size(); j ++) {
      if (!single_mask[m1][m2]) {
        // 该匹配点出界
        continue;
      }

      int tmp_index;
      vector<double> tmp_weights(4);
      Point2f point_1 = imgs[m1]->vertices[j];// 原始顶点
      Point2f point_2 = apap_pts[m1][m2][j];// 匹配点

      // 目标图像的顶点在自身的分量
      imgs[m1]->getInterpolateVertex(point_1, tmp_index, tmp_weights);
      index_1[j] = tmp_index;
      weights_1[j] = tmp_weights;

      for (int dim = 0; dim < 2; dim ++) {
        // dim: x, y
        for (int k = 0; k < 4; k ++) {
          // 目标图像的顶点在自身的分量
          _triplets.emplace_back(equation + eq_count + dim,
            dim + 2 * indices_1[index_1[j]][k],
            alignment_weight * weights_1[j][k]);
        }

        // 目标图像的匹配点在参考图像的分量
        if (dim == 0) {// x
          _b_vector.empty(equation + eq_count + dim, apap_pts[m1][m2][j].x);
        } else {// y
          _b_vector.empty(equation + eq_count + dim, apap_pts[m1][m2][j].y);
        }
      }

      eq_count += 2;// x, y
    }
  }

  LOG("alignment %d", eq_count);
  assert(eq_count == alignment_equation.second);
}

void MultiImages::prepareLocalSimilarityTerm(
    int _m1,
    vector<Triplet<double> > & _triplets, 
    vector<pair<int, double> > & _b_vector) {

  int eq_count = 0;

  const vector<pair<int, int> > edges = imgs[_m1]->edges;
  const vector<Point2f> vertices = imgs[_m1]->vertices;
  const vector<vector<int> > v_neighbors = imgs[_m1]->vertex_neighbors;
  const vector<vector<int> > e_neighbors = imgs[_m1]->edge_neighbors;// 和原意不同
  assert(e_neighbors.size() == edges.size());

  for (int i = 0; i < edges.size(); i ++) {
    // 获取边的两个端点
    const int ind_e1 = edges[i].first;
    const int ind_e2 = edges[i].second;
    const Point2f src = vertices[ind_e1];
    const Point2f dst = vertices[ind_e2];

    // 记录边端点的邻接顶点
    const vector<int> point_ind_set = e_neighbors[i];

    // 看不懂
    Mat Et, E_Main(2, 2, CV_64FC1), E((int)point_ind_set.size() * 2, 2, CV_64FC1);
    for (int j = 0; j < point_ind_set.size(); j ++) {
      Point2f e = vertices[point_ind_set[j]] - src;
      E.at<double>(2 * j    , 0) =  e.x;
      E.at<double>(2 * j    , 1) =  e.y;
      E.at<double>(2 * j + 1, 0) =  e.y;
      E.at<double>(2 * j + 1, 1) = -e.x;
    }
    transpose(E, Et);// 转置
    Point2f e_main = dst - src;
    E_Main.at<double>(0, 0) =  e_main.x;
    E_Main.at<double>(0, 1) =  e_main.y;
    E_Main.at<double>(1, 0) =  e_main.y;
    E_Main.at<double>(1, 1) = -e_main.x;

    Mat G_W = (Et * E).inv(DECOMP_SVD) * Et;
    Mat L_W = - E_Main * G_W;

    for (int j = 0; j < point_ind_set.size(); j ++) {
      for (int xy = 0; xy < 2; xy ++) {
        for (int dim = 0; dim < 2; dim ++) {
          // local
          _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
            2 * point_ind_set[j] + xy,
            local_similarity_weight * L_W.at<double>(dim, 2 * j + xy));
          _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
            2 * ind_e1 + xy,
            -local_similarity_weight * L_W.at<double>(dim, 2 * j + xy));
        }
      }
    }

    // local: x1, y1, x2, y2 
    _triplets.emplace_back(local_similarity_equation.first + eq_count    , 2 * ind_e2    ,
      local_similarity_weight);
    _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, 2 * ind_e2 + 1,
      local_similarity_weight);
    _triplets.emplace_back(local_similarity_equation.first + eq_count    , 2 * ind_e1    ,
      -local_similarity_weight);
    _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, 2 * ind_e1 + 1,
      -local_similarity_weight);
    
    eq_count += 2;
  }

  assert(eq_count == local_similarity_equation.second);
}

void MultiImages::prepareSensorTerm(
    int _m1,
    vector<Triplet<double> > & _triplets, 
    vector<pair<int, double> > & _b_vector) {
  // 给非重叠区域的顶点定位
  assert(!similarity_pts.empty());

  const int equation = global_similarity_equation.first;
  int eq_count = 0;

  for (int i = 0; i < pair_index[_m1].size(); i ++) {
    int m1 = _m1;
    int m2 = pair_index[_m1][i];

    for (int j = 0; j < total_mask[m1].size(); j ++) {
      if (total_mask[m1][j]) {
        // 该顶点至少与一张图片重合
        continue;
      }

      // x: dim = 0, y: dim = 1
      double x = similarity_pts[m1][m2][j].x;
      _triplets.emplace_back(equation + eq_count + 0, j * 2 + 0, 1 * sensor_weight);
      _b_vector.emplace_back(equation + eq_count + 0, x * sensor_weight);
      double y = similarity_pts[m1][m2][j].y;
      _triplets.emplace_back(equation + eq_count + 1, j * 2 + 1, 1 * sensor_weight);
      _b_vector.emplace_back(equation + eq_count + 1, y * sensor_weight);
      eq_count += 2;
    }
  }

  assert(eq_count == sensor_equation.second);
}

void MultiImages::getSolution(  
    vector<Triplet<double> > & _triplets, 
    vector<pair<int, double> > & _b_vector) {
  int equations = alignment_equation.second + local_similarity_equation.second + global_similarity_equation.second + sensor_equation.second;

  LeastSquaresConjugateGradient<SparseMatrix<double> > lscg;
  SparseMatrix<double> A(equations, imgs[0]->vertices.size() * 2);
  VectorXd b = VectorXd::Zero(equations), x;

  A.setFromTriplets(_triplets.begin(), _triplets.end());
  for (int i = 0; i < _b_vector.size(); i ++) {
    b[_b_vector[i].first] = _b_vector[i].second;
  }

  lscg.compute(A);
  x = lscg.solve(b);

  matching_pts[0].clear();
  for (int i = 0; i < imgs[0]->vertices.size() * 2; i += 2) {
    matching_pts[0].emplace_back(x[i], x[i + 1]);
  }
}

/***
  *
  * 图像形变:
    两个函数不会改变网格的位置, 且图像原点为(0, 0)
  *
  **/

void MultiImages::myWarping() {
  // 初始化
  pano_images.clear();
  pano_masks.clear();
  origin_masks.clear();

  // 对所有图像的网格点归一化(去除负值)
  Size2f tmp_size = normalizeVertices(matching_pts);
  pano_size = Size2i(ceil(tmp_size.width), ceil(tmp_size.height));
  
  // 获得每个形变的图片, 图片的起点都是(0, 0)?? TODO
  Mat warped_image;
  Mat image_mask;
  warpImage(
    imgs[0]->vertices,
    matching_pts[0],
    imgs[0]->triangle_indices,
    imgs[0]->data,
    warped_image,
    image_mask);

  // 将每个图片/mask平移到最终位置
  for (int i = 0; i < 2; i ++) {
    Mat tmp_image = Mat::zeros(pano_size, CV_8UC4);
    Mat tmp_mask = Mat::zeros(pano_size, CV_8UC1);
    // 计算目标矩阵
    Rect2f rect = getVerticesRects(matching_pts[i]);
    if (i == 1) {
      // 参考图片
      cvtColor(imgs[i]->data, tmp_image(rect), COLOR_RGB2RGBA);
      imgs[i]->mask.copyTo(tmp_mask(rect));
    } else {
      warped_image.copyTo(tmp_image(rect));
      image_mask.copyTo(tmp_mask(rect));
    }

    pano_images.emplace_back(tmp_image);
    pano_masks.emplace_back(tmp_mask);
    origin_masks.emplace_back(tmp_mask);
  }
}

void MultiImages::warpImage(
    vector<Point2f> _src_p, vector<Point2f> _dst_p,
    vector<vector<int> > _indices, // 三角形的线性索引
    Mat _src, Mat & _dst, Mat & _img_mask) {

  assert(_src_p.size() == _dst_p.size());

  Size2f dst_size = normalizeVertices(_dst_p);// 归一化形变后的网格点
  Rect2f dst_rect = getVerticesRects(_dst_p);// 获取图片的最终矩形

  /* 计算每个三角形的仿射变换 */
  const Point2f shift(0.5, 0.5);
  Mat polygon_index_mask(dst_rect.height + shift.y, dst_rect.width + shift.x, CV_32SC1, Scalar::all(NO_GRID));// 记录每个像素点对应三角形的索引矩阵, 并填充初始值
  vector<Mat> affine_transform;
  affine_transform.reserve(_indices.size());// 三角形数目
  
  for (int i = 0; i < _indices.size(); i ++) {
    const Point2i contour[] = {
      _dst_p[_indices[i][0]],
      _dst_p[_indices[i][1]],
      _dst_p[_indices[i][2]],
    };
    // 往索引矩阵中填充索引值
    fillConvexPoly(
      polygon_index_mask, // 索引矩阵
      contour,            // 三角形区域
      TRIANGLE_COUNT,     // 三个角
      i,
      LINE_AA,
      PRECISION);
    // 计算三角形的仿射变换
    Point2f src[] = {
      _dst_p[_indices[i][0]],
      _dst_p[_indices[i][1]],
      _dst_p[_indices[i][2]],
    };
    Point2f dst[] = {
      _src_p[_indices[i][0]],
      _src_p[_indices[i][1]],
      _src_p[_indices[i][2]],
    };
    // 按顺序保存(逆向的)仿射变换, 顺序对应三角形的索引
    affine_transform.emplace_back(getAffineTransform(src, dst));
  }

  /* 计算目标图像 */
  _dst = Mat::zeros(dst_rect.height + shift.y, dst_rect.width + shift.x, CV_8UC4);
  _img_mask = Mat::zeros(_dst.size(), CV_8UC1);

  for (int y = 0; y < _dst.rows; y ++) {
    for (int x = 0; x < _dst.cols; x ++) {
      int polygon_index = polygon_index_mask.at<int>(y, x);
      if (polygon_index != NO_GRID) {
        Point2f p_f = applyTransform2x3<float>(x, y, affine_transform[polygon_index]);// 根据(逆向的)仿射变换, 计算目标图像上每个像素对应的原图像坐标
        if (p_f.x >= 0 && p_f.y >= 0 && p_f.x <= _src.cols && p_f.y <= _src.rows) {// 计算出来的坐标没有出界
          Vec3b c = getSubpix<uchar, 3>(_src, p_f);
          _dst.at<Vec4b>(y, x) = Vec4b(c[0], c[1], c[2], 255);// TODO 透明度通道
          _img_mask.at<uchar>(y, x) = 255;
        }
      }
    }
  }
}

void MultiImages::warpImage2(
    vector<Point2f> _src_p, vector<Point2f> _dst_p,
    vector<vector<int> > _indices, // 四边形的线性索引
    Mat _src, Mat & _dst, Mat & _img_mask) {
}

/***
  *
  * 特征点形变
  *
  **/

void MultiImages::warpPoints(
    vector<Point2f> _src_vertices, vector<Point2f> _dst_vertices,
    vector<vector<int> > _indices,
    vector<Point2f> _src_features, vector<Point2f> & _dst_features) {

  assert(_src_vertices.size() == _dst_vertices.size());

  Size2f src_size = normalizeVertices(_src_vertices);
  Rect2f src_rect = getVerticesRects(_src_vertices);

  /* 计算每个三角形的仿射变换 */
  const Point2f shift(0.5, 0.5);
  Mat polygon_index_mask(src_rect.height + shift.y, src_rect.width + shift.x, CV_32SC1, Scalar::all(NO_GRID));// 记录每个像素点对应三角形的索引矩阵, 并填充初始值
  vector<Mat> affine_transform;
  affine_transform.reserve(_indices.size());// 三角形数目
  
  for (int i = 0; i < _indices.size(); i ++) {
    const Point2i contour[] = {
      _src_vertices[_indices[i][0]],
      _src_vertices[_indices[i][1]],
      _src_vertices[_indices[i][2]],
    };
    // 往索引矩阵中填充索引值
    fillConvexPoly(
      polygon_index_mask, // 索引矩阵
      contour,            // 三角形区域
      TRIANGLE_COUNT,     // 三个角
      i,
      LINE_AA,
      PRECISION);
    // 计算三角形的仿射变换
    Point2f src[] = {
      _src_vertices[_indices[i][0]],
      _src_vertices[_indices[i][1]],
      _src_vertices[_indices[i][2]],
    };
    Point2f dst[] = {
      _dst_vertices[_indices[i][0]],
      _dst_vertices[_indices[i][1]],
      _dst_vertices[_indices[i][2]],
    };
    // 按顺序保存(顺向的)仿射变换, 顺序对应三角形的索引
    affine_transform.emplace_back(getAffineTransform(src, dst));
  }

  /* 计算目标特征点位置 */
  _dst_features.clear();
  for (int i = 0; i < _src_features.size(); i ++) {
    int x = _src_features[i].x;
    int y = _src_features[i].y;
    int polygon_index = polygon_index_mask.at<int>(y, x);
    assert(polygon_index != NO_GRID);
    Point2f p_f = applyTransform2x3<float>(x, y, affine_transform[polygon_index]);// 计算目标特征点位置, (顺向的)仿射变换
    _dst_features.emplace_back(p_f);
  }
}


/***
 *
 * 寻找接缝线
 *
 **/

void MultiImages::getSeam() {
  Ptr<MySeamFinder> seam_finder = new MySeamFinder(10000, 1000);

  // 图像类型转换
  vector<UMat> pano_images_f(2);
  vector<UMat> pano_masks_gpu(2);
  for (int i = 0; i < 2; i ++) {
    pano_images[i].convertTo(pano_images_f[i], CV_32F);
    cvtColor(pano_images_f[i], pano_images_f[i], CV_RGBA2RGB);
    pano_masks[i].copyTo(pano_masks_gpu[i]);
  }

  // 记录每个图像最终的起点位置
  vector<Point2i> img_origins;
  for (int i = 0; i < 2; i ++) {
    img_origins.emplace_back(0, 0);
  }
  seam_finder->find(pano_images_f, img_origins, pano_masks_gpu);

  // 同步Mat和UMat
  for (int i = 0; i < 2; i ++) {
    pano_masks_gpu[i].copyTo(pano_masks[i]);
  }

  // 计算接缝线位置
  vector<Point2f> seam_pts;
  getSeamPts(pano_masks[0], pano_masks[1], seam_pts);

  myBlending();// 先进行图像融合
  drawPoints(pano_result, seam_pts);
}

/***
  *
  * 图像融合
  *
  **/

void MultiImages::myBlending() {
  int pano_rows = pano_masks[0].rows;
  int pano_cols = pano_masks[0].cols;

  // 图像扩充
  getExpandMat(pano_images[0], pano_images[1], pano_masks[0], pano_masks[1]);
  getExpandMat(pano_images[1], pano_images[0], pano_masks[1], pano_masks[0]);

  // 线性融合mask计算
  getGradualMat(
    pano_images[0], pano_images[1], 
    origin_masks[0], origin_masks[1], 
    pano_masks[0], pano_masks[1]);

  vector<Point2f> img_origins;
  blend_weight_mask.clear();
  blend_weight_mask.resize(2);
  for (int i = 0; i < 2; i ++) {
    img_origins.emplace_back(0, 0);
    // show_img(pano_images[i], "image %d", i);
    pano_masks[i].convertTo(blend_weight_mask[i], CV_32FC1);
  }
  
  bool ignore_weight_mask = false;
  pano_result = Blending(
    pano_images,
    img_origins,
    pano_size,
    blend_weight_mask,
    ignore_weight_mask);
}

/***
  *
  * DEBUG
  *
  **/

void MultiImages::drawPoints(Mat _img, vector<Point2f> _points) {
  Mat result;
  vector<Point2f> tmp_points;
  tmp_points.assign(_points.begin(), _points.end());
  Size2f result_size = normalizeVertices(tmp_points);

  if (result_size.width < _img.cols || result_size.height < _img.rows) {
    result = _img.clone();
    tmp_points.assign(_points.begin(), _points.end());
  } else {
    result = Mat::zeros(result_size, _img.type());
  }

  for (int i = 0; i < tmp_points.size(); i ++) {
    Point2f p = tmp_points[i];
    Scalar color1(255, 0, 255, 255);
    circle(result, p, 2, color1, -1);// circle_size = 1
  }
  show_img("result", result);
}