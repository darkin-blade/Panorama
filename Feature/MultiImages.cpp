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
  * 统一预处理
  *
  **/

void MultiImages::init() {
  // 初始化旋转角度
  assert(rotations.empty());
  for (int i = 0; i < img_num; i ++) {
    rotations.emplace_back(0);// 负为逆时针
  }

  // 图像网格化
  for (int i = 0; i < img_num; i ++) {
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
}

/***
  *
  * 计算特征匹配
  *
  **/

vector<pair<int, int> > MultiImages::getInitialFeaturePairs(int _m1, int _m2) {  
  const int nearest_size = 2;
  const bool ratio_test = true;

  int size_1 = imgs[_m1]->feature_points.size();
  int size_2 = imgs[_m2]->feature_points.size();
  const int feature_size[2] = { size_1, size_2 };

  const int another_feature_size = feature_size[1];
  const int nearest_k = min(nearest_size, another_feature_size);// 只可能为0, 1, 2
  const vector<vector<Mat> > &feature_descriptors_1 = imgs[_m1]->descriptors;
  const vector<vector<Mat> > &feature_descriptors_2 = imgs[_m2]->descriptors;

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

  vector<pair<int, int> > initial_indices = getInitialFeaturePairs(_m1, _m2);

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

  // 没有任何配对信息, 只能从前往后配对
  for (int i = 1; i < img_num; i ++) {
    int m1 = i - 1;
    int m2 = i;

    // 每次匹配需要清空临时数据    
    initial_pairs.clear();
    feature_pairs.clear();

    // 特征点匹配
    getFeaturePairs(m1, m2);

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

void MultiImages::similarityTransform(int _mode) {
  // 必须是弧度
  assert(!rotations.empty());
  // 初始化
  similarity_pts.resize(img_num);
  scales.resize(img_num);
  translations.resize(img_num);

  
  // 没有任何配对信息, 只能从前往后配对
  for (int i = 0; i < img_num; i ++) {// 第0张图片的位置为(0, 0)
    if (i == 0) {
      // 起始图像
      scales[i] = 1;
      translations[i].x = 0;
      translations[i].y = 0;
    } else {
      // 其他目标图像
      int m1 = i;
      int m2 = i - 1;
      // 计算等式数目
      int equations = feature_points[m1][m2].size();// 数目与[m2][m1]相同

      if (_mode == 0) {
        // 只计算平移, 未知数为(x, y)
        MatrixXd A = MatrixXd::Zero(equations * 2, 2);// 2个未知数
        VectorXd b = VectorXd::Zero(equations * 2);
        double rotate = rotations[m1];
        int eq_count = 0;// 检验等式数目是否正确
        for (int k = 0; k < feature_points[m1][m2].size(); k ++) {
          // x + 0 = x2 - (x1 cos - y1 sin)
          // 0 + y = y2 - (x1 sin + y1 cos)
          double x1 = feature_points[m1][m2][k].x;
          double y1 = feature_points[m1][m2][k].y;
          double _x2 = feature_points[m2][m1][k].x;
          double _y2 = feature_points[m2][m1][k].y;
          double x2 = (_x2 * cos(rotations[m2]) - _y2 * sin(rotations[m2])) * scales[m2] + translations[m2].x;
          double y2 = (_x2 * sin(rotations[m2]) + _y2 * cos(rotations[m2])) * scales[m2] + translations[m2].y;
          A(eq_count + 0, 0) = 1;
          b(eq_count + 0)    = x2 - (x1 * cos(rotate) - y1 * sin(rotate));
          A(eq_count + 1, 1) = 1;
          b(eq_count + 1)    = y2 - (x1 * sin(rotate) + y1 * cos(rotate));
          eq_count += 2;
        }
        assert(eq_count == equations * 2);
        // 得出结果
        VectorXd solution = A.colPivHouseholderQr().solve(b);
        scales[m1]  = 1;
        translations[m1].x = solution(0);
        translations[m1].y = solution(1);
      } else if (_mode == 1) {
        // 计算平移和缩放, 未知数为(s, x, y)
        // s: 以图像(0, 0)为中心放缩图像s倍
        MatrixXd A = MatrixXd::Zero(equations * 2, 3);// 3 个未知数
        VectorXd b = VectorXd::Zero(equations * 2);
        double rotate = rotations[m1];
        int eq_count = 0;// 检验等式数目是否正确
        for (int m1 = 0; m1 < feature_points[m1][m2].size(); m1 ++) {
          // (x1 cos - y1 sin) * s + x + 0 = x2
          // (x1 sin + y1 cos) * s + 0 + y = y2
          double x1 = feature_points[m1][m2][m1].x;
          double y1 = feature_points[m1][m2][m1].y;
          double _x2 = feature_points[m2][m1][m1].x;
          double _y2 = feature_points[m2][m1][m1].y;
          double x2 = (_x2 * cos(rotations[m2]) - _y2 * sin(rotations[m2])) * scales[m2] + translations[m2].x;
          double y2 = (_x2 * sin(rotations[m2]) + _y2 * cos(rotations[m2])) * scales[m2] + translations[m2].y;
          A(eq_count + 0, 0) = x1 * cos(rotate) - y1 * sin(rotate);
          A(eq_count + 0, 1) = 1;
          b(eq_count + 0)    = x2;
          A(eq_count + 1, 0) = x1 * sin(rotate) + y1 * cos(rotate);
          A(eq_count + 1, 2) = 1;
          b(eq_count + 1)    = y2;
          eq_count += 2;
        }
        assert(eq_count == equations * 2);
        // 得出结果
        VectorXd solution = A.colPivHouseholderQr().solve(b);
        scales[m1]  = solution(0);
        translations[m1].x = solution(1);
        translations[m1].y = solution(2);
      }
    }

    // 计算相似变换后的结果
    LOG("r=%lf, s=%lf, x=%lf, y=%lf", rotations[i], scales[i], translations[i].x, translations[i].y);
    for (int j = 0; j < imgs[0]->vertices.size(); j ++) {
      double x = imgs[i]->vertices[j].x;
      double y = imgs[i]->vertices[j].y;
      double new_x = (x * cos(rotations[i]) - y * sin(rotations[i])) * scales[i] + translations[i].x;
      double new_y = (x * sin(rotations[i]) + y * cos(rotations[i])) * scales[i] + translations[i].y;
      similarity_pts[i].emplace_back(new_x, new_y);
    }
  }
}

void MultiImages::getImagePairs() {
  assert(img_pairs.empty());

  vector<vector<double> > img_dis;
  img_dis.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    img_dis[i].resize(img_num);
  }

  // 计算所有图片之间的距离
  priority_queue <ImageDistance> que;
  for (int i = 0; i < img_num; i ++) {
    for (int j = i + 1; j < img_num; j ++) {
      Point2f tmp_dis = translations[i] - translations[j];
      double dis = norm(tmp_dis);
      LOG("%d %d %lf", i, j, dis);
      que.push(ImageDistance(i, j, dis));
    }
  }

  vector<int> visited(img_num);
  ImageDistance id = que.top();
  int visited_num = 0;
  double dis_tresh = id.distance * 1.5;// 距离最近的两张图片的距离
  vector<vector<int> > adjList(img_num);// 无向图的邻接列表

  // 最小生成树
  while (!que.empty()) {
    id = que.top();
    que.pop();
    int u = id.u;
    int v = id.v;
    double distance = id.distance;
    if (visited_num < img_num || distance < dis_tresh) {
      // 未构成连通图, 无条件加边
      // 或
      // 距离小于设定的阈值, 添加该边
      adjList[u].emplace_back(v);
      adjList[v].emplace_back(u);
      LOG("add %d %d %lf", u, v, distance);
    } else {
      break;
    }
    // 检查是否访问过
    if (!visited[v]) {
      visited[v] = 1;
      visited_num ++;
    }
    if (!visited[u]) {
      visited[u] = 1;
      visited_num ++;
    }
  }

  // 重建图片之间的配对关系
  int start_index = 0;
  int max_neibors = -1;
  for (int i = 0; i < img_num; i ++) {
    if (adjList[i].size() > max_neibors) {
      start_index = i;
      max_neibors = adjList[i].size();
    }
  }

  // bfs搜索重新建图
  for (int i = 0; i < img_num; i ++) {
    visited[i] = 0;
  }
  int depth = 1;
  pair_index.resize(img_num);
  queue<int> q;
  q.push(start_index);
  visited[start_index] = depth;
  assert(image_order.empty());
  while (!q.empty()) {
    int u = q.front();
    depth = visited[u] + 1;
    q.pop();
    image_order.emplace_back(u);
    for (int i = 0; i < adjList[u].size(); i ++) {
      int v = adjList[u][i];
      if (!visited[v]) {
        visited[v] = depth + 1;
        q.push(v);
      }
      // 反向配对
      if (visited[v] > visited[u]) {
        LOG("%d %d", v, u);
        pair_index[v].emplace_back(u);
      }
    }
  }

  assert(image_order.size() == img_num);
}

void MultiImages::getMeshInfo() {
  // 初始化数据
  apap_pts.resize(img_num);
  total_mask.resize(img_num);
  single_mask.resize(img_num);

  for (int i = 0; i < img_num; i ++) {
    // 先要初始化网格, 再初始化变量
    apap_pts[i].resize(img_num);
    single_mask[i].resize(img_num);
    total_mask[i].resize(imgs[i]->vertices.size());// 初始化为false
  }

  // apap匹配点
  assert(!img_pairs.empty());
  for (int i = 0; i < img_pairs.size(); i ++) {
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

/***
  *
  * 计算APAP的结果
  *
  **/
void MultiImages::getTmpResult() {
  assert(matching_pts.empty());
  matching_pts.resize(img_num);
  // matching_pts[0].assign(imgs[0]->vertices.begin(), imgs[0]->vertices.end());
  // matching_pts[1].assign(apap_pts[1][0].begin(), apap_pts[1][0].end());
  // matching_pts[1].assign(imgs[1]->vertices.begin(), imgs[1]->vertices.end());
  // matching_pts[0].assign(apap_pts[0][1].begin(), apap_pts[0][1].end());
  for (int i = 0; i < img_num; i ++) {
    matching_pts[i].assign(similarity_pts[i].begin(), similarity_pts[i].end());
  }
}

/***
  *
  * 网格优化
  *
  **/

void MultiImages::meshOptimization() {
  // 初始化结果
  matching_pts.resize(img_num);
  
  vector<Triplet<double> > triplets;
  vector<pair<int, double> > b_vector;

  for (int i = 0; i < img_num; i ++) {
    if (i == 0) {
      matching_pts[image_order[i]].assign(imgs[image_order[i]]->vertices.begin(), imgs[image_order[i]]->vertices.end());
    } else {
      // 按顺序添加图片
      reserveData(image_order[i], triplets, b_vector);
      prepareAlignmentTerm(image_order[i], triplets, b_vector);
      prepareLocalSimilarityTerm(image_order[i], triplets, b_vector);
      prepareSensorTerm(image_order[i], triplets, b_vector);
      getSolution(image_order[i], triplets, b_vector);
    }
  }
}

void MultiImages::reserveData(
    int _m1,
    vector<Triplet<double> > & _triplets,
    vector<pair<int, double> > & _b_vector) {
  // 清空数据
  _triplets.clear();
  _b_vector.clear();

  alignment_equation.second = 0;
  local_similarity_equation.second = 0;
  sensor_equation.second = 0;

  // 相似项处理
  int edge_count = imgs[_m1]->edges.size();// 边的数目
  int edge_neighbor_vertices_count = 0;// 每条边的每个顶点的相邻顶点数目和(不包括自身)
  for (int i = 0; i < imgs[_m1]->edge_neighbors.size(); i ++) {
    edge_neighbor_vertices_count += imgs[_m1]->edge_neighbors[i].size();
  }
  local_similarity_equation.second = edge_count * 2;// 每条边对应两个顶点

  // 对齐项
  for (int i = 0; i < pair_index[_m1].size(); i ++) {
    int m1 = _m1;
    int m2 = pair_index[_m1][i];

    // 计算目标图像在参考图像上的未出界顶点数目和(可能与多张图片有重合)
    for (int j = 0; j < single_mask[m1][m2].size(); j ++) {
      if (single_mask[m1][m2][j]) {
        alignment_equation.second += 2;
      }
    }
  }

  // 传感器处理
  for (int i = 0; i < total_mask[_m1].size(); i ++) {
    // 计算未成功匹配的数目和
    if (!total_mask[_m1][i]) {
      sensor_equation.second += 2;// 非重叠顶点数 * 2
    }
  }

  // 对齐项
  alignment_equation.first = 0;

  // 局部相似项
  local_similarity_equation.first = alignment_equation.first + alignment_equation.second;

  // 传感器定位
  sensor_equation.first = local_similarity_equation.first + local_similarity_equation.second;

  // triplet的大小并不对应equations的数目
  // triplet(row, col, val):
  // row: 第row个等式, 不会超过equations
  // col: 第col个matching points
  // val: 系数
  int triplet_size = 
      alignment_equation.second * 4 // 对齐项: 每个匹配点需要grid的4个顶点
    + edge_neighbor_vertices_count * 4 + edge_count * 2 // 局部相似项: 等式 == edge的2个端点 + 2维 * 2维 * edge的邻接顶点
    // + edge_neighbor_vertices_count * 4// 全局相似项: 等式 == 2维 * 2维 * edge的邻接顶点, 作废
    + sensor_equation.second;
  _triplets.reserve(triplet_size);

  // b向量不为零的数目
  int b_vector_size = alignment_equation.second + sensor_equation.second;
  _b_vector.reserve(b_vector_size);
}

void MultiImages::prepareAlignmentTerm(
    int _m1,
    vector<Triplet<double> > & _triplets,
    vector<pair<int, double> > & _b_vector) {

  int eq_count = 0;
  if (alignment_equation.second <= 0) {
    assert(eq_count == alignment_equation.second);
    return;
  }

  for (int i = 0; i < pair_index[_m1].size(); i ++) {
    int m1 = _m1;
    int m2 = pair_index[_m1][i];

    // 记录顶点所在polygon索引, 以及在polygon中的分量
    const vector<vector<int> > indices_1 = imgs[m1]->polygons_indices;
    const vector<vector<int> > indices_2 = imgs[m2]->polygons_indices;
    vector<int>             index_1(single_mask[m1][m2].size());
    vector<vector<double> > weights_1(single_mask[m1][m2].size());
    vector<int>             index_2(single_mask[m1][m2].size());
    vector<vector<double> > weights_2(single_mask[m1][m2].size());

    for (int j = 0; j < single_mask[m1][m2].size(); j ++) {
      if (!single_mask[m1][m2][j]) {
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

      // 目标图像的匹配点在参考图像的分量
      imgs[m2]->getInterpolateVertex(point_2, tmp_index, tmp_weights);
      index_2[j] = tmp_index;
      weights_2[j] = tmp_weights;

      for (int dim = 0; dim < 2; dim ++) {
        // dim: x, y
        double b_sum = 0;
        for (int k = 0; k < 4; k ++) {
          // 目标图像的顶点在自身的分量
          _triplets.emplace_back(alignment_equation.first + eq_count + dim,
            dim + 2 * indices_1[index_1[j]][k],
            alignment_weight * weights_1[j][k]);
          
          // 目标图像的匹配点在参考图像的分量, 此时参考图像的顶点位置已经确定, 分量和是一个定值
          int index_of_m2 = indices_2[index_2[j]][k];
          if (dim == 0) {// x
            b_sum += alignment_weight * matching_pts[m2][index_of_m2].x * weights_2[j][k];
          } else {
            b_sum += alignment_weight * matching_pts[m2][index_of_m2].y * weights_2[j][k];
          }
        }
        _b_vector.emplace_back(alignment_equation.first + eq_count + dim, b_sum);
      }

      eq_count += 2;// x, y
    }
  }

  assert(eq_count == alignment_equation.second);
}

void MultiImages::prepareLocalSimilarityTerm(
    int _m1,
    vector<Triplet<double> > & _triplets, 
    vector<pair<int, double> > & _b_vector) {

  int eq_count = 0;
  if (local_similarity_equation.second <= 0) {
    assert(eq_count == local_similarity_equation.second);
    return;
  }

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

  int eq_count = 0;
  if (sensor_equation.second <= 0) {
    assert(eq_count == sensor_equation.second);
    return;
  }

  assert(!similarity_pts.empty());

  for (int i = 0; i < total_mask[_m1].size(); i ++) {
    if (total_mask[_m1][i]) {
      // 该顶点至少与一张图片重合
      continue;
    }

    // x: dim = 0, y: dim = 1
    double x = similarity_pts[_m1][i].x;
    _triplets.emplace_back(sensor_equation.first + eq_count + 0, i * 2 + 0, 1 * sensor_weight);// 不要使用eq_count
    _b_vector.emplace_back(sensor_equation.first + eq_count + 0, x * sensor_weight);
    double y = similarity_pts[_m1][i].y;
    _triplets.emplace_back(sensor_equation.first + eq_count + 1, i * 2 + 1, 1 * sensor_weight);
    _b_vector.emplace_back(sensor_equation.first + eq_count + 1, y * sensor_weight);

    eq_count += 2;
  }

  assert(eq_count == sensor_equation.second);
}

void MultiImages::getSolution(
    int _m1,
    vector<Triplet<double> > & _triplets, 
    vector<pair<int, double> > & _b_vector) {
  int equations = alignment_equation.second + local_similarity_equation.second + sensor_equation.second;

  LeastSquaresConjugateGradient<SparseMatrix<double> > lscg;
  SparseMatrix<double> A(equations, imgs[_m1]->vertices.size() * 2);
  VectorXd b = VectorXd::Zero(equations), x;

  A.setFromTriplets(_triplets.begin(), _triplets.end());
  for (int i = 0; i < _b_vector.size(); i ++) {
    b[_b_vector[i].first] = _b_vector[i].second;
  }

  lscg.compute(A);
  x = lscg.solve(b);

  assert(matching_pts[_m1].empty());
  for (int i = 0; i < imgs[_m1]->vertices.size() * 2; i += 2) {
    matching_pts[_m1].emplace_back(x[i], x[i + 1]);
  }
}

/***
  *
  * 图像形变:
    两个函数不会改变网格的位置, 且图像原点为(0, 0)
  *
  **/

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
 * 寻找接缝线
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
  // for (int i = 0; i < img_num; i ++) {
  //   for (int j = 0; j < matching_pts[i].size(); j ++) {
  //     LOG("%d %d %lf %lf", i, j, matching_pts[i][j].x, matching_pts[i][j].y);
  //   }
  // }
  
  for (int i = 0; i < img_num; i ++) {
    // 获得每个形变的图片, 图片的起点都是(0, 0)?? TODO
    Mat warped_image;
    Mat image_mask;
    warpImage(
      imgs[i]->vertices,
      matching_pts[i],
      imgs[i]->triangle_indices,
      imgs[i]->data,
      warped_image,
      image_mask);

    // 将每个图片/mask平移到最终位置
    Mat tmp_image = Mat::zeros(pano_size, CV_8UC4);
    Mat tmp_mask = Mat::zeros(pano_size, CV_8UC1);

    Rect2f rect = getVerticesRects(matching_pts[i]);
    warped_image.copyTo(tmp_image(rect));
    image_mask.copyTo(tmp_mask(rect));

    pano_images.emplace_back(tmp_image);
    pano_masks.emplace_back(tmp_mask);
    origin_masks.emplace_back(tmp_mask);// 存储每张图片初始的mask
  }
}

void MultiImages::getSeam() {
  Ptr<MySeamFinder> seam_finder = new MySeamFinder(10000, 1000);

  vector<UMat> pano_images_f(2);
  vector<UMat> pano_masks_gpu(2);
  // 起点位置
  vector<Point2i> img_origins;
  for (int i = 0; i < 2; i ++) {
    img_origins.emplace_back(0, 0);
  }

  // 存储中间结果
  // 预处理参考图像
  Mat result_mask, result_image;
  pano_images[reference_index].copyTo(result_image);
  pano_masks[reference_index].copyTo(result_mask);

  // 逐一计算接缝线
  for (int i = 0; i < img_num; i ++) {
    if (i == reference_index) {
      // 跳过参考图像
      continue;
    }

    // 类型转换
    int _l = 0, _r = 1;
    // 参考图像
    result_image.convertTo(pano_images_f[_l], CV_32F);
    cvtColor(pano_images_f[_l], pano_images_f[_l], CV_RGBA2RGB);
    result_mask.copyTo(pano_masks_gpu[_l]);
    // 目标图像
    pano_images[i].convertTo(pano_images_f[_r], CV_32F);
    cvtColor(pano_images_f[_r], pano_images_f[_r], CV_RGBA2RGB);
    pano_masks[i].copyTo(pano_masks_gpu[_r]);

    // 接缝线
    seam_finder->find(pano_images_f, img_origins, pano_masks_gpu);

    // TODO 同步mask的Mat和UMat
    Mat src_mask, dst_mask;
    pano_masks_gpu[_l].copyTo(src_mask);
    pano_masks_gpu[_r].copyTo(dst_mask);

    // 计算接缝线位置
    vector<Point2f> seam_pts;
    getSeamPts(src_mask, dst_mask, seam_pts);// TODO 需要进行修改

    // 临时变量
    Mat src_image, dst_image;
    result_image.copyTo(src_image);
    pano_images[i].copyTo(dst_image);

    Mat src_origin, dst_origin;
    result_mask.copyTo(src_origin);
    pano_masks[i].copyTo(dst_origin);

    // 根据计算的mask进行图像融合
    myBlending(
      src_image,
      dst_image,
      src_mask,
      dst_mask,
      src_origin,
      dst_origin,
      result_image,
      result_mask
    );

    // 描绘接缝线
    drawPoints(result_image, seam_pts);
  }
}

/***
  *
  * 图像融合
  *
  **/

void MultiImages::myBlending(
    Mat src_image, // 参考图像
    Mat dst_image, // 目标图像
    Mat src_mask,
    Mat dst_mask,
    const Mat src_origin, // 用于扩展
    const Mat dst_origin,
    Mat & result_image,
    Mat & result_mask
) {
  assert(src_image.size() == dst_image.size());

  int pano_rows = src_image.rows;
  int pano_cols = src_image.cols;

  // 图像扩充
  getExpandMat(src_image, dst_image, src_mask, dst_mask);
  getExpandMat(dst_image, src_image, dst_mask, src_mask);

  // 线性融合mask计算
  vector<Mat> blend_weight_mask(2);
  getGradualMat(
    src_image, dst_image, 
    src_origin, dst_origin, 
    src_mask, dst_mask);
  src_mask.convertTo(blend_weight_mask[0], CV_32FC1);
  dst_mask.convertTo(blend_weight_mask[1], CV_32FC1);

  // 图像起点, float
  vector<Point2f> img_origins;
  for (int i = 0; i < 2; i ++) {
    img_origins.emplace_back(0, 0);
  }

  // 不能够使用pano_images
  vector<Mat> tmp_images;
  tmp_images.emplace_back(src_image);
  tmp_images.emplace_back(dst_image);
  
  // 计算融合的图像
  bool ignore_weight_mask = false;
  result_image = Blending(
    tmp_images,
    img_origins,
    pano_size,
    blend_weight_mask,
    ignore_weight_mask);

  // 计算融合的mask
  result_mask = src_mask | dst_mask;
}

/***
  *
  * DEBUG
  *
  **/

Mat MultiImages::drawPoints(Mat _img, vector<Point2f> _points) {
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
  return result;
}