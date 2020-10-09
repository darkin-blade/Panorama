#include "MultiImages.h"

MultiImages::MultiImages() {
  img_num = 0;
}

void MultiImages::read_img(const char *img_path) {
  // 读取图片Mat
  ImageData *imageData = new ImageData();
  imageData->init_data(img_path);

  // 检查图片数量
  imgs.push_back(imageData);
  img_num ++;
  assert(img_num == imgs.size());
}

void MultiImages::do_matching() {
  // 手写配对矩阵
  assert(img_pairs.empty() == false);
  images_match_graph.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    images_match_graph[i].resize(img_num, false);
  }
  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;
    assert(m1 < m2);
    images_match_graph[m1][m2] = true;
  }

  // 初始化匹配点信息
  for (int i = 0; i < img_num; i ++) {
    imgs[i]->matching_points.resize(img_num);
    imgs[i]->homographies.resize(img_num);
  }

  // 计算匹配点
  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;
    assert(m2 > m1);

    APAP_Stitching::apap_project(feature_points[m1][m2],
                                 feature_points[m2][m1],
                                 imgs[m1]->getVertices(),
                                 imgs[m1]->matching_points[m2],
                                 imgs[m1]->homographies[m2]);

    APAP_Stitching::apap_project(feature_points[m2][m1],
                                 feature_points[m1][m2],
                                 imgs[m2]->getVertices(),
                                 imgs[m2]->matching_points[m1],
                                 imgs[m2]->homographies[m1]);
  }
  // 将各自的homography保存到multi_images
  apap_homographies.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    apap_homographies[i].resize(img_num);
    for (int j = 0; j < img_num; j ++) {
      apap_homographies[i][j] = imgs[i]->homographies[j];
    }
  }
  // 将各自的matching points保存到multi_images
  apap_matching_points.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    apap_matching_points[i].resize(img_num);
    for (int j = 0; j < img_num; j ++) {
      apap_matching_points[i][j] = imgs[i]->matching_points[j];
    }
  }

  // 所有mesh点都算作keypoint
  image_features.resize(img_num);
  image_features_mask.resize(img_num);
  keypoints_pairs.resize(img_num);
  apap_overlap_mask.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    vector<Point2f> tmp_points = imgs[i]->getVertices();
    image_features_mask[i].resize(tmp_points.size());
    keypoints_pairs[i].resize(img_num);
    for (int j = 0; j < tmp_points.size(); j ++) {
      image_features[i].keypoints.emplace_back(tmp_points[j], 0);
    }
    apap_overlap_mask[i].resize(img_num);
    for (int j = 0; j < img_num; j ++) {
      apap_overlap_mask[i][j].resize(tmp_points.size());
    }
  }

  // 记录keypoint下标的配对信息
  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;
    assert(m2 > m1);
    Mat another_img;
    vector<Point2f> tmp_points;

    // 正向配对
    tmp_points = imgs[m1]->matching_points[m2];// m1 在 m2 上的匹配点
    another_img = imgs[m2]->data;
    for (int k = 0; k < tmp_points.size(); k ++) {
      if (tmp_points[k].x >= 0
        && tmp_points[k].y >= 0
        && tmp_points[k].x <= another_img.cols
        && tmp_points[k].y <= another_img.rows) {// x对应cols, y对应rows
        // 如果对应的匹配点没有出界
        keypoints_pairs[m1][m2].emplace_back(make_pair(k, image_features[m2].keypoints.size()));
        apap_overlap_mask[m1][m2][k] = true;

        image_features_mask[m1][k] = true;// TODO 标记可行
        image_features[m2].keypoints.emplace_back(tmp_points[k], 0);
      }
    }
    // 反向配对
    tmp_points = imgs[m2]->matching_points[m1];// m1 在 m2 上的匹配点
    another_img = imgs[m1]->data;
    for (int k = 0; k < tmp_points.size(); k ++) {
      if (tmp_points[k].x >= 0
        && tmp_points[k].y >= 0
        && tmp_points[k].x <= another_img.cols
        && tmp_points[k].y <= another_img.rows) {// x对应cols, y对应rows
        // 如果对应的匹配点没有出界
        keypoints_pairs[m2][m1].emplace_back(make_pair(image_features[m1].keypoints.size(), k));
        apap_overlap_mask[m2][m1][k] = true;

        image_features_mask[m2][k] = true;// TODO 标记可行
        image_features[m1].keypoints.emplace_back(tmp_points[k], 0);
      }
    }
  }
}

vector<pair<int, int> > MultiImages::getVlfeatFeaturePairs(const int m1, const int m2) {  
  const int nearest_size = 2;
  const bool ratio_test = true;

  int size_1 = imgs[m1]->feature_points.size();
  int size_2 = imgs[m2]->feature_points.size();
  const int feature_size[2] = { size_1, size_2 };
  const int pair_match[2] = { m1, m2 };

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
      initial_indices.emplace_back(feature_pairs_result[i].feature_index[0],
          feature_pairs_result[i].feature_index[1]);
    }
  }
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

void MultiImages::getFeaturePairs() {
  // 获取feature points下标的配对信息
  // 初始化vector大小
  initial_pairs.resize(img_num);
  feature_pairs.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    initial_pairs[i].resize(img_num);
    feature_pairs[i].resize(img_num);
  }

  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;
    vector<pair<int, int> > initial_indices = getVlfeatFeaturePairs(m1, m2);

    // 将所有成功配对的特征点进行筛选
    const vector<Point2f> & m1_fpts = imgs[m1]->feature_points;
    const vector<Point2f> & m2_fpts = imgs[m2]->feature_points;
    vector<Point2f> X, Y;
    X.reserve(initial_indices.size());
    Y.reserve(initial_indices.size());
    for (int j = 0; j < initial_indices.size(); j ++) {
      const pair<int, int> it = initial_indices[j];
      X.emplace_back(m1_fpts[it.first ]);
      Y.emplace_back(m2_fpts[it.second]);
    }
    initial_pairs[m1][m2] = initial_indices;
    feature_pairs[m1][m2] = getFeaturePairsBySequentialRANSAC(X, Y, initial_indices);

    LOG("%d %d has feature pairs %ld", m1, m2, feature_pairs[m1][m2].size());

    // TODO 记录反向的pairs
    for (int k = 0; k < feature_pairs[m1][m2].size(); k ++) {
      feature_pairs[m2][m1].emplace_back(make_pair(feature_pairs[m1][m2][k].second, feature_pairs[m1][m2][k].first));
    }

    assert(feature_pairs[m1][m2].empty() == false);
  }
}

vector<vector<InterpolateVertex> > MultiImages::getInterpolateVerticesOfMatchingPoints() {
  if (mesh_interpolate_vertex_of_matching_pts.empty()) {
    mesh_interpolate_vertex_of_matching_pts.resize(img_num);
    for (int i = 0; i < mesh_interpolate_vertex_of_matching_pts.size(); i ++) {
      mesh_interpolate_vertex_of_matching_pts[i].reserve(image_features[i].keypoints.size());
      for (int j = 0; j < image_features[i].keypoints.size(); j ++) {
        mesh_interpolate_vertex_of_matching_pts[i].emplace_back(imgs[i]->getInterpolateVertex(image_features[i].keypoints[j].pt));
      }
    }
  }
  return mesh_interpolate_vertex_of_matching_pts;
}

vector<int> MultiImages::getImagesVerticesStartIndex() {
  // 每幅图片的第1个vertex的总索引
  if (images_vertices_start_index.empty()) {
    images_vertices_start_index.reserve(img_num);
    int index = 0;
    for (int i = 0; i < img_num; i ++) {
      images_vertices_start_index.emplace_back(index);
      index += imgs[i]->getVertices().size() * DIMENSION_2D;
    }
  }
  return images_vertices_start_index;
}

vector<vector<double> > MultiImages::getImagesGridSpaceMatchingPointsWeight(const double _global_weight_gamma) {
  if (_global_weight_gamma && images_polygon_space_matching_pts_weight.empty()) {
    images_polygon_space_matching_pts_weight.resize(img_num);
    const vector<vector<bool> > images_features_mask = image_features_mask;
    const vector<vector<InterpolateVertex> > mesh_interpolate_vertex_of_matching_pts = getInterpolateVerticesOfMatchingPoints();
    for (int i = 0; i < images_polygon_space_matching_pts_weight.size(); i ++) {
      const int polygons_count = (int)imgs[i]->getPolygonsIndices().size();
      vector<bool> polygons_has_matching_pts(polygons_count, false);
      for (int j = 0; j < images_features_mask[i].size(); j ++) {
        if (images_features_mask[i][j]) {
          polygons_has_matching_pts[mesh_interpolate_vertex_of_matching_pts[i][j].polygon] = true;
        }
      }
      images_polygon_space_matching_pts_weight[i].reserve(polygons_count);
      priority_queue<dijkstraNode> que;

      for (int j = 0; j < polygons_has_matching_pts.size(); j ++) {
        if (polygons_has_matching_pts[j]) {
          polygons_has_matching_pts[j] = false;
          images_polygon_space_matching_pts_weight[i].emplace_back(0);
          que.push(dijkstraNode(j, j, 0));
        } else {
          images_polygon_space_matching_pts_weight[i].emplace_back(MAXFLOAT);
        }
      }
      const vector<vector<int> > polygons_neighbors = imgs[i]->getPolygonsNeighbors();
      const vector<Point2f> polygons_center = imgs[i]->getPolygonsCenter();
      while (que.empty() == false) {
        const dijkstraNode now = que.top();
        const int index = now.pos;
        que.pop();
        if (polygons_has_matching_pts[index] == false) {
          polygons_has_matching_pts[index] = true;
          for (int j = 0; j < polygons_neighbors[index].size(); j ++) {
            const int n = polygons_neighbors[index][j];
            if (polygons_has_matching_pts[n] == false) {
              const double dis = norm(polygons_center[n] - polygons_center[now.from]);
              if (images_polygon_space_matching_pts_weight[i][n] > dis) {
                images_polygon_space_matching_pts_weight[i][n] = dis;
                que.push(dijkstraNode(now.from, n, dis));
              }
            }
          }
        }
      }
      const double normalize_inv = 1. / norm(Point2i(imgs[i]->data.cols, imgs[i]->data.rows));
      for (int j = 0; j < images_polygon_space_matching_pts_weight[i].size(); j ++) {
        images_polygon_space_matching_pts_weight[i][j] = images_polygon_space_matching_pts_weight[i][j] * normalize_inv;
      }
    }
  }
  return images_polygon_space_matching_pts_weight;
}

vector<CameraParams> MultiImages::getCameraParams() {
  // 获取图像焦距
  if (camera_params.empty()) {
    camera_params.resize(img_num);
    /*** Focal Length ***/
    assert(apap_overlap_mask.size() > 0);
    assert(apap_homographies.size() > 0);

    vector<Mat> translation_matrix;
    translation_matrix.reserve(img_num);
    for (int i = 0; i < img_num; i ++) {
      Mat T(3, 3, CV_64FC1);
      T.at<double>(0, 0) = T.at<double>(1, 1) = T.at<double>(2, 2) = 1;
      T.at<double>(0, 2) = imgs[i]->data.cols * 0.5;
      T.at<double>(1, 2) = imgs[i]->data.rows * 0.5;
      T.at<double>(0, 1) = T.at<double>(1, 0) = T.at<double>(2, 0) = T.at<double>(2, 1) = 0;
      translation_matrix.emplace_back(T);
    }
    vector<vector<double> > image_focal_candidates;
    image_focal_candidates.resize(img_num);
    for (int i = 0; i < img_num; i ++) {
      for (int j = 0; j < img_num; j ++) {
        for (int k = 0; k < apap_overlap_mask[i][j].size(); k ++) {
          if (apap_overlap_mask[i][j][k]) {
            double f0, f1;
            bool f0_ok, f1_ok;
            Mat H = translation_matrix[j].inv() * apap_homographies[i][j][k] * translation_matrix[i];
            focalsFromHomography(H / H.at<double>(2, 2),
                f0, f1, f0_ok, f1_ok);
            if (f0_ok && f1_ok) {
              image_focal_candidates[i].emplace_back(f0);
              image_focal_candidates[j].emplace_back(f1);
            }
          }
        }
      }
    }
    for (int i = 0; i < camera_params.size(); i ++) {
      if (image_focal_candidates[i].empty()) {
        camera_params[i].focal = imgs[i]->data.cols + imgs[i]->data.rows;
      } else {
        Statistics::getMedianWithoutCopyData(image_focal_candidates[i], camera_params[i].focal);
      }
    }
    /********************/
    /*** 3D Rotations ***/
    vector<vector<Mat> > relative_3D_rotations;
    relative_3D_rotations.resize(img_num);
    for (int i = 0; i < relative_3D_rotations.size(); i ++) {
      relative_3D_rotations[i].resize(img_num);
    }
    assert(image_features.size() > 0);
    assert(img_pairs.size() > 0);
    for (int i = 0; i < img_pairs.size(); i ++) {
      int m1 = img_pairs[i].first;
      int m2 = img_pairs[i].second;
      const double & focal1 = camera_params[m1].focal;
      const double & focal2 = camera_params[m2].focal;

      MatrixXd A = MatrixXd::Zero((keypoints_pairs[m1][m2].size() + keypoints_pairs[m2][m1].size()) * DIMENSION_2D, HOMOGRAPHY_VARIABLES_COUNT);
      
      // 正向
      for (int j = 0; j < keypoints_pairs[m1][m2].size(); j ++) {
        int index = j + 0;
        Point2d p1 = Point2d(image_features[m1].keypoints[keypoints_pairs[m1][m2][j].first].pt) -
          Point2d(translation_matrix[m1].at<double>(0, 2), translation_matrix[m1].at<double>(1, 2));
        Point2d p2 = Point2d(image_features[m2].keypoints[keypoints_pairs[m1][m2][j].second].pt) -
          Point2d(translation_matrix[m2].at<double>(0, 2), translation_matrix[m2].at<double>(1, 2));
        A(2*index    , 0) =  p1.x;
        A(2*index    , 1) =  p1.y;
        A(2*index    , 2) =         focal1;
        A(2*index    , 6) = -p2.x *   p1.x / focal2;
        A(2*index    , 7) = -p2.x *   p1.y / focal2;
        A(2*index    , 8) = -p2.x * focal1 / focal2;

        A(2*index + 1, 3) =  p1.x;
        A(2*index + 1, 4) =  p1.y;
        A(2*index + 1, 5) =         focal1;
        A(2*index + 1, 6) = -p2.y *   p1.x / focal2;
        A(2*index + 1, 7) = -p2.y *   p1.y / focal2;
        A(2*index + 1, 8) = -p2.y * focal1 / focal2;
      }
      // 反向
      for (int j = 0; j < keypoints_pairs[m2][m1].size(); j ++) {
        int index = j + keypoints_pairs[m1][m2].size();
        Point2d p1 = Point2d(image_features[m1].keypoints[keypoints_pairs[m2][m1][j].first].pt) -
          Point2d(translation_matrix[m1].at<double>(0, 2), translation_matrix[m1].at<double>(1, 2));
        Point2d p2 = Point2d(image_features[m2].keypoints[keypoints_pairs[m2][m1][j].second].pt) -
          Point2d(translation_matrix[m2].at<double>(0, 2), translation_matrix[m2].at<double>(1, 2));
        A(2*index    , 0) =  p1.x;
        A(2*index    , 1) =  p1.y;
        A(2*index    , 2) =         focal1;
        A(2*index    , 6) = -p2.x *   p1.x / focal2;
        A(2*index    , 7) = -p2.x *   p1.y / focal2;
        A(2*index    , 8) = -p2.x * focal1 / focal2;

        A(2*index + 1, 3) =  p1.x;
        A(2*index + 1, 4) =  p1.y;
        A(2*index + 1, 5) =         focal1;
        A(2*index + 1, 6) = -p2.y *   p1.x / focal2;
        A(2*index + 1, 7) = -p2.y *   p1.y / focal2;
        A(2*index + 1, 8) = -p2.y * focal1 / focal2;
      }

      JacobiSVD<MatrixXd, HouseholderQRPreconditioner> jacobi_svd(A, ComputeThinV);
      MatrixXd V = jacobi_svd.matrixV();
      Mat R(3, 3, CV_64FC1);
      for (int j = 0; j < V.rows(); j ++) {
        R.at<double>(j / 3, j % 3) = V(j, V.rows() - 1);
      }
      SVD svd(R, SVD::FULL_UV);
      relative_3D_rotations[m1][m2] = svd.u * svd.vt;
    }
    queue<int> que;
    vector<bool> labels(img_num, false);

    que.push(center_index);
    relative_3D_rotations[center_index][center_index] = Mat::eye(3, 3, CV_64FC1);

    while (que.empty() == false) {
      int now = que.front();
      que.pop();
      labels[now] = true;
      for (int i = 0; i < img_num; i ++) {
        if (labels[i] == false) {
          if (images_match_graph[now][i]) {
            relative_3D_rotations[i][i] = relative_3D_rotations[now][i] * relative_3D_rotations[now][now];
            que.push(i);
          }
          if (images_match_graph[i][now]) {
            relative_3D_rotations[i][i] = relative_3D_rotations[i][now].inv() * relative_3D_rotations[now][now];
            que.push(i);
          }
        }
      }
    }
    /********************/
    for (int i = 0; i < camera_params.size(); i ++) {
      camera_params[i].aspect = 1;
      camera_params[i].ppx = translation_matrix[i].at<double>(0, 2);
      camera_params[i].ppy = translation_matrix[i].at<double>(1, 2);
      camera_params[i].t = Mat::zeros(3, 1, CV_64FC1);
      camera_params[i].R = relative_3D_rotations[i][i].inv();
      camera_params[i].R.convertTo(camera_params[i].R, CV_32FC1);
    }

    Ptr<detail::BundleAdjusterBase> adjuster = makePtr<detail::BundleAdjusterReproj>();
    adjuster->setTermCriteria(TermCriteria(TermCriteria::EPS, CRITERIA_MAX_COUNT, CRITERIA_EPSILON));

    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    refine_mask(0, 0) = 1; /* (0, 0)->focal, (0, 2)->ppx, (1, 2)->ppy, (1, 1)->aspect */
    adjuster->setConfThresh(1.f);
    adjuster->setRefinementMask(refine_mask);

    // TODO 临时构建pairwise_matches
    pairwise_matches.resize(img_num * img_num);
    for (int i = 0; i < img_pairs.size(); i ++) {
      int m1 = img_pairs[i].first;
      int m2 = img_pairs[i].second;
      assert(m1 < m2);
      int m_index = m1 * img_num + m2;
      // 正向
      for (int j = 0; j < keypoints_pairs[m1][m2].size(); j ++) {
        int queryIdx = keypoints_pairs[m1][m2][j].first;
        int trainIdx = keypoints_pairs[m1][m2][j].second;
        pairwise_matches[m_index].matches.emplace_back(queryIdx, trainIdx, 0);
      }
      // 反向
      for (int j = 0; j < keypoints_pairs[m2][m1].size(); j ++) {
        int queryIdx = keypoints_pairs[m2][m1][j].first;
        int trainIdx = keypoints_pairs[m2][m1][j].second;
        pairwise_matches[m_index].matches.emplace_back(queryIdx, trainIdx, 0);
      }
      pairwise_matches[m_index].confidence  = 2.; /*** need > 1.f ***/
      pairwise_matches[m_index].src_img_idx = m1;
      pairwise_matches[m_index].dst_img_idx = m2;
      pairwise_matches[m_index].inliers_mask.resize(pairwise_matches[m_index].matches.size(), 1);// TODO
      pairwise_matches[m_index].num_inliers = (int)pairwise_matches[m_index].matches.size();
      pairwise_matches[m_index].H = apap_homographies[m1][m2].front(); /*** for OpenCV findMaxSpanningTree funtion ***/
    }

    if (!(*adjuster)(image_features, pairwise_matches, camera_params)) {
      assert(0);
    }

    Mat center_rotation_inv = camera_params[center_index].R.inv();
    for (int i = 0; i < camera_params.size(); i ++) {
      camera_params[i].R = center_rotation_inv * camera_params[i].R;
    }
    /* wave correction */
    if (WAVE_CORRECT != WAVE_X) {
      vector<Mat> rotations;
      rotations.reserve(camera_params.size());
      for (int i = 0; i < camera_params.size(); i ++) {
        rotations.emplace_back(camera_params[i].R);
      }
      waveCorrect(rotations, ((WAVE_CORRECT == WAVE_H) ? detail::WAVE_CORRECT_HORIZ : detail::WAVE_CORRECT_VERT));
      for (int i = 0; i < camera_params.size(); i ++) {
        camera_params[i].R = rotations[i];
      }
    }
    /*******************/
  }
  return camera_params;
}

vector<SimilarityElements> MultiImages::getImagesSimilarityElements() {
  if (images_similarity_elements.empty()) {
    // TODO 自定义旋转角度
    img_rotations.emplace_back(0.0);
    img_rotations.emplace_back(0.0);
    if (img_rotations.size() < img_num) {
      for (int i = img_rotations.size(); i < img_num; i ++) {
        img_rotations.emplace_back(0);// 全部置为0
      }
    }
    images_similarity_elements.reserve(img_num);

    // 通过相机焦距判断缩放比
    const vector<CameraParams> & camera_params = getCameraParams();

    for (int i = 0; i < img_num; i ++) {
      images_similarity_elements.emplace_back(fabs(camera_params[center_index].focal / camera_params[i].focal),
          img_rotations[i]);
      // similarity elements: 顺时针为正, 逆时针为负, 单位是弧度
      // rotation: 顺时针为负, 逆时针为正, 单位是弧度
    }
  }
  assert(images_similarity_elements.size() == img_num);

  return images_similarity_elements;
}

void MultiImages::warpImages() {
  // 对mesh顶点归一化(去除负值), 必须在计算图像左上角原点之前计算
  target_size = normalizeVertices(image_mesh_points);// 最终Mat大小

  vector<vector<Point2f> > vertices(image_mesh_points);

  vector<Mat> weight_mask;
  vector<Rect2f> rects = getVerticesRects<float>(vertices);// 获取每幅图片的矩形大小(height, width, x, y)


  vector<Mat> tmp_imgs;
  for (int i = 0; i < img_num; i ++) {
    tmp_imgs.push_back(imgs[i]->data);
  }

  if (true) {// linear
    weight_mask = getMatsLinearBlendWeight(tmp_imgs);// TODO
  }

  assert(vertices.size() == img_num);
  images_warped.reserve(img_num);// 图片数
  masks_warped.reserve(img_num);
  origins.reserve(img_num);
  blend_weight_mask.reserve(img_num);

  const bool isLinear = true;
  const int SCALE = pow(2, PRECISION);

  for (int i = 0; i < img_num; i ++) {
    const vector<Point2f> src_vertices = imgs[i]->getVertices();// 所有mesh点
    const vector<vector<int> > polygons_indices = imgs[i]->getPolygonsIndices();// TODO mesh点的线性索引
    const Point2f origin(rects[i].x, rects[i].y);// 矩形坐标(左上角)
    const Point2f shift(0, 0);// TODO 原值为0.5, 不知道有什么用
    vector<Mat> affine_transform;
    affine_transform.reserve(polygons_indices.size() * (imgs[i]->getTriangulationIndices().size()));// 每个(三角形)区域的单应变换矩阵
    Mat polygon_index_mask(rects[i].height + shift.y, rects[i].width + shift.x, CV_32SC1, Scalar::all(NO_GRID));// 图片每个像素对应的(三角形)区域索引
    int label = 0;
    // 计算每个网格的但应变换
    for (int j = 0; j < polygons_indices.size(); j ++) {
      for (int k = 0; k < imgs[i]->getTriangulationIndices().size(); k ++) {// 分两次填充一个矩形mesh区域
        const vector<int> index = imgs[i]->getTriangulationIndices()[k];// 每次填充矩形的一半(两个邻边加上对角线所构成的三角形部分)
        const Point2i contour[] = {
          (vertices[i][polygons_indices[j][index[0]]] - origin) * SCALE,
          (vertices[i][polygons_indices[j][index[1]]] - origin) * SCALE,
          (vertices[i][polygons_indices[j][index[2]]] - origin) * SCALE,
        };
        // 创造一个图片大小的索引矩阵, 里面填充成对应像素所在的区域的索引
        fillConvexPoly(polygon_index_mask, // 总的索引矩阵
            contour,            // pts 三角形区域
            TRIANGLE_COUNT,     // npts
            label,              // color
            LINE_AA,            // lineType = LINE_8
            PRECISION);         // shift = 0
        Point2f src[] = {
          vertices[i][polygons_indices[j][index[0]]] - origin,
          vertices[i][polygons_indices[j][index[1]]] - origin,
          vertices[i][polygons_indices[j][index[2]]] - origin
        };// mesh点经过单应变换后的坐标
        Point2f dst[] = {
          src_vertices[polygons_indices[j][index[0]]],
          src_vertices[polygons_indices[j][index[1]]],
          src_vertices[polygons_indices[j][index[2]]]
        };// mesh点原始坐标
        affine_transform.emplace_back(getAffineTransform(src, dst));
        label ++;
      }
    }

    Mat image = Mat::zeros(rects[i].height + shift.y, rects[i].width + shift.x, CV_8UC4);// 新建空图
    Mat image_mask = Mat::zeros(rects[i].height + shift.y, rects[i].width + shift.x, CV_8UC1);// 图像的mask
    Mat w_mask = Mat();

    if (isLinear) {// linear
      w_mask = Mat::zeros(image.size(), CV_32FC1);
    }

    for (int y = 0; y < image.rows; y ++) {
      for (int x = 0; x < image.cols; x ++) {
        int polygon_index = polygon_index_mask.at<int>(y, x);
        if (polygon_index != NO_GRID) {// -1
          Point2f p_f = applyTransform2x3<float>(x, y, affine_transform[polygon_index]);
          if (p_f.x >= 0 && p_f.y >= 0 &&
              p_f.x <= imgs[i]->data.cols &&
              p_f.y <= imgs[i]->data.rows) {
            Vec<uchar, 1> alpha = getSubpix<uchar, 1>(imgs[i]->alpha_mask, p_f);// TODO 透明度掩码好像没改过
            Vec3b c = getSubpix<uchar, 3>(imgs[i]->data, p_f);
            image.at<Vec4b>(y, x) = Vec4b(c[0], c[1], c[2], alpha[0]);
            image_mask.at<uchar>(y, x) = 255;// 保存图像的mask

            if (isLinear) {// linear
              w_mask.at<float>(y, x) = getSubpix<float>(weight_mask[i], p_f);
            }
          }
        }
      }
    }

    polygon_index_masks.emplace_back(polygon_index_mask);// 保存区域索引 bug
    affine_transforms.emplace_back(affine_transform);// 保存单应矩阵变换
    images_warped.emplace_back(image);
    masks_warped.emplace_back(image_mask);
    origins.emplace_back(rects[i].x, rects[i].y);
    if (isLinear) {// linear
      blend_weight_mask.emplace_back(w_mask);
    }

    LOG("%d warped", i);
  }

  // 预处理
  // 去除透明通道, 同步UMat和Mat
  for (int i = 0; i < img_num; i ++) {
    corners.emplace_back((int) origins[i].x, (int) origins[i].y);// 不要用括号把x, y括起来
    UMat tmp_img, tmp_mask;
    cvtColor(images_warped[i], tmp_img, COLOR_RGBA2RGB);// 不要使用convertTo
    masks_warped[i].copyTo(tmp_mask);// 不要使用getMat, 否则会产生关联

    gpu_images_warped.emplace_back(tmp_img);
    gpu_masks_warped.emplace_back(tmp_mask);
    LOG("%d (%d, %d)", i, corners[i].x, corners[i].y);
  }
}

void MultiImages::exposureCompensate() {
  // 曝光补偿
  Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(ExposureCompensator::GAIN);// 使用增益补偿
  compensator->feed(corners, gpu_images_warped, gpu_masks_warped);
  for (int i = 0; i < img_num; i ++) {
    compensator->apply(i, origins[i], gpu_images_warped[i], gpu_masks_warped[i]);
    cvtColor(gpu_images_warped[i], images_warped[i], COLOR_RGB2RGBA);// 同步UMat和Mat, Mat要添加透明通道
  }
}

void MultiImages::getSeam() {
  // 寻找接缝线
  // 根据像素相似度修改图像的mask

  Ptr<SeamFinder> seam_finder;
  seam_finder = makePtr<detail::VoronoiSeamFinder>();
  // seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);// 动态规划法
  // seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);// 图割法
  // 图像类型转换
  vector<UMat> images_warped_f(img_num);
  for (int i = 0; i < img_num; i ++) {
    gpu_images_warped[i].convertTo(images_warped_f[i], CV_32F);
  }
  seam_finder->find(images_warped_f, corners, gpu_masks_warped);
  // 显示结果
  char tmp_name[32];
  for (int i = 0; i < img_num; i ++) {
    sprintf(tmp_name, "mask%d", i);
    show_img(tmp_name, gpu_masks_warped[i].getMat(ACCESS_READ));
    // sprintf(tmp_name, "img%d", i);
    // show_img(tmp_name, gpu_images_warped[i].getMat(ACCESS_READ));
  }
}

Mat MultiImages::textureMapping() {

  // 将图片改成纯色
  // vector<Mat> borders_warped;
  // Vec4b colors[3] = { Vec4b(255, 0, 0, 255), Vec4b(0, 0, 255, 255), Vec4b(0, 255, 0, 255) };
  // for (int i = 0; i < img_num; i ++) {
  //   borders_warped.emplace_back(Mat::zeros(masks_warped[i].size(), CV_8UC4));
  //   int channels = masks_warped[0].channels();
  //   int rows = masks_warped[i].rows;
  //   int cols = masks_warped[i].cols * channels;
  //   if (masks_warped[0].isContinuous()) {
  //     cols *= rows;
  //     rows = 1;
  //   }
  //   for (int j = 0; j < rows; j ++) {
  //     uchar *weight_p = masks_warped[i].ptr<uchar>(j);
  //     Vec4b *border_p = borders_warped[i].ptr<Vec4b>(j);
  //     for (int k = 0; k < cols; k ++) {
  //       if (weight_p[k] == 255) {
  //         border_p[k] = colors[i % 3];
  //       }
  //     }
  //   }
  //   show_img("mask", masks_warped[i]);
  //   show_img("border", borders_warped[i]);
  // }

  // 修改blend权重
  // for (int i = 0; i < img_num; i ++) {
  //   gpu_masks_warped[i].copyTo(masks_warped[i]);
  //   if (using_seam_finder) {
  //     // blend_weight_mask.emplace_back(Mat::zeros(masks_warped[i].size(), CV_32FC1));
  //     assert(1 == masks_warped[i].channels());
  //     int rows = blend_weight_mask[i].rows;
  //     int cols = blend_weight_mask[i].cols * 1;
  //     for (int j = 0; j < rows; j ++) {
  //       uchar *mask_p = masks_warped[i].ptr<uchar>(j);
  //       float *weight_p = blend_weight_mask[i].ptr<float>(j);
  //       for (int k = 0; k < cols; k ++) {
  //         if (mask_p[k] == 255) {
  //           weight_p[k] = 12000;
  //         } else if (k > 0 && mask_p[k - 1] == 255) {
  //           weight_p[k] = 12000;
  //         } else {
  //           weight_p[k] = 0;
  //         }
  //       }
  //     }
  //   }
  //   Mat tmp_weight;
  //   blend_weight_mask[i].convertTo(tmp_weight, CV_8UC4);
  //   show_img("weight", tmp_weight);
  // }

  vector<Size> sizes;
  for (int i = 0; i < img_num; i ++) {
    sizes.emplace_back(gpu_images_warped[i].size());
  }
  // 为结果生成区域
  Size dst_sz = resultRoi(corners, sizes).size();
  float blend_width = sqrt(static_cast<float>(dst_sz.area())) * 5 / 100.f;
  Ptr<Blender> blender;
  if (true) {
    // 多频带融合
    blender = Blender::createDefault(Blender::MULTI_BAND, false);// try_cuda = false
    MultiBandBlender *mb = dynamic_cast<MultiBandBlender*>(blender.get());
    mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
  } else {
    // 羽化融合
    blender = Blender::createDefault(Blender::FEATHER);
    FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
    fb->setSharpness(1.f/blend_width);
  }
  blender->prepare(corners, sizes);
  // 纹理映射
  for (int i = 0; i < img_num; i ++) {
    // 膨胀运算
    Mat dilated_mask, seam_mask, mask_warped;
    gpu_images_warped[i].copyTo(mask_warped);
    dilate(gpu_masks_warped[i], dilated_mask, Mat());
    // 统一Mat的大小
    resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
    mask_warped = seam_mask & dilated_mask;
    // 转换图像格式
    Mat images_warped_s;
    gpu_images_warped[i].convertTo(images_warped_s, CV_16S);
    blender->feed(images_warped_s, mask_warped, corners[i]);
  }
  Mat blend_result, blend_mask;
  blender->blend(blend_result, blend_mask);
  blend_result.convertTo(blend_result, CV_8UC3);
  cvtColor(blend_result, blend_result, COLOR_RGB2RGBA);
  return blend_result;

  // return Blending(
  //     images_warped,
  //     origins,
  //     target_size,
  //     blend_weight_mask, // 最小0, 最大12000
  //     false);// 不能ignore
}
