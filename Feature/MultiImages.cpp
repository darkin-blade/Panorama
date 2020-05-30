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

  LOG("starting apap");

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

    LOG("apap [%d, %d] finish", m1, m2);
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
  image_features.resize(img_num);// TODO
  keypoints_mask.resize(img_num);
  keypoints_pairs.resize(img_num);
  apap_overlap_mask.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    vector<Point2f> tmp_points = imgs[i]->getVertices();
    keypoints_mask[i].resize(tmp_points.size());
    keypoints_pairs[i].resize(img_num);
    for (int j = 0; j < tmp_points.size(); j ++) {
      image_features[i].keypoints.emplace_back(tmp_points[j], 0);// TODO keypoints
    }
    apap_overlap_mask[i].resize(img_num);
    for (int j = 0; j < img_num; j ++) {
      apap_overlap_mask[i][j].resize(tmp_points.size());
    }
  }

  // 记录keypoint下标的配对信息
  matching_indices.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    matching_indices[i].resize(img_num);
  }
  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;
    assert(m2 > m1);
    Mat another_img;
    vector<Point2f> tmp_points;

    // 正向配对
    vector<int> & forward_indices = matching_indices[m1][m2];// 记录配对信息
    tmp_points = imgs[m1]->matching_points[m2];// m1 在 m2 上的匹配点
    another_img = imgs[m2]->data;
    for (int k = 0; k < tmp_points.size(); k ++) {
      if (tmp_points[k].x >= 0
        && tmp_points[k].y >= 0
        && tmp_points[k].x <= another_img.cols
        && tmp_points[k].y <= another_img.rows) {// x对应cols, y对应rows
        // 如果对应的匹配点没有出界
        forward_indices.emplace_back(k);// 记录可行的匹配点
        
        keypoints_pairs[m1][m2].emplace_back(make_pair(k, image_features[m2].keypoints.size()));
        apap_overlap_mask[m1][m2][k] = true;

        keypoints_mask[m1][k] = true;// TODO 标记可行
        image_features[m2].keypoints.emplace_back(tmp_points[k], 0);// TODO keypoints
      }
    }
    // 反向配对
    vector<int> & backward_indices = matching_indices[m2][m1];// 记录配对信息
    tmp_points = imgs[m2]->matching_points[m1];// m1 在 m2 上的匹配点
    another_img = imgs[m1]->data;
    for (int k = 0; k < tmp_points.size(); k ++) {
      if (tmp_points[k].x >= 0
        && tmp_points[k].y >= 0
        && tmp_points[k].x <= another_img.cols
        && tmp_points[k].y <= another_img.rows) {// x对应cols, y对应rows
        // 如果对应的匹配点没有出界
        backward_indices.emplace_back(k);// 记录可行的匹配点
        
        keypoints_pairs[m2][m1].emplace_back(make_pair(image_features[m1].keypoints.size(), k));
        apap_overlap_mask[m2][m1][k] = true;

        keypoints_mask[m2][k] = true;// TODO 标记可行
        image_features[m1].keypoints.emplace_back(tmp_points[k], 0);// TODO keypoints
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
    feature_distance_set.insert(FeatureDistance(MAXFLOAT, 0, -1, -1));// TODO 存放计算结果
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
  feature_pairs.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    feature_pairs[i].resize(img_num);
  }

  // ****************************** TODO *********************************
  if (0) {
    // 需要自动检测图片配对关系
    for (int i = 0; i < img_num; i ++) {
      for (int j = i + 1; j < img_num; j ++) {
        assert(j > i);

        // 先计算两张图的原始配对
        int m1 = i;
        int m2 = j;
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
        feature_pairs[m1][m2] = getFeaturePairsBySequentialRANSAC(X, Y, initial_indices);
        // feature_pairs[m1][m2] = initial_indices;

        LOG("%d %d has feature pairs %ld", m1, m2, feature_pairs[m1][m2].size());
        if (feature_pairs[m1][m2].size() > 20) {
          LOG("%d %d can match", m1, m2);
          // 两幅图片能够配对
          img_pairs.emplace_back(make_pair(m1, m2));  

          // 记录反向的pairs
          for (int k = 0; k < feature_pairs[m1][m2].size(); k ++) {
            feature_pairs[m2][m1].emplace_back(make_pair(feature_pairs[m1][m2][k].second, feature_pairs[m1][m2][k].first));
          }

          assert(feature_pairs[m1][m2].empty() == false);
        } else {
          LOG("%d %d can't match", m1, m2);
        }
      }
    }
  } else {
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
      feature_pairs[m1][m2] = getFeaturePairsBySequentialRANSAC(X, Y, initial_indices);
      // feature_pairs[m1][m2] = initial_indices;

      LOG("%d %d has feature pairs %ld", m1, m2, feature_pairs[m1][m2].size());

      // 记录反向的pairs
      for (int k = 0; k < feature_pairs[m1][m2].size(); k ++) {
        feature_pairs[m2][m1].emplace_back(make_pair(feature_pairs[m1][m2][k].second, feature_pairs[m1][m2][k].first));
      }

      assert(feature_pairs[m1][m2].empty() == false);
    }
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
    const vector<vector<bool> > images_features_mask = keypoints_mask;// TODO
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

      MatrixXd A = MatrixXd::Zero((keypoints_pairs[m1][m2].size() + keypoints_pairs[m2][m1].size()) * DIMENSION_2D,// TODO num_inliers
          HOMOGRAPHY_VARIABLES_COUNT);
      
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
    // if (!(*adjuster)(image_features, pairwise_matches, camera_params)) {
    //   assert(0);
    // }

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

vector<vector<pair<double, double> > > & MultiImages::getImagesRelativeRotationRange() {
  if (images_relative_rotation_range.empty()) {
    images_relative_rotation_range.resize(img_num);
    for (int i = 0; i < images_relative_rotation_range.size(); i ++) {
      images_relative_rotation_range[i].resize(images_relative_rotation_range.size(), make_pair(0, 0));
    }
    assert(apap_overlap_mask.empty() == false);
    assert(apap_matching_points.empty() == false);
    for (int i = 0; i < img_pairs.size(); i ++) {
      const pair<int, int> & match_pair = img_pairs[i];
      const int & m1 = match_pair.first, & m2 = match_pair.second;
      const vector<Edge> & m1_edges = imgs[m1]->getEdges();
      const vector<Edge> & m2_edges = imgs[m2]->getEdges();
      const vector<const vector<Edge> *> & edges = { &m1_edges, &m2_edges };
      const vector<pair<int, int> > pair_index = { make_pair(m1, m2), make_pair(m2, m1) };

      vector<Point2f> mesh_pts_1 = imgs[m1]->getVertices();
      vector<Point2f> mesh_pts_2 = imgs[m2]->getVertices();
      const vector<pair<vector<Point2f> *, vector<Point2f> *> > & vertices_pair = {
        make_pair(&mesh_pts_1, &apap_matching_points[m1][m2]),
        make_pair(&mesh_pts_2, &apap_matching_points[m2][m1])
      };
      
      vector<double> positive, negative;
      const vector<bool> sign_mapping = { false, true, true, false };
      for (int j = 0; j < edges.size(); j ++) {
        for (int k = 0; k < edges[j]->size(); k ++) {
          const Edge & e = (*edges[j])[k];
          if (apap_overlap_mask[pair_index[j].first][pair_index[j].second][e.indices[0]] &&
              apap_overlap_mask[pair_index[j].first][pair_index[j].second][e.indices[1]]) {
            const Point2d a = (*vertices_pair[j].first )[e.indices[0]] - (*vertices_pair[j].first )[e.indices[1]];
            const Point2d b = (*vertices_pair[j].second)[e.indices[0]] - (*vertices_pair[j].second)[e.indices[1]];
            const double theta = acos(a.dot(b) / (norm(a) * norm(b)));
            const double direction = a.x * b.y - a.y * b.x;
            int map = ((direction > 0) << 1) + j;
            if (sign_mapping[map]) {
              positive.emplace_back( theta);
            } else {
              negative.emplace_back(-theta);
            }
          }
        }
      }
      sort(positive.begin(), positive.end());
      sort(negative.begin(), negative.end());

      if (positive.empty() == false && negative.empty() == false) {
        if (positive.back() - negative.front() < M_PI) {
          images_relative_rotation_range[m1][m2].first  = negative.front() + 2 * M_PI;
          images_relative_rotation_range[m1][m2].second = positive.back()  + 2 * M_PI;
          images_relative_rotation_range[m2][m1].first  = 2 * M_PI - positive.back();
          images_relative_rotation_range[m2][m1].second = 2 * M_PI - negative.front();
        } else {
          images_relative_rotation_range[m1][m2].first  = positive.front();
          images_relative_rotation_range[m1][m2].second = negative.back()  + 2 * M_PI;
          images_relative_rotation_range[m2][m1].first  =          - negative.back();
          images_relative_rotation_range[m2][m1].second = 2 * M_PI - positive.front();

        }
      } else if (positive.empty() == false) {
        images_relative_rotation_range[m1][m2].first  =            positive.front();
        images_relative_rotation_range[m1][m2].second =            positive.back();
        images_relative_rotation_range[m2][m1].first  = 2 * M_PI - positive.back();
        images_relative_rotation_range[m2][m1].second = 2 * M_PI - positive.front();
      } else {
        images_relative_rotation_range[m1][m2].first  =  negative.front() + 2 * M_PI;
        images_relative_rotation_range[m1][m2].second =  negative.back()  + 2 * M_PI;
        images_relative_rotation_range[m2][m1].first  = -negative.back();
        images_relative_rotation_range[m2][m1].second = -negative.front();
      }
    }
  }
  return images_relative_rotation_range;
}

double MultiImages::getImagesMinimumLineDistortionRotation(const int _from, const int _to) {
  // if (images_minimum_line_distortion_rotation.empty()) {
  //   images_minimum_line_distortion_rotation.resize(img_num);
  //   for (int i = 0; i < images_minimum_line_distortion_rotation.size(); i ++) {
  //     images_minimum_line_distortion_rotation[i].resize(img_num, std::numeric_limits<float>::max());
  //   }
  // }
  // if (images_minimum_line_distortion_rotation[_from][_to] == std::numeric_limits<float>::max()) {
  //   const vector<LineData> & from_lines   = images_data[_from].getLines();
  //   const vector<LineData> &   to_lines   = images_data[_to  ].getLines();
  //   const vector<Point2>   & from_project = getImagesLinesProject(_from, _to);
  //   const vector<Point2>   &   to_project = getImagesLinesProject(_to, _from);

  //   const vector<const vector<LineData> *> & lines    = { &from_lines,   &to_lines   };
  //   const vector<const vector<Point2  > *> & projects = { &from_project, &to_project };
  //   const vector<int> & img_indices = { _to, _from };
  //   const vector<int> sign_mapping = { -1, 1, 1, -1 };

  //   vector<pair<double, double> > theta_weight_pairs;
  //   for (int i = 0; i < lines.size(); i ++) {
  //     const int & rows = images_data[img_indices[i]].img.rows;
  //     const int & cols = images_data[img_indices[i]].img.cols;
  //     const vector<pair<Point2, Point2> > & boundary_edgs = {
  //       make_pair(Point2(0,       0), Point2(cols,    0)),
  //       make_pair(Point2(cols,    0), Point2(cols, rows)),
  //       make_pair(Point2(cols, rows), Point2(   0, rows)),
  //       make_pair(Point2(   0, rows), Point2(   0,    0))
  //     };
  //     for (int j = 0; j < lines[i]->size(); j ++) {
  //       const Point2 & p1 = (*projects[i])[EDGE_VERTEX_SIZE * j    ];
  //       const Point2 & p2 = (*projects[i])[EDGE_VERTEX_SIZE * j + 1];
  //       const bool p1_in_img = (p1.x >= 0 && p1.x <= cols && p1.y >= 0 && p1.y <= rows);
  //       const bool p2_in_img = (p2.x >= 0 && p2.x <= cols && p2.y >= 0 && p2.y <= rows);

  //       const bool p_in_img[EDGE_VERTEX_SIZE] = { p1_in_img, p2_in_img };

  //       Point2 p[EDGE_VERTEX_SIZE] = { p1, p2 };

  //       if (!p1_in_img || !p2_in_img) {
  //         vector<double> scales;
  //         for (int k = 0; k < boundary_edgs.size(); k ++) {
  //           double s1;
  //           if (isEdgeIntersection(p1, p2, boundary_edgs[k].first, boundary_edgs[k].second, &s1)) {
  //             scales.emplace_back(s1);
  //           }
  //         }
  //         assert(scales.size() <= EDGE_VERTEX_SIZE);
  //         if (scales.size() == EDGE_VERTEX_SIZE) {
  //           assert(!p1_in_img && !p2_in_img);
  //           if (scales.front() > scales.back()) {
  //             iter_swap(scales.begin(), scales.begin() + 1);
  //           }
  //           for (int k = 0; k < scales.size(); k ++) {
  //             p[k] = p1 + scales[k] * (p2 - p1);
  //           }
  //         } else if (!scales.empty()){
  //           for (int k = 0; k < EDGE_VERTEX_SIZE; k ++) {
  //             if (!p_in_img[k]) {
  //               p[k] = p1 + scales.front() * (p2 - p1);
  //             }
  //           }
  //         } else {
  //           continue;
  //         }
  //       }
  //       const Point2d a = (*lines[i])[j].data[1] - (*lines[i])[j].data[0];
  //       const Point2d b = p2 - p1;
  //       const double theta = acos(a.dot(b) / (norm(a) * norm(b)));
  //       const double direction = a.x * b.y - a.y * b.x;
  //       const int map = ((direction > 0) << 1) + i;
  //       const double b_length_2 = sqrt(b.x * b.x + b.y * b.y);
  //       theta_weight_pairs.emplace_back(theta * sign_mapping[map],
  //           (*lines[i])[j].length * (*lines[i])[j].width * b_length_2);
  //     }
  //   }
  //   Point2 dir(0, 0);
  //   for (int i = 0; i < theta_weight_pairs.size(); i ++) {
  //     const double & theta = theta_weight_pairs[i].first;
  //     dir += (theta_weight_pairs[i].second * Point2(cos(theta), sin(theta)));
  //   }
  //   images_minimum_line_distortion_rotation[_from][_to] = acos(dir.x / (norm(dir))) * (dir.y > 0 ? 1 : -1);
  //   images_minimum_line_distortion_rotation[_to][_from] = -images_minimum_line_distortion_rotation[_from][_to];
  // }
  return images_minimum_line_distortion_rotation[_from][_to];
}

vector<SimilarityElements> MultiImages::getImagesSimilarityElements() {
  if (0) {
    if (images_similarity_elements.empty()) {
      images_similarity_elements.resize(img_num);
      for (int i = 0; i < images_similarity_elements.size(); i ++) {
        images_similarity_elements[i].scale = 1;
        images_similarity_elements[i].theta = 0;
      }
    }
  } else {
    if (images_similarity_elements.empty()) {
      images_similarity_elements.reserve(img_num);
      const vector<CameraParams> & camera_params = getCameraParams();
      for (int i = 0; i < img_num; i ++) {
        images_similarity_elements.emplace_back(fabs(camera_params[center_index].focal / camera_params[i].focal),
            -getEulerZXYRadians<float>(camera_params[i].R)[2]);
      }
      double rotate_theta = 0;// TODO 自定义的参照图片旋转角度
      for (int i = 0; i < img_num; i ++) {
        double a = (images_similarity_elements[i].theta - rotate_theta) * 180 / M_PI;
        images_similarity_elements[i].theta = normalizeAngle(a) * M_PI / 180;
      }

      const vector<vector<pair<double, double> > > & images_relative_rotation_range = getImagesRelativeRotationRange();

      for (int i = 0; i < img_pairs.size(); i ++) {
        int m1 = img_pairs[i].first;
        int m2 = img_pairs[i].second;
        double theta_min = images_relative_rotation_range[m1][m2].first;
        double theta_max = images_relative_rotation_range[m1][m2].second;
        LOG("%lf %lf", theta_min, theta_max);
      }

      for (int i = 0; i < img_num; i ++) {
        LOG("%lf %lf", images_similarity_elements[i].scale, images_similarity_elements[i].theta);
      }
  
      if (1) {
        // do nothing
      } else if (0) {
        // // 2D method
        // class RotationNode {
        //   public:
        //     int index, parent;
        //     RotationNode(const int _index, const int _parent) {
        //       index = _index, parent = _parent;
        //     }
        //   private:

        // };
        // const double TOLERANT_THETA = TOLERANT_ANGLE * M_PI / 180;
        // vector<pair<int, double> > theta_constraints;
        // vector<bool> decided(img_num, false);
        // vector<RotationNode> priority_que;
        // theta_constraints.emplace_back(center_index, images_similarity_elements[center_index].theta);
        // decided[center_index] = true;
        // priority_que.emplace_back(center_index, -1);
        // while(priority_que.empty() == false) {
        //   RotationNode node = priority_que.front();
        //   priority_que.erase(priority_que.begin());
        //   if (!decided[node.index]) {
        //     decided[node.index] = true;
        //     images_similarity_elements[node.index].theta = images_similarity_elements[node.parent].theta + getImagesMinimumLineDistortionRotation(node.parent, node.index);
        //   }
        //   for (int i = 0; i < decided.size(); i ++) {
        //     if (!decided[i]) {
        //       const int e[EDGE_VERTEX_SIZE] = { node.index, i };
        //       for (int j = 0; j < EDGE_VERTEX_SIZE; j ++) {
        //         if (images_match_graph[e[j]][e[!j]]) {
        //           RotationNode new_node(i, node.index);
        //           if (isRotationInTheRange<double>(0, images_similarity_elements[node.index].theta + images_relative_rotation_range[node.index][i].first  - TOLERANT_THETA,
        //                 images_similarity_elements[node.index].theta + images_relative_rotation_range[node.index][i].second + TOLERANT_THETA)) {
        //             priority_que.insert(priority_que.begin(), new_node);
        //             images_similarity_elements[i].theta = 0;
        //             decided[i] = true;
        //             theta_constraints.emplace_back(i, 0);
        //           } else {
        //             priority_que.emplace_back(new_node);
        //           }
        //           break;
        //         }
        //       }
        //     }
        //   }
        // }
        // const int equations_count = (int)(img_pairs.size() + theta_constraints.size()) * DIMENSION_2D;
        // SparseMatrix<double> A(equations_count, img_num * DIMENSION_2D);
        // VectorXd b = VectorXd::Zero(equations_count);
        // vector<Triplet<double> > triplets;
        // triplets.reserve(theta_constraints.size() * 2 + img_pairs.size() * 6);

        // int equation = 0;
        // for (int i = 0; i < theta_constraints.size(); i ++) {
        //   triplets.emplace_back(equation    , DIMENSION_2D * theta_constraints[i].first    , STRONG_CONSTRAINT);
        //   triplets.emplace_back(equation + 1, DIMENSION_2D * theta_constraints[i].first + 1, STRONG_CONSTRAINT);
        //   b[equation    ] = STRONG_CONSTRAINT * cos(theta_constraints[i].second);
        //   b[equation + 1] = STRONG_CONSTRAINT * sin(theta_constraints[i].second);
        //   equation += DIMENSION_2D;
        // } 
        // for (int i = 0; i < img_pairs.size(); i ++) {
        //   const pair<int, int> & match_pair = img_pairs[i];
        //   const int & m1 = match_pair.first, & m2 = match_pair.second;
        //   const double & MLDR_theta = getImagesMinimumLineDistortionRotation(m1, m2);
        //   triplets.emplace_back(equation    , DIMENSION_2D * m1    ,  cos(MLDR_theta));
        //   triplets.emplace_back(equation    , DIMENSION_2D * m1 + 1, -sin(MLDR_theta));
        //   triplets.emplace_back(equation    , DIMENSION_2D * m2    ,               -1);
        //   triplets.emplace_back(equation + 1, DIMENSION_2D * m1    ,  sin(MLDR_theta));
        //   triplets.emplace_back(equation + 1, DIMENSION_2D * m1 + 1,  cos(MLDR_theta));
        //   triplets.emplace_back(equation + 1, DIMENSION_2D * m2 + 1,               -1);
        //   equation += DIMENSION_2D;
        // }
        // assert(equation == equations_count);
        // A.setFromTriplets(triplets.begin(), triplets.end());
        // LeastSquaresConjugateGradient<SparseMatrix<double> > lscg(A);
        // VectorXd x = lscg.solve(b);

        // for (int i = 0; i < img_num; i ++) {
        //   images_similarity_elements[i].theta = atan2(x[DIMENSION_2D * i + 1], x[DIMENSION_2D * i]);
        // }
      } else {
        // 3D method
        const int equations_count = (int)img_pairs.size() * DIMENSION_2D + DIMENSION_2D;
        SparseMatrix<double> A(equations_count, img_num * DIMENSION_2D);
        VectorXd b = VectorXd::Zero(equations_count);
        vector<Triplet<double> > triplets;
        triplets.reserve(img_pairs.size() * 6 + DIMENSION_2D);

        b[0] = STRONG_CONSTRAINT * cos(images_similarity_elements[center_index].theta);
        b[1] = STRONG_CONSTRAINT * sin(images_similarity_elements[center_index].theta);
        triplets.emplace_back(0, DIMENSION_2D * center_index    , STRONG_CONSTRAINT);
        triplets.emplace_back(1, DIMENSION_2D * center_index + 1, STRONG_CONSTRAINT);
        int equation = DIMENSION_2D;
        for (int i = 0; i < img_pairs.size(); i ++) {
          const pair<int, int> & match_pair = img_pairs[i];
          const int & m1 = match_pair.first, & m2 = match_pair.second;
          const double guess_theta = images_similarity_elements[m2].theta - images_similarity_elements[m1].theta;
          double decision_theta, weight;
          if (isRotationInTheRange(guess_theta,
                images_relative_rotation_range[m1][m2].first,
                images_relative_rotation_range[m1][m2].second)) {
            decision_theta = guess_theta;
            weight = LAMBDA_GAMMA;
          } else {
            decision_theta = getImagesMinimumLineDistortionRotation(m1, m2);
            weight = 1;
          }
          triplets.emplace_back(equation    , DIMENSION_2D * m1    , weight *  cos(decision_theta));
          triplets.emplace_back(equation    , DIMENSION_2D * m1 + 1, weight * -sin(decision_theta));
          triplets.emplace_back(equation    , DIMENSION_2D * m2    ,                       -weight);
          triplets.emplace_back(equation + 1, DIMENSION_2D * m1    , weight *  sin(decision_theta));
          triplets.emplace_back(equation + 1, DIMENSION_2D * m1 + 1, weight *  cos(decision_theta));
          triplets.emplace_back(equation + 1, DIMENSION_2D * m2 + 1,                       -weight);

          equation += DIMENSION_2D;
        }
        assert(equation == equations_count);
        A.setFromTriplets(triplets.begin(), triplets.end());
        LeastSquaresConjugateGradient<SparseMatrix<double> > lscg(A);
        VectorXd x = lscg.solve(b);

        for (int i = 0; i < img_num; i ++) {
          images_similarity_elements[i].theta = atan2(x[DIMENSION_2D * i + 1], x[DIMENSION_2D * i]);
        }
      }
    }
  }

  return images_similarity_elements;
}

Mat MultiImages::textureMapping(vector<vector<Point2f> > &_vertices,// 对应所有图片的匹配点
    int _blend_method) {
  Size2f target_size = normalizeVertices(_vertices);// 最终Mat大小
  vector<Mat> warp_images;// 存放wrap后的Mat

  vector<Mat> weight_mask, new_weight_mask;
  vector<Point2f> origins;
  vector<Rect2f> rects = getVerticesRects<float>(_vertices);// 获取每幅图片的矩形大小(height, width, x, y)


  vector<Mat> tmp_imgs;
  for (int i = 0; i < img_num; i ++) {
    tmp_imgs.push_back(imgs[i]->data);
  }

  if (_blend_method) {// linear
    weight_mask = getMatsLinearBlendWeight(tmp_imgs);// TODO
  }

  warp_images.reserve(_vertices.size());
  origins.reserve(_vertices.size());
  new_weight_mask.reserve(_vertices.size());

  const int NO_GRID = -1, TRIANGLE_COUNT = 3, PRECISION = 0;
  const int SCALE = pow(2, PRECISION);

  for (int i = 0; i < img_num; i ++) {
    const vector<Point2f> src_vertices = imgs[i]->getVertices();// 所有mesh点
    const vector<vector<int> > polygons_indices = imgs[i]->getPolygonsIndices();// TODO mesh点的线性索引
    const Point2f origin(rects[i].x, rects[i].y);// 矩形坐标(左上角)
    const Point2f shift(0, 0);// 原值为0.5, 不知道有什么用
    vector<Mat> affine_transforms;
    affine_transforms.reserve(polygons_indices.size() * (imgs[i]->getTriangulationIndices().size()));// TODO
    Mat polygon_index_mask(rects[i].height + shift.y, rects[i].width + shift.x, CV_32SC1, Scalar::all(NO_GRID));
    int label = 0;
    for (int j = 0; j < polygons_indices.size(); j ++) {
      for (int k = 0; k < imgs[i]->getTriangulationIndices().size(); k ++) {// 分两次填充矩形区域
        const vector<int> index = imgs[i]->getTriangulationIndices()[k];// 每次填充矩形的一半(两个邻边加上对角线所构成的三角形部分)
        const Point2i contour[] = {
          (_vertices[i][polygons_indices[j][index[0]]] - origin) * SCALE,
          (_vertices[i][polygons_indices[j][index[1]]] - origin) * SCALE,
          (_vertices[i][polygons_indices[j][index[2]]] - origin) * SCALE,
        };
        // 多边形填充
        fillConvexPoly(polygon_index_mask, // img 绘制后的图像Mat
            contour,            // pts 三角形区域
            TRIANGLE_COUNT,     // npts
            label,              // color
            LINE_AA,            // lineType = LINE_8
            PRECISION);         // shift = 0
        Point2f src[] = {
          _vertices[i][polygons_indices[j][index[0]]] - origin,
          _vertices[i][polygons_indices[j][index[1]]] - origin,
          _vertices[i][polygons_indices[j][index[2]]] - origin
        };// mesh点经过单应变换后的坐标
        Point2f dst[] = {
          src_vertices[polygons_indices[j][index[0]]],
          src_vertices[polygons_indices[j][index[1]]],
          src_vertices[polygons_indices[j][index[2]]]
        };// mesh点原始坐标
        affine_transforms.emplace_back(getAffineTransform(src, dst));
        label ++;
      }
    }

    LOG("%d affine", i);

    Mat image = Mat::zeros(rects[i].height + shift.y, rects[i].width + shift.x, CV_8UC4);// 新建空图
    Mat w_mask = Mat();

    if (_blend_method) {// linear
      w_mask = Mat::zeros(image.size(), CV_32FC1);
    }

    for (int y = 0; y < image.rows; y ++) {
      for (int x = 0; x < image.cols; x ++) {
        int polygon_index = polygon_index_mask.at<int>(y, x);
        if (polygon_index != NO_GRID) {// -1
          Point2f p_f = applyTransform2x3<float>(x, y, affine_transforms[polygon_index]);
          if (p_f.x >= 0 && p_f.y >= 0 &&
              p_f.x <= imgs[i]->data.cols &&
              p_f.y <= imgs[i]->data.rows) {
            Vec<uchar, 1> alpha = getSubpix<uchar, 1>(imgs[i]->alpha_mask, p_f);// TODO
            Vec3b c = getSubpix<uchar, 3>(imgs[i]->data, p_f);
            image.at<Vec4b>(y, x) = Vec4b(c[0], c[1], c[2], alpha[0]);

            if (_blend_method) {// linear
              w_mask.at<float>(y, x) = getSubpix<float>(weight_mask[i], p_f);
            }
          }
        }
      }
    }

    warp_images.emplace_back(image);
    origins.emplace_back(rects[i].x, rects[i].y);
    if (_blend_method) {// linear
      new_weight_mask.emplace_back(w_mask);
    }
  }

  LOG("%ld", warp_images.size());

  // return warp_images[1];

  return Blending(warp_images,
      origins,
      target_size,
      new_weight_mask,
      1 - _blend_method);// TODO
}
