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
      LOG("break: homography points");
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
      LOG("break: feature count");
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

  if (auto_match) {
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
      mesh_interpolate_vertex_of_matching_pts[i].reserve(keypoints[i].size());
      for (int j = 0; j < keypoints[i].size(); j ++) {
        mesh_interpolate_vertex_of_matching_pts[i].emplace_back(imgs[i]->getInterpolateVertex(keypoints[i][j]));
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
      index += imgs[i]->getMeshPoints().size() * DIMENSION_2D;
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

vector<CameraParams> MultiImages::getCameraParams() const {
  if(camera_params.empty()) {
    camera_params.resize(images_data.size());
    /*** Focal Length ***/
    const vector<vector<vector<bool> > > & apap_overlap_mask = getAPAPOverlapMask();
    const vector<vector<vector<Mat> > >  & apap_homographies = getAPAPHomographies();

    vector<Mat> translation_matrix;
    translation_matrix.reserve(images_data.size());
    for(int i = 0; i < images_data.size(); ++i) {
      Mat T(3, 3, CV_64FC1);
      T.at<double>(0, 0) = T.at<double>(1, 1) = T.at<double>(2, 2) = 1;
      T.at<double>(0, 2) = images_data[i].img.cols * 0.5;
      T.at<double>(1, 2) = images_data[i].img.rows * 0.5;
      T.at<double>(0, 1) = T.at<double>(1, 0) = T.at<double>(2, 0) = T.at<double>(2, 1) = 0;
      translation_matrix.emplace_back(T);
    }
    vector<vector<double> > image_focal_candidates;
    image_focal_candidates.resize(images_data.size());
    for(int i = 0; i < images_data.size(); ++i) {
      for(int j = 0; j < images_data.size(); ++j) {
        for(int k = 0; k < apap_overlap_mask[i][j].size(); ++k) {
          if(apap_overlap_mask[i][j][k]) {
            double f0, f1;
            bool f0_ok, f1_ok;
            Mat H = translation_matrix[j].inv() * apap_homographies[i][j][k] * translation_matrix[i];
            detail::focalsFromHomography(H / H.at<double>(2, 2),
                f0, f1, f0_ok, f1_ok);
            if(f0_ok && f1_ok) {
              image_focal_candidates[i].emplace_back(f0);
              image_focal_candidates[j].emplace_back(f1);
            }
          }
        }
      }
    }
    for(int i = 0; i < camera_params.size(); ++i) {
      if(image_focal_candidates[i].empty()) {
        camera_params[i].focal = images_data[i].img.cols + images_data[i].img.rows;
      } else {
        Statistics::getMedianWithoutCopyData(image_focal_candidates[i], camera_params[i].focal);
      }
    }
    /********************/
    /*** 3D Rotations ***/
    RED("3D rotations");
    vector<vector<Mat> > relative_3D_rotations;
    relative_3D_rotations.resize(images_data.size());
    for(int i = 0; i < relative_3D_rotations.size(); ++i) {
      relative_3D_rotations[i].resize(images_data.size());
    }
    const vector<detail::ImageFeatures> & images_features    = getImagesFeaturesByMatchingPoints();
    const vector<detail::MatchesInfo>   & pairwise_matches   = getPairwiseMatchesByMatchingPoints();
    const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
    for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
      const pair<int, int> & match_pair = images_match_graph_pair_list[i];
      const int & m1 = match_pair.first, & m2 = match_pair.second;
      const int m_index = m1 * (int)images_data.size() + m2;
      const detail::MatchesInfo & matches_info = pairwise_matches[m_index];
      const double & focal1 = camera_params[m1].focal;
      const double & focal2 = camera_params[m2].focal;

      MatrixXd A = MatrixXd::Zero(matches_info.num_inliers * DIMENSION_2D,
          HOMOGRAPHY_VARIABLES_COUNT);

      for(int j = 0; j < matches_info.num_inliers; ++j) {
        Point2d p1 = Point2d(images_features[m1].keypoints[matches_info.matches[j].queryIdx].pt) -
          Point2d(translation_matrix[m1].at<double>(0, 2), translation_matrix[m1].at<double>(1, 2));
        Point2d p2 = Point2d(images_features[m2].keypoints[matches_info.matches[j].trainIdx].pt) -
          Point2d(translation_matrix[m2].at<double>(0, 2), translation_matrix[m2].at<double>(1, 2));
        A(2*j  , 0) =  p1.x;
        A(2*j  , 1) =  p1.y;
        A(2*j  , 2) =         focal1;
        A(2*j  , 6) = -p2.x *   p1.x / focal2;
        A(2*j  , 7) = -p2.x *   p1.y / focal2;
        A(2*j  , 8) = -p2.x * focal1 / focal2;

        A(2*j+1, 3) =  p1.x;
        A(2*j+1, 4) =  p1.y;
        A(2*j+1, 5) =         focal1;
        A(2*j+1, 6) = -p2.y *   p1.x / focal2;
        A(2*j+1, 7) = -p2.y *   p1.y / focal2;
        A(2*j+1, 8) = -p2.y * focal1 / focal2;
      }
      JacobiSVD<MatrixXd, HouseholderQRPreconditioner> jacobi_svd(A, ComputeThinV);
      MatrixXd V = jacobi_svd.matrixV();
      Mat R(3, 3, CV_64FC1);
      for(int j = 0; j < V.rows(); ++j) {
        R.at<double>(j / 3, j % 3) = V(j, V.rows() - 1);
      }
      SVD svd(R, SVD::FULL_UV);
      relative_3D_rotations[m1][m2] = svd.u * svd.vt;
    }
    queue<int> que;
    vector<bool> labels(images_data.size(), false);
    const int & center_index = parameter.center_image_index;
    const vector<vector<bool> > & images_match_graph = parameter.getImagesMatchGraph();

    que.push(center_index);
    relative_3D_rotations[center_index][center_index] = Mat::eye(3, 3, CV_64FC1);

    while(que.empty() == false) {
      int now = que.front();
      que.pop();
      labels[now] = true;
      for(int i = 0; i < images_data.size(); ++i) {
        if(labels[i] == false) {
          if(images_match_graph[now][i]) {
            relative_3D_rotations[i][i] = relative_3D_rotations[now][i] * relative_3D_rotations[now][now];
            que.push(i);
          }
          if(images_match_graph[i][now]) {
            relative_3D_rotations[i][i] = relative_3D_rotations[i][now].inv() * relative_3D_rotations[now][now];
            que.push(i);
          }
        }
      }
    }
    /********************/
    for(int i = 0; i < camera_params.size(); ++i) {
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

    if (!(*adjuster)(images_features, pairwise_matches, camera_params)) {
      printError("F(getCameraParams) camera parameters adjuster failed");
    }

    Mat center_rotation_inv = camera_params[parameter.center_image_index].R.inv();
    for(int i = 0; i < camera_params.size(); ++i) {
      camera_params[i].R = center_rotation_inv * camera_params[i].R;
    }
    /* wave correction */
    if(WAVE_CORRECT != WAVE_X) {
      vector<Mat> rotations;
      rotations.reserve(camera_params.size());
      for(int i = 0; i < camera_params.size(); ++i) {
        rotations.emplace_back(camera_params[i].R);
      }
      waveCorrect(rotations, ((WAVE_CORRECT == WAVE_H) ? detail::WAVE_CORRECT_HORIZ : detail::WAVE_CORRECT_VERT));
      for(int i = 0; i < camera_params.size(); ++i) {
        camera_params[i].R = rotations[i];
      }
    }
    /*******************/
  }
  return camera_params;
}

vector<SimilarityElements> MultiImages::getImagesSimilarityElements() {// TODO
  // if (images_similarity_elements.empty()) {
  //   images_similarity_elements.resize(img_num);
  //   for (int i = 0; i < images_similarity_elements.size(); i ++) {
  //     images_similarity_elements[i].scale = 1;
  //     images_similarity_elements[i].theta = 0;
  //   }
  // }
  // return images_similarity_elements;

  if (images_similarity_elements.empty()) {
    images_similarity_elements.reserve(img_num);
    const vector<CameraParams> & camera_params = getCameraParams();
    for(int i = 0; i < images_data.size(); ++i) {
      images_similarity_elements.emplace_back(fabs(camera_params[parameter.center_image_index].focal / camera_params[i].focal),
          -getEulerZXYRadians<float>(camera_params[i].R)[2]);
    }
    double rotate_theta = parameter.center_image_rotation_angle;
    for(int i = 0; i < images_data.size(); ++i) {
      double a = (images_similarity_elements[i].theta - rotate_theta) * 180 / M_PI;
      images_similarity_elements[i].theta = normalizeAngle(a) * M_PI / 180;
    }

    const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
    const vector<vector<pair<double, double> > > & images_relative_rotation_range = getImagesRelativeRotationRange();

    switch (_global_rotation_method) {
      case GLOBAL_ROTATION_2D_METHOD:
        {
          class RotationNode {
            public:
              int index, parent;
              RotationNode(const int _index, const int _parent) {
                index = _index, parent = _parent;
              }
            private:

          };
          const double TOLERANT_THETA = TOLERANT_ANGLE * M_PI / 180;
          vector<pair<int, double> > theta_constraints;
          vector<bool> decided(images_data.size(), false);
          vector<RotationNode> priority_que;
          theta_constraints.emplace_back(parameter.center_image_index, images_similarity_elements[parameter.center_image_index].theta);
          decided[parameter.center_image_index] = true;
          priority_que.emplace_back(parameter.center_image_index, -1);
          const vector<vector<bool> > & images_match_graph = parameter.getImagesMatchGraph();
          while(priority_que.empty() == false) {
            RotationNode node = priority_que.front();
            priority_que.erase(priority_que.begin());
            if(!decided[node.index]) {
              decided[node.index] = true;
              images_similarity_elements[node.index].theta = images_similarity_elements[node.parent].theta + getImagesMinimumLineDistortionRotation(node.parent, node.index);
            }
            for(int i = 0; i < decided.size(); ++i) {
              if(!decided[i]) {
                const int e[EDGE_VERTEX_SIZE] = { node.index, i };
                for(int j = 0; j < EDGE_VERTEX_SIZE; ++j) {
                  if(images_match_graph[e[j]][e[!j]]) {
                    RotationNode new_node(i, node.index);
                    if(isRotationInTheRange<double>(0, images_similarity_elements[node.index].theta + images_relative_rotation_range[node.index][i].first  - TOLERANT_THETA,
                          images_similarity_elements[node.index].theta + images_relative_rotation_range[node.index][i].second + TOLERANT_THETA)) {
                      priority_que.insert(priority_que.begin(), new_node);
                      images_similarity_elements[i].theta = 0;
                      decided[i] = true;
                      theta_constraints.emplace_back(i, 0);
                    } else {
                      priority_que.emplace_back(new_node);
                    }
                    break;
                  }
                }
              }
            }
          }
          const int equations_count = (int)(images_match_graph_pair_list.size() + theta_constraints.size()) * DIMENSION_2D;
          SparseMatrix<double> A(equations_count, images_data.size() * DIMENSION_2D);
          VectorXd b = VectorXd::Zero(equations_count);
          vector<Triplet<double> > triplets;
          triplets.reserve(theta_constraints.size() * 2 + images_match_graph_pair_list.size() * 6);

          int equation = 0;
          for(int i = 0; i < theta_constraints.size(); ++i) {
            triplets.emplace_back(equation    , DIMENSION_2D * theta_constraints[i].first    , STRONG_CONSTRAINT);
            triplets.emplace_back(equation + 1, DIMENSION_2D * theta_constraints[i].first + 1, STRONG_CONSTRAINT);
            b[equation    ] = STRONG_CONSTRAINT * cos(theta_constraints[i].second);
            b[equation + 1] = STRONG_CONSTRAINT * sin(theta_constraints[i].second);
            equation += DIMENSION_2D;
          } 
          for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
            const pair<int, int> & match_pair = images_match_graph_pair_list[i];
            const int & m1 = match_pair.first, & m2 = match_pair.second;
            const FLOAT_TYPE & MLDR_theta = getImagesMinimumLineDistortionRotation(m1, m2);
            triplets.emplace_back(equation    , DIMENSION_2D * m1    ,  cos(MLDR_theta));
            triplets.emplace_back(equation    , DIMENSION_2D * m1 + 1, -sin(MLDR_theta));
            triplets.emplace_back(equation    , DIMENSION_2D * m2    ,               -1);
            triplets.emplace_back(equation + 1, DIMENSION_2D * m1    ,  sin(MLDR_theta));
            triplets.emplace_back(equation + 1, DIMENSION_2D * m1 + 1,  cos(MLDR_theta));
            triplets.emplace_back(equation + 1, DIMENSION_2D * m2 + 1,               -1);
            equation += DIMENSION_2D;
          }
          assert(equation == equations_count);
          A.setFromTriplets(triplets.begin(), triplets.end());
          LeastSquaresConjugateGradient<SparseMatrix<double> > lscg(A);
          VectorXd x = lscg.solve(b);

          for(int i = 0; i < images_data.size(); ++i) {
            images_similarity_elements[i].theta = atan2(x[DIMENSION_2D * i + 1], x[DIMENSION_2D * i]);
          }
        }
        break;
      case GLOBAL_ROTATION_3D_METHOD:
        {
          const int equations_count = (int)images_match_graph_pair_list.size() * DIMENSION_2D + DIMENSION_2D;
          SparseMatrix<double> A(equations_count, images_data.size() * DIMENSION_2D);
          VectorXd b = VectorXd::Zero(equations_count);
          vector<Triplet<double> > triplets;
          triplets.reserve(images_match_graph_pair_list.size() * 6 + DIMENSION_2D);

          b[0] = STRONG_CONSTRAINT * cos(images_similarity_elements[parameter.center_image_index].theta);
          b[1] = STRONG_CONSTRAINT * sin(images_similarity_elements[parameter.center_image_index].theta);
          triplets.emplace_back(0, DIMENSION_2D * parameter.center_image_index    , STRONG_CONSTRAINT);
          triplets.emplace_back(1, DIMENSION_2D * parameter.center_image_index + 1, STRONG_CONSTRAINT);
          int equation = DIMENSION_2D;
          for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
            const pair<int, int> & match_pair = images_match_graph_pair_list[i];
            const int & m1 = match_pair.first, & m2 = match_pair.second;
            const double guess_theta = images_similarity_elements[m2].theta - images_similarity_elements[m1].theta;
            FLOAT_TYPE decision_theta, weight;
            if(isRotationInTheRange(guess_theta,
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

          for(int i = 0; i < images_data.size(); ++i) {
            images_similarity_elements[i].theta = atan2(x[DIMENSION_2D * i + 1], x[DIMENSION_2D * i]);
          }
        }
        break;
      default:
        printError("F(getImagesSimilarityElements) NISwGSP_ROTATION_METHOD");
        break;
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
    const vector<Point2f> src_vertices = imgs[i]->getMeshPoints();// 所有mesh点
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