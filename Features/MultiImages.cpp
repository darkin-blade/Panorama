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

vector<pair<int, int> > MultiImages::getOpencvFeaturePairs(const int m1, const int m2) {
  Ptr<SIFT> core = SIFT::create();
  vector<KeyPoint> key_points[2];
  Mat gray_imgs[2];

  // 计算灰色图
  cvtColor(imgs[m1]->data, gray_imgs[0], CV_BGR2GRAY);// TODO TODO TODO
  cvtColor(imgs[m2]->data, gray_imgs[1], CV_BGR2GRAY);

  // 检测特征点
  core->detect(gray_imgs[0], key_points[0]);
  core->detect(gray_imgs[1], key_points[1]);
  // 保存特征点
  for (int i = 0; i < key_points[0].size(); i ++) {
    imgs[m1]->feature_points.push_back(key_points[0][i].pt);
  }
  for (int i = 0; i < key_points[1].size(); i ++) {
    imgs[m2]->feature_points.push_back(key_points[1][i].pt);
  }

  LOG("opencv detect finish");

  Mat descriptors[2];
  core->compute(gray_imgs[0], key_points[0], descriptors[0]);
  core->compute(gray_imgs[1], key_points[1], descriptors[1]);

  LOG("opencv compute finish");

  // 特征点匹配
  vector<DMatch> feature_pairs_result;// 存储配对信息
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
  matcher->match(descriptors[0], descriptors[1], feature_pairs_result);

  LOG("opencv match finish");

  vector<pair<int, int> > initial_indices;
  for (int i = 0; i < feature_pairs_result.size(); i ++) {
    initial_indices.push_back(make_pair(feature_pairs_result[i].queryIdx, feature_pairs_result[i].trainIdx));
  }

  return initial_indices;
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
    LOG("%ld", tmp_X.size());

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
  // 初始化vector大小
  feature_pairs.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    feature_pairs[i].resize(img_num);
  }

  for (int i = 0; i < img_num; i ++) {
    for (int j = 0; j < img_num; j ++) {
      if (i == j) continue;

      // 先计算两张图的原始配对
      int m1 = i;
      int m2 = j;
#if defined(using_opencv)
      vector<pair<int, int> > initial_indices = getOpencvFeaturePairs(m1, m2);
#else
      vector<pair<int, int> > initial_indices = getVlfeatFeaturePairs(m1, m2);
#endif

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
      
      assert(feature_pairs[m1][m2].empty() == false);
    }
  }
}

Mat MultiImages::textureMapping(vector<vector<Point2f> > &_vertices,// 对应所有图片的匹配点
                                int _blend_method) {

  Size2f target_size = normalizeVertices(_vertices);// 最终Mat大小
  vector<Mat> warp_images;// 存放wrap后的Mat

  vector<Mat> weight_mask, new_weight_mask;
  vector<Point2f> origins;
  for (int i = 0; i < _vertices.size(); i ++) {
    for (int j = 0; j < _vertices[i].size(); j ++) {
      LOG("%d %f %f", i, _vertices[i][j].x, _vertices[i][j].y);
    }
  }
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
    const vector<Point2f> & src_vertices = imgs[i]->mesh_points;// 所有mesh点
    const vector<vector<int> > & polygons_indices = imgs[i]->polygons_indices;// TODO mesh点的线性索引
    const Point2f origin(rects[i].x, rects[i].y);// 矩形坐标(左上角)
    const Point2f shift(0, 0);// 原值为0.5, 不知道有什么用
    vector<Mat> affine_transforms;
    affine_transforms.reserve(polygons_indices.size() * (imgs[i]->triangulation_indices.size()));// TODO
    LOG("%f %f", rects[i].height + shift.y, rects[i].width + shift.x);
    Mat polygon_index_mask(rects[i].height + shift.y, rects[i].width + shift.x, CV_32SC1, Scalar::all(NO_GRID));
    int label = 0;
    for (int j = 0; j < polygons_indices.size(); j ++) {
      for (int k = 0; k < imgs[i]->triangulation_indices.size(); k ++) {// 分两次填充矩形区域
        const vector<int> & index = imgs[i]->triangulation_indices[k];// 每次填充矩形的一半(两个邻边加上对角线所构成的三角形部分)
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