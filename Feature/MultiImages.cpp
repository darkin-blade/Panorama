#include "MultiImages.h"

MultiImages::MultiImages() {
  img_num = 0;
}

void MultiImages::readImg(const char *img_path) {
  // 读取图片Mat
  ImageData *imageData = new ImageData();
  imageData->readImg(img_path);

  // 检查图片数量
  imgs.push_back(imageData);
  img_num ++;
  assert(img_num == imgs.size());
}

/***
  *
  * 计算特征匹配
  *
  **/

vector<pair<int, int> > MultiImages::getInitialFeaturePairs(const int m1, const int m2) {  
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
    vector<pair<int, int> > initial_indices = getInitialFeaturePairs(m1, m2);

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

/***
  *
  * 图像配准
  *
  **/

void MultiImages::rotateImage(vector<double> _angle, vector<Point2f> _src_p, vector<Point2f> & _dst_p) {
  // 旋转图片, 计算透视变换 TODO
  assert(_angle.size() == 3);
  assert(_src_p.size() == 4);
  // 计算图像中心
  double x = 0, y = 0;
  for (int i = 0; i < 4; i ++) {
    x += _src_p[i].x;
    y += _src_p[i].y;
  }
  Point2f center(x / 2, y / 2);
}

void MultiImages::getFeatureInfo() {
  for (int i = 0; i < img_num; i ++) {
    Mat grey_img;
    cvtColor(imgs[i]->data, grey_img, CV_BGR2GRAY);
    FeatureController::detect(grey_img,
                              imgs[i]->feature_points,
                              imgs[i]->descriptors);
    LOG("[picture %d] feature points: %ld", i, imgs[i]->feature_points.size());
  }

  // 特征点匹配
  getFeaturePairs();

  // 筛选所有图片的成功匹配的特征点
  feature_points.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    feature_points[i].resize(img_num);
  }
  for (int i = 0; i < img_pairs.size(); i ++) {
    int m1 = img_pairs[i].first;
    int m2 = img_pairs[i].second;
    assert(m2 > m1);

    const vector<Point2f> & m1_fpts = imgs[m1]->feature_points;
    const vector<Point2f> & m2_fpts = imgs[m2]->feature_points;
    for (int k = 0; k < feature_pairs[m1][m2].size(); k ++) {
      const pair<int, int> it = feature_pairs[m1][m2][k];
      feature_points[m1][m2].emplace_back(m1_fpts[it.first ]);
      feature_points[m2][m1].emplace_back(m2_fpts[it.second]);
    }
  }
}

void MultiImages::getMeshInfo() {
  // 初始化图像网格
  vector<double> col_r, row_r;
  for (int i = 0; i <= 20; i ++) {
    col_r.emplace_back(0.05 * i);
    row_r.emplace_back(0.05 * i);
  }
  imgs[0]->initVertices(col_r, row_r);
  col_r.clear();
  row_r.clear();
  for (int i = 0; i <= 1; i ++) {
    col_r.emplace_back(1 * i);
    row_r.emplace_back(1 * i);
  }
  imgs[1]->initVertices(col_r, row_r);
  
  // 计算网格形变
  assert(matching_pts.empty());
  matching_pts.resize(img_num * 2);
  Homographies::compute(
    feature_points[0][1],
    feature_points[1][0],
    imgs[0]->vertices,
    matching_pts[0],
    imgs[0]->homographies
  );
  matching_pts[1].assign(imgs[1]->vertices.begin(), imgs[1]->vertices.end());
}

void MultiImages::getHomographyInfo() {
  int equations = feature_points[0][1].size();
  // 式子是特征点数目的两倍, 未知数为x和y
  MatrixXd A = MatrixXd::Zero(equations * 2, 2);
  VectorXd b = VectorXd::Zero(equations * 2);
  for (int i = 0; i < equations; i ++) {
    // x + delX = x', 1delX + 0Dely = x' - x
    // y + delY = y', 0delX + 1Dely = y' - y
    A(i*2 + 0, 0) = 1;
    b(i*2 + 0)    = feature_points[1][0][i].x - feature_points[0][1][i].x;
    A(i*2 + 1, 1) = 1;
    b(i*2 + 1)    = feature_points[1][0][i].y - feature_points[0][1][i].y;
  }
  VectorXd x = A.colPivHouseholderQr().solve(b);
  LOG("x=%lf, y=%lf", x(0), x(1));

  // 进行平移
  assert(matching_pts.size() == 2 * img_num);
  for (int i = 0; i < imgs[0]->vertices.size(); i ++) {
    matching_pts[0 + img_num].emplace_back(imgs[0]->vertices[i] + Point2f(x(0), x(1)));
  }
  matching_pts[1 + img_num].assign(imgs[1]->vertices.begin(), imgs[1]->vertices.end());
}

/***
  *
  * 图像形变:
    两个函数不会改变网格的位置, 且图像原点为(0, 0)
  *
  **/

void MultiImages::warpImage(vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 三角形的线性索引
      Mat _src, Mat & _dst, Mat & _mask) {

  assert(_src_p.size() == _dst_p.size());
  bool ignore_weight_mask = false;

  Size2f target_size = normalizeVertices(_dst_p);// 归一化形变后的网格点
  Rect2f rect        = getVerticesRects(_dst_p);// 获取图片的最终矩形

  /* 计算每个三角形的仿射变换 */
  const Point2f shift(0.5, 0.5);
  Mat polygon_index_mask(rect.height + shift.y, rect.width + shift.x, CV_32SC1, Scalar::all(NO_GRID));// 记录每个像素点对应三角形的索引矩阵, 并填充初始值
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
  _dst = Mat::zeros(rect.height + shift.y, rect.width + shift.x, CV_8UC4);
  Mat weight_mask;
  // 线性融合
  weight_mask = getMatOfLinearBlendWeight(_src);// TODO
  _mask = Mat::zeros(_dst.size(), CV_32FC1);

  for (int y = 0; y < _dst.rows; y ++) {
    for (int x = 0; x < _dst.cols; x ++) {
      int polygon_index = polygon_index_mask.at<int>(y, x);
      if (polygon_index != NO_GRID) {
        Point2f p_f = applyTransform2x3<float>(x, y, affine_transform[polygon_index]);// 根据(逆向的)仿射变换, 计算目标图像上每个像素对应的原图像坐标
        if (p_f.x >= 0 && p_f.y >= 0 && p_f.x <= _src.cols && p_f.y <= _src.rows) {// 计算出来的坐标没有出界
          Vec3b c = getSubpix<uchar, 3>(_src, p_f);
          _dst.at<Vec4b>(y, x) = Vec4b(c[0], c[1], c[2], 255);// TODO 透明度通道
          _mask.at<float>(y, x) = getSubpix<float>(weight_mask, p_f);// 线性权值
        }
      }
    }
  }
}

void MultiImages::warpImage2(vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 四边形的线性索引
      Mat _src, Mat & _dst, Mat & _mask) {

  assert(_src_p.size() == _dst_p.size());
  bool ignore_weight_mask = false;

  Size2f target_size = normalizeVertices(_dst_p);// 归一化形变后的网格点
  Rect2f rect        = getVerticesRects(_dst_p);// 获取图片的最终矩形

  /* 计算每个四边形的透视变换 */
  const Point2f shift(0.5, 0.5);
  Mat polygon_index_mask(rect.height + shift.y, rect.width + shift.x, CV_32SC1, Scalar::all(NO_GRID));// 记录每个像素点对应四边形的索引矩阵, 并填充初始值
  vector<Mat> perspective_transform;
  perspective_transform.reserve(_indices.size());// 四边形数目
  
  for (int i = 0; i < _indices.size(); i ++) {
    const Point2i contour[] = {
      _dst_p[_indices[i][0]],
      _dst_p[_indices[i][1]],
      _dst_p[_indices[i][2]],
      _dst_p[_indices[i][3]],
    };
    // 往索引矩阵中填充索引值
    fillConvexPoly(
      polygon_index_mask, // 索引矩阵
      contour,            // 四边形区域
      4,                  // 四个角
      i,
      LINE_AA,
      PRECISION);
    // 计算四边形的透视变换
    Point2f src[] = {
      _dst_p[_indices[i][0]],
      _dst_p[_indices[i][1]],
      _dst_p[_indices[i][2]],
      _dst_p[_indices[i][3]],
    };
    Point2f dst[] = {
      _src_p[_indices[i][0]],
      _src_p[_indices[i][1]],
      _src_p[_indices[i][2]],
      _src_p[_indices[i][3]],
    };
    // 按顺序保存(逆向的)透视变换
    perspective_transform.emplace_back(getPerspectiveTransform(src, dst));
  }

  /* 计算目标图像 */
  _dst = Mat::zeros(rect.height + shift.y, rect.width + shift.x, CV_8UC4);
  Mat weight_mask;
  // 线性融合
  weight_mask = getMatOfLinearBlendWeight(_src);// TODO
  _mask = Mat::zeros(_dst.size(), CV_32FC1);


  for (int y = 0; y < _dst.rows; y ++) {
    for (int x = 0; x < _dst.cols; x ++) {
      int polygon_index = polygon_index_mask.at<int>(y, x);
      if (polygon_index != NO_GRID) {
        Point2f p_f = applyTransform3x3<float>(x, y, perspective_transform[polygon_index]);// 根据(逆向的)透视变换, 计算目标图像上每个像素对应的原图像坐标
        if (p_f.x >= 0 && p_f.y >= 0 && p_f.x <= _src.cols && p_f.y <= _src.rows) {// 计算出来的坐标没有出界
          Vec3b c = getSubpix<uchar, 3>(_src, p_f);
          _dst.at<Vec4b>(y, x) = Vec4b(c[0], c[1], c[2], 255);// TODO 透明度通道
          _mask.at<float>(y, x) = getSubpix<float>(weight_mask, p_f);// 线性权值
        }
      }
    }
  }
}

/***
  *
  * 图像融合
  *
  **/

void MultiImages::repairWarpping() {
  /*
    图像修正: 待添加图像=>全景图像
    计算待添加图像的网格顶点与全景图像mask的差, 并对这些剩下的网格顶点修正
  */

  Mat mask(imgs[1]->data.cols, imgs[1]->data.rows, CV_8UC1, Scalar::all(255));
  vector<bool> pts_mask(matching_pts[0].size());
  
  for (int i = 0; i < matching_pts[0].size(); i ++) {
    double x = matching_pts[0][i].x;
    double y = matching_pts[0][i].y;
    if (x < 0 || y < 0 || x > imgs[1]->data.cols || y > imgs[1]->data.rows) {// 出界
      pts_mask[i] = true;
    } else {
      uchar c = mask.at<uchar>(x, y);
      if (c == 0) {
        pts_mask[i] = true;
      } else {
        pts_mask[i] = false;
      }
    }
  }

  // 修正网格顶点
  for (int i = 0; i < matching_pts[0].size(); i ++) {
    if (pts_mask[i]) {
      double origin_x = matching_pts[0 + img_num][i].x;
      double origin_y = matching_pts[0 + img_num][i].y;
      double x = matching_pts[0][i].x - origin_x;
      double y = matching_pts[0][i].y - origin_y;
      // 修正长度
      // double delta_x = x < 0 ? -sqrt(-x) : sqrt(x);
      // double delta_y = y < 0 ? -sqrt(-y) : sqrt(y);
      double delta_x = x / 2;
      double delta_y = y / 2;
      matching_pts[0][i].x = origin_x + delta_x;
      matching_pts[0][i].y = origin_y + delta_y;
      LOG("%d %lf %lf", i, x, y);
    }
  }
}

Mat MultiImages::textureMapping(int _mode) {
  int MODE_CHOICE = 0;
  if (_mode == 1) {
    MODE_CHOICE = img_num;
  }

  // 获得每个形变的图片, 图片的起点都是(0, 0)
  vector<Mat> images_warped;
  vector<Mat> blend_weight_mask;
  for (int i = 0; i < img_num; i ++) {
    Mat warped_image;
    Mat weight_mask;
    warpImage2(
      imgs[i]->vertices,
      matching_pts[i + MODE_CHOICE],// TODO
      imgs[i]->rectangle_indices,
      imgs[i]->data,
      warped_image,
      weight_mask);
    images_warped.emplace_back(warped_image);
    blend_weight_mask.emplace_back(weight_mask);
  }

  // 对所有图像的网格点归一化(去除负值)
  pano_size = normalizeVertices(matching_pts);

  // 记录每个图像最终的起点位置
  vector<Point2f> img_origins;
  for (int i = 0; i < img_num; i ++) {
    img_origins.emplace_back(matching_pts[i + MODE_CHOICE][0]);
  }

  bool ignore_weight_mask = false;
  return Blending(
    images_warped,
    img_origins,
    pano_size,
    blend_weight_mask,
    ignore_weight_mask);
}