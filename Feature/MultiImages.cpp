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
  LOG("%d %d initial pairs %ld", m1, m2, initial_indices.size());
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

  // double base = 0;
  // for (int i = 0; i <= 7; i ++) {
  //   col_r.emplace_back(base);
  //   base += (1 - base) / 2.5;
  // }
  // col_r.emplace_back(1);
  int mesh_size = 5;
  for (int i = 0; i <= mesh_size; i ++) {
    col_r.emplace_back((double)i / mesh_size);
    row_r.emplace_back((double)i / mesh_size);
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

void MultiImages::similarityTransform(int _mode, double _angle) {
  // _angle必须是弧度
  int equations = feature_points[0][1].size();
  // 式子是特征点数目的两倍

  if (_mode == 0) {
    // 只计算平移, 未知数为(x, y)

    MatrixXd A = MatrixXd::Zero(equations * 2, 2);
    VectorXd b = VectorXd::Zero(equations * 2);
    for (int i = 0; i < equations; i ++) {
      // x + 0 = x2 - (x1 cos - y1 sin)
      // 0 + y = y2 - (x1 sin + y1 cos)
      double x1 = feature_points[0][1][i].x;
      double y1 = feature_points[0][1][i].y;
      double x2 = feature_points[1][0][i].x;
      double y2 = feature_points[1][0][i].y;
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
      double x1 = feature_points[0][1][i].x;
      double y1 = feature_points[0][1][i].y;
      double x2 = feature_points[1][0][i].x;
      double y2 = feature_points[1][0][i].y;
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
      A(i*2 + 0, 0) = feature_points[0][1][i].x;
      A(i*2 + 0, 1) = -feature_points[0][1][i].y;
      A(i*2 + 0, 2) = 1;
      b(i*2 + 0)    = feature_points[1][0][i].x;
      A(i*2 + 1, 0) = feature_points[0][1][i].y;
      A(i*2 + 1, 1) = feature_points[0][1][i].x;
      A(i*2 + 1, 3) = 1;
      b(i*2 + 1)    = feature_points[1][0][i].y;
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

  // 进行相似变换
  LOG("r=%lf, s=%lf, x=%lf, y=%lf", rotate, scale, shift_x, shift_y);
  assert(matching_pts.size() == 2 * img_num);
  for (int i = 0; i < imgs[0]->vertices.size(); i ++) {
    double x = imgs[0]->vertices[i].x;
    double y = imgs[0]->vertices[i].y;
    double new_x = (x * cos(rotate) - y * sin(rotate)) * scale + shift_x;
    double new_y = (x * sin(rotate) + y * cos(rotate)) * scale + shift_y;
    matching_pts[0 + img_num].emplace_back(new_x, new_y);
  }
  matching_pts[1 + img_num].assign(imgs[1]->vertices.begin(), imgs[1]->vertices.end());

  // 计算参考点
  double dst_x = imgs[0]->data.cols / 2;
  double dst_y = imgs[0]->data.rows / 2;
  double src_x = (dst_x * cos(rotate) - dst_y * sin(rotate)) + shift_x;
  double src_y = (dst_x * sin(rotate) + dst_y * cos(rotate)) + shift_y;
  double delta_x = dst_x - src_x;
  double delta_y = dst_y - src_y;
  // 反向旋转(顺时针旋转rotate弧度
  double vec_x = delta_x * cos(-rotate) - delta_y * sin(-rotate);
  double vec_y = delta_x * sin(-rotate) + delta_y * cos(-rotate);
  // 修正缩放比(缩小scale比例)
  vec_x /= scale;
  vec_y /= scale;
  shift_vec = Point2f(vec_x, vec_y);
  LOG("vector(%lf, %lf)", vec_x, vec_y);
}

void MultiImages::myWarping() {
  // 获得每个形变的图片, 图片的起点都是(0, 0)
  vector<Mat> images_warped;
  vector<Mat> masks_warped;

  for (int i = 0; i < img_num; i ++) {
    Mat warped_image;
    Mat img_mask;
    warpImage(
      imgs[i]->vertices,
      matching_pts[i],
      imgs[i]->triangle_indices,
      imgs[i]->data,
      warped_image,
      img_mask);
    images_warped.emplace_back(warped_image);
    masks_warped.emplace_back(img_mask);
  }

  // 对所有图像的网格点归一化(去除负值)
  pano_size = normalizeVertices(matching_pts);

  // 将每个图片/mask平移到最终位置
  pano_images.clear();
  pano_masks.clear();
  origin_masks.clear();
  origin_masks.resize(2);
  for (int i = 0; i < img_num; i ++) {
    Mat tmp_image = Mat::zeros(pano_size, CV_8UC4);
    Mat tmp_mask = Mat::zeros(pano_size, CV_8UC1);
    Mat tmp_weight = Mat::zeros(pano_size, CV_32FC1);
    // 计算目标矩阵
    Rect2f rect = getVerticesRects(matching_pts[i]);
    Mat dst_image = Mat(tmp_image, rect);
    Mat dst_mask = Mat(tmp_mask, rect);
    Mat dst_weight = Mat(tmp_weight, rect);

    images_warped[i].copyTo(dst_image);
    masks_warped[i].copyTo(dst_mask);

    pano_images.emplace_back(tmp_image);
    pano_masks.emplace_back(tmp_mask);
    tmp_mask.copyTo(origin_masks[i]);
  }

  // 比对形变后图像的相似度
  Mat intersect = pano_masks[0] & pano_masks[1];
  Scalar ssim = SSIM(pano_images[0], pano_images[1], intersect);
  LOG("score:");
  cout << ssim << endl;
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

  vector<int>  pts_distance(matching_pts[0].size());
  queue<int> q;
  
  // 过滤重合的点
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
        // 进入队列
        pts_distance[i] = 1;
        q.push(i);
      }
    }
  }

  // 对网格顶点进行bfs
  int steps[4][2] = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
  while (!q.empty()) {
    int u = q.front();
    q.pop();

    int x = u % imgs[0]->cols;
    int y = (u - x) / imgs[0]->cols;
    int depth = pts_distance[u];// 距离深度
    // LOG("%d %d %d", x, y, depth);
    
    for (int i = 0; i < 4; i ++) {
      int next_x = x + steps[i][0]; 
      int next_y = y + steps[i][1]; 
      if (next_x >= 0 && next_y >= 0
          && next_x < imgs[0]->cols && next_y < imgs[0]->rows) {
        int index = next_x + next_y * imgs[0]->cols;
        if (pts_distance[index] == 0) {
          pts_distance[index] = depth + 1;
          q.push(index);
        }
      }
    }
  }


  Point2f center(imgs[0]->data.cols / 2, imgs[0]->data.rows / 2);
  double tangent = center.x * center.x + center.y * center.y;

  // 透视点修正
  // Mat canny_img;
  // GaussianBlur(imgs[0]->grey_data, canny_img, Size(5, 5), 2);

  // Ptr<FastLineDetector> fld = createFastLineDetector(
  //   10,         // 最小长度 10
  //   1.41,          // distance 1.41421356
  //   50.0,       // canny 50
  //   50.0,       // canny 50
  //   3,          // canny 不要动 3
  //   true);
  // vector<Vec4f> lines_fld;
  // fld->detect(canny_img, lines_fld);
  // Mat line_result(imgs[0]->data);
  // fld->drawSegments(line_result, lines_fld);
  // show_img("lines", line_result);

  // 边缘修正
  // for (int i = 0; i < imgs[0]->rows; i ++) {
  //   {
  //     Point2f a = matching_pts[0][i * imgs[0]->cols + 0];
  //     Point2f b = matching_pts[0][i * imgs[0]->cols + imgs[0]->cols - 3 + 1];
  //     Point2f c = matching_pts[0][i * imgs[0]->cols + imgs[0]->cols - 3 + 2];
  //     Point2f ab = b - a;
  //     Point2f bc = c - b;
  //     double relative = fabs(ab.x*bc.x + ab.y*bc.y)/(sqrt(ab.x*ab.x + ab.y*ab.y)*sqrt(bc.x*bc.x + bc.y*bc.y));
  //     LOG("%d %lf", i, relative);
  //   }

  //   for (int j = 2; j < imgs[0]->cols; j ++) {
  //     int col = imgs[0]->cols - 1 - j;
  //     int index = i * imgs[0]->cols + col;
  //     Point2f a = matching_pts[0][index];
  //     Point2f b = matching_pts[0][index + 1];
  //     Point2f c = matching_pts[0][index + 2];
  //     Point2f ab = b - a;
  //     Point2f bc = c - b;

  //     // 修正朝向
  //     double relative = fabs(ab.x*bc.x + ab.y*bc.y)/(sqrt(ab.x*ab.x + ab.y*ab.y)*sqrt(bc.x*bc.x + bc.y*bc.y));
  //     // LOG("%d %lf", index, relative);
  //     if (relative < 0.99) {
  //       matching_pts[0][index].x = b.x - bc.x;
  //       matching_pts[0][index].y = b.y - bc.y;
  //       // matching_pts[0][index] = matching_pts[0 + img_num][index];
  //     }

  //     // 修正间距
  //     a = matching_pts[0][index];
  //     ab = b - a;
  //     Point2f _a = matching_pts[0 + img_num][index];
  //     Point2f _b = matching_pts[0 + img_num][index + 1];
  //     Point2f _ab = _b - _a;
  //     relative = fabs(sqrt(ab.x*ab.x + ab.y*ab.y) - sqrt(_ab.x*_ab.x + _ab.y*_ab.y)) / sqrt(tangent);
  //     // LOG("%d %lf", index, relative);
  //     if (relative > 0.06) {
  //       double rate = sqrt((ab.x*ab.x + ab.y*ab.y)/(_ab.x*_ab.x + _ab.y*_ab.y));
  //       matching_pts[0][index].x = b.x - ab.x / rate; 
  //       matching_pts[0][index].y = b.y - ab.y / rate; 
  //     }
  //   }
  // }

  // 按距离修正
  vector<Point2f> tmp_pts(matching_pts[0]);
  Size2f warped_size = normalizeVertices(tmp_pts);// 正偏移不能被修正
  double distance = warped_size.width * warped_size.width + warped_size.height * warped_size.height;
  double base_weight = sqrt(distance / tangent);
  LOG("over warped %lf", distance / tangent);

  for (int i = 0; i < matching_pts[0].size(); i ++) {
    if (pts_mask[i]) {
      // 计算权值
      Point2f d = imgs[0]->vertices[i] - (shift_vec + center);
      double weight = fabs(d.x * shift_vec.x + d.y * shift_vec.y) / tangent;// 计算偏差相对图像的比例

      double origin_x = matching_pts[0 + img_num][i].x;
      double origin_y = matching_pts[0 + img_num][i].y;
      double x = matching_pts[0][i].x - origin_x;
      double y = matching_pts[0][i].y - origin_y;
      // 修正长度
      weight = exp(weight) * base_weight * base_weight * base_weight;
      // weight = 2;
      double delta_x = x / weight;
      double delta_y = y / weight;
      LOG("%d %lf", i, weight);
      matching_pts[0][i].x = origin_x + delta_x;
      matching_pts[0][i].y = origin_y + delta_y;
    }
  }
}

void MultiImages::myBlending() {
  // pano_result = Mat::zeros(pano_size, CV_8UC4);
  // for (int i = 0; i < img_num; i ++) {
  //   Mat dst_image = Mat(pano_result, Rect(0, 0, pano_images[i].cols, pano_images[i].rows));
  //   pano_images[i].copyTo(dst_image, pano_masks[i]);
  // }

  int pano_rows = pano_masks[0].rows;
  int pano_cols = pano_masks[0].cols;

  // 图像扩充
  getExpandMat(pano_images[0], pano_masks[0], pano_masks[1]);
  getExpandMat(pano_images[1], pano_masks[1], pano_masks[0]);
  // 线性融合mask计算
  getGradualMat(
    pano_images[0], pano_images[1], 
    origin_masks[0], origin_masks[1], 
    pano_masks[0], pano_masks[1]);

  vector<Point2f> img_origins;
  blend_weight_mask.clear();
  blend_weight_mask.resize(2);
  for (int i = 0; i < img_num; i ++) {
    img_origins.emplace_back(0, 0);
    // show_img(pano_masks[i], "mask %d", i);
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
 * 寻找接缝线
 *
 **/

void MultiImages::getMask() {
  // 计算形变之后特征点的位置
  assert(img_num == 2);
  vector<vector<Point2f> > pano_feature_points(img_num);
  for (int i = 0; i < img_num; i ++) {
    warpPoints(
      imgs[i]->vertices,
      matching_pts[i],
      imgs[i]->triangle_indices,
      feature_points[i][1 - i],
      pano_feature_points[i]);
    // 描绘最终特征点
    drawPoints(pano_images[i], pano_feature_points[i]);
  }

  // 1 计算两个pano mask的交集
  Mat common_mask = pano_masks[0] & pano_masks[1];
  vector<Mat> masks(2);
  masks[0] = pano_masks[0] ^ common_mask;
  masks[1] = pano_masks[1] ^ common_mask;

  // 2 连接每个特征点最近的一个特征点
  int points_num = pano_feature_points[0].size();
  for (int i = 0; i < points_num; i ++) {
    int nearst = i;
    double min_dis = 9999999;
    for (int j = 0; j < i; j ++) {
      Point2f d = pano_feature_points[0][i] - pano_feature_points[0][j];
      double dis = d.x * d.x + d.y * d.y;
      if (dis < min_dis) {
        nearst = j;
        min_dis = dis;
      }
    }
    circle(masks[0], pano_feature_points[0][i], 50, Scalar(255), -1);
    circle(masks[1], pano_feature_points[1][i], 50, Scalar(255), -1);
    line(masks[0], pano_feature_points[0][i], pano_feature_points[0][nearst], Scalar(255), 10, LINE_AA);
    line(masks[1], pano_feature_points[1][i], pano_feature_points[1][nearst], Scalar(255), 10, LINE_AA);
    
    circle(common_mask, pano_feature_points[0][i], 50, Scalar(0), -1);
    circle(common_mask, pano_feature_points[1][i], 50, Scalar(0), -1);
  }
}

void MultiImages::getSeam() {
  // Ptr<SeamFinder> seam_finder;
  // seam_finder = makePtr<detail::VoronoiSeamFinder>();
  // seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);// 动态规划法
  // seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);// 图割法
  Ptr<MySeamFinder> seam_finder = new MySeamFinder(10000, 1000);

  // 图像类型转换
  vector<UMat> pano_images_f(img_num);
  vector<UMat> pano_masks_gpu(img_num);
  for (int i = 0; i < img_num; i ++) {
    pano_images[i].convertTo(pano_images_f[i], CV_32F);
    cvtColor(pano_images_f[i], pano_images_f[i], CV_RGBA2RGB);
    pano_masks[i].copyTo(pano_masks_gpu[i]);
  }

  // 记录每个图像最终的起点位置
  vector<Point2i> img_origins;
  for (int i = 0; i < img_num; i ++) {
    img_origins.emplace_back(0, 0);
  }
  seam_finder->find(pano_images_f, img_origins, pano_masks_gpu);

  // 同步Mat和UMat
  for (int i = 0; i < img_num; i ++) {
    pano_masks_gpu[i].copyTo(pano_masks[i]);
  }

  myBlending();
}

/***
  *
  * DEBUG
  *
  **/

void MultiImages::drawPoints(Mat _img, vector<Point2f> _points) {
  Mat result = _img.clone();
  for (int i = 0; i < _points.size(); i ++) {
    Point2f p = _points[i];
    Scalar color1(255, 0, 255, 255);
    circle(result, p, CIRCLE_SIZE, color1, -1);
  }
  show_img("result", result);
}