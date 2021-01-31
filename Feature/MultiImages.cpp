#include "MultiImages.h"

MultiImages::MultiImages() {
  img_num = 0;
}

void MultiImages::readImg(const char *img_path) {
  // 读取原始图片Mat
  origin_data.emplace_back(imread(img_path));
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

void MultiImages::getFeaturePairs() {
  // 获取feature points下标的配对信息

  vector<pair<int, int> > initial_indices = getInitialFeaturePairs();

  // 将所有成功配对的特征点进行筛选
  const vector<Point2f> & m1_fpts = imgs[0]->feature_points;
  const vector<Point2f> & m2_fpts = imgs[1]->feature_points;
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

  LOG("feature pairs %ld", feature_pairs.size());
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
  // 初始化
  initial_pairs.clear();
  feature_pairs.clear();
  feature_points_1.clear();
  feature_points_2.clear();

  for (int i = 0; i < 2; i ++) {
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
  const vector<Point2f> & m1_fpts = imgs[0]->feature_points;
  const vector<Point2f> & m2_fpts = imgs[1]->feature_points;
  for (int i = 0; i < feature_pairs.size(); i ++) {
    const pair<int, int> it = feature_pairs[i];
    feature_points_1.emplace_back(m1_fpts[it.first ]);
    feature_points_2.emplace_back(m2_fpts[it.second]);
  }
}

void MultiImages::getMeshInfo() {
  // 初始化图像网格
  matching_pts.clear();
  vector<double> col_r, row_r;

  // double base = 0;
  // for (int i = 0; i <= 7; i ++) {
  //   col_r.emplace_back(base);
  //   base += (1 - base) / 2.5;
  // }
  // col_r.emplace_back(1);
  int mesh_size = 10;
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
  matching_pts.resize(2);
  Homographies::compute(
    feature_points_1,
    feature_points_2,
    imgs[0]->vertices,
    matching_pts[0],
    imgs[0]->homographies
  );
}

void MultiImages::similarityTransform(int _mode, double _angle) {
  // _angle必须是弧度
  int equations = feature_points_1.size();
  // 式子是特征点数目的两倍

  if (_mode == 0) {
    // 只计算平移, 未知数为(x, y)

    MatrixXd A = MatrixXd::Zero(equations * 2, 2);
    VectorXd b = VectorXd::Zero(equations * 2);
    for (int i = 0; i < equations; i ++) {
      // x + 0 = x2 - (x1 cos - y1 sin)
      // 0 + y = y2 - (x1 sin + y1 cos)
      double x1 = feature_points_1[i].x;
      double y1 = feature_points_1[i].y;
      double x2 = feature_points_2[i].x;
      double y2 = feature_points_2[i].y;
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
      double x1 = feature_points_1[i].x;
      double y1 = feature_points_1[i].y;
      double x2 = feature_points_2[i].x;
      double y2 = feature_points_2[i].y;
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
      A(i*2 + 0, 0) = feature_points_1[i].x;
      A(i*2 + 0, 1) = -feature_points_1[i].y;
      A(i*2 + 0, 2) = 1;
      b(i*2 + 0)    = feature_points_2[i].x;
      A(i*2 + 1, 0) = feature_points_1[i].y;
      A(i*2 + 1, 1) = feature_points_1[i].x;
      A(i*2 + 1, 3) = 1;
      b(i*2 + 1)    = feature_points_2[i].y;
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
  assert(matching_pts.size() == 2);
  for (int i = 0; i < imgs[0]->vertices.size(); i ++) {
    double x = imgs[0]->vertices[i].x;
    double y = imgs[0]->vertices[i].y;
    double new_x = (x * cos(rotate) - y * sin(rotate)) * scale + shift_x;
    double new_y = (x * sin(rotate) + y * cos(rotate)) * scale + shift_y;
    matching_pts[2].emplace_back(new_x, new_y);
  }
  matching_pts[3].assign(imgs[1]->vertices.begin(), imgs[1]->vertices.end());

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
  // 初始化
  pano_images.clear();
  pano_masks.clear();
  origin_masks.clear();
  
  // 获得每个形变的图片, 图片的起点都是(0, 0)
  Mat warped_image;
  Mat image_mask;
  warpImage(
    imgs[0]->vertices,
    matching_pts[0],
    imgs[0]->triangle_indices,
    imgs[0]->data,
    warped_image,
    image_mask);

  // 对所有图像的网格点归一化(去除负值)
  matching_pts[1].assign(imgs[1]->vertices.begin(), imgs[1]->vertices.end());
  Size2f tmp_size = normalizeVertices(matching_pts);
  pano_size = Size2i(ceil(tmp_size.width), ceil(tmp_size.height));

  // 将每个图片/mask平移到最终位置
  for (int i = 0; i < 2; i ++) {
    Mat tmp_image = Mat::zeros(pano_size, CV_8UC4);
    Mat tmp_mask = Mat::zeros(pano_size, CV_8UC1);
    // 计算目标矩阵
    Rect2f rect = getVerticesRects(matching_pts[i]);

    cout << rect << endl;
    if (i == 1) {
      // 参考图片
      LOG("%d %d", imgs[i]->mask.cols, imgs[i]->mask.rows);
      cvtColor(imgs[i]->data, tmp_image(rect), COLOR_RGB2RGBA);
      imgs[i]->mask.copyTo(tmp_mask(rect));
    } else {
      LOG("%d %d", image_mask.cols, image_mask.rows);
      LOG("%d %d", tmp_mask.cols, tmp_mask.rows);
      warped_image.copyTo(tmp_image(rect));
      image_mask.copyTo(tmp_mask(rect));
    }

    pano_images.emplace_back(tmp_image);
    pano_masks.emplace_back(tmp_mask);
    origin_masks.emplace_back(tmp_mask);
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

void MultiImages::repairWarpping() {
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

  // 计算接缝线质量, BUG: 出界
//   int seam_size = seam_pts.size();
//   LOG("seam pts num %d", seam_size);
//   double total_error = 0;
//   vector<Point2f> tmp_pts;
//   for (int i = 0; i < seam_size; i ++) {
//     // 在接缝线上选择合适的点
//     Point2f patch_center = seam_pts[i];
//     Rect seam_rect(patch_center + Point2f(-8, -8), patch_center + Point2f(+8, +8));

//     Mat result_patch = pano_result(seam_rect);
//     Mat patch_1 = pano_images[0](seam_rect);
//     Mat patch_2 = pano_images[1](seam_rect);
//     Mat ssam_mask = Mat(patch_1.size(), CV_8UC1, Scalar(255));

//     // 计算与两幅图片的相似度
//     Scalar ssim_1 = SSIM(result_patch, patch_1, ssam_mask, 1);
//     Scalar ssim_2 = SSIM(result_patch, patch_2, ssam_mask, 1);
//     double seam_error = max(ssim_1[0], ssim_2[0]);
//     total_error += seam_error;
//   }
//   LOG("total error %lf", total_error / seam_size);
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
  mask_result = pano_masks[0] | pano_masks[1];
  // show_img("mask", mask_result);
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
    circle(result, p, 1, color1, -1);// circle_size = 1
  }
  show_img("result", result);
}