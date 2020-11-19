#include "Translate.h"

Translate::Translate(int _useless) {
  // 初始化内参矩阵
  K = Mat::zeros(3, 3, CV_64FC1);
  K.at<double>(0, 0) = 646.3999802324365;// fx
  K.at<double>(0, 1) = 0; 
  K.at<double>(0, 2) = 439.6819232174948;// cx
  K.at<double>(1, 0) = 0; 
  K.at<double>(1, 1) = 695.073515875543;// fy
  K.at<double>(1, 2) = 395.4597934067736;// cy
  K.at<double>(2, 0) = 0; 
  K.at<double>(2, 1) = 0; 
  K.at<double>(2, 2) = 1; 
}

void Translate::init(vector<Mat> imgs) {
  // 清空所有数据
  imgRGBA.clear();
  imgGray.clear();

  // 处理成灰度图
  imgNum = imgs.size();
  for (int i = 0; i < imgNum; i ++) {
    Mat tmpImg = imgs[i];

    // 图像放缩
    double imgSize = tmpImg.rows * tmpImg.cols;
    if (imgSize > SIZE_SMALL) {
      double scale = sqrt(SIZE_SMALL / imgSize);
      resize(tmpImg, tmpImg, Size(), scale, scale);
    }

    // 保存rgba和灰度图
    Mat tmpRGBA, tmpGray;
    if (tmpImg.channels() == 3) {
      cvtColor(tmpImg, tmpRGBA, CV_RGB2RGBA);
      cvtColor(tmpImg, tmpGray, CV_RGB2GRAY);
    } else if (tmpImg.channels() == 4) {
      tmpImg.copyTo(tmpRGBA);
      cvtColor(tmpImg, tmpGray, CV_RGBA2GRAY);
    } else {
      LOG("invalid img");
      assert(0);
    }
    imgRGBA.emplace_back(tmpRGBA);
    imgGray.emplace_back(tmpGray);
  }
}

void Translate::init(vector<Mat> imgs, vector<vector<double> > _rotations) {
  // 读取图片
  init(imgs);

  angles.clear();

  // 计算旋转矩阵
  assert(_rotations.size() == imgNum);
  for (int i = 0; i < imgNum; i ++) {
    // 将欧拉角转换成旋转矩阵
    Mat R_x = Mat::zeros(3, 3, CV_64FC1);
    Mat R_y = Mat::zeros(3, 3, CV_64FC1);
    Mat R_z = Mat::zeros(3, 3, CV_64FC1);
    double alpha = _rotations[i][0];
    double beta  = _rotations[i][1];
    double gamma = _rotations[i][2];

    R_x.at<double>(0, 0) = 1;
    R_x.at<double>(1, 1) = cos(alpha);
    R_x.at<double>(1, 2) = -sin(alpha);
    R_x.at<double>(2, 1) = sin(alpha);
    R_x.at<double>(2, 2) = cos(alpha);

    R_y.at<double>(0, 0) = cos(beta);
    R_y.at<double>(0, 2) = sin(beta);
    R_y.at<double>(1, 1) = 1;
    R_y.at<double>(2, 0) = -sin(beta);
    R_y.at<double>(2, 2) = cos(beta);

    R_z.at<double>(0, 0) = cos(gamma);
    R_z.at<double>(0, 1) = -sin(gamma);
    R_z.at<double>(1, 0) = sin(gamma);
    R_z.at<double>(1, 1) = cos(gamma);
    R_z.at<double>(2, 2) = 1;

    // 计算旋转矩阵: https://zhuanlan.zhihu.com/p/144032401
    angles.emplace_back(R_x * R_y * R_z);
  }
}

void Translate::computeTranslate(int _m1, int _m2) {
  // 计算两张图片的平移

  // drawFeature(_m1, _m2);

  LOG("%d %d matches", _m1, _m2);
  // 筛选特征点对
  vector<Point2f> X, Y;// 筛选后的特征点
  for (int k = 0; k < indices[_m1][_m2].size(); k ++) {
    int src = indices[_m1][_m2][k].first;
    int dst = indices[_m1][_m2][k].second;
    X.emplace_back(origin_features[_m1][src]);
    Y.emplace_back(origin_features[_m2][dst]);
  }

  // 计算旋转矩阵
  R = angles[_m2] * angles[_m1].t();
  // cout << R << endl;

  // 坐标齐次化
  Mat points1, points2;
  homogenization(X, points1);
  homogenization(Y, points2);
  // 将像素坐标转换至相机坐标系
  points1 = K.inv() * points1;
  points2 = K.inv() * points2;

  // 列方程
  points1 = points1.t();
  int equations = points2.cols;
  Mat tmpA = Mat::zeros(equations + 1, 3, CV_64FC1);// TODO 去除奇异解
  for (int i = 0; i < equations; i ++) {
    Mat tmp1 = points1.row(i);
    Mat tmp2 = points2.col(i);
    tmp2 = R * tmp2;
    double a = tmp1.at<double>(0, 0);
    double b = tmp1.at<double>(0, 1);
    double c = tmp1.at<double>(0, 2);
    double d = tmp2.at<double>(0, 0);
    double e = tmp2.at<double>(1, 0);
    double f = tmp2.at<double>(2, 0);
    tmpA.at<double>(i, 0) = c*e - b*f;
    tmpA.at<double>(i, 1) = a*f - c*d;
    tmpA.at<double>(i, 2) = b*d - a*e;
  }
  MatrixXd A(equations, 3);
  cv2eigen(tmpA, A);
  // 去除奇异解
  A(equations, 0) = 1;
  VectorXd b(equations + 1);
  for (int i = 0; i < equations; i ++) {
    b(i) = 0;
  }
  b(equations) = 1;

  VectorXd x = A.colPivHouseholderQr().solve(b);
  // cout << A << endl;
  // cout << b << endl;
  // cout << x << endl;

  /*
           0 -z  y
      T =  z  0 -x
          -y  x  0
  */

  // T = E * R.t();
  // cout << T << endl;
  t = Mat::zeros(3, 1, R.type());
  t.at<double>(0, 0) = x(0);
  t.at<double>(1, 0) = x(1);
  t.at<double>(2, 0) = x(2);

  // 对解进行筛选: 对t进行符号判断
  selectSolution(X, Y);
  // officialResult(X, Y);
  cout << t << endl;

  // 5 保存无量纲的结果
  if (translations.empty()) {
    translations.resize(imgNum);
    for (int i = 0; i < imgNum; i ++) {
      translations[i].resize(imgNum);
    }
  }
  if (rotations.empty()) {
    rotations.resize(imgNum);
    for (int i = 0; i < imgNum; i ++) {
      rotations[i].resize(imgNum);
    }
  }
  translations[_m1][_m2] = t * 1.0;
  translations[_m2][_m1] = -t * 1.0;
  rotations[_m1][_m2]    = R * 1.0;
  rotations[_m2][_m1]    = R.t() * 1.0;
}

void Translate::computeDistance(int _m1, int _m2, int _m3) {
  // 求解(_m1, _m2) = a(_m2, _m3) + b(_m1, _m3)
  MatrixXd A(3, 2);
  VectorXd b(3);
  A(0, 0) = translations[_m2][_m3].at<double>(0, 0);
  A(1, 0) = translations[_m2][_m3].at<double>(1, 0);
  A(2, 0) = translations[_m2][_m3].at<double>(2, 0);
  A(0, 1) = translations[_m1][_m3].at<double>(0, 0);
  A(1, 1) = translations[_m1][_m3].at<double>(1, 0);
  A(2, 1) = translations[_m1][_m3].at<double>(2, 0);
  b(0)    = translations[_m1][_m2].at<double>(0, 0);
  b(1)    = translations[_m1][_m2].at<double>(1, 0);
  b(2)    = translations[_m1][_m2].at<double>(2, 0);
  VectorXd x = A.colPivHouseholderQr().solve(b);
  double rate1 = x(0);
  double rate2 = x(1);

  // 进行缩放
  LOG("(%d,%d) = %lf(%d,%d) + %lf(%d,%d)", _m1, _m2, rate1, _m2, _m3, rate2, _m1, _m3);
  LOG("%d %d t:", _m2, _m3);
  cout << translations[_m2][_m3] * rate1 << endl;
  LOG("%d %d t:", _m1, _m3);
  cout << translations[_m1][_m3] * rate2 << endl;
  translations[_m2][_m3] *= rate1;
  translations[_m3][_m2] *= -rate1;
  translations[_m1][_m2] *= rate2;
  translations[_m2][_m1] *= -rate2;
}

void Translate::computeOrigin(vector<pair<int, int> > _img_pairs) {
  // 计算每幅图像的相对原点
  computeDistance(0, 1, 2);
  computeDistance(0, 2, 3);
  computeDistance(0, 1, 2);
  computeDistance(0, 2, 3);
}

void Translate::pixel2Cam(InputArray _src, Mat & _dst) {
  // TODO 将像素坐标转换为相机坐标
  _src.getMat().convertTo(_dst, CV_64F);
  int npoints = _dst.checkVector(2);
  if (_dst.channels() > 1) {
    _dst = _dst.reshape(1, npoints);
  }
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);
  _dst.col(0) = (_dst.col(0) - cx) / fx;
  _dst.col(1) = (_dst.col(1) - cy) / fy;
  _dst = _dst.t();
}

void Translate::homogenization(InputArray _src, Mat & _dst) {
  // TODO 坐标齐次化
  Mat src;
  _src.getMat().convertTo(src, CV_64F);
  int npoints = src.checkVector(2);
  if (src.channels() > 1) {
    src = src.reshape(1, npoints);
  }

  int dimension = src.cols;// 坐标的维度
  _dst = Mat::ones(src.rows, dimension + 1, src.type());// 对于单通道, 所有元素为1
  _dst(Range::all(), Range(0, dimension)) = src * 1.0;
  _dst = _dst.t();
}

void Translate::selectSolution(InputArray _points1, InputArray _points2) {
  // 初始化特征点对
  Mat points1, points2;
  pixel2Cam(_points1, points1);
  pixel2Cam(_points2, points2);

  // 初始化
  Mat P0 = Mat::eye(3, 4, R.type());
  Mat P1(3, 4, R.type()), P2(3, 4, R.type());
  P1(Range::all(), Range(0, 3)) = R * 1.0;
  P1.col(3) = t * 1.0;
  P2(Range::all(), Range(0, 3)) = R * 1.0;
  P2.col(3) = -t * 1.0;
  Mat Q;
  double distanceThresh = 50;

  // 第一组解
  triangulatePoints(P0, P1, points1, points2, Q);
  Q.convertTo(Q, CV_64FC1);
  Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask1 = (Q.row(2) < distanceThresh) & mask1;
  Q = P1 * Q;
  mask1 = (Q.row(2) > 0) & mask1;
  mask1 = (Q.row(2) < distanceThresh) & mask1;

  // 第二组解
  triangulatePoints(P0, P2, points1, points2, Q);
  Q.convertTo(Q, CV_64FC1);
  Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask2 = (Q.row(2) < distanceThresh) & mask2;
  Q = P2 * Q;
  mask2 = (Q.row(2) > 0) & mask2;
  mask2 = (Q.row(2) < distanceThresh) & mask2;

  // 计算景深为正的点
  int good1 = countNonZero(mask1);
  int good2 = countNonZero(mask2);
  if (good1 < good2) {
    t = -t;
  }
}

void Translate::officialResult(InputArray _points1, InputArray _points2) {
  // opencv官方结果作比较
  // Mat R1, R2;
  // decomposeEssentialMat(E, R1, R2, t);
  // cout << R1 << endl;
  // cout << R2 << endl;
  Mat tmpE, tmpR, tmpt;

  // 正向
  // LOG("forward");
  tmpE = findEssentialMat(_points1, _points2, K);
  recoverPose(tmpE, _points1, _points2, K, tmpR, tmpt);
  // cout << tmpR << endl;
  // cout << tmpt << endl;
  // TODO 保存结果
  E = tmpE;
  R = tmpR;
  t = tmpt;
  // 反向
  // LOG("backward");
  // tmpE = findEssentialMat(_points2, _points1, K);
  // recoverPose(tmpE, _points2, _points1, K, tmpR, tmpt);
  // cout << tmpR << endl;
  // cout << tmpt << endl;
}

void Translate::getFeaturePairs() {
  // 清空数据
  origin_features.clear();
  descriptors.clear();

  origin_features.resize(imgNum);
  descriptors.resize(imgNum);

  // 特征检测
  for (int i = 0; i < imgNum; i ++) {
    getDescriptors(imgGray[i], origin_features[i], descriptors[i]);
    LOG("image %d feature points %ld", i, origin_features[i].size());
  }

  initial_indices.clear();
  indices.clear();
  initial_indices.resize(imgNum);
  indices.resize(imgNum);
  for (int i = 0; i < imgNum; i ++) {
    initial_indices[i].resize(imgNum);
    indices[i].resize(imgNum);
  }
  // 初步匹配
  for (int i = 0; i < imgNum; i ++) {
    for (int j = i + 1; j < imgNum; j ++) {
      initial_indices[i][j] = getInitialFeaturePairs(i, j);
      LOG("%d %d has initial pairs %ld", i, j, initial_indices[i][j].size());
      
      // RANSAC 筛选
      vector<Point2f> X, Y;
      X.reserve(initial_indices[i][j].size());
      Y.reserve(initial_indices[i][j].size());
      for (int k = 0; k < initial_indices[i][j].size(); k ++) {
        const pair<int, int> tmpPair = initial_indices[i][j][k];
        X.emplace_back(origin_features[i][tmpPair.first ]);
        Y.emplace_back(origin_features[j][tmpPair.second]);
      }

      indices[i][j] = getFeaturePairsBySequentialRANSAC(X, Y, initial_indices[i][j]);
      // 反向配对
      for (int k = 0; k < indices[i][j].size(); k ++) {
        indices[j][i].emplace_back(make_pair(indices[i][j][k].second, indices[i][j][k].first));
      }
      // 将反向配对进行排序
      sort(indices[j][i].begin(), indices[j][i].end(), my_cmp);
      LOG("%d %d has feature pairs %ld", i, j, indices[i][j].size());
    }
  }
}

vector<pair<int, int> > Translate::getInitialFeaturePairs(int _m1, int _m2) {  
  const int nearest_size = 2;
  const bool ratio_test = true;


  int size_1 = origin_features[_m1].size();
  int size_2 = origin_features[_m2].size();
  const int feature_size[2] = { size_1, size_2 };

  const int another_feature_size = feature_size[1];
  const int nearest_k = min(nearest_size, another_feature_size);// 只可能为0, 1, 2

  // 对每个点计算最相近的特征点
  vector<FeatureDistance> feature_pairs_result;
  for (int f1 = 0; f1 < feature_size[0]; f1 ++) {
    set<FeatureDistance> feature_distance_set;// set 中每个元素都唯一, 降序
    feature_distance_set.insert(FeatureDistance(MAXFLOAT, 0, -1, -1));// 存放计算结果
    for (int f2 = 0; f2 < feature_size[1]; f2 ++) {
      const double dist = getDistance(descriptors[_m1][f1], descriptors[_m2][f2],
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
  vector<pair<int, int> > result;
  result.reserve(feature_pairs_result.size());
  for (int i = 0; i < feature_pairs_result.size(); i ++) {
    if (feature_pairs_result[i].distance < OUTLIER_THRESHOLD) {// 如果没有超出范围
      result.emplace_back(feature_pairs_result[i].feature_index[0],
          feature_pairs_result[i].feature_index[1]);
    }
  }
  return result;
}

vector<pair<int, int> > Translate::getFeaturePairsBySequentialRANSAC(
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

    tmp_X = next_X;
    tmp_Y = next_Y;
  }

  vector<pair<int, int> > result;
  for (int i = 0; i < final_mask.size(); i ++) {
    if (final_mask[i]) {
      result.emplace_back(_initial_indices[i]);
    }
  }
  return result;
}

void Translate::getDescriptors(
    const Mat & _grey_img,
    vector<Point2f> & _feature_points,
    vector<vector<Mat> > & _feature_descriptors) {
  Mat grey_img_float = _grey_img.clone();
  grey_img_float.convertTo(grey_img_float, CV_32FC1);

  const int  width = _grey_img.cols;
  const int height = _grey_img.rows;

  VlSiftFilt * vlSift = vl_sift_new(width, height,
      log2(min(width, height)),  // noctaves
      3,                         // nlevels
      2);                        // o_min

  vl_sift_set_peak_thresh(vlSift, 0.);
  vl_sift_set_edge_thresh(vlSift, 10.);

  if (vl_sift_process_first_octave(vlSift, (vl_sift_pix const *) grey_img_float.data) != VL_ERR_EOF) {
    do {
      vl_sift_detect(vlSift);// 特征点检测
      for (int i = 0; i < vlSift->nkeys; i ++) {
        double angles[4];
        _feature_points.emplace_back(vlSift->keys[i].x, vlSift->keys[i].y);
        vector<Mat> descriptor;
        // 计算每个极值点的方向，包括主方向和辅方向，最多4个方向
        int angleCount = vl_sift_calc_keypoint_orientations(vlSift, angles, &vlSift->keys[i]);
        for (int j = 0; j < angleCount; j ++) {
          // 计算每个方向的描述符
          Mat descriptor_array(1, SIFT_DESCRIPTOR_DIM, CV_32FC1);
          vl_sift_calc_keypoint_descriptor(vlSift, (vl_sift_pix *) descriptor_array.data, &vlSift->keys[i], angles[j]);
          descriptor.emplace_back(descriptor_array);
        }
        _feature_descriptors.emplace_back(descriptor);
      }
    } while (vl_sift_process_next_octave(vlSift) != VL_ERR_EOF);
  }
  vl_sift_delete(vlSift);
}

double Translate::getDistance(
    const vector<Mat> & _descriptor1,
    const vector<Mat> & _descriptor2,
    const double _threshold) {
  double result = MAXFLOAT;
  for (int i = 0; i < _descriptor1.size(); i ++) {
    for (int j = 0; j < _descriptor2.size(); j ++) {
      double distance = 0;
      for (int k = 0; k < SIFT_DESCRIPTOR_DIM; k ++) {// 128维的SIFT特征向量
        /* at<vl_sift_pix>(k) == at<vl_sift_pix>(0, k) */
        distance += ((_descriptor1[i].at<vl_sift_pix>(k) - _descriptor2[j].at<vl_sift_pix>(k)) *
                     (_descriptor1[i].at<vl_sift_pix>(k) - _descriptor2[j].at<vl_sift_pix>(k)));

        if (distance >= _threshold) {
          break;// 不计算result
        }
      }
      result = min(result, distance);// 取_threshold内最小的平方
    }
  }
  return result;
}

void Translate::drawFeature(int _m1, int _m2) {
  if (_m1 > _m2) {
    int tmp = _m1;
    _m1 = _m2;
    _m2 = tmp;
  }

  Mat result;
  Mat left, right;
  result = Mat::zeros(max(imgRGBA[_m1].rows, imgRGBA[_m2].rows), imgRGBA[_m1].cols + imgRGBA[_m2].cols, CV_8UC4);
  left  = Mat(result, Rect(0, 0, imgRGBA[_m1].cols, imgRGBA[_m1].rows));
  right = Mat(result, Rect(imgRGBA[_m1].cols, 0, imgRGBA[_m2].cols, imgRGBA[_m2].rows));
  imgRGBA[_m1].copyTo(left);
  imgRGBA[_m2].copyTo(right);

  for (int i = 0; i < indices[_m1][_m2].size(); i ++) {
    int src = indices[_m1][_m2][i].first;
    int dst = indices[_m1][_m2][i].second;

    Point2f src_p, dst_p;
    src_p = origin_features[_m1][src];
    dst_p = origin_features[_m2][dst];

    Scalar color(rand() % 256, rand() % 256, rand() % 256, 255);
    circle(result, src_p, SIZE_CIRCLE, color, -1);
    line(result, src_p, dst_p + Point2f(imgRGBA[_m1].cols, 0), color, SIZE_LINE, LINE_AA);
    circle(result, dst_p + Point2f(imgRGBA[_m1].cols, 0), SIZE_CIRCLE, color, -1);
  }
  show_img(result, "%d %d matches", _m1, _m2);
}