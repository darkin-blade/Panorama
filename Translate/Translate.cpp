#include "Translate.h"

Translate::Translate(Mat _img1, Mat _img2,
    double _alpha1, double _beta1, double _gamma1,
    double _alpha2, double _beta2, double _gamma2) {

  // 将欧拉角转换成旋转矩阵
  Mat R_x = Mat::zeros(3, 3, CV_64FC1);
  Mat R_y = Mat::zeros(3, 3, CV_64FC1);
  Mat R_z = Mat::zeros(3, 3, CV_64FC1);
  double alpha = _alpha2 - _alpha1;
  double beta  = _beta2 - _beta1;
  double gamma = _gamma2 - _gamma1;

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
  R = R_x * R_y * R_z;

  // 处理成灰度图
  assert(_img1.channels() == _img2.channels());
  _img1.copyTo(img1);
  if (img1.channels() == 3) {
    cvtColor(img1, grey1, CV_RGB2GRAY);
    cvtColor(img1, img1, CV_RGB2RGBA);
  } else if (img1.channels() == 4) {
    cvtColor(img1, grey1, CV_RGBA2GRAY);
  }
  _img2.copyTo(img2);
  if (img2.channels() == 3) {
    cvtColor(img2, grey2, CV_RGB2GRAY);
    cvtColor(img2, img2, CV_RGB2RGBA);
  } else if (img2.channels() == 4) {
    cvtColor(img2, grey2, CV_RGBA2GRAY);
  }

  // 放缩
  float size1 = img1.rows * img1.cols;
  if (size1 > DOWN_SAMPLE_IMAGE_SIZE) {
    float scale = sqrt(DOWN_SAMPLE_IMAGE_SIZE / size1);
    resize(grey1, grey1, Size(), scale, scale);
    resize(img1, img1, Size(), scale, scale);
  }
  float size2 = img2.rows * img2.cols;
  if (size2 > DOWN_SAMPLE_IMAGE_SIZE) {
    float scale = sqrt(DOWN_SAMPLE_IMAGE_SIZE / size2);
    resize(grey2, grey2, Size(), scale, scale);
    resize(img2, img2, Size(), scale, scale);
  }
}

Mat Translate::computeIntrinsic() {
  // 计算相机的内参矩阵

  clock_t begin_time, end_time;
  begin_time = clock();

  getFeaturePairs();
  // drawFeature();

  H = findHomography(feature_pair1, feature_pair2);
  K = H * R.inv();
  cout << R << endl;
  cout << H << endl;
  cout << K << endl;

  end_time = clock();
  LOG("totoal time %lf", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

  return K;
}

Mat Translate::computeTranslate() {
  // 计算两张图片的平移

  clock_t begin_time, end_time;
  begin_time = clock();

  getFeaturePairs();
  // drawFeature();

  E = findFundamentalMat(feature_pair1, feature_pair2, FM_RANSAC, 3, 0.99);
  T = E;

  end_time = clock();
  LOG("totoal time %lf", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

  return T;
}

void Translate::getFeaturePairs() {

  // 特征检测
  getDescriptors(grey1, feature_points1, descriptor1);
  LOG("image 1 feature points %ld", feature_points1.size());
  getDescriptors(grey2, feature_points2, descriptor2);
  LOG("image 2 feature points %ld", feature_points2.size());

  // 初步匹配
  getInitialFeaturePairs();
  vector<Point2f> X, Y;
  X.reserve(initial_indices.size());
  Y.reserve(initial_indices.size());
  for (int i = 0; i < initial_indices.size(); i ++) {
    const pair<int, int> it = initial_indices[i];
    X.emplace_back(feature_points1[it.first ]);
    Y.emplace_back(feature_points2[it.second]);
  }

  // RANSAC 筛选
  getFeaturePairsBySequentialRANSAC(X, Y);

  // 记录结果
  for (int i = 0; i < initial_indices.size(); i ++) {
    const pair<int, int> it = initial_indices[i];
    feature_pair1.emplace_back(feature_points1[it.first ]);
    feature_pair2.emplace_back(feature_points2[it.second]);
  }
}

void Translate::getInitialFeaturePairs() {  
  const int nearest_size = 2;
  const bool ratio_test = true;

  int size_1 = feature_points1.size();
  int size_2 = feature_points2.size();
  const int feature_size[2] = { size_1, size_2 };

  const int another_feature_size = feature_size[1];
  const int nearest_k = min(nearest_size, another_feature_size);// 只可能为0, 1, 2

  // 对每个点计算最相近的特征点
  vector<FeatureDistance> feature_pairs_result;
  for (int f1 = 0; f1 < feature_size[0]; f1 ++) {
    set<FeatureDistance> feature_distance_set;// set 中每个元素都唯一, 降序
    feature_distance_set.insert(FeatureDistance(MAXFLOAT, 0, -1, -1));// 存放计算结果
    for (int f2 = 0; f2 < feature_size[1]; f2 ++) {
      const double dist = getDistance(descriptor1[f1],
          descriptor2[f2],
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
  initial_indices.reserve(feature_pairs_result.size());
  for (int i = 0; i < feature_pairs_result.size(); i ++) {
    if (feature_pairs_result[i].distance < OUTLIER_THRESHOLD) {// 如果没有超出范围
      initial_indices.emplace_back(feature_pairs_result[i].feature_index[0],
          feature_pairs_result[i].feature_index[1]);
    }
  }
}

void Translate::getFeaturePairsBySequentialRANSAC(
    const vector<Point2f> & _X,
    const vector<Point2f> & _Y) {
  vector<char> final_mask(initial_indices.size(), 0);// 存储最终的结果
  findHomography(_X, _Y, CV_RANSAC, GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST, final_mask, GLOBAL_MAX_ITERATION);

  vector<Point2f> tmp_X = _X, tmp_Y = _Y;

  vector<int> mask_indices(initial_indices.size(), 0);
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

  for (int i = 0; i < final_mask.size(); i ++) {
    if (final_mask[i]) {
      indices.emplace_back(initial_indices[i]);
    }
  }
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

void Translate::drawFeature() {
  Mat result;
  Mat left, right;
  result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC4);
  left  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
  right = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
  img1.copyTo(left);
  img2.copyTo(right);
  for (int i = 0; i < indices.size(); i ++) {
    int src = indices[i].first;
    int dst = indices[i].second;

    Point2f src_p, dst_p;
    src_p = feature_points1[src];
    dst_p = feature_points2[dst];

    Scalar color(rand() % 256, rand() % 256, rand() % 256, 255);
    circle(result, src_p, CIRCLE_SIZE, color, -1);
    line(result, src_p, dst_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
    circle(result, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
  }
  show_img("feature pairs", result);
}