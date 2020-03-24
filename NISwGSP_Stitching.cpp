#include "NISwGSP_Stitching.h"
#include "APAP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(MultiImages &multiImages) {
  this->multiImages = &multiImages;
}

void NISwGSP_Stitching::sift_1(Mat img1, Mat img2) {
  Ptr<SIFT> my_sift = SIFT::create();
  vector<vector<KeyPoint> > key_points(2);

  // 检测特征点
  vl_sift_pix *sift_img1 = new vl_sift_pix[img1.cols * img1.rows];
  for (int i = 0, k = 0; i < img1.rows; i ++) {
    uint8_t *p = img1.ptr<uint8_t>(i);
    for (int j = 0; j < img1.cols; j ++) {
      sift_img1[k + j] = p[j];
    }
    k += img1.cols;
  }

  if (true) {
    for (int i = 0; i < 2; i ++) {
      VlSiftFilt *SiftFilt = NULL;
      SiftFilt = vl_sift_new(img1.cols, img1.rows,
          4,// noctaves
          2,// nlevels
          0);// o_min
      if (vl_sift_process_first_octave(SiftFilt, sift_img1) != VL_ERR_EOF) {
        do {
          vl_sift_detect(SiftFilt);
          LOG("cur key points %d", SiftFilt->nkeys);

          // 描绘特征点
          VlSiftKeypoint *point_p = SiftFilt->keys;
          for (int j = 0; j < SiftFilt->nkeys; j ++) {
            VlSiftKeypoint cur_point = *point_p;
            point_p ++;
            key_points[i].push_back(KeyPoint(cur_point.x, cur_point.y, cur_point.s,
                  -1, 0, cur_point.o, -1));
          }
        } while (vl_sift_process_next_octave(SiftFilt) != VL_ERR_EOF);
      }
      vl_sift_delete(SiftFilt);
    }
  } else {
    my_sift->detect(img1, key_points[0]);
    my_sift->detect(img2, key_points[1]);
  }
  multiImages->key_points.push_back(key_points[0]);
  multiImages->key_points.push_back(key_points[1]);

  LOG("sift finished");

  // TODO 匹配类型转换
  vector<Mat> descrip(2);
  my_sift->compute(img1, key_points[0], descrip[0]);
  my_sift->compute(img2, key_points[1], descrip[1]);
  if (false) {
    if (descrip[0].type() != CV_32F || descrip[1].type() != CV_32F) {
      descrip[0].convertTo(descrip[0], CV_32F);
      descrip[1].convertTo(descrip[1], CV_32F);
    }
  }

  LOG("compute finished");

  // 特征点匹配
  Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");
  vector<DMatch> feature_matches;// 存储配对信息
  descriptor_matcher->match(descrip[0], descrip[1], feature_matches);// 进行匹配
  multiImages->descriptor.push_back(descrip[0]);
  multiImages->descriptor.push_back(descrip[1]);

  LOG("match finished");

  // 过滤bad特征匹配
  double max_dis = 0;// 最大的匹配距离
  for (int i = 0; i < descrip[0].rows; i ++) {
    double tmp_dis = feature_matches[i].distance;
    if (tmp_dis > max_dis) {
      max_dis = tmp_dis;
    }
  }

  LOG("max dis: %lf", max_dis);

  multiImages->feature_points.resize(2);
  for (int i = 0; i < feature_matches.size(); i ++) {
    double tmp_dis = feature_matches[i].distance;
    if (tmp_dis < max_dis * 0.1) {
      multiImages->feature_matches.push_back(feature_matches[i]);// 存储好的特征匹配
      int src  = feature_matches[i].queryIdx;
      int dest = feature_matches[i].trainIdx;
      multiImages->feature_points[0].push_back(key_points[0][src].pt);
      multiImages->feature_points[1].push_back(key_points[1][dest].pt);
    }
  }

  LOG("get good mathces %ld", multiImages->feature_points[0].size());
}

void NISwGSP_Stitching::sift_2(Mat img1, Mat img2) {
  Ptr<SIFT> my_sift = SIFT::create();
  vector<ImageFeatures> features(2);
  computeImageFeatures(my_sift, img1, features[0]);
  computeImageFeatures(my_sift, img2, features[1]);

  LOG("compute finish");

  // 特征匹配
  vector<MatchesInfo> pairwise_matches;
  Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestMatcher>(false, 0.3f, 6, 6);// TODO
  (*matcher)(features, pairwise_matches);

  LOG("match finish");

  for (int i = 0; i < pairwise_matches.size(); i ++) {
    LOG("pairwise[%d] size: %ld", i, (long) pairwise_matches[i].matches.size());
  }

  // 保存特征点
  for (int i = 0; i < 2; i ++) {
    multiImages->key_points.push_back(features[i].keypoints);
  }

  // 过滤bad特征匹配
  double max_dis = 0;// 最大的匹配距离
  for (int i = 0; i < pairwise_matches[1].matches.size(); i ++) {
    double tmp_dis = pairwise_matches[1].matches[i].distance;
    if (tmp_dis > max_dis) {
      max_dis = tmp_dis;
    }
  }

  LOG("max distance %lf", max_dis);

  multiImages->feature_points.resize(2);
  for (int i = 0; i < pairwise_matches[1].matches.size(); i ++) {
    double tmp_dis = pairwise_matches[1].matches[i].distance;
    if (tmp_dis < max_dis * 0.25) {
      multiImages->feature_matches.push_back(pairwise_matches[1].matches[i]);// 存储好的特征匹配
      int src  = pairwise_matches[1].matches[i].queryIdx;
      int dest = pairwise_matches[1].matches[i].trainIdx;
      multiImages->feature_points[0].push_back(features[0].keypoints[src].pt);
      multiImages->feature_points[1].push_back(features[1].keypoints[dest].pt);
    }
  }

  LOG("get good mathces %ld", (long) multiImages->feature_matches.size());
}

Mat NISwGSP_Stitching::draw_matches() {
  // 匹配特征点
  sift_1(multiImages->imgs[0], multiImages->imgs[1]);// 特征点匹配

  // 描绘特征点
  Mat result_1;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  Mat img1 = multiImages->imgs[0];
  Mat img2 = multiImages->imgs[1];
  result_1 = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
  left_1  = Mat(result_1, Rect(0, 0, img1.cols, img1.rows));
  right_1 = Mat(result_1, Rect(img1.cols, 0, img2.cols, img2.rows));
  // 复制图片
  img1.copyTo(left_1);
  img2.copyTo(right_1);

  if (1) {
    // 匹配所有特征点
    for (int i = 0; i < multiImages->feature_points[0].size(); i++) {
      // 获取特征点
      Point2f src_p, dest_p;
      src_p = multiImages->feature_points[0][i];
      dest_p = multiImages->feature_points[1][i];

      // 描绘
      Scalar color(rand() % 256, rand() % 256, rand() % 256);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
      line(result_1, src_p, dest_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
      circle(result_1, dest_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  } else {
    // 描绘所有关键点
    for (int i = 0; i < multiImages->key_points[0].size(); i ++) {
      Point2f src_p = multiImages->key_points[0][i].pt;
      Scalar color(255, 0, 0);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
    }
    for (int i = 0; i < multiImages->key_points[1].size(); i ++) {
      Point2f src_p = multiImages->key_points[1][i].pt;
      Scalar color(255, 0, 0);
      circle(result_1, src_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  }

  LOG("draw feature matching finished");

  return result_1;
}

Mat NISwGSP_Stitching::get_matching_pts() {
  int img_num = 2;
  // 划分图像mesh
  multiImages->img_mesh.resize(2);
  for (int i = 0; i < img_num; i ++) {
    int cols = multiImages->imgs[i].cols;
    int rows = multiImages->imgs[i].rows;
    // 计算间距
    double ratio = ((double) cols) / rows;
    int row_num = 20;
    int col_num = (int) (ratio * 20);
    double col_step = ((double) cols) / col_num;
    double row_step = ((double) rows) / row_num;
    // 添加mesh
    for (int j = 0; j <= col_num; j ++) {
      for (int k = 0; k <= row_num; k ++) {
        multiImages->img_mesh[i].push_back(Point2f(j * col_step, k * row_step));
      }
    }
  }

  LOG("get mesh");

  // 计算单应矩阵
  multiImages->matching_points.resize(img_num);
  multiImages->homographies.resize(img_num);

  LOG("starting apap");

  APAP_Stitching::apap_project(multiImages->feature_points[0],
                               multiImages->feature_points[1],
                               multiImages->img_mesh[0],
                               multiImages->matching_points[0],
                               multiImages->homographies[0]);

  LOG("apap finished");

  // 计算匹配点
  Mat result_1;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  Mat img1 = multiImages->imgs[0];
  Mat img2 = multiImages->imgs[1];
  result_1 = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
  left_1  = Mat(result_1, Rect(0, 0, img1.cols, img1.rows));
  right_1 = Mat(result_1, Rect(img1.cols, 0, img2.cols, img2.rows));
  // 复制图片
  img1.copyTo(left_1);
  img2.copyTo(right_1);

  // 剔除出界点
  multiImages->matching_pairs.resize(2);
  vector<pair<int, int> > &matching_pairs = multiImages->matching_pairs[0];// 配对信息
  const vector<Point2f> *tmp_p = &multiImages->matching_points[0];// 匹配点位置
  for (int i = 0; i < tmp_p->size(); i ++) {
    if ((*tmp_p)[i].x >= 0 && (*tmp_p)[i].y >= 0 && (*tmp_p)[i].x <= img2.cols && (*tmp_p)[i].y <= img2.rows) {// x对应cols, y对应rows
      matching_pairs.push_back(pair<int, int>(i, i));
    }
  }

  // 描绘匹配点
  if (1 == 2) {
    // 匹配点配对
    for (int i = 0; i < multiImages->matching_pairs[0].size(); i ++) {
      // 获取mesh
      int index = multiImages->matching_pairs[0][i].first;
      Point2f src_p, dest_p;// TODO
      src_p = multiImages->img_mesh[0][index];
      dest_p = multiImages->matching_points[0][index];

      // 描绘
      Scalar color(rand() % 256, rand() % 256, rand() % 256);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
      line(result_1, src_p, dest_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
      circle(result_1, dest_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  } else {
    // 描绘所有匹配点
    for (int i = 0; i < multiImages->img_mesh[0].size(); i ++) {
      // 获取mesh
      Point2f src_p, dest_p;// TODO
      src_p = multiImages->img_mesh[0][i];
      dest_p = multiImages->matching_points[0][i];

      // 描绘
      Scalar color1(255, 0, 0);
      circle(result_1, src_p, CIRCLE_SIZE, color1, -1);
      Scalar color2(0, 0, 255);
      circle(result_1, dest_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color2, -1);
    }
  }

  LOG("match finished");

  return result_1;
}

void NISwGSP_Stitching::show_img(const char *window_name, Mat img) {
#if defined(UBUNTU)
  namedWindow(window_name, WINDOW_AUTOSIZE);
  imshow(window_name, img);
  waitKey(0);
#endif
}

// vector<pair<int, int> > NISwGSP_Stitching::getFeaturePairs(const pair<int, int> &_match_pair) {
// }
