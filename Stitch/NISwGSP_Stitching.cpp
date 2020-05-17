#include "NISwGSP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(MultiImages & _multi_images) : MeshOptimization(_multi_images) {
}

void NISwGSP_Stitching::prepare() {
  // 对图片预处理
  int img_num = multi_images->img_num;

  multi_images->imgs[0]->data = change_image(multi_images->imgs[0]->data, +0.00, 1.0).clone();
  multi_images->imgs[1]->data = change_image(multi_images->imgs[1]->data, +1.57, 1.0).clone();
  // multi_images->imgs[2]->data = change_image(multi_images->imgs[2]->data, -0.51, 1.0);
  // multi_images->imgs[3]->data = change_image(multi_images->imgs[3]->data, +2.12, 1.3);
  // multi_images->imgs[4]->data = change_image(multi_images->imgs[4]->data, +1.68, 1.2);
  assert(0);
}

Mat NISwGSP_Stitching::change_image(Mat img, double angle, double scale) {
  double tmp = 3.1415926 / 180;

  double width = img.cols;
  double height = img.rows;
  double diagonal = sqrt(width * width + height * height);

  double new_width;
  double new_height;
  double tmp_angle = fabs(asin(sin(angle)));
  new_width = diagonal * cos(asin(height / diagonal) - tmp_angle);
  new_height = diagonal * cos(asin(width / diagonal) - tmp_angle);

  // 平移变换
  Mat translate = Mat::zeros(2, 3, CV_32FC1);
  translate.at<float>(0, 0) = 1;
  translate.at<float>(0, 2) = (new_width - width) / 2;// 水平偏移
  translate.at<float>(1, 1) = 1;
  translate.at<float>(1, 2) = (new_height - height) / 2;// 垂直偏移
  Mat result_1;
  warpAffine(img, result_1, translate, Size(new_width, new_height));
  show_img("0", img);
  show_img("1", result_1);

  // 旋转变换
  Point2f center(new_width / 2, new_height / 2);
  Mat rotation = getRotationMatrix2D(center, angle / tmp, 1.0);
  Mat result_2;
  warpAffine(result_1, result_2, rotation, Size(new_width, new_height));
  show_img("2", result_2);

  // 缩放变换
  Mat result_3;
  resize(result_2, result_3, Size(new_width * 1, new_height * 1), 0, 0, INTER_LINEAR);
  show_img("3", result_3);

  return result_3;
}

Mat NISwGSP_Stitching::feature_match() {
  int img_num = multi_images->img_num;

  for (int i = 0; i < img_num; i ++) {
    Mat grey_img;
    cvtColor(multi_images->imgs[i]->data, grey_img, CV_BGR2GRAY);
    FeatureController::detect(grey_img,
                              multi_images->imgs[i]->feature_points,
                              multi_images->imgs[i]->descriptors);
    LOG("[picture %d] feature points: %ld", i, multi_images->imgs[i]->feature_points.size());
  }

  // 特征点匹配
  multi_images->getFeaturePairs();

  LOG("get feature pairs");

  // 筛选所有图片的成功匹配的特征点
  multi_images->feature_points.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    multi_images->feature_points[i].resize(img_num);
  }
  for (int i = 0; i < multi_images->img_pairs.size(); i ++) {
    int m1 = multi_images->img_pairs[i].first;
    int m2 = multi_images->img_pairs[i].second;
    assert(m2 > m1);

    const vector<Point2f> & m1_fpts = multi_images->imgs[m1]->feature_points;
    const vector<Point2f> & m2_fpts = multi_images->imgs[m2]->feature_points;
    for (int k = 0; k < multi_images->feature_pairs[m1][m2].size(); k ++) {
      const pair<int, int> it = multi_images->feature_pairs[m1][m2][k];
      multi_images->feature_points[m1][m2].emplace_back(m1_fpts[it.first ]);
      multi_images->feature_points[m2][m1].emplace_back(m2_fpts[it.second]);
    }
  }


  // 描绘特征点
  Mat result_1;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  if (multi_images->img_pairs.size() > 0) {
    int m1 = multi_images->img_pairs[0].first;
    int m2 = multi_images->img_pairs[0].second;
    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    result_1 = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    left_1  = Mat(result_1, Rect(0, 0, img1.cols, img1.rows));
    right_1 = Mat(result_1, Rect(img1.cols, 0, img2.cols, img2.rows));
    // 复制图片
    img1.copyTo(left_1);
    img2.copyTo(right_1);

    if (0) {
      // 匹配所有特征点
      for (int i = 0; i < multi_images->feature_pairs[m1][m2].size(); i ++) {
        // 计算索引
        int src = multi_images->feature_pairs[m1][m2][i].first;
        int dst = multi_images->feature_pairs[m1][m2][i].second;

        // 获取特征点
        Point2f src_p, dst_p;
        src_p = multi_images->imgs[m1]->feature_points[src];
        dst_p = multi_images->imgs[m2]->feature_points[dst];

        // 描绘
        Scalar color(rand() % 256, rand() % 256, rand() % 256);
        circle(result_1, src_p, CIRCLE_SIZE, color, -1);
        line(result_1, src_p, dst_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
        circle(result_1, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
      }
    } else {
      // 描绘所有特征点
      for (int i = 0; i < multi_images->imgs[m1]->feature_points.size(); i ++) {
        Point2f src_p = multi_images->imgs[m1]->feature_points[i];
        Scalar color(255, 0, 0);
        circle(result_1, src_p, CIRCLE_SIZE, color, -1);
      }
      for (int i = 0; i < multi_images->imgs[m2]->feature_points.size(); i ++) {
        Point2f src_p = multi_images->imgs[m2]->feature_points[i];
        Scalar color(255, 0, 0);
        circle(result_1, src_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
      }
    }
  }
  LOG("draw feature matching finished");
  return result_1;
}

Mat NISwGSP_Stitching::matching_match() {
  multi_images->do_matching();

  // 描绘匹配点
  Mat result_1;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  if (multi_images->img_pairs.size() > 0) {
    int m1 = multi_images->img_pairs[0].first;
    int m2 = multi_images->img_pairs[0].second;

    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    result_1 = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    left_1  = Mat(result_1, Rect(0, 0, img1.cols, img1.rows));
    right_1 = Mat(result_1, Rect(img1.cols, 0, img2.cols, img2.rows));
    // 复制图片
    img1.copyTo(left_1);
    img2.copyTo(right_1);

    if (0) {
      // 描绘匹配点配对
      for (int i = 0; i < multi_images->matching_indices[m1][m2].size(); i ++) {
        int index = multi_images->matching_indices[m1][m2][i];
        Point2f src_p, dst_p;
        src_p = multi_images->imgs[m1]->getMeshPoints()[index];
        dst_p = multi_images->imgs[m1]->matching_points[m2][index];

        Scalar color(rand() % 256, rand() % 256, rand() % 256);
        circle(result_1, src_p, CIRCLE_SIZE, color, -1);
        line(result_1, src_p, dst_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
        circle(result_1, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
      }
    } else {
      // 描绘所有匹配点
      for (int i = 0; i < multi_images->imgs[m1]->getMeshPoints().size(); i ++) {
        Point2f src_p, dst_p;
        src_p = multi_images->imgs[m1]->getMeshPoints()[i];
        dst_p = multi_images->imgs[m1]->matching_points[m2][i];

        Scalar color1(255, 0, 0);
        circle(result_1, src_p, CIRCLE_SIZE, color1, -1);
        Scalar color2(0, 0, 255);
        circle(result_1, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color2, -1);
      }
    }
  }
  LOG("match finished");
  return result_1;
}

void NISwGSP_Stitching::get_solution() {
  alignment_weight               = 1;
  local_similarity_weight        = 0.56;
  global_similarity_weight_beta  = 6;
  global_similarity_weight_gamma = 20;

  vector<Triplet<double> > triplets;
  vector<pair<int, double> > b_vector;

  reserveData(triplets, b_vector, 0);// DIMENSION_2D

  triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
  triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
  b_vector.emplace_back(0,    STRONG_CONSTRAINT);
  b_vector.emplace_back(1,    STRONG_CONSTRAINT);

  prepareAlignmentTerm(triplets);
  // int tmp_size = triplets.size() - 1;
  // for (int i = 0; i < 100; i ++) {
  //   LOG("%lf", triplets[tmp_size - i].value());
  // }
  prepareSimilarityTerm(triplets, b_vector);
  getImageMeshPoints(triplets, b_vector);
}

Mat NISwGSP_Stitching::texture_mapping() {
  // vector<vector<Point2f> > result_1;
  // result_1.resize(2);
  // 2->1
  // result_1[0] = multi_images->imgs[0]->getMeshPoints();// 图1的mesh
  // for (int i = 0; i < multi_images->imgs[1]->matching_points[0].size(); i ++) {// 图2的mesh
  //   Point2f tmp_mesh = multi_images->imgs[1]->matching_points[0][i];// TODO
  //   result_1[1].push_back(tmp_mesh);
  // }
  // 1->2
  // result_1[1] = multi_images->imgs[1]->getMeshPoints();// 图1的mesh
  // for (int i = 0; i < multi_images->imgs[0]->matching_points[1].size(); i ++) {// 图2的mesh
  //   Point2f tmp_mesh = multi_images->imgs[0]->matching_points[1][i];// TODO
  //   result_1[0].push_back(tmp_mesh);
  // }

  if (0) {
    // 只绘制最终mesh点
    Size2f target_size = normalizeVertices(multi_images->image_mesh_points);
    Mat result_1;
    result_1 = Mat::zeros(round(target_size.height), round(target_size.width), CV_8UC4);
    vector<vector<Point2f> > & image_mesh_points = multi_images->image_mesh_points;
    for (int i = 0; i < multi_images->img_num; i ++) {
      for (int j = 0; j < image_mesh_points[i].size(); j ++) {
        Point2f tmp = image_mesh_points[i][j];
        if (i) {
          circle(result_1, tmp, CIRCLE_SIZE, Scalar(255, 0, 0), -1);
        } else {
          circle(result_1, tmp, CIRCLE_SIZE, Scalar(255, 255, 0), -1);
        }
      }
    }
    return result_1;
  } else {
    return multi_images->textureMapping(multi_images->image_mesh_points, 1);
  }
}

void NISwGSP_Stitching::show_img(const char *window_name, Mat img) {
#if defined(UBUNTU)
  namedWindow(window_name, WINDOW_AUTOSIZE);
  imshow(window_name, img);
  waitKey(0);

  // 保存图片
  char img_name[128];
  int savable = 0;
  for (int i = 0; i < 100; i ++) {
    sprintf(img_name, "../../result_%d.jpg", i);
    if (fopen(img_name, "r") == NULL) {
      savable = 1;
      break;
    }
  }
  if (savable) {
    imwrite(img_name, img);
  } else {
    LOG("can't save img");
  }
#endif
}
