#include "NISwGSP_Stitching.h"
#include "APAP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(MultiImages &multiImages) {
  this->multiImages = &multiImages;
}

Mat NISwGSP_Stitching::feature_match() {
  int img_num = multiImages->img_num;

  // 检测特征点
  for (int i = 0; i < img_num; i ++) {
    FeatureController::detect(multiImages->imgs[i]->data,
                              multiImages->imgs[i]->feature_points,
                              multiImages->imgs[i]->descriptors);
    LOG("[picture %d] feature points: %ld", i, multiImages->imgs[i]->feature_points.size());
  }


  // 特征点匹配

  // 描绘特征点
  Mat result_1;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  Mat img1 = multiImages->imgs[0]->data;
  Mat img2 = multiImages->imgs[1]->data;
  result_1 = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
  left_1  = Mat(result_1, Rect(0, 0, img1.cols, img1.rows));
  right_1 = Mat(result_1, Rect(img1.cols, 0, img2.cols, img2.rows));
  // 复制图片
  img1.copyTo(left_1);
  img2.copyTo(right_1);

  if (1) {
    // 匹配所有特征点
    for (int i = 0; i < multiImages->feature_pairs[0][1].size(); i ++) {
      // 计算索引
      int src  = multiImages->feature_pairs[0][1][i].first;
      int dest = multiImages->feature_pairs[0][1][i].second;

      // 获取特征点
      Point2f src_p, dest_p;
      src_p  = multiImages->imgs[0]->feature_points[src];
      dest_p = multiImages->imgs[1]->feature_points[dest];

      // 描绘
      Scalar color(rand() % 256, rand() % 256, rand() % 256);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
      line(result_1, src_p, dest_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
      circle(result_1, dest_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  } else {
    // 描绘所有特征点
    for (int i = 0; i < multiImages->imgs[0]->feature_points.size(); i ++) {
      Point2f src_p = multiImages->imgs[0]->feature_points[i];
      Scalar color(255, 0, 0);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
    }
    for (int i = 0; i < multiImages->imgs[1]->feature_points.size(); i ++) {
      Point2f src_p = multiImages->imgs[1]->feature_points[i];
      Scalar color(255, 0, 0);
      circle(result_1, src_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  }

  LOG("draw feature matching finished");

  return result_1;
}

    /*
Mat NISwGSP_Stitching::matching_match() {
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
     */

void NISwGSP_Stitching::show_img(const char *window_name, Mat img) {
#if defined(UBUNTU)
  namedWindow(window_name, WINDOW_AUTOSIZE);
  imshow(window_name, img);
  waitKey(0);
#endif
}
