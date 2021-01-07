#include "My_Stitching.h"

My_Stitching::My_Stitching(MultiImages & _multi_images) {
  multi_images = & _multi_images;
}

Mat My_Stitching::getMyResult() {
  int img_num = multi_images->img_num;
  assert(img_num == 2);
  multi_images->getFeatureInfo();
  multi_images->getMeshInfo();
  // multi_images->similarityTransform(1, 0.03);
  // multi_images->repairWarpping();


  // multi_images->textureMapping(0);
  // drawFeatureMatch();

  multi_images->myWarping();
  multi_images->getSeam();
  
  // drawMatchingPts();
  show_img("result", multi_images->pano_result);

  return multi_images->pano_result;
}

void My_Stitching::drawFeatureMatch() {
  // 描绘特征点
  Mat result;// 存储结果
  Mat left, right;// 分割矩阵
  if (multi_images->img_pairs.size() > 0) {
    int m1 = multi_images->img_pairs[0].first;
    int m2 = multi_images->img_pairs[0].second;
    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    left  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
    right = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    // 复制图片
    img1.copyTo(left);
    img2.copyTo(right);

    if (0) {
      // 匹配RANSAC之前的所有特征点
      for (int i = 0; i < multi_images->initial_pairs[m1][m2].size(); i ++) {
        // 计算索引
        int src = multi_images->initial_pairs[m1][m2][i].first;
        int dst = multi_images->initial_pairs[m1][m2][i].second;

        // 获取特征点
        Point2f src_p, dst_p;
        src_p = multi_images->imgs[m1]->feature_points[src];
        dst_p = multi_images->imgs[m2]->feature_points[dst] + Point2f(img1.cols, 0);

        // 描绘
        Scalar color(rand() % 256, rand() % 256, rand() % 256, 255);
        circle(result, src_p, CIRCLE_SIZE, color, -1);
        line(result, src_p, dst_p, color, LINE_SIZE, LINE_AA);
        circle(result, dst_p, CIRCLE_SIZE, color, -1);
      }
    } else if (1) {
      // 匹配RANSAC之后的所有特征点
      for (int i = 0; i < multi_images->feature_points[m1][m2].size(); i ++) {
        // 获取特征点
        Point2f src_p, dst_p;
        src_p = multi_images->feature_points[m1][m2][i];
        dst_p = multi_images->feature_points[m2][m1][i] + Point2f(img1.cols, 0);

        // 描绘
        Scalar color(rand() % 256, rand() % 256, rand() % 256, 255);
        circle(result, src_p, CIRCLE_SIZE, color, -1);
        line(result, src_p, dst_p, color, LINE_SIZE, LINE_AA);
        circle(result, dst_p, CIRCLE_SIZE, color, -1);
      }
    } else {
      // 描绘所有特征点
      for (int i = 0; i < multi_images->imgs[m1]->feature_points.size(); i ++) {
        Point2f src_p = multi_images->imgs[m1]->feature_points[i];
        Scalar color(255, 0, 0);
        circle(result, src_p, CIRCLE_SIZE, color, -1);
      }
      for (int i = 0; i < multi_images->imgs[m2]->feature_points.size(); i ++) {
        Point2f src_p = multi_images->imgs[m2]->feature_points[i];
        Scalar color(255, 0, 0);
        circle(result, src_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
      }
    }
  }
  show_img("feature pairs", result);
}

void My_Stitching::drawMatchingPts() {
  int m1 = 0;
  int m2 = 1;
  Mat result = Mat::zeros(multi_images->pano_size, CV_8UC3);

  // 描绘所有匹配点
  for (int i = 0; i < multi_images->imgs[m1]->vertices.size(); i ++) {
    Point2f src_p, dst_p;
    // src_p = multi_images->matching_pts[m1 + multi_images->img_num][i];
    dst_p = multi_images->matching_pts[m1][i];

    // Scalar color1(255, 0, 0, 255);
    // circle(result, src_p, CIRCLE_SIZE, color1, -1);
    Scalar color2(0, 0, 255, 255);
    circle(result, dst_p, CIRCLE_SIZE, color2, -1);
    // Scalar color3(0, 100, 0, 255);
    // line(result, src_p, dst_p, color3, LINE_SIZE, LINE_AA);

    // debug
  }

  show_img("matching pts", result);
}