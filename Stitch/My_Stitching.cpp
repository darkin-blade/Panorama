#include "My_Stitching.h"

My_Stitching::My_Stitching(MultiImages & _multi_images) {
  multi_images = & _multi_images;
}

Mat My_Stitching::getMyResult() {
  int img_num = multi_images->img_num;
  multi_images->imgs.resize(2);
  multi_images->imgs[0] = new ImageData();
  multi_images->imgs[1] = new ImageData();

  for (int i = 1; i < img_num; i ++) {
    // 初始化
    multi_images->imgs[0]->initData();
    multi_images->imgs[1]->initData();

    if (i == 1) {
      // 使用原始图片
      // 目标图片
      multi_images->imgs[0]->readImg(multi_images->origin_data[i - 1], 1);
      // 参考图片
      multi_images->imgs[1]->readImg(multi_images->origin_data[i], 1);
    } else {
      // 使用之前的结果
      // 目标图片
      multi_images->imgs[0]->readImg(multi_images->origin_data[i], 1);
      // 参考图片
      multi_images->imgs[1]->readImg(multi_images->pano_result, 2);
    }
    
    // 进行拼接

    multi_images->getFeatureInfo();
    multi_images->getMeshInfo();
    // multi_images->similarityTransform(1, 0.03);
    // multi_images->repairWarpping();

    // multi_images->textureMapping(0);
    // drawFeatureMatch();

    multi_images->myWarping();
    multi_images->getSeam();
    
    // drawMatchingPts();
    // show_img("result", multi_images->pano_result);
  }

  return multi_images->pano_result;
}

void My_Stitching::drawFeatureMatch() {
  // 描绘特征点
  int m1 = 0;
  int m2 = 1;
  Mat result;// 存储结果
  Mat left, right;// 分割矩阵
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
    for (int i = 0; i < multi_images->initial_pairs.size(); i ++) {
      // 计算索引
      int src = multi_images->initial_pairs[i].first;
      int dst = multi_images->initial_pairs[i].second;

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
    for (int i = 0; i < multi_images->feature_points_1.size(); i ++) {
      // 获取特征点
      Point2f src_p, dst_p;
      src_p = multi_images->feature_points_1[i];
      dst_p = multi_images->feature_points_2[i] + Point2f(img1.cols, 0);

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
  show_img("feature pairs", result);
}

void My_Stitching::drawMatchingPts() {
  int m1 = 1;
  int m2 = 0;
  // Mat result = multi_images->pano_images[m1];
  Mat result = Mat::zeros(multi_images->pano_size, CV_8UC3);
  LOG("total size %ld %ld", multi_images->pano_size.width, multi_images->pano_size.height);

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