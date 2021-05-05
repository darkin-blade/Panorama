#include "My_Stitching.h"

My_Stitching::My_Stitching(MultiImages & _multi_images) {
  multi_images = & _multi_images;
}

Mat My_Stitching::getMyResult() {
  LOG("o_min %d nlevels %d", SIFT_MINIMUM_OCTAVE_INDEX, SIFT_LEVEL_COUNT);
  int img_num = multi_images->img_num;

  assert(multi_images->imgs.empty());
  for (int i = 0; i < img_num; i ++) {
    // 读取图像: 从multi_images到ImageData类中
    multi_images->imgs.emplace_back(new ImageData());
    // multi_images->imgs[i]->initData();// 单纯地清空所有数据
    multi_images->imgs[i]->readMat(multi_images->origin_data[i], 1);
  }

  // 初始化
  multi_images->init();
  multi_images->getImagePairs();// TODO

  // 初步特征点匹配
  multi_images->getFeatureInfo();

  // 相似变换
  multi_images->similarityTransform(0);

  // apap网格计算
  // multi_images->getMeshInfo();

  // 网格优化
  multi_images->meshOptimization();

  // 计算中间结果
  multi_images->getTmpResult();

  // 计算每幅图像的全景位置
  multi_images->myWarping();

  // 接缝线算法
  multi_images->getSeam();// 图像融合过程内嵌在这个函数中
  
  // debug
  drawFeature();
  drawAPAP();

  return Mat();// TODO
}

void My_Stitching::debug() {
  assert(0);
}

void My_Stitching::drawAPAP() {
  // 绘制原始网格
  for (int i = 0; i < multi_images->img_pairs.size(); i ++) {
    int m1 = multi_images->img_pairs[i].first;
    int m2 = multi_images->img_pairs[i].second;
    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    Mat result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    Mat left  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
    Mat right = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    img1.copyTo(left);
    img2.copyTo(right);

    Scalar color1(255, 0, 0, 255);
    for (int j = 0; j < multi_images->imgs[m1]->vertices.size(); j ++) {
      Point2f src_p = multi_images->imgs[m1]->vertices[j];
      Point2f dst_p = multi_images->imgs[m2]->vertices[j] + Point2f(img1.cols, 0);
      circle(result, src_p, CIRCLE_SIZE, color1, -1);
      // circle(result, dst_p, CIRCLE_SIZE, color1, -1);
    }
    show_img("initial vertices", result);
  }

  // 绘制形变后的网格点位置
  for (int i = 0; i < multi_images->img_pairs.size(); i ++) {
    int m1 = multi_images->img_pairs[i].first;
    int m2 = multi_images->img_pairs[i].second;
    Mat result;
    multi_images->pano_images[m2].copyTo(result);

    Scalar color1(255, 0, 0, 255);
    Scalar color2(0, 0, 255, 255);
    for (int j = 0; j < multi_images->single_mask[m1][m2].size(); j ++) {
      Point2f src_p = multi_images->matching_pts[m1][j];
      if (!multi_images->single_mask[m1][m2][j]) {
        // 出界
        circle(result, src_p, CIRCLE_SIZE + 2, color1, -1);
      } else {
        circle(result, src_p, CIRCLE_SIZE + 2, color2, -1);
      }
    }
    show_img("apap vertices", result);
  }

  // 绘制网格优化后的所有顶点
  {
    Mat result = Mat::zeros(multi_images->pano_size, CV_8UC4);
    for (int i = 0; i < multi_images->matching_pts.size(); i ++) {
      if (i == 1) continue;
      Scalar color1(0, 0, 255, 255);// TODO
      for (int j = 0; j < multi_images->matching_pts[i].size(); j ++) {
        Point2f src_p = multi_images->matching_pts[i][j];
        circle(result, src_p, CIRCLE_SIZE + 1, color1, -1);
      }
    }
    show_img("mesh optimization", result);
  }
}

void My_Stitching::drawFeature() {
  // 描绘所有图像的特征点
  for (int i = 0; i < multi_images->img_pairs.size(); i ++) {
    int m1 = multi_images->img_pairs[i].first;
    int m2 = multi_images->img_pairs[i].second;
    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    Mat result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    Mat left  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
    Mat right = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    img1.copyTo(left);
    img2.copyTo(right);

    Scalar color1(255, 0, 0, 255);
    for (int j = 0; j < multi_images->imgs[m1]->features.size(); j ++) {
      Point2f src_p = multi_images->imgs[m1]->features[j];
      circle(result, src_p, CIRCLE_SIZE, color1, -1);
    }
    for (int j = 0; j < multi_images->imgs[m2]->features.size(); j ++) {
      Point2f dst_p = multi_images->imgs[m2]->features[j] + Point2f(img1.cols, 0);
      circle(result, dst_p, CIRCLE_SIZE, color1, -1);
    }
    show_img("feature pts", result);
  }

  // 绘制初步匹配结果
  for (int i = 0; i < multi_images->img_pairs.size(); i ++) {
    int m1 = multi_images->img_pairs[i].first;
    int m2 = multi_images->img_pairs[i].second;
    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    Mat result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    Mat left  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
    Mat right = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    img1.copyTo(left);
    img2.copyTo(right);

    Scalar color1(255, 0, 0, 255);
    Scalar color2(0, 255, 0, 255);
    for (int j = 0; j < multi_images->initial_pairs[m1][m2].size(); j ++) {
      pair<int, int> tmp_pair =  multi_images->initial_pairs[m1][m2][j];
      Point2f src_p = multi_images->imgs[m1]->features[tmp_pair.first];
      Point2f dst_p = multi_images->imgs[m2]->features[tmp_pair.second] + Point2f(img1.cols, 0);
      line(result, src_p, dst_p, color2, LINE_SIZE, LINE_AA);
      circle(result, src_p, CIRCLE_SIZE, color1, -1);
      circle(result, dst_p, CIRCLE_SIZE, color1, -1);
    }
    show_img("init pair", result);
  }

  // 绘制RANSAC之后的结果
  for (int i = 0; i < multi_images->img_pairs.size(); i ++) {
    int m1 = multi_images->img_pairs[i].first;
    int m2 = multi_images->img_pairs[i].second;
    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    Mat result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    Mat left  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
    Mat right = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    img1.copyTo(left);
    img2.copyTo(right);

    Scalar color1(255, 0, 0, 255);
    Scalar color2(0, 255, 0, 255);
    for (int j = 0; j < multi_images->filtered_pairs[m1][m2].size(); j ++) {
      pair<int, int> tmp_pair =  multi_images->filtered_pairs[m1][m2][j];
      Point2f src_p = multi_images->imgs[m1]->features[tmp_pair.first];
      Point2f dst_p = multi_images->imgs[m2]->features[tmp_pair.second] + Point2f(img1.cols, 0);
      line(result, src_p, dst_p, color2, LINE_SIZE, LINE_AA);
      circle(result, src_p, CIRCLE_SIZE, color1, -1);
      circle(result, dst_p, CIRCLE_SIZE, color1, -1);
    }
    show_img("after RANSAC", result);
  }
}