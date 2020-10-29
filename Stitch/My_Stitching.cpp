#include "My_Stitching.h"

My_Stitching::My_Stitching(MultiImages & _multi_images) : MeshOptimization(_multi_images) {
}

Mat My_Stitching::getMyResult() {

  set_progress(5, MODE_MY);
  multi_images->getFeatureInfo();
  set_progress(10, MODE_MY);
  multi_images->getMeshInfo();
  set_progress(20, MODE_MY);

  /**** 网格优化 ****/

  alignment_weight               = 1;// 1
  local_similarity_weight        = 0.56;// 0.56
  global_similarity_weight_beta  = 6;// 6
  global_similarity_weight_gamma = 20;// 20

  vector<Triplet<double> > triplets;
  vector<pair<int, double> > b_vector;

  reserveData(triplets, b_vector, 0);// DIMENSION_2D

  triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
  triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
  b_vector.emplace_back(0,    STRONG_CONSTRAINT);
  b_vector.emplace_back(1,    STRONG_CONSTRAINT);

  prepareAlignmentTerm(triplets);
  prepareSimilarityTerm(triplets, b_vector);

  getImageVerticesBySolving(triplets, b_vector);

  /*****************/

  set_progress(70, MODE_MY);
  multi_images->ignore_weight_mask = true;
  multi_images->warpImages();
  set_progress(80, MODE_MY);
  // 曝光补偿
  // multi_images->exposureCompensate();
  // 寻找接缝线
  multi_images->getSeam();
  set_progress(90, MODE_MY);
  
  Mat result = multi_images->blending();
  set_progress(100, MODE_MY);
  return result;
}

Mat My_Stitching::getNISResult() {
  
  set_progress(5, MODE_MY);
  multi_images->getFeatureInfo();
  set_progress(10, MODE_MY);
  multi_images->getMeshInfo();
  set_progress(20, MODE_MY);

  /**** 网格优化 ****/

  alignment_weight               = 1;// 1
  local_similarity_weight        = 0.56;// 0.56
  global_similarity_weight_beta  = 6;// 6
  global_similarity_weight_gamma = 20;// 20

  vector<Triplet<double> > triplets;
  vector<pair<int, double> > b_vector;

  reserveData(triplets, b_vector, 0);// DIMENSION_2D

  triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
  triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
  b_vector.emplace_back(0,    STRONG_CONSTRAINT);
  b_vector.emplace_back(1,    STRONG_CONSTRAINT);

  prepareAlignmentTerm(triplets);
  prepareSimilarityTerm(triplets, b_vector);

  getImageVerticesBySolving(triplets, b_vector);

  /*****************/

  set_progress(80, MODE_MY);
  multi_images->ignore_weight_mask = false;
  multi_images->warpImages();
  set_progress(90, MODE_MY);
  Mat result = multi_images->textureMapping();
  return result;
}

Mat My_Stitching::getAPAPResult() {

  multi_images->getFeatureInfo();
  multi_images->getMeshInfo();

  // 只适用于两张图片
  multi_images->image_mesh_points.emplace_back(multi_images->imgs[0]->matching_points[1]);
  multi_images->image_mesh_points.emplace_back(multi_images->imgs[1]->getVertices());

  multi_images->ignore_weight_mask = false;
  multi_images->warpImages();
  return multi_images->textureMapping();
}

void My_Stitching::drawFeatureMatch() {
  // 描绘特征点
  Mat result;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  if (multi_images->img_pairs.size() > 0) {
    int m1 = multi_images->img_pairs[0].first;
    int m2 = multi_images->img_pairs[0].second;
    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    left_1  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
    right_1 = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    // 复制图片
    img1.copyTo(left_1);
    img2.copyTo(right_1);

    if (0) {
      // 匹配RANSAC之前的所有特征点
      for (int i = 0; i < multi_images->initial_pairs[m1][m2].size(); i ++) {
        // 计算索引
        int src = multi_images->initial_pairs[m1][m2][i].first;
        int dst = multi_images->initial_pairs[m1][m2][i].second;

        // 获取特征点
        Point2f src_p, dst_p;
        src_p = multi_images->imgs[m1]->feature_points[src];
        dst_p = multi_images->imgs[m2]->feature_points[dst];

        // 描绘
        Scalar color(rand() % 256, rand() % 256, rand() % 256);
        circle(result, src_p, CIRCLE_SIZE, color, -1);
        line(result, src_p, dst_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
        circle(result, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
      }
    } else if (1) {
      // 匹配RANSAC之后的所有特征点
      for (int i = 0; i < multi_images->feature_points[m1][m2].size(); i ++) {
        // 获取特征点
        Point2f src_p, dst_p;
        src_p = multi_images->feature_points[m1][m2][i];
        dst_p = multi_images->feature_points[m2][m1][i];

        // 描绘
        Scalar color(rand() % 256, rand() % 256, rand() % 256);
        circle(result, src_p, CIRCLE_SIZE, color, -1);
        line(result, src_p, dst_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
        circle(result, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
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

void My_Stitching::drawMatchingMatch() {
  // 描绘匹配点
  Mat result;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  if (multi_images->img_pairs.size() > 0) {
    int m1 = multi_images->img_pairs[0].first;
    int m2 = multi_images->img_pairs[0].second;

    Mat img1 = multi_images->imgs[m1]->data;
    Mat img2 = multi_images->imgs[m2]->data;
    result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    left_1  = Mat(result, Rect(0, 0, img1.cols, img1.rows));
    right_1 = Mat(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    // 复制图片
    img1.copyTo(left_1);
    img2.copyTo(right_1);

    
    // 描绘所有匹配点
    for (int i = 0; i < multi_images->imgs[m1]->getVertices().size(); i ++) {
      Point2f src_p, dst_p;
      src_p = multi_images->imgs[m1]->getVertices()[i];
      dst_p = multi_images->imgs[m1]->matching_points[m2][i];

      Scalar color1(255, 0, 0);
      circle(result, src_p, CIRCLE_SIZE, color1, -1);
      Scalar color2(0, 0, 255);
      circle(result, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color2, -1);
    }
  }
  show_img("matching pairs", result);
}

void My_Stitching::drawResult(Mat _result) {
  
  if (1) {
    // 只绘制最终mesh点
    Size2f target_size = normalizeVertices(multi_images->image_mesh_points);
    vector<vector<Point2f> > & image_mesh_points = multi_images->image_mesh_points;
    for (int i = 0; i < multi_images->img_num; i ++) {
      for (int j = 0; j < image_mesh_points[i].size(); j ++) {
        Point2f tmp = image_mesh_points[i][j];
        if (i) {
          circle(_result, tmp, CIRCLE_SIZE, Scalar(255, 0, 0), -1);
        } else {
          circle(_result, tmp, CIRCLE_SIZE, Scalar(255, 255, 0), -1);
        }
      }
    }
  } else if (0) {
    // 图像描边
    int line_thickness = 1;// 描边的线宽
    Mat imgs_border(_result.size() + Size(line_thickness * 6, line_thickness * 6), CV_8UC4);
    Point2f shift(line_thickness * 3, line_thickness * 3);// 偏移
    Rect rect(shift, _result.size());
    _result.copyTo(imgs_border);
    for (int i = 0; i < multi_images->img_num; i ++) {
      Scalar color(255, 255. * i / (multi_images->img_num - 1), 255 - 255. * i / (multi_images->img_num - 1), 255);
      vector<Edge> edges = multi_images->imgs[i]->getEdges();
      vector<int> edge_indices;
      if (0) {// 只描绘边框
        edge_indices = multi_images->imgs[i]->getBoundaryVertexIndices();
      } else {// 描绘网格
        edge_indices.reserve(edges.size());
        for (int j = 0; j < edges.size(); j ++) {
          edge_indices.emplace_back(j);
        }
      }
      for (int j = 0; j < edge_indices.size(); j ++) {
        line(imgs_border,
             multi_images->image_mesh_points[i][edges[edge_indices[j]].indices[0]] + shift,
             multi_images->image_mesh_points[i][edges[edge_indices[j]].indices[1]] + shift,
             color, line_thickness, LINE_8);
      }
    }
  }
  show_img("result", _result);
}