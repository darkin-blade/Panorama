#include "NISwGSP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(MultiImages & _multi_images) : MeshOptimization(_multi_images) {
}

void NISwGSP_Stitching::featureMatch() {
  int img_num = multi_images->img_num;

  for (int i = 0; i < img_num; i ++) {
    Mat grey_img;
    cvtColor(multi_images->imgs[i]->data, grey_img, CV_BGR2GRAY);
    FeatureController::detect(grey_img,
                              multi_images->imgs[i]->feature_points,
                              multi_images->imgs[i]->descriptors);
    LOG("[picture %d] feature points: %ld", i, multi_images->imgs[i]->feature_points.size());
  }

  // 特征点匹配(内含自动检测图片匹配)
  multi_images->getFeaturePairs();

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
}

void NISwGSP_Stitching::matchingMatch() {
  multi_images->do_matching();
}

Mat NISwGSP_Stitching::apapResult() {
  show_img("feature", drawFeatureMatch());

  // 修改最终mesh结果
  multi_images->image_mesh_points.emplace_back(multi_images->imgs[0]->matching_points[1]);
  multi_images->image_mesh_points.emplace_back(multi_images->imgs[1]->getVertices());
  multi_images->warpImages();
  return multi_images->textureMapping();
}

void NISwGSP_Stitching::getMesh() {
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
}

Mat NISwGSP_Stitching::textureMapping() {
  multi_images->using_seam_finder = true;// 如果启用, 那么Blending不会平滑过渡
  // 手动对图像进行形变
  multi_images->warpImages();
  // 曝光补偿
  multi_images->exposureCompensate();
  // 图像分块
  // multi_images->getBlock();
  // 寻找接缝线
  multi_images->getSeam();

  if (0) {
    // 只绘制最终mesh点
    Size2f target_size = normalizeVertices(multi_images->image_mesh_points);
    Mat result;
    result = Mat::zeros(round(target_size.height), round(target_size.width), CV_8UC4);
    vector<vector<Point2f> > & image_mesh_points = multi_images->image_mesh_points;
    for (int i = 0; i < multi_images->img_num; i ++) {
      for (int j = 0; j < image_mesh_points[i].size(); j ++) {
        Point2f tmp = image_mesh_points[i][j];
        if (i) {
          circle(result, tmp, CIRCLE_SIZE, Scalar(255, 0, 0), -1);
        } else {
          circle(result, tmp, CIRCLE_SIZE, Scalar(255, 255, 0), -1);
        }
      }
    }
    return result;
  } else if (0) {
    Mat result = multi_images->textureMapping();
    // 图像描边
    int line_thickness = 1;// 描边的线宽
    Mat imgs_border(result.size() + Size(line_thickness * 6, line_thickness * 6), CV_8UC4);
    Point2f shift(line_thickness * 3, line_thickness * 3);// 偏移
    Rect rect(shift, result.size());
    result.copyTo(imgs_border);
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
    return result;
  }
}

Mat NISwGSP_Stitching::drawFeatureMatch() {
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
      // 匹配RANSAC之后的所有特征点
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
    } else if (1) {
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
  return result_1;
}

Mat NISwGSP_Stitching::drawMatchingMatch() {
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
    } else {
      // 描绘所有匹配点
      for (int i = 0; i < multi_images->imgs[m1]->getVertices().size(); i ++) {
        Point2f src_p, dst_p;
        src_p = multi_images->imgs[m1]->getVertices()[i];
        dst_p = multi_images->imgs[m1]->matching_points[m2][i];

        Scalar color1(255, 0, 0);
        circle(result_1, src_p, CIRCLE_SIZE, color1, -1);
        Scalar color2(0, 0, 255);
        circle(result_1, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color2, -1);
      }
    }
  }
  return result_1;
}