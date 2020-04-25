#include "NISwGSP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(MultiImages & _multi_images) : MeshOptimization(_multi_images) {
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

  // 描绘特征点
  Mat result_1;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  Mat img1 = multi_images->imgs[0]->data;
  Mat img2 = multi_images->imgs[1]->data;
  result_1 = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
  left_1  = Mat(result_1, Rect(0, 0, img1.cols, img1.rows));
  right_1 = Mat(result_1, Rect(img1.cols, 0, img2.cols, img2.rows));
  // 复制图片
  img1.copyTo(left_1);
  img2.copyTo(right_1);

  if (0) {
    // 匹配所有特征点
    for (int i = 0; i < multi_images->feature_pairs[0][1].size(); i ++) {
      // 计算索引
      int src = multi_images->feature_pairs[0][1][i].first;
      int dst = multi_images->feature_pairs[0][1][i].second;

      // 获取特征点
      Point2f src_p, dst_p;
      src_p = multi_images->imgs[0]->feature_points[src];
      dst_p = multi_images->imgs[1]->feature_points[dst];

      // 描绘
      Scalar color(rand() % 256, rand() % 256, rand() % 256);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
      line(result_1, src_p, dst_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
      circle(result_1, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  } else {
    // 描绘所有特征点
    for (int i = 0; i < multi_images->imgs[0]->feature_points.size(); i ++) {
      Point2f src_p = multi_images->imgs[0]->feature_points[i];
      Scalar color(255, 0, 0);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
    }
    for (int i = 0; i < multi_images->imgs[1]->feature_points.size(); i ++) {
      Point2f src_p = multi_images->imgs[1]->feature_points[i];
      Scalar color(255, 0, 0);
      circle(result_1, src_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  }

  LOG("draw feature matching finished");

  return result_1;
}

Mat NISwGSP_Stitching::matching_match() {
  int img_num = multi_images->img_num;

  // 初始化匹配点信息
  for (int i = 0; i < img_num; i ++) {
    multi_images->imgs[i]->matching_points.resize(img_num);
    multi_images->imgs[i]->homographies.resize(img_num);
  }

  LOG("starting apap");

  // 计算匹配点
  for (int i = 0; i < img_num; i ++) {
    for (int j = 0; j < img_num; j ++) {
      if (i == j) continue;
      int m1 = i;
      int m2 = j;

      // 筛选成功匹配的特征点
      const vector<Point2f> & m1_fpts = multi_images->imgs[m1]->feature_points;
      const vector<Point2f> & m2_fpts = multi_images->imgs[m2]->feature_points;
      vector<Point2f> X, Y;
      for (int k = 0; k < multi_images->feature_pairs[m1][m2].size(); k ++) {
        const pair<int, int> it = multi_images->feature_pairs[m1][m2][k];
        X.emplace_back(m1_fpts[it.first ]);
        Y.emplace_back(m2_fpts[it.second]);
      }

      APAP_Stitching::apap_project(X,
                                   Y,
                                   multi_images->imgs[m1]->getMeshPoints(),
                                   multi_images->imgs[m1]->matching_points[m2],
                                   multi_images->imgs[m1]->homographies[m2]);
      LOG("apap [%d, %d] finish", m1, m2);
    }
  }

  multi_images->keypoints.resize(img_num);
  multi_images->keypoints_mask.resize(img_num);
  multi_images->keypoints_pairs.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    vector<Point2f> tmp_points = multi_images->imgs[i]->getMeshPoints();
    multi_images->keypoints_mask[i].resize(tmp_points.size());
    multi_images->keypoints_pairs[i].resize(img_num);
    for (int j = 0; j < tmp_points.size(); j ++) {
      multi_images->keypoints[i].emplace_back(tmp_points[j]);
    }
  }

  // 记录匹配信息
  multi_images->matching_indices.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    multi_images->matching_indices[i].resize(img_num);
    for (int j = 0; j < img_num; j ++) {
      if (i == j) continue;
      int m1 = i;
      int m2 = j;
      Mat another_img;
      vector<int> & matching_indices = multi_images->matching_indices[m1][m2];// 配对信息
      
      vector<Point2f> tmp_points = multi_images->imgs[m1]->matching_points[m2];// m1 在 m2 上的匹配点
      another_img = multi_images->imgs[m2]->data;
      // LOG("%d %d", another_img.cols, another_img.rows);
      for (int k = 0; k < tmp_points.size(); k ++) {
        cout << k << " " << tmp_points[k].x << " " << tmp_points[k].y << endl;
        if (tmp_points[k].x >= 0
         && tmp_points[k].y >= 0
         && tmp_points[k].x <= another_img.cols
         && tmp_points[k].y <= another_img.rows) {// x对应cols, y对应rows
          // 如果对应的匹配点没有出界
          matching_indices.emplace_back(k);// 记录可行的匹配点
          
          if (m1 < m2) {// 正
            multi_images->keypoints_pairs[m1][m2].emplace_back(make_pair(k, multi_images->keypoints[m2].size()));
            // cout << k << " " << multi_images->keypoints[m2].size() << endl; 
          } else {// 反
            multi_images->keypoints_pairs[m1][m2].emplace_back(make_pair(multi_images->keypoints[m2].size(), k));
            // cout << multi_images->keypoints[m2].size() << " " << k << endl;
          }
          multi_images->keypoints_mask[m1][k] = true;// TODO 标记可行
          multi_images->keypoints[m2].emplace_back(tmp_points[k]);
        }
      }
    }
  }
  
  // 描绘匹配点
  Mat result_1;// 存储结果
  Mat left_1, right_1;// 分割矩阵
  Mat img1 = multi_images->imgs[0]->data;
  Mat img2 = multi_images->imgs[1]->data;
  result_1 = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
  left_1  = Mat(result_1, Rect(0, 0, img1.cols, img1.rows));
  right_1 = Mat(result_1, Rect(img1.cols, 0, img2.cols, img2.rows));
  // 复制图片
  img1.copyTo(left_1);
  img2.copyTo(right_1);

  if (0) {
    // 描绘匹配点配对
    for (int i = 0; i < multi_images->matching_indices[0][1].size(); i ++) {
      int index = multi_images->matching_indices[0][1][i];
      Point2f src_p, dst_p;
      src_p = multi_images->imgs[0]->getMeshPoints()[index];
      dst_p = multi_images->imgs[0]->matching_points[1][index];

      Scalar color(rand() % 256, rand() % 256, rand() % 256);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
      line(result_1, src_p, dst_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
      circle(result_1, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  } else {
    // 描绘所有匹配点
    for (int i = 0; i < multi_images->imgs[0]->getMeshPoints().size(); i ++) {
      Point2f src_p, dst_p;
      src_p = multi_images->imgs[0]->getMeshPoints()[i];
      dst_p = multi_images->imgs[0]->matching_points[1][i];

      Scalar color1(255, 0, 0);
      circle(result_1, src_p, CIRCLE_SIZE, color1, -1);
      Scalar color2(0, 0, 255);
      circle(result_1, dst_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color2, -1);
    }
  }

  LOG("match finished");

  return result_1;
}

void NISwGSP_Stitching::get_solution() {
  alignment_weight               =    1;
  local_similarity_weight        = 0.56;
  global_similarity_weight_beta  =    6;
  global_similarity_weight_gamma =   20;

  vector<Triplet<double> > triplets;
  vector<pair<int, double> > b_vector;

  reserveData(triplets, b_vector, DIMENSION_2D);// TODO

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
