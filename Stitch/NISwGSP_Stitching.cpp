#include "NISwGSP_Stitching.h"
#include "APAP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(MultiImages &multiImages) {
  this->multiImages = &multiImages;
}

Mat NISwGSP_Stitching::feature_match() {
  int img_num = multiImages->img_num;

  // 检测特征点
#if !defined(using_opencv)
  for (int i = 0; i < img_num; i ++) {
    Mat grey_img;
    cvtColor(multiImages->imgs[i]->data, grey_img, CV_BGR2GRAY);
    FeatureController::detect(grey_img,
                              multiImages->imgs[i]->feature_points,
                              multiImages->imgs[i]->descriptors);
    LOG("[picture %d] feature points: %ld", i, multiImages->imgs[i]->feature_points.size());
  }
#endif

  // 特征点匹配
  multiImages->getFeaturePairs();

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

  if (0) {
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

Mat NISwGSP_Stitching::matching_match() {
  int img_num = multiImages->img_num;

  // 划分图像mesh
  for (int i = 0; i < img_num; i ++) {
    int cols = multiImages->imgs[i]->data.cols;
    int rows = multiImages->imgs[i]->data.rows;
    
    // 计算间距
    double ratio = ((double) cols) / rows;
    int row_num = 20;
    int col_num = (int) (ratio * 20);
    double col_step = ((double) cols) / col_num;
    double row_step = ((double) rows) / row_num;

    // 添加mesh
    for (int j = 0; j <= col_num; j ++) {
      for (int k = 0; k <= row_num; k ++) {
        multiImages->imgs[i]->mesh_points.push_back(Point2f(j * col_step, k * row_step));
      }
    }
  }

  LOG("get mesh");

  // 初始化匹配点信息
  for (int i = 0; i < img_num; i ++) {
    multiImages->imgs[i]->matching_points.resize(img_num);
    multiImages->imgs[i]->homographies.resize(img_num);
  }

  LOG("starting apap");

  // 计算匹配点
  for (int i = 0; i < img_num; i ++) {
    for (int j = 0; j < img_num; j ++) {
      if (i == j) continue;
      int m1 = i;
      int m2 = j;

      // 筛选成功匹配的特征点
      const vector<Point2f> & m1_fpts = multiImages->imgs[m1]->feature_points;
      const vector<Point2f> & m2_fpts = multiImages->imgs[m2]->feature_points;
      vector<Point2f> X, Y;
      for (int k = 0; k < multiImages->feature_pairs[m1][m2].size(); k ++) {
        const pair<int, int> it = multiImages->feature_pairs[m1][m2][k];
        X.emplace_back(m1_fpts[it.first ]);
        Y.emplace_back(m2_fpts[it.second]);
      }

      APAP_Stitching::apap_project(X,
                                   Y,
                                   multiImages->imgs[m1]->mesh_points,
                                   multiImages->imgs[m1]->matching_points[m2],
                                   multiImages->imgs[m1]->homographies[m2]);
      LOG("apap [%d, %d] finish", m1, m2);
    }
  }

  // 记录匹配信息
  multiImages->matching_pairs.resize(img_num);
  for (int i = 0; i < img_num; i ++) {
    multiImages->matching_pairs[i].resize(img_num);

    for (int j = 0; j < img_num; j ++) {
      if (i == j) continue;
      int m1 = i;
      int m2 = j;
      Mat another_img = multiImages->imgs[m2]->data;

      // 剔除出界点
      vector<pair<int, int> > &matching_pairs = multiImages->matching_pairs[m1][m2];// 配对信息
      const vector<Point2f> *tmp_p = &multiImages->imgs[m1]->matching_points[m2];// 匹配点位置
      for (int k = 0; k < tmp_p->size(); k ++) {
        if ((*tmp_p)[k].x >= 0
          && (*tmp_p)[k].y >= 0
          && (*tmp_p)[k].x <= another_img.cols
          && (*tmp_p)[k].y <= another_img.rows) {// x对应cols, y对应rows
          // 如果对应的匹配点没有出界
          matching_pairs.push_back(pair<int, int>(k, k));
        }
      }
    }
  }
  
  // 描绘匹配点
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

  if (0) {
    // 描绘匹配点配对
    for (int i = 0; i < multiImages->matching_pairs[0][1].size(); i ++) {
      int index = multiImages->matching_pairs[0][1][i].first;// first == second
      Point2f src_p, dest_p;
      src_p  = multiImages->imgs[0]->mesh_points[index];
      dest_p = multiImages->imgs[0]->matching_points[1][index];

      Scalar color(rand() % 256, rand() % 256, rand() % 256);
      circle(result_1, src_p, CIRCLE_SIZE, color, -1);
      line(result_1, src_p, dest_p + Point2f(img1.cols, 0), color, LINE_SIZE, LINE_AA);
      circle(result_1, dest_p + Point2f(img1.cols, 0), CIRCLE_SIZE, color, -1);
    }
  } else {
    // 描绘所有匹配点
    for (int i = 0; i < multiImages->imgs[0]->mesh_points.size(); i ++) {
      Point2f src_p, dest_p;
      src_p  = multiImages->imgs[0]->mesh_points[i];
      dest_p = multiImages->imgs[0]->matching_points[1][i];

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

  // 保存图片
  char img_name[100];
  char full_name[128];
  LOG("input file name:");
  scanf("%s", img_name);
  sprintf(full_name, "../../%s.jpg", img_name);
  if (strlen(full_name) > 12) {
    imwrite(full_name, img);
  } else {
    LOG("invalid name");
  }
#endif
}
