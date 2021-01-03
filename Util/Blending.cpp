#include "Blending.h"

Mat Blending(const vector<Mat> & images,
    const vector<Point2f> & origins,
    const Size2f target_size,
    const vector<Mat> & weight_mask,
    const bool ignore_weight_mask) 
{

  Mat result = Mat::zeros(round(target_size.height), round(target_size.width), CV_8UC4);

  vector<Rect2f> rects;
  rects.reserve(origins.size());
  for (int i = 0; i < origins.size(); i ++) {
    rects.emplace_back(origins[i], images[i].size());
  }
  for (int y = 0; y < result.rows; y ++) {
    for (int x = 0; x < result.cols; x ++) {
      Point2i p(x, y);
      Vec3f pixel_sum(0, 0, 0);
      float weight_sum = 0.f;
      for (int i = 0; i < rects.size(); i ++) {
        Point2i pv(round(x - origins[i].x), round(y - origins[i].y));
        if (pv.x >= 0 && pv.x < images[i].cols &&
            pv.y >= 0 && pv.y < images[i].rows) {
          Vec4b v = images[i].at<Vec4b>(pv);
          Vec3f value = Vec3f(v[0], v[1], v[2]);
          if (ignore_weight_mask) {
            if (v[3] > 127) {
              pixel_sum += value;
              weight_sum += 1.f;
            }
          } else {
            float weight = weight_mask[i].at<float>(pv);
            pixel_sum += weight * value;
            weight_sum += weight;
          }
        }
      }
      if (weight_sum) {
        pixel_sum /= weight_sum;
        result.at<Vec4b>(p) = Vec4b(round(pixel_sum[0]), round(pixel_sum[1]), round(pixel_sum[2]), 255);
      }
    }
  }
  return result;
}

void getGradualMat(
  const Mat & dst_mat,    // dst - src 为待填充区域
  Mat & src_mat)          // 初始区域
{

  assert(src_mat.size() == dst_mat.size());

  int rows = dst_mat.rows;
  int cols = dst_mat.cols;
  int steps[8][2] = {
    {-1 ,-1}, {-1, 0}, {-1, 1}, 
    {0, -1}, {0, 1},
    {1, -1}, {1, 0}, {1, 1}
  };
  Mat visit = Mat::zeros(rows, cols, CV_8UC1);

  // BFS
  queue<pair<int, int> > q;
  for (int i = 0; i < rows; i ++) {
    for (int j = 0; j < cols; j ++) {
      if (src_mat.at<uchar>(i, j)) {
        q.push(make_pair(i, j));
        visit.at<uchar>(i, j) = 255;
      }
    }
  }

  int min_depth = 255;
  while (!q.empty()) {
    pair<int, int> u = q.front();
    q.pop();
    int r = u.first;
    int c = u.second;
    int depth = src_mat.at<uchar>(r, c);
    int next_depth = depth - 20;
    for (int i = 0; i < 8; i ++) {
      int next_r = r + steps[i][0];
      int next_c = c + steps[i][1];
      if (next_r >= 0 && next_c >= 0 && next_r < rows && next_c < cols) {
        // 未出界
        if (dst_mat.at<uchar>(next_r, next_c) && !visit.at<uchar>(next_r, next_c)) {
          // 未访问, 未出界
          q.push(make_pair(next_r, next_c));
          src_mat.at<uchar>(next_r, next_c) = max(0, next_depth);
          min_depth = min(next_depth, min_depth);
          visit.at<uchar>(next_r, next_c) = 255;
        }
      }
    }
  }
}