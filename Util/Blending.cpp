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

void getExpandMat(
  Mat & src_image,
  const Mat & src_mask,
  const Mat & dst_mask
)
{
  assert(src_image.channels() == 4);
  assert(dst_mask.size() == src_mask.size());

  Mat expand = (Scalar(255) - src_mask) & dst_mask;
  int rows = dst_mask.rows;
  int cols = dst_mask.cols;
  int steps[4][2] = {
    {-1, 0}, {0, -1}, {1, 0}, {0, 1}
  };
  Mat visit = Mat::zeros(rows, cols, CV_8UC1);

  // BFS
  queue<pair<int, int> > q;
  for (int i = 0; i < rows; i ++) {
    for (int j = 0; j < cols; j ++) {
      if (src_mask.at<uchar>(i, j)) {
        q.push(make_pair(i, j));
      }
    }
  }

  while (!q.empty()) {
    pair<int, int> u = q.front();
    q.pop();
    int r = u.first;
    int c = u.second;
    Vec4b pix = src_image.at<Vec4b>(r, c);
    for (int i = 0; i < 4; i ++) {
      int next_r = r + steps[i][0];
      int next_c = c + steps[i][1];
      if (next_r >= 0 && next_c >= 0 && next_r < rows && next_c < cols) {
        // 未出界
        if (expand.at<uchar>(next_r, next_c) && !visit.at<uchar>(next_r, next_c)) {
          q.push(make_pair(next_r, next_c));
          src_image.at<Vec4b>(next_r, next_c) = pix;
          visit.at<uchar>(next_r, next_c) = 255;
        }
      }
    }
  }
}

void getGradualMat(
  const Mat & image_1,
  const Mat & image_2,
  const Mat & dst_1,
  const Mat & dst_2,
  Mat & mask_1,    
  Mat & mask_2)
{
  assert(mask_1.size() == mask_2.size());

  // 计算像素梯度
  Mat dx, dy, gradient_1, gradient_2;
  Sobel(image_1, dx, CV_8U, 1, 0);
  Sobel(image_1, dy, CV_8U, 0, 1);
  gradient_1 = dx + dy;
  cvtColor(gradient_1, gradient_1, COLOR_RGBA2GRAY);
  Sobel(image_2, dx, CV_8U, 1, 0);
  Sobel(image_2, dy, CV_8U, 0, 1);
  gradient_2 = dx + dy;
  cvtColor(gradient_2, gradient_2, COLOR_RGBA2GRAY);

  int rows = mask_1.rows;
  int cols = mask_1.cols;
  int steps[8][2] = {
    {-1 ,-1}, {-1, 0}, {-1, 1}, 
    {0, -1}, {0, 1},
    {1, -1}, {1, 0}, {1, 1}
  };
  Mat visit;
  queue<pair<int, int> > q;

  int inner = 1;
  int outer = 10;

  // BFS 1
  for (int i = 0; i < rows; i ++) {
    for (int j = 0; j < cols; j ++) {
      if (mask_1.at<uchar>(i, j)) {
        q.push(make_pair(i, j));
      }
    }
  }

  while (!q.empty()) {
    pair<int, int> u = q.front();
    q.pop();
    int r = u.first;
    int c = u.second;
    int depth = mask_1.at<uchar>(r, c);
    if (depth == 0) {
      continue;
    }
    for (int i = 0; i < 8; i ++) {
      int next_r = r + steps[i][0];
      int next_c = c + steps[i][1];
      if (next_r >= 0 && next_c >= 0 && next_r < rows && next_c < cols) {
        // 未出界
        if (dst_2.at<uchar>(next_r, next_c)) {
          // 图像扩展部分
          if (gradient_1.at<uchar>(next_r, next_c) < 5
           && gradient_2.at<uchar>(next_r, next_c) < 5) {
            int next_depth = max(depth - outer, 0);
            if (mask_1.at<uchar>(next_r, next_c) < next_depth) {
              q.push(make_pair(next_r, next_c));
              mask_1.at<uchar>(next_r, next_c) = next_depth;
            }
          }
        } else if (dst_1.at<uchar>(next_r, next_c)) {
          // 图像原有部分
          int next_depth = max(depth - inner, 0);
          if (mask_1.at<uchar>(next_r, next_c) < next_depth) {
            // 权值覆盖
            q.push(make_pair(next_r, next_c));
            mask_1.at<uchar>(next_r, next_c) = next_depth;
          }
        }
      }
    }
  }

  // BFS 2
  for (int i = 0; i < rows; i ++) {
    for (int j = 0; j < cols; j ++) {
      if (mask_2.at<uchar>(i, j)) {
        q.push(make_pair(i, j));
      }
    }
  }

  while (!q.empty()) {
    pair<int, int> u = q.front();
    q.pop();
    int r = u.first;
    int c = u.second;
    int depth = mask_2.at<uchar>(r, c);
    if (depth == 0) {
      continue;
    }
    for (int i = 0; i < 8; i ++) {
      int next_r = r + steps[i][0];
      int next_c = c + steps[i][1];
      if (next_r >= 0 && next_c >= 0 && next_r < rows && next_c < cols) {
        // 未出界
        if (dst_1.at<uchar>(next_r, next_c)) {
          // 图像扩展部分
          if (gradient_2.at<uchar>(next_r, next_c) < 2
           && gradient_1.at<uchar>(next_r, next_c) < 2) {
            int next_depth = max(depth - outer, 0);
            if (mask_2.at<uchar>(next_r, next_c) < next_depth) {
              q.push(make_pair(next_r, next_c));
              mask_2.at<uchar>(next_r, next_c) = next_depth;
            }
          }
        } else if (dst_2.at<uchar>(next_r, next_c)) {
          // 图像原有部分
          int next_depth = max(depth - inner, 0);
          if (mask_2.at<uchar>(next_r, next_c) < next_depth) {
            // 权值覆盖
            q.push(make_pair(next_r, next_c));
            mask_2.at<uchar>(next_r, next_c) = next_depth;
          }
        }
      }
    }
  }

  show_img("g 2", gradient_2);
  show_img("1", mask_1);
  show_img("2", mask_2);
}