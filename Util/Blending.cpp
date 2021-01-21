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
  const Mat & dst_image,
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

  // 使用单一颜色填充带扩展区域
  while (!q.empty()) {
    pair<int, int> u = q.front();
    q.pop();
    int r = u.first;
    int c = u.second;
    Vec4b src_pix = src_image.at<Vec4b>(r, c);
    for (int i = 0; i < 4; i ++) {
      int next_r = r + steps[i][0];
      int next_c = c + steps[i][1];
      Vec4b dst_pix = dst_image.at<Vec4b>(next_r, next_c);
      if (next_r >= 0 && next_c >= 0 && next_r < rows && next_c < cols) {
        // 未出界
        if (expand.at<uchar>(next_r, next_c)) {
          uchar diff = abs(src_pix[0] - dst_pix[0]) + abs(src_pix[1] - dst_pix[1]) + abs(src_pix[2] - dst_pix[2]);
          if (diff < 5000 && !visit.at<uchar>(next_r, next_c)) {
            // 未访问的需要扩展的区域
            src_image.at<Vec4b>(next_r, next_c) = src_pix;
            q.push(make_pair(next_r, next_c));
            visit.at<uchar>(next_r, next_c) = 255;
          }
        }
      }
    }
  }

  // 羽化扩充区域
  // Mat blured;
  // src_image.copyTo(blured);
  // blur(blured, blured, Size(9, 9));
  // blured.copyTo(src_image, dst_mask);
}

void getGradualMat(
  const Mat & image_1,
  const Mat & image_2,
  const Mat & origin_1,
  const Mat & origin_2,
  Mat & mask_1,    
  Mat & mask_2)
{
  assert(mask_1.size() == mask_2.size());

  // 计算像素梯度
  // Mat dx, dy, gradient_1, gradient_2;
  // Sobel(image_1, dx, CV_8U, 1, 0);
  // Sobel(image_1, dy, CV_8U, 0, 1);
  // gradient_1 = dx + dy;
  // cvtColor(gradient_1, gradient_1, COLOR_RGBA2GRAY);
  // Sobel(image_2, dx, CV_8U, 1, 0);
  // Sobel(image_2, dy, CV_8U, 0, 1);
  // gradient_2 = dx + dy;
  // cvtColor(gradient_2, gradient_2, COLOR_RGBA2GRAY);

  int rows = mask_1.rows;
  int cols = mask_1.cols;
  int steps[8][2] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  };
  Mat visit;
  queue<pair<int, int> > q;

  for (int k = 0; k < 2; k ++) {
    Mat m1, m2;// mask
    Mat o1, o2;// origin
    Mat i1, i2;// image
    if (k == 0) {
      m1 = mask_1, m2 = mask_2;
      o1 = origin_1, o2 = origin_2;
      i1 = image_1, i2 = image_2;
    } else {
      m1 = mask_2, m2 = mask_1;
      o1 = origin_2, o2 = origin_1;
      i1 = image_2, i2 = image_1;
    }

    visit = Mat::zeros(rows, cols, CV_8UC1);
    for (int i = 0; i < rows; i ++) {
      for (int j = 0; j < cols; j ++) {
        if (m1.at<uchar>(i, j)) {
          q.push(make_pair(i, j));
          visit.at<uchar>(i, j) = 255;
        }
      }
    }

    while (!q.empty()) {
      pair<int, int> u = q.front();
      q.pop();
      int r = u.first;
      int c = u.second;
      int depth = m1.at<uchar>(r, c);
      for (int i = 0; i < 4; i ++) {
        int next_r = r + steps[i][0];
        int next_c = c + steps[i][1];
        if (next_r >= 0 && next_c >= 0 && next_r < rows && next_c < cols) {
          // 未出界
          if (o1.at<uchar>(next_r, next_c)
          || o2.at<uchar>(next_r, next_c)) {// 图像原有/扩展部分
            // 比对色差
            Vec4b pix_1 = i1.at<Vec4b>(next_r, next_c);
            Vec4b pix_2 = i2.at<Vec4b>(next_r, next_c);
            int color_dis = (abs(pix_1[0] - pix_2[0]) + abs(pix_1[1] - pix_2[1]) + abs(pix_1[2] - pix_2[2])) / 3;

            int next_depth = depth - color_dis;
            if (next_depth <= 0) {
              continue;
            } else if (!visit.at<uchar>(next_r, next_c) && m1.at<uchar>(next_r, next_c) < next_depth) {
              q.push(make_pair(next_r, next_c));
              m1.at<uchar>(next_r, next_c) = next_depth;
              visit.at<uchar>(next_r, next_c) = 255;
            }
          }
        }
      }
    }
  }
}