#if !defined(Blending_H)
#define Blending_H

#include "../common.h"

Mat Blending(
  const vector<Mat> & images,
  const vector<Point2f> & origins,
  const Size2f target_size,
  const vector<Mat> & weight_mask,
  const bool ignore_weight_mask = true);

void getGradualMat(
  const Mat & dst_mat,    // dst - src 为待填充区域
  Mat & src_mat);         // 初始区域

#endif