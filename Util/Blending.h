#if !defined(Blending_H)
#define Blending_H

#include "../common.h"

Mat Blending(
  const vector<Mat> & images,
  const vector<Point2f> & origins,
  const Size2f target_size,
  const vector<Mat> & weight_mask,
  const bool ignore_weight_mask = true);


// 记录接缝线位置
void getSeamPts(
  const Mat & src_mask,
  const Mat & dst_mask,
  vector<Point2f> & seam_pts);
void getSeamPts(
  const UMat & src_mask,
  const UMat & dst_mask,
  vector<Point2f> & seam_pts);

// 暴力图像扩展
void getExpandMat(
  Mat & src_image,
  const Mat & dst_image,
  const Mat & src_mask,
  const Mat & dst_mask);

// 计算线型融合的mask
void getGradualMat(
  const Mat & image_1,
  const Mat & image_2,
  const Mat & origin_1,
  const Mat & origin_2,
  Mat & mask_1,    
  Mat & mask_2);

#endif