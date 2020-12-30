#if !defined(GraphCut_H)
#define GraphCut_H

#include "../common.h"

class MySeamFinder
{
public:
  MySeamFinder(
    float terminal_cost = 10000.f,
    float bad_region_penalty = 1000.f);

  ~MySeamFinder() {}

  int cost_type_;
  float terminal_cost_;
  float bad_region_penalty_;

  // 从前往后两两配对
  void find(const std::vector<UMat> &src, const std::vector<Point> &corners, std::vector<UMat> &masks);
  // 两两计算接缝线
  void findInPair(size_t first, size_t second, Rect roi);
  // 建图
  void setGraphWeightsColor(
      const Mat &img1, const Mat &img2, const Mat &dx1, const Mat &dx2,
      const Mat &dy1, const Mat &dy2, const Mat &mask1, const Mat &mask2,
      GCGraph<float> &graph);

  std::vector<Mat> dx_, dy_;
  std::vector<UMat> images_;
  std::vector<Size> sizes_;
  std::vector<Point> corners_;
  std::vector<UMat> masks_;
};

#endif