#if !defined(GraphCut_H)
#define GraphCut_H

#include "../common.h"

class MySeamFinderBase
{
public:
  /** FinderBase **/

  MySeamFinderBase(int cost_type, float terminal_cost, float bad_region_penalty)
    : cost_type_(cost_type), terminal_cost_(terminal_cost), bad_region_penalty_(bad_region_penalty) {}

  ~MySeamFinderBase() {}

  std::vector<Mat> dx_, dy_;
  int cost_type_;
  float terminal_cost_;
  float bad_region_penalty_;

  /** PairwiseSeamFinder **/

  void find(const std::vector<UMat> &src, const std::vector<Point> &corners, std::vector<UMat> &masks);
  void findInPair(size_t first, size_t second, Rect roi);

  void setGraphWeightsColor(
      const Mat &img1, const Mat &img2,
      const Mat &mask1, const Mat &mask2, GCGraph<float> &graph);
  void setGraphWeightsColorGrad(
      const Mat &img1, const Mat &img2, const Mat &dx1, const Mat &dx2,
      const Mat &dy1, const Mat &dy2, const Mat &mask1, const Mat &mask2,
      GCGraph<float> &graph);

  void run();

  std::vector<UMat> images_;
  std::vector<Size> sizes_;
  std::vector<Point> corners_;
  std::vector<UMat> masks_;
};

class MySeamFinder
{
public:
  enum CostType { COST_COLOR, COST_COLOR_GRAD };

  MySeamFinder(
    int cost_type = COST_COLOR, 
    float terminal_cost = 10000.f,
    float bad_region_penalty = 1000.f);

  ~MySeamFinder() {}

  void find(
    const std::vector<UMat> &src, 
    const std::vector<Point> &corners, 
    std::vector<UMat> &masks);

  Ptr<MySeamFinderBase> impl_;
};

#endif