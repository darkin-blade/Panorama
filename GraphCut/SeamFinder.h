#if !defined(GraphCut_H)
#define GraphCut_H

#include "../common.h"

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

  class Impl;
  Ptr<PairwiseSeamFinder> impl_;
};

#endif