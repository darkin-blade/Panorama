#include "SeamFinder.h"


/***
  *
  * MySeamFinder
  *
  **/

MySeamFinder::MySeamFinder(
    float terminal_cost, 
    float bad_region_penalty)
{
  terminal_cost_ = terminal_cost;
  bad_region_penalty_ = bad_region_penalty;
}

void MySeamFinder::find(
    const std::vector<UMat> &src, 
    const std::vector<Point> &corners,
    std::vector<UMat> &masks)
{
  if (src.size() == 0)
      return;

  // 计算图像梯度
  for (size_t i = 0; i < src.size(); ++i)
  {
    CV_Assert(src[i].channels() == 3);
  }

  images_ = src;
  sizes_.resize(src.size());
  for (size_t i = 0; i < src.size(); ++i)
    sizes_[i] = src[i].size();
  corners_ = corners;
  masks_ = masks;
  
  for (size_t i = 0; i < sizes_.size() - 1; ++i)
  {
    for (size_t j = i + 1; j < sizes_.size(); ++j)
    {
      Rect roi;
      if (overlapRoi(corners_[i], corners_[j], sizes_[i], sizes_[j], roi))
        findInPair(i, j, roi);
    }
  }
}

void MySeamFinder::findInPair(size_t first, size_t second, Rect roi)
{
  Mat img1 = images_[first].getMat(ACCESS_READ), img2 = images_[second].getMat(ACCESS_READ);
  Mat mask1 = masks_[first].getMat(ACCESS_RW), mask2 = masks_[second].getMat(ACCESS_RW);
  Point tl1 = corners_[first], tl2 = corners_[second];

  const int gap = 10;
  Mat subimg1(roi.height + 2 * gap, roi.width + 2 * gap, CV_32FC3);
  Mat subimg2(roi.height + 2 * gap, roi.width + 2 * gap, CV_32FC3);
  Mat submask1(roi.height + 2 * gap, roi.width + 2 * gap, CV_8U);
  Mat submask2(roi.height + 2 * gap, roi.width + 2 * gap, CV_8U);

  // Cut subimages and submasks with some gap
  for (int y = -gap; y < roi.height + gap; ++y)
  {
    for (int x = -gap; x < roi.width + gap; ++x)
    {
      int y1 = roi.y - tl1.y + y;
      int x1 = roi.x - tl1.x + x;
      if (y1 >= 0 && x1 >= 0 && y1 < img1.rows && x1 < img1.cols)
      {
        subimg1.at<Point3f>(y + gap, x + gap) = img1.at<Point3f>(y1, x1);
        submask1.at<uchar>(y + gap, x + gap) = mask1.at<uchar>(y1, x1);
      }
      else
      {
        subimg1.at<Point3f>(y + gap, x + gap) = Point3f(0, 0, 0);
        submask1.at<uchar>(y + gap, x + gap) = 0;
      }

      int y2 = roi.y - tl2.y + y;
      int x2 = roi.x - tl2.x + x;
      if (y2 >= 0 && x2 >= 0 && y2 < img2.rows && x2 < img2.cols)
      {
        subimg2.at<Point3f>(y + gap, x + gap) = img2.at<Point3f>(y2, x2);
        submask2.at<uchar>(y + gap, x + gap) = mask2.at<uchar>(y2, x2);
      }
      else
      {
        subimg2.at<Point3f>(y + gap, x + gap) = Point3f(0, 0, 0);
        submask2.at<uchar>(y + gap, x + gap) = 0;
      }
    }
  }

  const int vertex_count = (roi.height + 2 * gap) * (roi.width + 2 * gap);
  const int edge_count = (roi.height - 1 + 2 * gap) * (roi.width + 2 * gap) +
    (roi.width - 1 + 2 * gap) * (roi.height + 2 * gap);
  GCGraph<float> graph(vertex_count, edge_count);

  setGraphWeightsColor(subimg1, subimg2, submask1, submask2, graph);

  graph.maxFlow();

  for (int y = 0; y < roi.height; ++y)
  {
    for (int x = 0; x < roi.width; ++x)
    {
      if (graph.inSourceSegment((y + gap) * (roi.width + 2 * gap) + x + gap))
      {
        if (mask1.at<uchar>(roi.y - tl1.y + y, roi.x - tl1.x + x))
          mask2.at<uchar>(roi.y - tl2.y + y, roi.x - tl2.x + x) = 0;
      }
      else
      {
        if (mask2.at<uchar>(roi.y - tl2.y + y, roi.x - tl2.x + x))
          mask1.at<uchar>(roi.y - tl1.y + y, roi.x - tl1.x + x) = 0;
      }
    }
  }
}

void MySeamFinder::setGraphWeightsColor(
    const Mat &img1, const Mat &img2,
    const Mat &mask1, const Mat &mask2, GCGraph<float> &graph)
{
  const Size img_size = img1.size();

  // Set terminal weights
  for (int y = 0; y < img_size.height; ++y)
  {
    for (int x = 0; x < img_size.width; ++x)
    {
      int v = graph.addVtx();
      graph.addTermWeights(v, mask1.at<uchar>(y, x) ? terminal_cost_ : 0.f,
          mask2.at<uchar>(y, x) ? terminal_cost_ : 0.f);
    }
  }

  // Set regular edge weights
  const float weight_eps = 1.f;
  for (int y = 0; y < img_size.height; ++y)
  {
    for (int x = 0; x < img_size.width; ++x)
    {
      int v = y * img_size.width + x;
      if (x < img_size.width - 1)
      {
        float weight = normL2(img1.at<Point3f>(y, x), img2.at<Point3f>(y, x)) +
          normL2(img1.at<Point3f>(y, x + 1), img2.at<Point3f>(y, x + 1)) +
          weight_eps;
        if (!mask1.at<uchar>(y, x) || !mask1.at<uchar>(y, x + 1) ||
            !mask2.at<uchar>(y, x) || !mask2.at<uchar>(y, x + 1))
          weight += bad_region_penalty_;
        graph.addEdges(v, v + 1, weight, weight);
      }
      if (y < img_size.height - 1)
      {
        float weight = normL2(img1.at<Point3f>(y, x), img2.at<Point3f>(y, x)) +
          normL2(img1.at<Point3f>(y + 1, x), img2.at<Point3f>(y + 1, x)) +
          weight_eps;
        if (!mask1.at<uchar>(y, x) || !mask1.at<uchar>(y + 1, x) ||
            !mask2.at<uchar>(y, x) || !mask2.at<uchar>(y + 1, x))
          weight += bad_region_penalty_;
        graph.addEdges(v, v + img_size.width, weight, weight);
      }
    }
  }
}