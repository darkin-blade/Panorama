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
  dx_.resize(src.size());
  dy_.resize(src.size());
  Mat dx, dy;
  for (size_t i = 0; i < src.size(); ++i)
  {
    CV_Assert(src[i].channels() == 3);
    Sobel(src[i], dx, CV_32F, 1, 0);
    Sobel(src[i], dy, CV_32F, 0, 1);

    // dx += dy;
    convertScaleAbs(dx, dx_[i]);
    convertScaleAbs(dy, dy_[i]);
    // dx.copyTo(dx_[i]);
    // dx.copyTo(dy_[i]);
    // dx_[i].create(src[i].size(), CV_32F);
    // dy_[i].create(src[i].size(), CV_32F);
    // for (int y = 0; y < src[i].rows; ++y)
    // {
    //   const Point3f* dx_row = dx.ptr<Point3f>(y);
    //   const Point3f* dy_row = dy.ptr<Point3f>(y);
    //   float* dx_row_ = dx_[i].ptr<float>(y);
    //   float* dy_row_ = dy_[i].ptr<float>(y);
    //   for (int x = 0; x < src[i].cols; ++x)
    //   {
    //     dx_row_[x] = normL2(dx_row[x]);
    //     dy_row_[x] = normL2(dy_row[x]);
    //   }
    // }
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
  Mat dx1 = dx_[first], dx2 = dx_[second];
  Mat dy1 = dy_[first], dy2 = dy_[second];
  Mat mask1 = masks_[first].getMat(ACCESS_RW), mask2 = masks_[second].getMat(ACCESS_RW);
  Point tl1 = corners_[first], tl2 = corners_[second];

  const int gap = 10;
  Mat subimg1(roi.height + 2 * gap, roi.width + 2 * gap, CV_32FC3);
  Mat subimg2(roi.height + 2 * gap, roi.width + 2 * gap, CV_32FC3);
  Mat submask1(roi.height + 2 * gap, roi.width + 2 * gap, CV_8U);
  Mat submask2(roi.height + 2 * gap, roi.width + 2 * gap, CV_8U);
  Mat subdx1(roi.height + 2 * gap, roi.width + 2 * gap, CV_32F);
  Mat subdy1(roi.height + 2 * gap, roi.width + 2 * gap, CV_32F);
  Mat subdx2(roi.height + 2 * gap, roi.width + 2 * gap, CV_32F);
  Mat subdy2(roi.height + 2 * gap, roi.width + 2 * gap, CV_32F);

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
        subdx1.at<float>(y + gap, x + gap) = dx1.at<float>(y1, x1);
        subdy1.at<float>(y + gap, x + gap) = dy1.at<float>(y1, x1);
      }
      else
      {
        subimg1.at<Point3f>(y + gap, x + gap) = Point3f(0, 0, 0);
        submask1.at<uchar>(y + gap, x + gap) = 0;
        subdx1.at<float>(y + gap, x + gap) = 0.f;
        subdy1.at<float>(y + gap, x + gap) = 0.f;
      }

      int y2 = roi.y - tl2.y + y;
      int x2 = roi.x - tl2.x + x;
      if (y2 >= 0 && x2 >= 0 && y2 < img2.rows && x2 < img2.cols)
      {
        subimg2.at<Point3f>(y + gap, x + gap) = img2.at<Point3f>(y2, x2);
        submask2.at<uchar>(y + gap, x + gap) = mask2.at<uchar>(y2, x2);
        subdx2.at<float>(y + gap, x + gap) = dx2.at<float>(y2, x2);
        subdy2.at<float>(y + gap, x + gap) = dy2.at<float>(y2, x2);
      }
      else
      {
        subimg2.at<Point3f>(y + gap, x + gap) = Point3f(0, 0, 0);
        submask2.at<uchar>(y + gap, x + gap) = 0;
        subdx2.at<float>(y + gap, x + gap) = 0.f;
        subdy2.at<float>(y + gap, x + gap) = 0.f;
      }
    }
  }

  const int vertex_count = (roi.height + 2 * gap) * (roi.width + 2 * gap);
  const int edge_count = (roi.height - 1 + 2 * gap) * (roi.width + 2 * gap) + (roi.width - 1 + 2 * gap) * (roi.height + 2 * gap);
  GCGraph<float> graph(vertex_count, edge_count);

  setGraphWeightsColor(subimg1, subimg2, subdx1, subdx2, subdy1, subdy2, submask1, submask2, graph);

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
    const Mat &img1, const Mat &img2, const Mat &dx1, const Mat &dx2,
    const Mat &dy1, const Mat &dy2, const Mat &mask1, const Mat &mask2,
    GCGraph<float> &graph)
{
  const Size img_size = img1.size();

  // Set terminal weights
  for (int y = 0; y < img_size.height; ++y)
  {
    for (int x = 0; x < img_size.width; ++x)
    {
      int v = graph.addVtx();
      float weight1, weight2;
      Point2i p(x, y);
      if (mask1.at<uchar>(p)) {
        weight1 = terminal_cost_ - 0.02 * (dx1.at<uchar>(p) + dy1.at<uchar>(p));
      } else {
        weight1 = 0;
      }
      if (mask2.at<uchar>(p)) {
        weight2 = terminal_cost_ - 0.02 * (dx2.at<uchar>(p) + dy2.at<uchar>(p));
      } else {
        weight2 = 0;
      }
      graph.addTermWeights(v, weight1, weight2);
    }
  }

  // Set regular edge weights
  const float weight_eps = 1.f;
  const float alpha = 0.f;
  for (int y = 0; y < img_size.height; ++y)
  {
    for (int x = 0; x < img_size.width; ++x)
    {
      int v = y * img_size.width + x;
      if (x < img_size.width - 1)
      {// 向右
        Point2i p(x, y), q(x + 1, y);
        float weight_Dp = normL2(img1.at<Point3f>(p), img2.at<Point3f>(p));
          // + alpha * (dx1.at<uchar>(p) - dx2.at<uchar>(p)) * (dx1.at<uchar>(p) - dx2.at<uchar>(p));
        float weight_Dq = normL2(img1.at<Point3f>(q), img2.at<Point3f>(q));
          // + alpha * (dx1.at<uchar>(q) - dx2.at<uchar>(q)) * (dx1.at<uchar>(q) - dx2.at<uchar>(q));
        float weight = weight_Dp + weight_Dq;
        assert(weight_Dp >= 0 && weight_Dq >= 0);

        if (!mask1.at<uchar>(p) || !mask1.at<uchar>(q) ||
            !mask2.at<uchar>(p) || !mask2.at<uchar>(q))
          weight += bad_region_penalty_;
        graph.addEdges(v, v + 1, weight, weight);
      }
      if (y < img_size.height - 1)
      {// 向下
        Point2i p(x, y), q(x, y + 1);
        float weight_Dp = normL2(img1.at<Point3f>(p), img2.at<Point3f>(p));
          // + alpha * (dy1.at<uchar>(p) - dy2.at<uchar>(p)) * (dy1.at<uchar>(p) - dy2.at<uchar>(p));
        float weight_Dq = normL2(img1.at<Point3f>(q), img2.at<Point3f>(q));
          // + alpha * (dy1.at<uchar>(q) - dy2.at<uchar>(q)) * (dy1.at<uchar>(q) - dy2.at<uchar>(q));
        float weight = weight_Dp + weight_Dq;
        assert(weight_Dp >= 0 && weight_Dq >= 0);

        if (!mask1.at<uchar>(p) || !mask1.at<uchar>(q) ||
            !mask2.at<uchar>(p) || !mask2.at<uchar>(q))
          weight += bad_region_penalty_;
        graph.addEdges(v, v + img_size.width, weight, weight);
      }
    }
  }
}
