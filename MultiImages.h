#if !defined(COMMON_H)
#include "common.h"
#endif

using namespace std;

using namespace cv;
using namespace cv::detail;
using namespace cv::xfeatures2d;

class MultiImages {// 注意reserve与resize的区别
public:
  int img_num;
  vector<Mat> imgs;
  vector<vector<Point2f> > img_mesh;// 划分图像
  vector<vector<KeyPoint> > key_points;// 关键点
  vector<vector<Point2f> > feature_points;// 特征点
  vector<vector<Point2f> > matching_points;// 匹配点 要记得初始化
  vector<vector<Mat> > homographies;// 单应矩阵 要记得初始化

  vector<vector<pair<int, int> > > matching_pairs;// 匹配点配对信息
  vector<Mat> descriptor;
  vector<DMatch> feature_matches;
};
