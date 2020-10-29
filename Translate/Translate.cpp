#include "Translate.h"

void Translate::computeTranslation(Mat _img1, Mat _img2, double _r1, double _r2, double _r3, Mat & translate) {
  // 计算两张图片的平移

  clock_t begin_time, end_time;
  begin_time = clock();

  // 0 处理成灰度图
  assert(_img1.channels() == _img2.channels());
  if (_img1.channels() == 3) {
    cvtColor(_img1, _img1, CV_RGB2GRAY);
  } else if (_img1.channels() == 4) {
    cvtColor(_img1, _img1, CV_RGBA2GRAY);
  }
  // 1 检测特征点
  vector<KeyPoint> keyPoints1;
  vector<KeyPoint> keyPoints2;
  Ptr<SiftFeatureDetector> detector = SiftFeatureDetector::create(2000);
  detector->detect(_img1, keyPoints1);
  detector->detect(_img2, keyPoints2);
  // 2 获取特征点描述
  Ptr<SiftDescriptorExtractor> extractor = SiftDescriptorExtractor::create();
  Mat descriptor1;
  Mat descriptor2;
  extractor->compute(_img1, keyPoints1, descriptor1);
  extractor->compute(_img2, keyPoints2, descriptor2);
  if (descriptor1.type() != CV_32F) {
    descriptor1.convertTo(descriptor1, CV_32F);
  }
  if (descriptor2.type() != CV_32F) {
    descriptor2.convertTo(descriptor2, CV_32F);
  }
  // 3 特征点配对
  Ptr<FlannBasedMatcher> matcher = FlannBasedMatcher::create();
  vector<DMatch> initialMatches;
  matcher->match(descriptor1, descriptor2, initialMatches, Mat());
  // 4 特征点对筛选
  // DEBUG
  Mat result;
  drawMatches(_img1, keyPoints1, _img2, keyPoints2, initialMatches, result);

  end_time = clock();
  LOG("totoal time %lf", (double)(end_time - begin_time)/CLOCKS_PER_SEC);
  show_img("fast", result);
}