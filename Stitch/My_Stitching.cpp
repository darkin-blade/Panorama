#include "My_Stitching.h"

My_Stitching::My_Stitching(MultiImages & _multi_images) {
  multi_images = & _multi_images;
}

Mat My_Stitching::getMyResult() {
  int img_num = multi_images->img_num;

  assert(multi_images->imgs.empty());
  for (int i = 0; i < img_num; i ++) {
    // 读取图像: 从multi_images到ImageData类中
    multi_images->imgs.emplace_back(new ImageData());
    // multi_images->imgs[i]->initData();// 单纯地清空所有数据
    multi_images->imgs[i]->readImg(multi_images->origin_data[i], 1);
  }

  // 特征检测及特征点匹配
  multi_images->getFeatureInfo();
  // apap网格计算
  multi_images->getMeshInfo();

  // 相似变换
  vector<double> angles;
  for (int i = 0; i < img_num; i ++) {
    angles.emplace_back(0);// 负为逆时针
  }
  multi_images->similarityTransform(1, angles);

  // 网格优化
  multi_images->meshOptimization();

  // 不进行网格优化, 直接计算APAP结果
  // multi_images->getAPAPResult();

  // 接缝线算法
  multi_images->myWarping();
  multi_images->getSeam();// 图像融合过程内嵌在这个函数中

  return Mat();// TODO
}

void My_Stitching::debug() {
  assert(0);
}