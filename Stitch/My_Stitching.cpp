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

  // 初始化
  multi_images->init();

  // 初步特征点匹配
  multi_images->getFeatureInfo();

  // 相似变换
  multi_images->similarityTransform(0);
  multi_images->getImagePairs();
  // 补充特征点匹配
  multi_images->getFeatureInfo();

  // apap网格计算
  multi_images->getMeshInfo();

  // 网格优化
  multi_images->meshOptimization();

  // 计算中间结果
  // multi_images->getTmpResult();

  // 接缝线算法
  multi_images->myWarping();
  multi_images->getSeam();// 图像融合过程内嵌在这个函数中

  return Mat();// TODO
}

void My_Stitching::debug() {
  assert(0);
}