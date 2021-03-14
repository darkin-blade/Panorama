#include "My_Stitching.h"

My_Stitching::My_Stitching(MultiImages & _multi_images) {
  multi_images = & _multi_images;
}

Mat My_Stitching::getMyResult() {
  int img_num = multi_images->img_num;
  multi_images->imgs.resize(2);
  multi_images->imgs[0] = new ImageData();
  multi_images->imgs[1] = new ImageData();

  for (int i = 1; i < img_num; i ++) {
    // 初始化
    multi_images->imgs[0]->initData();
    multi_images->imgs[1]->initData();

    // 目标图片序号为0, 参考图片序号为1
    if (i == 1) {
      // 使用原始图片
      // 目标图片
      multi_images->imgs[0]->readImg(multi_images->origin_data[i], 1);
      // 参考图片
      multi_images->imgs[1]->readImg(multi_images->origin_data[i - 1], 1);
    } else {
      // 使用之前的结果
      // 目标图片
      multi_images->imgs[0]->readImg(multi_images->origin_data[i], 1);
      // 参考图片
      multi_images->imgs[1]->readImg(multi_images->pano_result, 2);
    }
    
    // 进行拼接

    multi_images->getMeshInfo();
    multi_images->similarityTransform(1, 0);// 负为逆时针
    multi_images->meshOptimization();

    // multi_images->textureMapping(0);
    // drawFeatureMatch();

    multi_images->myWarping();
    multi_images->getSeam();
    
    // drawMatchingPts();
    // show_img("result", multi_images->pano_result);
  }

  // 读取图像
  // 特征检测
  // apap网格计算
  // 相似变换
  // 网格优化
  // 接缝线算法

  return multi_images->pano_result;
}

void My_Stitching::debug() {
  assert(0);
}