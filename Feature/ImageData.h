#if !defined(ImageData_H)
#define ImageData_H

#include "../common.h"
#include "../Util/Statistics.h"

class ImageData {
  public:

    char *name;
    Mat data;// 原始数据
    Mat grey_data;// 灰度数据
    Mat rgba_data;// 携带透明度的数据
    Mat alpha_mask;// 图像原有的透明度信息, 从rgba分离得到的

    vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
    vector<Point2f> feature_points;// 特征点(全部)
    vector<vector<Point2f> > matching_points;// 此图片在第i个图片的匹配点

    void readImg(const char *img_path);
};

#endif
