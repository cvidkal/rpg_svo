#include <svo/sparse_img_align.h>
#include <svo/feature.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

class testWithTUM{
public:

    testWithTUM(std::string datasetPath,std::string assocFilePath) {
      assoc = fopen(assocFilePath.c_str(),"r");
        printf("hh");
        char rgb_[200],depth_[200];
        while(fscanf(assoc,"%s %s",rgb_,depth_)!=EOF){
            auto i = cv::imread(datasetPath+string(rgb_));
            img.push_back(i);
            i = cv::imread(datasetPath+string(depth_),CV_LOAD_IMAGE_UNCHANGED);
            depth.push_back(i);
        }
        fclose(assoc);
    }
    FILE* assoc;
    std::vector<cv::Mat> img;
    std::vector<cv::Mat> depth;
};




int main(){
   auto *test = new testWithTUM("/mnt/e/dataset/rgbd_dataset_freiburg2_xyz/",
"/mnt/e/dataset/rgbd_dataset_freiburg2_xyz/assoc.txt");

}