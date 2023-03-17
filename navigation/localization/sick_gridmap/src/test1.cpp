#include <opencv2/opencv.hpp>
//#include <ros.h>
#include <time.h>

using namespace cv;
using namespace std;

void Dilate_Img(cv::Mat src, cv::Mat &dst,int hight, int width);

int main()
{
  clock_t t1, t2;

  Mat mat_hight = cv::Mat(1000, 1000, CV_8UC1, cv::Scalar(0)), dst4 = cv::Mat(1000, 1000, CV_8UC1, cv::Scalar(0));
  for (size_t i = 0; i < 120; i++)
    for (size_t j = 0; j < 120; j++)
    {
      mat_hight.at<uchar>(i, j) = 255;
      dst4.at<uchar>(i, j) = 255;
    }
  t1 = clock();
  cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
  cv::Mat dst;
  // mat_hight.clone(dst4);
  cv::dilate(mat_hight, dst, element1);
  Dilate_Img(dst4, dst4,1, 1);
  //cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(600, 1));
  // cv::dilate(dst, dst, element2);

  // for(size_t i=0;i<1000;i++)
  // {
  //   if(dst.at<uchar>(50,i)==255) std::cout<<"  "<<i<<std::endl;
  //   // if(dst4.at<uchar>(50,i)==255) std::cout<<"  "<<i<<std::endl;
  // }

  // for(size_t i=0;i<1000;i++)
  // {
  //   // if(dst.at<uchar>(50,i)==255) std::cout<<"  "<<i<<std::endl;
  //   if(dst4.at<uchar>(50,i)==255) std::cout<<"  "<<i<<std::endl;
  // }
  t2 = clock();
  cout << "\ndilate time  (ms) = " << (double)(clock() - t1) * 1000 / CLOCKS_PER_SEC << endl;
  cv::imshow("origin", mat_hight);
  cv::imshow("mat_hight", dst4);
  cv::imshow("dst", dst);
  cv::waitKey(0);
}

void Dilate_Img(cv::Mat src, cv::Mat &dst, int width, int hight)
{
  int width_img = src.cols;
  int hight_img = src.rows;
  int img[1000][1000];
  int img_point = 0;
  for (size_t i = 0; i < hight_img; i++)
  {
    for (size_t j = 0; j < width_img; j++)
    {
      if (src.at<uchar>(i, j) == 255)
        img_point = 1;
      else
        img_point = 0;
      img[i][j] = 0;
      if (i == 0 || j == 0)
        continue;
      img[i][j] = img_point + img[i - 1][j] + img[i][j - 1] - img[i - 1][j - 1];
    }
  }
  int i_hight = 0;
  int j_width = 0;
  int i__hight = 0;
  int j__width = 0;

  for (size_t i = 1; i < hight_img; i++)
    for (size_t j = 1; j < width_img; j++)
    {
      if (src.at<uchar>(i, j) == 255)
      { 
        dst.at<uchar>(i, j) = 255;
        continue;
      }
      i_hight = i + hight / 2;
      j_width = j + width / 2;
      i__hight = i - hight / 2;
      j__width = j - width / 2;

      if (i_hight > hight_img-1)
        i_hight = hight_img-1;
      if (j_width > width_img-1)
        j_width = width_img-1;
      if (i__hight < 0)
        i__hight = 0;
      if (j__width < 0)
        j__width = 0;
      if (img[i_hight][j_width] - img[i__hight][j_width] - img[i_hight][j__width] + img[i__hight][j__width] > 1)
        dst.at<uchar>(i, j) = 255;
    }
}
