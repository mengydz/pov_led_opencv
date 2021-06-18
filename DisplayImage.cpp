#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
#define PI 3.1415926

    uchar interpolate_bilinear(cv::Mat& mat_src, double ri, int rf, int rc, double ti, int tf, int tc)  
    {  
        double inter_value = 0.0;  
      
        if (rf == rc && tc == tf)  
        {  
            inter_value = mat_src.ptr<uchar>(rc)[tc];  
        }  
        else if (rf == rc)  
        {  
            inter_value = (ti - tf) * mat_src.ptr<uchar>(rf)[tc] + (tc - ti) * mat_src.ptr<uchar>(rf)[tf];  
        }  
        else if (tf == tc)  
        {  
            inter_value = (ri - rf) * mat_src.ptr<uchar>(rc)[tf] + (rc - ri) * mat_src.ptr<uchar>(rf)[tf];  
        }  
        else  
        {  
            double inter_r1 = (ti - tf) * mat_src.ptr<uchar>(rf)[tc] + (tc - ti) * mat_src.ptr<uchar>(rf)[tf];  
            double inter_r2 = (ti - tf) * mat_src.ptr<uchar>(rc)[tc] + (tc - ti) * mat_src.ptr<uchar>(rc)[tf];  
      
            inter_value = (ri - rf) * inter_r2 + (rc - ri) * inter_r1;  
        }  
      
        return (uchar)inter_value;  
    }  

bool cartesian_to_polar(cv::Mat& mat_c, cv::Mat& mat_p, int img_d)  
{  
	mat_p = cv::Mat::zeros(img_d, img_d, CV_8UC1);  

	int line_len = mat_c.rows;  
	int line_num = mat_c.cols;  

	double delta_r = (2.0*line_len) / (img_d - 1); //半径因子  
	double delta_t = 2.0 * PI / line_num; //角度因子  

	double center_x = (img_d - 1) / 2.0;  
	double center_y = (img_d - 1) / 2.0;  

	for (int i = 0; i < img_d; i++)  
	{  
	    for (int j = 0; j < img_d; j++)  
	    {  
		double rx = j - center_x; //图像坐标转世界坐标  
		double ry = center_y - i; //图像坐标转世界坐标  

		double r = std::sqrt(rx*rx + ry*ry);  

		if (r <= (img_d - 1) / 2.0)  
		{  
		    double ri = r * delta_r;  
		    int rf = (int)std::floor(ri);  
		    int rc = (int)std::ceil(ri);  

		    if (rf < 0)  
		    {  
		        rf = 0;  
		    }  
		    if (rc > (line_len - 1))  
		    {  
		        rc = line_len - 1;  
		    }  

		    double t = std::atan2(ry, rx);  

		    if (t < 0)  
		    {  
		        t = t + 2.0 * PI;  
		    }  

		    double ti = t / delta_t;  
		    int tf = (int)std::floor(ti);  
		    int tc = (int)std::ceil(ti);  

		    if (tf < 0)  
		    {  
		        tf = 0;  
		    }  
		    if (tc > (line_num - 1))  
		    {  
		        tc = line_num - 1;  
		    }  

		    mat_p.ptr<uchar>(i)[j] = interpolate_bilinear(mat_c, ri, rf, rc, ti, tf, tc);  
		}  
	    }  
	}  

	return true;  
}  

    bool polar_to_cartesian(cv::Mat& mat_p, cv::Mat& mat_c, int rows_c, int cols_c)  
    {  
        mat_c = cv::Mat::zeros(rows_c, cols_c, CV_8UC1);  
      
        int polar_d = mat_p.cols;  
        double polar_r = polar_d / 2.0; // 圆图半径  
      
        double delta_r = polar_r / rows_c; //半径因子  
        double delta_t = 2.0*PI / cols_c;  //角度因子  
      
        double center_polar_x = (polar_d - 1) / 2.0;  
        double center_polar_y = (polar_d - 1) / 2.0;  
      
        for (int i = 0; i < cols_c; i++)  
        {  
            double theta_p = i * delta_t; //方图第i列在圆图对应线的角度  
            double sin_theta = std::sin(theta_p);  
            double cos_theta = std::cos(theta_p);  
      
            for (int j = 0; j < rows_c; j++)  
            {  
                double temp_r = j * delta_r; //方图第j行在圆图上对应的半径长度  
      
                int polar_x = (int)(center_polar_x + temp_r * cos_theta);  
                int polar_y = (int)(center_polar_y - temp_r * sin_theta);  
      
                mat_c.ptr<uchar>(j)[i] = mat_p.ptr<uchar>(polar_y)[polar_x];  
            }  
        }  
        return true;  
    }  

int main()
{
    cv::Mat image,image1;
    image = cv::imread("image.bmp",cv::IMREAD_GRAYSCALE);
    polar_to_cartesian(image,image1,image.cols/2.0f,image.cols*PI);
    cv::imwrite("image1.bmp",image1);
    cv::imshow("image",image);
    cv::imshow("image1",image1);

    cv::waitKey(0);
    return 0;
}
