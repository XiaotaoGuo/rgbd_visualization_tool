#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6,1> Vector6d;

//图像格式转换，高3位为置信度量，低13位为深度
void cvtDepth2Color(const cv::Mat src,cv::Mat& dst){
    if(true){
        int nr=src.rows;
        int nc=src.cols;
        int min=10000,max=0;
        for(int k=0;k<nr;k++)
        {
            const ushort* inData=src.ptr<ushort>(k);
            ushort* outData=dst.ptr<ushort>(k);
            for(int i=0;i<nc;i++)
            {

                //int deepnum=(inData[i*2+1]&0x1f)*256+inData[i*2];
                outData[i]=inData[i]&0x1fff;
                //std::cout << outData[i] << std::endl;

            }
        }
        // cv::applyColorMap(gray,dst,cv::COLORMAP_RAINBOW);
        return;
    }
}

//绘图函数，不用改
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud, cv::Mat& rgb_img);

int main(int argc, char **argv) {

    string rgb_path, dep_path;

    cv::Mat rgb_img, dep_img;
    if(argc == 3) {  //rgb, depth
        rgb_path = argv[1];
        dep_path = argv[2];
    }
    else if(argc == 2){ //only depth provided
        dep_path = argv[1];
    }
    else{
        cout << "depth image needed!" << endl;
        return 0;
    }

    // 内参和图像
    const double fx = 358.20;
    const double fy = 358.20;
    const double cx = 243.90;
    const double cy = 137.00;

    dep_img = cv::imread(dep_path, cv::IMREAD_ANYDEPTH); //RGB图和深度图读取方式根据需要更改
    //cvtDepth2Color(dep_img, dep_img);

    if(argc == 3){
        rgb_img = cv::imread(rgb_path);
        cv::Rect crop_area(0, 200,1280, 320); //rgb图大小位置和深度图对准
        rgb_img = rgb_img(crop_area);
    }


    // 生成点云
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    float factor = 256;
    for (int v = 0; v < dep_img.rows; v++)
        for (int u = 0; u < dep_img.cols; u++) {

            Vector6d point;
            if(argc == 3){
                point[3] = float(rgb_img.at<cv::Vec3b>(v, u)[2])/255;
                point[4] = float(rgb_img.at<cv::Vec3b>(v, u)[1])/255;
                point[5] = float(rgb_img.at<cv::Vec3b>(v, u)[0])/255;
            }
            else{//没有rgb图，默认红色
                point[3] = 1.0;
                point[4] = 0.0;
                point[5] = 0.0;
            }

            unsigned short val = dep_img.at<unsigned short>(v, u);
            float z = static_cast<float>(val);

            point[2] = z / factor;
            point[0] = point[2]*(u - cx)/fx;
            point[1] = point[2]*(v - cy)/fy;
            if(point[2] > 0) pointcloud.push_back(point);
        }

    // 画出点云
    showPointCloud(pointcloud, rgb_img);
    return 0;
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud, cv::Mat& rgb_img) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[4], p[5]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
