//
// Created by liuhang on 18-10-22.
//

#ifndef LIDE_FEATURE_H
#define LIDE_FEATURE_H

//c++
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <omp.h>

//liblas
#include <liblas/liblas.hpp>

//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

//添加命名空间
using namespace std;

//定义点和点云的类型
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

//读取文件中的文件列表
inline vector<string> getFilename(string fileDir, string format = "")
{
    fileDir += "/";

    DIR *dir;
    struct dirent *ptr;
    //char base[1000];

    vector<string> imgFiles;
    if ((dir=opendir(fileDir.c_str())) == NULL)
    {
        printf ("Open dir error...%s\n", fileDir.c_str ());
        return imgFiles;
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            //printf("d_name:%s\n",ptr->d_name);
            string imgName = ptr->d_name;
            if(format.empty())
                imgFiles.push_back(fileDir+ptr->d_name);
            else if(imgName.substr(imgName.find_first_of('.')+1) == format)
                imgFiles.push_back(fileDir+ptr->d_name);
        }
        else if(ptr->d_type == 10)    ///link file
            //printf("d_name:%s\n",ptr->d_name);
            continue;
        else if(ptr->d_type == 4)    ///dir
        {
            continue;
            imgFiles.push_back(ptr->d_name);
            /*
               memset(base,'\0',sizeof(base));
               strcpy(base,basePath);
               strcat(base,"/");
               strcat(base,ptr->d_nSame);
               readFileList(base);
           */
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    sort(imgFiles.begin(), imgFiles.end());
    return imgFiles;
}

//将las文件转为pcd文件
void const las2pcd(string _las_filename, CloudT::Ptr _init_cloud)
{
    std::ifstream ifs;
    ifs.open(_las_filename, std::ios::in|std::ios::binary);

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();

    int count = header.GetPointRecordsCount();
    _init_cloud->resize(count);

    for(int i=0; i<count; i++)
    {
        liblas::Point const& p = reader.GetPoint();
        //由于las中存储的数据超出了float
        //通过减去一个数使得数据不溢出
        _init_cloud->points[i].x = p.GetX() - 40561478.9285;
        _init_cloud->points[i].y = p.GetY() - 3478956.6627;
        _init_cloud->points[i].z = p.GetZ() - 8;
        _init_cloud->points[i].intensity = p.GetIntensity();

        reader.ReadNextPoint();
    }
    //必须要加上这两句，否则你会发现噪声点都过滤不掉
    _init_cloud->height = 1;
    _init_cloud->width = _init_cloud->points.size();

    cout<<"las2pcd completed."<<endl;
}

//对点云进行滤波,去除孤立的噪声点
void filter(CloudT::Ptr _init_cloud, CloudT::Ptr _filter_cloud)
{
    //创建设置StatisticaloutlierRemoval滤波器移除离群点
    pcl::StatisticalOutlierRemoval<PointT> statistical;
    statistical.setInputCloud(_init_cloud);
    //统计时考虑查询点临近点的个数
    statistical.setMeanK(20);
    //判断是否为离群点的阈值，１个标准差
    statistical.setStddevMulThresh(3);
    //false表示保留去除噪声点后剩下的点
    //true表示保留噪声点
    //默认是false
    statistical.setNegative(false);
    statistical.filter(*_filter_cloud);

    /*
    //创建设置RadiusOutlierRemoval滤波器
    pcl::RadiusOutlierRemoval<PointT> radius;
    radius.setInputCloud(cloud_in);
    //设置搜索半径
    radius.setRadiusSearch(0.2);
    //当半径内包含点数大于两个点时候认为是内点
    //半径内的点数不包含自己
    radius.setMinNeighborsInRadius(25);
    radius.setNegative(false);
    radius.filter(*cloud_out);
     */

    cout<<"filter completed."<<endl;
}

//使用体素格网的方法进行降采样
void downsample(CloudT::Ptr _filter_cloud, CloudT::Ptr _downsample_cloud)
{
    //创建设置体素化网格滤波器，对点云进行下采样
    //ApproximateVoxelGrid和VoxelGrid不同
    //前者使用格网中所有点的重心,后者使用格网的中心
    pcl::ApproximateVoxelGrid<PointT> voxel;
    voxel.setInputCloud(_filter_cloud);
    voxel.setLeafSize(0.05, 0.05, 0.05);
    voxel.filter(*_downsample_cloud);

    cout<<"downsample completed."<<endl;
}

//使用法向量与Z轴平行的思想获取路面边界
bool road_boundary(CloudT::Ptr _downsample_cloud, CloudT::Ptr _road_boundary_cloud)
{
//    //取出大致路面高程范围的数据
//    //普通的路面10-14肯定能够包括进去
//    //但是这对于高架就是一个大bug啊，还是不做这个处理了吧！
//    CloudT::Ptr z_filt_cloud(new CloudT());
//    for(int i=0; i<_downsample_cloud->points.size(); i++)
//    {
//        if(_downsample_cloud->points[i].z>10 && _downsample_cloud->points[i].z<50)
//        {
//            z_filt_cloud->push_back(_downsample_cloud->points[i]);
//        }
//    }
//    //保护程序不被中断
//    if(z_filt_cloud->points.size() == 0)
//    {
//        return false;
//    }

    CloudT::Ptr z_filt_cloud(new CloudT());
    *z_filt_cloud = *_downsample_cloud;

    //计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointT, pcl::Normal> n;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(z_filt_cloud);
    n.setInputCloud(z_filt_cloud);
    n.setSearchMethod(tree);
    n.setKSearch(100);
    n.compute(*normals);

    //将法向量与Z轴平行的点作为初始的地面点
    CloudT::Ptr road_init_cloud(new CloudT());
    double threshold = 0.01;
    for(int i=0; i<normals->points.size(); i++)
    {
        if(abs(normals->points[i].normal_x)<threshold && abs(normals->points[i].normal_y)<threshold)
        {
            road_init_cloud->points.push_back(z_filt_cloud->points[i]);
        }
    }

    //保护程序不被中断
    if(road_init_cloud->points.size() == 0)
    {
        return false;
    }

    //拟合道路平面方程
    pcl::PointIndices::Ptr plane_index(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);
    seg.setInputCloud(road_init_cloud);
    seg.segment(*plane_index, *coefficients);

    //提取拟合的道路平面点云
    CloudT::Ptr road_plane_cloud(new CloudT());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(road_init_cloud);
    extract.setIndices(plane_index);
    extract.setNegative(false);
    extract.filter(*road_plane_cloud);

    //保护程序不被中断
    if(road_plane_cloud->points.size() == 0)
    {
        return false;
    }

    //再对道路平面点云做一个滤波处理，去除远处草地中产生的噪点
    pcl::StatisticalOutlierRemoval<PointT> statistical;
    statistical.setInputCloud(road_plane_cloud);
    //统计时考虑查询点临近点的个数
    statistical.setMeanK(30);
    //判断是否为离群点的阈值，１个标准差
    statistical.setStddevMulThresh(1.5);
    //false表示保留去除噪声点后剩下的点
    //true表示保留噪声点,默认是false
    statistical.setNegative(false);
    statistical.filter(*_road_boundary_cloud);

    //保护程序不被中断
    if(_road_boundary_cloud->points.size() == 0)
    {
        return false;
    }

    cout<<"road boundary completed."<<endl;
    return true;
}

//提取路面点云、非路面范围点云、车辆点云和灯树点云
bool road_car(CloudT::Ptr _road_boundary_cloud, CloudT::Ptr _downsample_cloud, CloudT::Ptr _road_cloud,
              CloudT::Ptr _without_road_cloud, CloudT::Ptr _car_cloud, CloudT::Ptr _lamp_tree_cloud)
{
    //获取路面的OBB边界
    PointT min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotional_matrix_OBB;
    pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
    feature_extractor.setInputCloud(_road_boundary_cloud);
    feature_extractor.compute();
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotional_matrix_OBB);

    //将点云转到OBB的坐标系下
    CloudT::Ptr transform_downsample_cloud(new CloudT());
    CloudT::Ptr transform_road_boundary_cloud(new CloudT());
    Eigen::Matrix4f t_obb = Eigen::Matrix4f::Identity();
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            t_obb(i,j) = rotional_matrix_OBB(i,j);
        }
    }
    t_obb(0,3) = position_OBB.x;
    t_obb(1,3) = position_OBB.y;
    t_obb(2,3) = position_OBB.z;
    t_obb(3,0) = 0;
    t_obb(3,1) = 0;
    t_obb(3,2) = 0;
    t_obb(3,3) = 1;
    Eigen::Matrix4f t_obb_inv = t_obb.inverse();
    pcl::transformPointCloud(*_downsample_cloud, *transform_downsample_cloud, t_obb_inv);
    pcl::transformPointCloud(*_road_boundary_cloud, *transform_road_boundary_cloud, t_obb_inv);

    //计算地面的平均高度
    double z_ave_transform = 0;
    for(int i=0; i<transform_road_boundary_cloud->points.size(); i++)
    {
        z_ave_transform += transform_road_boundary_cloud->points[i].z / transform_road_boundary_cloud->points.size();
    }

    //区分路面点、非路面范围点、路灯树车辆
    pcl::IndicesPtr road_index(new std::vector<int>);
    pcl::IndicesPtr without_road_index(new std::vector<int>);
    pcl::IndicesPtr lamp_tree_car_index(new std::vector<int>);
    double xy_threshold = 0;
    double z_threshold = 0.25;
    for(int i=0; i<transform_downsample_cloud->points.size(); i++)
    {
        double x = transform_downsample_cloud->points[i].x;
        double y = transform_downsample_cloud->points[i].y;
        double z = transform_downsample_cloud->points[i].z;
        if(x>min_point_OBB.x+xy_threshold && x<max_point_OBB.x-xy_threshold && y>min_point_OBB.y+xy_threshold && y<max_point_OBB.y-xy_threshold)
        {
            if(abs(z-z_ave_transform)<z_threshold)
            {
                road_index->push_back(i);
            }
            else
            {
                lamp_tree_car_index->push_back(i);
            }
        }
        else
        {
            without_road_index->push_back(i);
        }
    }

    //提取路面点
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(_downsample_cloud);
    extract.setIndices(road_index);
    extract.setNegative(false);
    extract.filter(*_road_cloud);

    //提取非路面范围点
    extract.setInputCloud(_downsample_cloud);
    extract.setIndices(without_road_index);
    extract.setNegative(false);
    extract.filter(*_without_road_cloud);

    //提取路面范围内包含路灯树和车辆的点云
    CloudT::Ptr lamp_tree_car_cloud(new CloudT());
    extract.setInputCloud(_downsample_cloud);
    extract.setIndices(lamp_tree_car_index);
    extract.setNegative(false);
    extract.filter(*lamp_tree_car_cloud);

    //对包含路灯树和车辆的点云进行欧式聚类分割
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    if(lamp_tree_car_cloud->points.size() == 0)
    {
        return false;
    }
    tree->setInputCloud(lamp_tree_car_cloud);
    pcl::EuclideanClusterExtraction<PointT> ec;
    // 设置近邻搜索的搜索半径为2cm
    ec.setClusterTolerance(0.3);
    //设置一个聚类需要的最小和最大的点数目
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(lamp_tree_car_cloud);
    ec.extract(cluster_indices);

    //迭代访问点云索引cluster_indices,直到分割处所有聚类
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        //迭代容器中的点云的索引，并且分开保存索引的点云
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr car_cluster(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr lamp_tree_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cluster->points.push_back(lamp_tree_car_cloud->points[*pit]);
        }

        //根据聚类中最高点的高度区分车辆与树 路灯
        double z_ave = 0;
        for(int i=0; i<_road_boundary_cloud->points.size(); i++)
        {
            z_ave += _road_boundary_cloud->points[i].z / _road_boundary_cloud->points.size();
        }
        PointT minpt, maxpt;
        pcl::getMinMax3D(*cluster, minpt, maxpt);

        //计算聚类的长宽比,区分路中央的栅栏
        PointT min_cluster_OBB, max_cluster_OBB, position_cluster_OBB;
        Eigen::Matrix3f rotional_cluster_OBB;
        feature_extractor.setInputCloud(cluster);
        feature_extractor.compute();
        feature_extractor.getOBB(min_cluster_OBB, max_cluster_OBB, position_cluster_OBB, rotional_cluster_OBB);
        double xy_ratio = (max_cluster_OBB.x-min_cluster_OBB.x)/(max_cluster_OBB.y-min_cluster_OBB.y);

        //将聚类按照限制条件保存为车辆或树 路灯,设置车辆的最高高度
        if(maxpt.z>(z_ave+2.8) || xy_ratio<0.1 || xy_ratio>10)
        {
            lamp_tree_cluster = cluster;
        }
        else
        {
            car_cluster = cluster;
        }

        //把每个聚类累加到对应的待保存的点云中
        *_lamp_tree_cloud += *lamp_tree_cluster;
        *_car_cloud += *car_cluster;
    }

    //保护程序不被中断
    if(_road_cloud->points.size() == 0)
    { return false; }
    if(_without_road_cloud->points.size() == 0)
    { return false; }
    if(_car_cloud->points.size() == 0)
    { return false; }
    if(_lamp_tree_cloud->points.size() == 0)
    { return false; }

    cout<<"road_car complated."<<endl;
    return true;
}

//提取edge点云
bool edge(CloudT::Ptr _lamp_tree_cloud, CloudT::Ptr _edge_cloud)
{
    //防止程序崩溃
    if(_lamp_tree_cloud->points.size() == 0)
    { return false; }

    //边缘检测
    pcl::PointCloud<pcl::Boundary>::Ptr boundary(new pcl::PointCloud<pcl::Boundary>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> boundEst;
    pcl::NormalEstimationOMP<PointT, pcl::Normal> normEst;
    pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
    kdTree->setInputCloud(_lamp_tree_cloud);
    normEst.setInputCloud(_lamp_tree_cloud);
    normEst.setSearchMethod(kdTree);
    normEst.setKSearch(100);
    normEst.compute(*normals);
    boundEst.setInputCloud(_lamp_tree_cloud);
    boundEst.setInputNormals(normals);
    //一般这里的数值越高，最终边界识别的精度越好
    //boundEst.setKSearch(1000);
    boundEst.setRadiusSearch(1);
    boundEst.setAngleThreshold(M_PI);
    boundEst.setSearchMethod(kdTree);
    boundEst.compute(*boundary);

    //提取pcl::Boundary对象中的信息
    for(int i=0; i<_lamp_tree_cloud->points.size(); i++)
    {
        uint8_t x = (boundary->points[i].boundary_point);
        //强制类型转换
        int a = static_cast<int>(x);
        if(a == 1)
        {
            _edge_cloud->points.push_back(_lamp_tree_cloud->points[i]);
        }
    }

    //保护程序不被中断
    if(_edge_cloud->points.size() == 0)
    { return false; }

    cout<<"edge complated."<<endl;
    return true;
}

//使用体素格网降采样的方法从road_boundary_cloud点云中提取surf点云
bool surf_1(CloudT::Ptr _road_boundary_cloud, CloudT::Ptr _surf_road_cloud)
{
    pcl::ApproximateVoxelGrid<PointT> voxel;
    voxel.setInputCloud(_road_boundary_cloud);
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.filter(*_surf_road_cloud);

    cout<<"surf_1 complated."<<endl;
}

//通过平面拟合的方法获取lamp_tree_cloud和without_road_cloud中的平面点
bool surf_2(CloudT::Ptr _lamp_tree_cloud, CloudT::Ptr _surf_lamp_tree_cloud)
{
    /*
    //使用平面拟合的方法提取平面，完全不work！
    pcl::PointIndices::Ptr plane_index(new pcl::PointIndices);
    CloudT::Ptr remain_cloud(new CloudT());
    *remain_cloud = *_lamp_tree_cloud;
    do {
        //拟合平面
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setInputCloud(remain_cloud);
        seg.segment(*plane_index, *coefficients);

        //提取拟合平面中的点云
        CloudT::Ptr plane_cloud(new CloudT());
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(remain_cloud);
        extract.setIndices(plane_index);
        extract.setNegative(false);
        extract.filter(*plane_cloud);
        extract.setNegative(true);
        extract.filter(*remain_cloud);

        //维护所有的平面点云
        *_surf_lamp_tree_cloud += *plane_cloud;

        cout<<"plane_index->indices.size()   "<<plane_index->indices.size()<<endl;
        }while(plane_index->indices.size()>1000);
        */

    //使用区域生长的方法 region growing segmentation，进行平面提取
    //计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointT, pcl::Normal> n;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(_lamp_tree_cloud);
    n.setInputCloud(_lamp_tree_cloud);
    n.setSearchMethod(tree);
    n.setKSearch(70);
    n.compute(*normals);

    //区域生长
    std::vector<pcl::PointIndices> clusters;
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize(400);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(6);
    reg.setInputCloud(_lamp_tree_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(10/180.0*M_PI);
    reg.setCurvatureThreshold(0.15);
    reg.extract(clusters);

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            _surf_lamp_tree_cloud->points.push_back(_lamp_tree_cloud->points[*pit]);
        }
    }

    //对提取的面点进行降采样
    pcl::ApproximateVoxelGrid<PointT> voxel;
    voxel.setInputCloud(_surf_lamp_tree_cloud);
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.filter(*_surf_lamp_tree_cloud);

    if(_surf_lamp_tree_cloud->points.size() == 0)
    {
        cout<<"没有找到平面点！"<<endl;
        return false;
    }

    cout<<"surf_2 complated."<<endl;
    return true;
}

#endif //LIDE_FEATURE_H


