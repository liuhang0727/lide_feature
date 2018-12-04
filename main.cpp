//包含头文件
#include "lide_feature.hpp"

int main()
{
    /*
    //读取文件列表
    vector<string> las_filenames;
    las_filenames = getFilename("/home/liuhang/Documents/Momenta/data/lide_2_las/", "las");
     */

    //读取文件列表
    vector<string> pcd_filenames;
    pcd_filenames = getFilename("/home/liuhang/Documents/data/FAK353_2018_10_19_22_27_44/merge_full_global/", "pcd");

    //多线程运行
    omp_set_num_threads(5);
    #pragma omp parallel for

    //逐个文件处理
    for(int i=1; i<2; i++)
    {
        /*
        //将las转为pcd格式
        CloudT::Ptr init_cloud(new CloudT());
        las2pcd(las_filenames[i], init_cloud);
        string init_cloud_filename = "/home/liuhang/Documents/Momenta/data/lide_2_pcd/init/init_cloud_" +
                to_string(i) +".pcd";
        pcl::io::savePCDFileBinary(init_cloud_filename, *init_cloud);
        */

        //加载点云
        CloudT::Ptr init_cloud(new CloudT());
        pcl::io::loadPCDFile(pcd_filenames[i], *init_cloud);

        //对点云进行滤波,去除孤立的噪声点
        CloudT::Ptr filter_cloud(new CloudT());
        filter(init_cloud, filter_cloud);
        string filter_cloud_filename = "/home/liuhang/Documents/data/feature_map/filt/filter_cloud_" + to_string(i) +".pcd";
        pcl::io::savePCDFileBinary(filter_cloud_filename, *filter_cloud);

        //使用体素格网的方法进行降采样,后面的处理都是在downsample_cloud的基础上
        CloudT::Ptr downsample_cloud(new CloudT());
        downsample(filter_cloud, downsample_cloud);
        string downsample_cloud_filename = "/home/liuhang/Documents/data/feature_map/downsample/downsample_cloud_" + to_string(i) +".pcd";
        pcl::io::savePCDFileBinary(downsample_cloud_filename, *downsample_cloud);

        //提取路面边界点云
        CloudT::Ptr road_boundary_cloud(new CloudT());
        if(road_boundary(downsample_cloud, road_boundary_cloud))
        {
            string road_boundary_cloud_filename =
                    "/home/liuhang/Documents/data/feature_map/road_boundary/road_boundary_" + to_string(i) + ".pcd";
            pcl::io::savePCDFileBinary(road_boundary_cloud_filename, *road_boundary_cloud);

            //提取路面点云
            CloudT::Ptr road_cloud(new CloudT());
            CloudT::Ptr without_road_cloud(new CloudT());
            CloudT::Ptr car_cloud(new CloudT());
            CloudT::Ptr lamp_tree_cloud(new CloudT());
            if(road_car(road_boundary_cloud, downsample_cloud, road_cloud, without_road_cloud, car_cloud, lamp_tree_cloud))
            {
                string road_cloud_filename =
                        "/home/liuhang/Documents/data/feature_map/road/road_" + to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(road_cloud_filename, *road_cloud);
                string without_road_cloud_filename =
                        "/home/liuhang/Documents/data/feature_map/without_road/without_road_" + to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(without_road_cloud_filename, *without_road_cloud);
                string car_cloud_filename =
                        "/home/liuhang/Documents/data/feature_map/car/car_" + to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(car_cloud_filename, *car_cloud);
                string lamp_tree_cloud_filename =
                        "/home/liuhang/Documents/data/feature_map/lamp_tree/lamp_tree_" + to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(lamp_tree_cloud_filename, *lamp_tree_cloud);

                //提取线特征点云
                CloudT::Ptr edge_lamp_tree_cloud(new CloudT());
                CloudT::Ptr edge_without_road_cloud(new CloudT());
                edge(lamp_tree_cloud, edge_lamp_tree_cloud);
                edge(without_road_cloud, edge_without_road_cloud);
                string edge_lamp_tree_cloud_filename =
                        "/home/liuhang/Documents/data/feature_map/edge_lamp_tree/edge_lamp_tree_" + to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(edge_lamp_tree_cloud_filename, *edge_lamp_tree_cloud);
                string edge_without_road_cloud_filename =
                        "/home/liuhang/Documents/data/feature_map/edge_without_road/edge_without_road_" + to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(edge_without_road_cloud_filename, *edge_without_road_cloud);

                //提取面特征点云
                CloudT::Ptr surf_road_boundary_cloud(new CloudT());
                CloudT::Ptr surf_lamp_tree_cloud(new CloudT());
                CloudT::Ptr surf_without_road_cloud(new CloudT());
                surf_1(road_boundary_cloud, surf_road_boundary_cloud);
                string surf_road_boundary_cloud_filename =
                        "/home/liuhang/Documents/data/feature_map/surf_road_boundary/surf_road_boundary_" + to_string(i) + ".pcd";
                pcl::io::savePCDFileBinary(surf_road_boundary_cloud_filename, *surf_road_boundary_cloud);
                if(surf_2(lamp_tree_cloud, surf_lamp_tree_cloud))
                {
                    if(surf_2(without_road_cloud, surf_without_road_cloud))
                    {
                        string surf_lamp_tree_cloud_filename =
                                "/home/liuhang/Documents/data/feature_map/surf_lamp_tree/surf_lamp_tree_" +to_string(i) + ".pcd";
                        pcl::io::savePCDFileBinary(surf_lamp_tree_cloud_filename, *surf_lamp_tree_cloud);
                        string surf_without_road_cloud_filename =
                                "/home/liuhang/Documents/data/feature_map/surf_without_road/surf_without_road_" + to_string(i) + ".pcd";
                        pcl::io::savePCDFileBinary(surf_without_road_cloud_filename, *surf_without_road_cloud);
                    }
                }
            }
        }

    cout<<"the "<<i<<"th file complated";
    }
}
