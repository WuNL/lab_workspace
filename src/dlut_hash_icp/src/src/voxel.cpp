/*************************************************************************
  > File Name: voxel.cpp
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Wed 21 May 2014 09:35:49 AM CST
 ************************************************************************/

#include "voxel.h"
using namespace std;




void Voxelize::generateUmap(pcl::PointCloud<pcl::PointXYZ> &cloud,double resolution,boost::unordered_map<unordered_map_voxel,un_key> &m)//遍历每个点，生成栅格。然后计算每个栅格的参数
{
    double t0 = ros::Time::now().toSec();

    un_key tempp;  //key键不允许修改，全存在value中

    for(int i=0;i<cloud.size();i++)
    {
        unordered_map_voxel tempv(cloud[i].x,cloud[i].y,cloud[i].z,resolution);

        pair<umap::iterator,bool> pair_insert = m.insert(umap::value_type(tempv,tempp));//这个pair就是用来存储插入返回值的，看是否这个栅格已经存在

        pair_insert.first->second.vec_index_point.push_back(i);
        /*
           if(!pair_insert.second)//如果栅格已经存在，那么把value++
           {
           (pair_insert.first)->second.cout++;
           pair_insert.first->second.vec_index_point.push_back(i);
           }
           else{
           pair_insert.first->second.vec_index_point.push_back(i);
           }
           */
    }

    double t2 = ros::Time::now().toSec();
    //cout <<"划栅格的时间为"<<(t2-t0)<<endl;
    Matrix3d tempmat;
    tempmat.setZero();

    pcl::PointXYZ temppoint(0,0,0);
    for(umap::iterator iter=m.begin();iter!=m.end();)
    {
        temppoint.x=temppoint.y=temppoint.z=0;
        if(iter->second.vec_index_point.size()<10)
        {
            m.erase(iter++);
            continue;
        }

        for(int j = 0;j<iter->second.vec_index_point.size();j++)
        {
            double x_ = cloud[iter->second.vec_index_point[j]].x;
            double y_ = cloud[iter->second.vec_index_point[j]].y;
            double z_ = cloud[iter->second.vec_index_point[j]].z;

            tempmat += (Vector3d(x_,y_,z_) * RowVector3d(x_,y_,z_));//存储pipiT的和
            temppoint.x += x_;
            temppoint.y += y_;
            temppoint.z += z_;
        }

        int n = iter->second.vec_index_point.size();
        //cout <<"栅格大小为"<<n<<endl;
        Vector3d v(temppoint.x/n,temppoint.y/n,temppoint.z/n);//存储栅格重心
        RowVector3d vT(temppoint.x/n,temppoint.y/n,temppoint.z/n);

        tempmat /= n;
        tempmat -= v*vT;//S
        iter->second.matS = tempmat;
        //cout <<"S 矩阵= "<< tempmat <<endl;

        vector<eigen_sort> vector1(3);//用于排序

        //cout <<"tempmat="<<endl<<tempmat<<endl;
        EigenSolver<MatrixXd> es(tempmat);
        VectorXcd eivals = es.eigenvalues();
        MatrixXcd eigenvectors = es.eigenvectors();
        //cout << "特征值="<<eivals<<endl;
        vector1[0]=eigen_sort(eivals(0).real(),es.eigenvectors().col(0));
        vector1[1]=eigen_sort(eivals(1).real(),es.eigenvectors().col(1));
        vector1[2]=eigen_sort(eivals(2).real(),es.eigenvectors().col(2));
        sort(vector1.begin(),vector1.end());
        double lamada1 = vector1[0].eigen_value;
        double lamada2 = vector1[1].eigen_value;
        double lamada3 = vector1[2].eigen_value;
        //cout <<"lamada="<<lamada1<<" "<<lamada2<<" "<<lamada3<<endl;
        //
        if(vector1[0].eigen_vector(2).real()<0)
        {
            vector1[0].eigen_vector(2).real() = -vector1[0].eigen_vector(2).real();
        }
        if(vector1[2].eigen_vector(2).real()<0)
        {
            vector1[2].eigen_vector(2).real() = -vector1[2].eigen_vector(2).real();
        }


        iter->second.c = (lamada3-lamada2)/(lamada1+lamada2+lamada3);
        iter->second.p = 2*(lamada2-lamada1)/(lamada1+lamada2+lamada3);
        iter->second.u = v;
        iter->second.v1 = vector1[0].eigen_vector;
        iter->second.v3 = vector1[2].eigen_vector;

        double c = iter->second.c;
        double p = iter->second.p;


        iter->second.vector_9D << v(0),v(1),v(2),p*vector1[0].eigen_vector(0).real(),p*vector1[0].eigen_vector(1).real(),
            p*vector1[0].eigen_vector(2).real(),c*vector1[2].eigen_vector(0).real(),c*vector1[2].eigen_vector(1).real(),
            p*vector1[2].eigen_vector(2).real();

        //cout << "9D向量="<<iter->second.vector_9D<<endl;
        //ArrayXd ad = iter->second.vector_9D;
        //cout <<"ad = "<<ad<<endl;
        //cout <<"ad square="<<ad.square()<<endl;
        tempmat.setZero();

        iter++;

    }

    double tn = ros::Time::now().toSec();
    //cout << "对栅格计算处理的时间"<<(tn - t2)<<endl;
    //cout <<"栅格数量为" <<m.size()<<endl;

}


pair<umap::iterator,bool> Voxelize::neighbor_search( umap &m, umap &m1,const unordered_map_voxel &tempp )//搜索最近邻
{
    int x_b,x_t,y_b,y_t,z_b,z_t;
    double o_dis = 10000.0; //存储临近栅格之间，9D向量的欧式距离

    umap::iterator vox_iter = m1.find(tempp);
    if(vox_iter == m.end())
    {
        pair<umap::iterator,bool> return_value(m.end(),false);
        cout << "出错"<<endl;
        //return return_value;
    }

    if(tempp.x() == 1 || tempp.x()==2)
    {
        x_b = tempp.x() -3;
        x_t = tempp.x() +2;
    }
    else if(tempp.x()== -1 || tempp.x()== -2)
    {
        x_b = tempp.x() -2;
        x_t = tempp.x() +3;
    }
    else
    {
        x_b = tempp.x() -2;
        x_t = tempp.x() +2;
    }


    //对y处理
    if(tempp.y() == 1 || tempp.y()==2)
    {
        y_b = tempp.y() -3;
        y_t = tempp.y() +2;
    }
    else if(tempp.y()== -1 || tempp.y()== -2)
    {
        y_b = tempp.y() -2;
        y_t = tempp.y() +3;
    }
    else
    {
        y_b = tempp.y() -2;
        y_t = tempp.y() +2;
    }

    //对Z处理
    if(tempp.z() == 1 || tempp.z()==2)
    {
        z_b = tempp.z() -3;
        z_t = tempp.z() +2;
    }
    else if(tempp.z()== -1 || tempp.z()== -2)
    {
        z_b = tempp.z() -2;
        z_t = tempp.z() +3;
    }
    else
    {
        z_b = tempp.z() -2;
        z_t = tempp.z() +2;
    }
    //cout <<"坐标处理完毕,上下限为："<<endl;
    //cout <<x_b <<" " <<x_t <<" "<<y_b<<" "<<y_t <<" "<<z_b<<" "<<z_t <<endl;

    //后加的，请注意
    /*
       x_b = x_t =tempp.x();
       y_b = y_t =tempp.y();
       z_b = z_t =tempp.z();
       */


    int x_min,y_min,z_min; //距离最近栅格的坐标

    for(int i = x_b; i<= x_t ; i++)
    {
        for(int j = y_b; j <= y_t ; j++)
        {
            for(int k = z_b ; k <= z_t ; k++)
            {
                unordered_map_voxel temp_vox(i,j,k);
                if(m.count(temp_vox)>0 )
                {
                    if(m.find(temp_vox)->second.cout ==0)
                        continue;

                    //cout <<"ijk="<<i<<" "<<j <<" "<<k<<endl;
                    ArrayXd o_distance = m.find(temp_vox)->second.vector_9D- vox_iter->second.vector_9D;

                    //cout << "o_distance= "<<o_distance <<endl;
                    double o_dis_temp = o_distance.square().sum();
                    //cout <<"o_distance=="<< m.find(temp_vox)->second.vector_9D<<endl;
                    //cout << "平方和="<<o_dis_temp<<endl;

                    if(o_dis_temp < o_dis)
                    {
                        o_dis = o_dis_temp;
                        x_min = i;
                        y_min = j;
                        z_min = k;
                    }
                }
            }
        }
    }

    if(o_dis>= 0.5 )
    {
        //cout << "原始栅格坐标："<<tempp.x()<<" "<<tempp.y()<<" "<<tempp.z()<<"没有最近的类似栅格"<<endl;
        pair<umap::iterator,bool> return_value(m.find(unordered_map_voxel(x_min,y_min,z_min)),false);
        return return_value;
    }
    //cout <<"原始栅格坐标为："<<tempp.x()<<" "<<tempp.y()<<" "<<tempp.z()<<endl;
    //cout <<"最近的欧式距离为："<<o_dis<<"属于："<<x_min<<" "<<y_min <<" "<< z_min<<endl;
    pair<umap::iterator,bool> return_value(m.find(unordered_map_voxel(x_min,y_min,z_min)),true);
    return return_value;

}


