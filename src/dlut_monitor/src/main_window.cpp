/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <GL/glut.h>
#include <algorithm>
#include <QMessageBox>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include "../include/dlut_monitor/main_window.hpp"
#include "../include/dlut_monitor/glviewer.h"
#include "../include/dlut_monitor/motor.h"
#include "../include/dlut_monitor/getdata.h"
#include <QDebug>
#include "Grid3D.h"
#include "Point3d.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dlut_monitor {

using namespace Qt;
using namespace std;
/*
void abcconnect (MyGetDataThread* mt,t1* moth)
{
  connect(mt,SIGNAL(showdata()),moth,SLOT(showdata()));
}
*/
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	ui.tabWidget->setCurrentIndex(0);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    glviewer1 = new GLViewer (MyGetDataThread::point_cloud, 1, this);
    ui.gridLayout_showdata->addWidget(glviewer1);
    glviewer2 = new GLViewer (MyGetDataThread::point_cloud, 1, this);
    ui.gridLayout_show_file->addWidget(glviewer2);
    glviewer3 = new GLViewer (MyGetDataThread::point_cloud, 1, this);
    ui.gridLayout_show_people->addWidget(glviewer3);
   	setWindowIcon(QIcon(":/images/icon.png"));   
   	ui.DA_end->setEnabled(false);
   	mygetdata = new MyGetDataThread;
   	t2 = new t1;
   	t2->start();
   	connect(mygetdata,SIGNAL(showdata()),t2,SLOT(showdata()));
   	connect(mygetdata,SIGNAL(showpeople()),t2,SLOT(showpeople()));
   	connect(this,SIGNAL(showdata()),t2,SLOT(showdata()));
   	connect(t2,SIGNAL(update()),this,SLOT(update()));
   	connect(t2,SIGNAL(update_people()),this,SLOT(update_people()));
   	
   	
}

MainWindow::~MainWindow() {
	delete glviewer1;
	delete glviewer2;
	delete glviewer3;
	mygetdata->quit();
	delete mygetdata;
	t2->quit();
	delete t2;
	
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::update()
{	
	qDebug("into slot update");
  	glviewer1->point_cloud = MyGetDataThread::point_cloud;
  	glviewer1->reset();
}

void MainWindow::update_people()
{	
	qDebug("into slot update_people");
  	glviewer3->point_cloud = MyGetDataThread::point_cloud;
  	glviewer3->reset();
}
/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */






/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/


void MainWindow::closeEvent(QCloseEvent *event)
{
	//WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::on_DA_start_clicked(bool check)
{
	if(ui.lineEdit_runtime->text().toInt()==0 && ui.lineEdit_freqence->text().toInt()==0 )
	{
		QMessageBox::information(NULL, tr("Waring"), tr("param must be int")); 
		return;
	}
	if(ui.lineEdit_runtime->text().toInt()>=60 || ui.lineEdit_freqence->text().toInt()>=3000)
	{
		QMessageBox::information(NULL, tr("Waring"), tr("Please input a lower param"));
		return; 
	}	
	ROS_INFO("start");
	mt = new Motor(ui.lineEdit_runtime->text().toInt(),ui.lineEdit_freqence->text().toInt());
	mt->motorMoveWithStart();
	ui.DA_end->setEnabled(true);
	if(ui.checkBox_savefile->isChecked())
	{
		t2->is_save(true);
		qDebug("is checked");
	}
	else 
	{
		t2->is_save(false);
	}
	t2->messageemit();
	mygetdata->start();
	mygetdata->data_switch_fun(true);
	delete mt;
}
void MainWindow::on_DA_end_clicked(bool check)
{
	ROS_INFO("end");
	mt = new Motor();
	mt->motorMoveWithStop();
	ui.DA_end->setEnabled(false);
	mygetdata->data_switch_fun(false);
	mygetdata->terminate();
	delete mt;
}

void MainWindow::on_pushButton_openfile_clicked()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Open PCD File"), ".", tr("PCD Files(*.PCD)")); 
	if(path.length() == 0)
	{ 
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files.")); 
    } 
    else
    { 
        //QMessageBox::information(NULL, tr("Path"), tr("You selected ") + path); 
        std::string file = path.toLatin1().data();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> indices;
        pcl::PCDReader reader;

        reader.read<pcl::PointXYZ> (file, *cloud);
        cout << cloud->width <<" "<<cloud->height <<endl;
        pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

    	  //pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud);

	      pcl::PointXYZ temp_point;
	      std::vector < pcl::PointXYZ > temp_vector;
        vector<CPoint3d> laserdata;
        CPoint3d temp_CPoint;
        //temp_vector = (*cloud).points;
        cout << cloud->width <<" "<<cloud->height <<endl;


        for (size_t i = 0; i < cloud->points.size (); ++i)
    	{
	      temp_point.x = cloud->points[i].x;
	      temp_point.y = cloud->points[i].y;
	      temp_point.z = cloud->points[i].z;
	      temp_vector.push_back (temp_point);

	      temp_CPoint.x = cloud->points[i].x;
	      temp_CPoint.y = cloud->points[i].y;
	      temp_CPoint.z = cloud->points[i].z;
        laserdata.push_back(temp_CPoint);
	  	}
        
        gd3d.Voxel2ize(laserdata,0.2);
        /*
        //cout <<"grid[1].center.x="<< gd3d.grid[1].index_of_voxel<<endl;
        temp_vector.clear();
        for(int num = 0;num < gd3d.grid[14].indexs.size();num++)
        {
	          temp_point.x = laserdata[gd3d.grid[14].indexs[num]].x;
	          temp_point.y = laserdata[gd3d.grid[14].indexs[num]].y;
	          temp_point.z = laserdata[gd3d.grid[14].indexs[num]].z;
            temp_vector.push_back(temp_point);
        }
        cout << temp_vector.size();
        */


        cout << "vec_voxel.size=" << gd3d.vec_voxel.size()<<endl;

	  	glviewer2->point_cloud = temp_vector;
  		glviewer2->reset();
  		temp_vector.clear();
    } 
}

void MainWindow::on_pushButton_peopledetect_clicked()
{
	ROS_INFO("hello people");
	if(ui.lineEdit_freq_people->text().toInt() == 0 )
	{
		QMessageBox::information(NULL, tr("Waring"), tr("param must be int")); 
		return;
	}
	mt = new Motor("0",360,ui.lineEdit_freq_people->text().toInt());
	mt->motorMoveWithStart_people();
	if(ui.is_save_people->isChecked())
	{
		t2->is_save(true);
		qDebug("is checked");
	}
	else 
	{
		t2->is_save(false);
	}
	//t2->messageemit_people();
	mygetdata->setseq(0);
	mygetdata->start();
	mygetdata->data_switch_fun(true);
	mygetdata->data_switch_people(true);
	mygetdata->start_data_fun(true);
	delete mt;
}


void MainWindow::on_pushButton_detect_clicked()
{
    cout << "people detect fun works" <<endl;
	QString path = QFileDialog::getOpenFileName(this, tr("Open PCD File"), ".", tr("PCD Files(*.PCD)")); 
	if(path.length() == 0)
	{ 
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files.")); 
    } 
    else
    { 
        //QMessageBox::information(NULL, tr("Path"), tr("You selected ") + path); 
        std::string file = path.toLatin1().data();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    	pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud);

	    pcl::PointXYZ temp_point;
	    std::vector < pcl::PointXYZ > temp_vector;
      vector<CPoint3d> laserdata;
      CPoint3d temp_CPoint;
        for (size_t i = 0; i < cloud->points.size (); ++i)
    	{
	      temp_CPoint.x = cloud->points[i].x;
	      temp_CPoint.y = cloud->points[i].y;
	      temp_CPoint.z = cloud->points[i].z;
        laserdata.push_back(temp_CPoint);
	  	}

        gd3d_p.Voxel2ize(laserdata,0.2);

        cout << "vec_voxel.size=" << gd3d_p.vec_voxel.size()<<endl;
        cout << "gd3d.vec_voxel.size="<<gd3d.vec_voxel.size()<<endl;
        for(int i =0;i<gd3d.vec_voxel.size();i++)
        {
           // cout << gd3d.vec_voxel[i]<<endl;
        }
        /*
        temp_vector.clear();
        for(int num = 0;num < gd3d.grid[14].indexs.size();num++)
        {
	          temp_point.x = laserdata[gd3d.grid[14].indexs[num]].x;
	          temp_point.y = laserdata[gd3d.grid[14].indexs[num]].y;
	          temp_point.z = laserdata[gd3d.grid[14].indexs[num]].z;
            temp_vector.push_back(temp_point);
        }
        cout << temp_vector.size();
        */
        temp_vector.clear();
        gd3d.vec_voxel.push_back(-1);
        for(int _i = 1; _i < gd3d_p.vec_voxel.size() ; _i++)
        {
            std::vector<int>::iterator position=std::find(gd3d.vec_voxel.begin()+1,gd3d.vec_voxel.end()-1,gd3d_p.vec_voxel[_i]);

            //cout << *position <<endl;
            //end()指向的是最后一个元素的下一个位置，所以需要-1
            if(*position == -1)
            {
               //把第_i个grid中的点的数据赋值给一个vector 
                
               cout << "第-i个grid和背景不一样"<<_i<<endl; 
               for(int num=0;num<gd3d_p.grid[_i].indexs.size() && gd3d_p.grid[_i].indexs.size()>10; num++)
               {
                    temp_point.x = laserdata[gd3d_p.grid[_i].indexs[num]].x;
                    temp_point.y = laserdata[gd3d_p.grid[_i].indexs[num]].y;
                    temp_point.z = laserdata[gd3d_p.grid[_i].indexs[num]].z;
                    temp_vector.push_back(temp_point);
                    clouds.push_back(temp_point);
               }
            }
        }
        cout << "temp_vector.size=" << temp_vector.size();
    
        //return;

  		glviewer2->reset();
	  	glviewer2->point_cloud = temp_vector;
  		glviewer2->reset();
  		temp_vector.clear();
    } 
}
void MainWindow::on_pushButton_kmeans_clicked()
{
    cout << "kmeans fun works" << endl;
    cout << clouds.points.size()<<"clouds.size()="<<endl<<&clouds<<endl;    
}

}  // namespace dlut_monitor

