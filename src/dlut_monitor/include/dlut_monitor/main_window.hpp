/**
 * @file /include/dlut_monitor/main_window.hpp
 *
 * @brief Qt based gui for dlut_monitor.
 *
 * @date November 2010
 **/
#ifndef dlut_monitor_MAIN_WINDOW_H
#define dlut_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QMainWindow>
#include <QGridLayout>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "glviewer.h"
#include <pcl/point_types.h>
#include "motor.h"
#include "getdata.h"
#include "t1.h"
#include <pcl/ros/conversions.h>
#include "../../src/Grid3D.h"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace dlut_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	


	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
Q_SIGNALS:
	void showdata();
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void update();
	void update_people();

private Q_SLOTS:
	void on_DA_start_clicked(bool check);
	void on_DA_end_clicked(bool check);
	void on_pushButton_openfile_clicked();
	void on_pushButton_peopledetect_clicked();
  void on_pushButton_detect_clicked();
  void on_pushButton_kmeans_clicked();
    /******************************************
    ** Manual connections
    *******************************************/


private:
	Ui::MainWindowDesign ui;
	
    GLViewer* glviewer1;
    GLViewer* glviewer2;
    GLViewer* glviewer3;
    MyGetDataThread* mygetdata;
	QNode qnode;
	Motor* mt;
	t1* t2;

  seg::Grid3D gd3d;
  seg::Grid3D gd3d_p;
  pcl::PointCloud<pcl::PointXYZ> clouds;
  //(new pcl::PointCloud<pcl::PointXYZ ());


};  // namespace dlut_monitor
}
#endif // dlut_monitor_MAIN_WINDOW_H
