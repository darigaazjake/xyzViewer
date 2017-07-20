/* Yoshihiro Murakami
 * xyzファイルを描画するだけのプログラム
 */



#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
			<< "Options:\n"
			<< "-------------------------------------------\n"
			<< "-h           this help\n"
			<< "-s           Simple visualisation example\n"
			<< "-r           RGB colour visualisation example\n"
			<< "-c           Custom colour visualisation example\n"
			<< "-d           DOSEI-san\n"
			<< "\n\n";
}

/*
 * xyzファイルを読み込むだけ
 * https://github.com/PointCloudLibrary/pcl/blob/master/tools/xyz2pcd.cpp
 */
bool loadCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::ifstream fs;
	fs.open(filename.c_str(), ios::binary);
	if (!fs.is_open() || fs.fail())
	{
		PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
		fs.close();
		return (false);
	}

	std::string line;
	std::vector<std::string> st;

	while (!fs.eof())
	{
		getline(fs, line);
		if (line == "")
			continue;

		/* 行を区切る */
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r ,"), boost::token_compress_on);

		/* 要素数がちょうど3しか受け付けない */
		if (st.size() != 3){
			//std::cout << "    can't find 3 elements. skip this line. \n    ("<< line << "\n";
			continue;
		}

		/* PointXYZ を作って、そのままcloudに押し込む */
		cloud->push_back(pcl::PointXYZ(float(atof(st[0].c_str())), float(atof(st[1].c_str())), float(atof(st[2].c_str()))));
	}
	fs.close();

	cloud->width = uint32_t(cloud->size());
	cloud->height = 1;
	cloud->is_dense = true;

	return (true);
}

/*
 * [1] PCLVisualizerの最も簡単なサンプル
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	/* 1: viewerオブジェクトを生成する */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	/* 2: 背景色の設定
	 *     デフォルトは(0, 0, 0)なので、実はコールしなくても結果は同じ
	 */
	viewer->setBackgroundColor(0, 0, 0);
	
	/* 3:[最重要] viewerに点群データを追加する
	 *     コールするたびに、点群が追加されていく
	 *     追加するたびに、点群を識別するIDをstringで与えること
	 */
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

	/* 4: 点の大きさを指定する */
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	
	/* 5: 座標軸を表示する サイズは1.0
	 *    pclは可視化にVTKを用いており、VTKはOpenGLで作られている
	 *    --->右手座標系になる
	 */
	viewer->addCoordinateSystem(1.0);
	
	/* 6: いい感じにcameraパラメータを調整 */
	viewer->initCameraParameters();

	return (viewer);
}

/*
 * [2] 色をつける例
 *     [in]PointXYZRGB型に変わっていることに注意 色付きの点群データ
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	/* 1: rgbというハンドラを作成する */
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	
	/* 2: ハンドラを利用してaddする */
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");

	/* 3: サイズは3にする */
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

/*
 * [3] カスタムの色づけをするために、ハンドラを任意の色で作成
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	
	/* 1: 緑色になるように色ハンドラを生成 */
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	
	/* 2: 作成したハンドラを用いてadd */
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
	/* コマンドライン引数の処理 */
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}

	bool simple(false), rgb(false), custom_c(false);
	std::string xyzname("D:/work/pcl_viewer/cow/cow.xyz");

	if (pcl::console::find_argument(argc, argv, "-s") >= 0)
	{
		simple = true;
		std::cout << "Simple visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
	{
		custom_c = true;
		std::cout << "Custom colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-r") >= 0)
	{
		rgb = true;
		std::cout << "RGB colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-d") >= 0)
	{
		xyzname = "D:/work/pcl_viewer/dosei/dosei.xyz";
	}
	else
	{
		printUsage(argv[0]);
		return 0;
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	if (!loadCloud(xyzname, basic_cloud_ptr)){
		std::cout << "Reading xyz failed (" << xyzname << ") .....\n\n";
		return (-1);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	/*
	if (simple)
	{
	viewer = simpleVis(basic_cloud_ptr);
	}
	else if (rgb)
	{
	viewer = rgbVis(point_cloud_ptr);
	}
	else if (custom_c)
	{
	viewer = customColourVis(basic_cloud_ptr);
	}
	*/

	viewer = simpleVis(basic_cloud_ptr);

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}