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


#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
namespace fs = boost::filesystem;


// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" -D <target_directory> [options]\n\n"
	        << "-D           set target directory\n"
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
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("XYZ Viewer"));
	
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

/*
 * キーボードイベントを自前で定義する
 * registerKeyboardCallbackをコールしてviewerに設定すること
 * viewerにデフォルトで組み込まれているキーと被ると、動作も被るので注意すること
 */
bool waitflag = true;
unsigned int num = 1;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "y" && event.keyDown())
	{
		std::cout << "y was pressed.";
		
		if (waitflag){
			waitflag = false;
			std::cout << "   wait: true->[false]" << std::endl;
		}
		else{
			waitflag = true;
			std::cout << "   wait: false->[true]" << std::endl;
		}
	}
	if (event.getKeySym() == "m" && event.keyDown())
	{
		std::cout << "jump to 1st frame.";
		num = 0;
	}
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

	bool simple(true), custom_c(false), savess(false);

	if (pcl::console::find_argument(argc, argv, "-s") >= 0)
	{
		simple = true;
		std::cout << "Simple visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
	{
		custom_c = true;
		simple = false;
		std::cout << "Custom colour visualisation example\n";
	}

	if (pcl::console::find_argument(argc, argv, "-S") >= 0)
	{
		savess = true;
		std::cout << "Save screen shot\n";
	}
	
	std::string xyzdir = ".";
	if (pcl::console::parse(argc, argv, "-D", xyzdir) >= 0)
	{
		cout << "Using Directory:" << xyzdir << ".\n";
	}
	else{
		PCL_ERROR("You must set target directory with -D.\n");
		return 0;
	}

	if (!fs::is_directory(xyzdir)){
		PCL_ERROR("invalid directory %s\n", xyzdir.c_str());
		return 0;
	}

	const fs::path path(xyzdir);
	std::vector<std::string> xyzs;
	BOOST_FOREACH(const fs::path& p, std::make_pair(fs::directory_iterator(path), fs::directory_iterator())) {
		if (!fs::is_directory(p)){
			if (p.extension().string() == ".xyz"){
				xyzs.push_back(p.string());
			}
		}
	}

	if (xyzs.size() == 0){
		PCL_ERROR("No .xyz file exists under %s\n", xyzdir.c_str());
		return 0;
	}

	std::cout << xyzs.size() << " files found.\n";

	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	if (!loadCloud(xyzs[0], basic_cloud_ptr)){
		std::cout << "Reading xyz failed (" << xyzs[0] << ") .....\n\n";
		return (-1);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	if (simple)
	{
	    viewer = simpleVis(basic_cloud_ptr);
	}
	else if (custom_c)
	{
	    viewer = customColourVis(basic_cloud_ptr);
	}

	viewer->addText(xyzs[0], 0, 20, "fname");
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

	std::cout << "Press h key to show VTK help.\n\n";
	std::cout << "Press y key to start loop\n";

	/*
	while (waitflag){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
	
	

	
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		/* 表示の更新 */
		viewer->spinOnce(100);

		/* wait */
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

		/* waitflagがtrueのときは更新しない yキーで切り替え */
		if (waitflag == false){
			if (num >= xyzs.size()){
				num = 0;
			}

			basic_cloud_ptr->clear();
			loadCloud(xyzs[num], basic_cloud_ptr);

			viewer->updatePointCloud(basic_cloud_ptr, "sample cloud");
			viewer->updateText(xyzs[num], 0, 20, "fname");

			if (savess)
			{
				std::string ssname = "ss_" + std::to_string(num) + ".png";
				std::string camname = "ss_" + std::to_string(num) + ".cam";
				viewer->saveScreenshot(ssname);
				viewer->saveCameraParameters(camname);
			}

			num++;
		}

	}
}