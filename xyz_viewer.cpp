/* Yoshihiro Murakami
 * xyz�t�@�C����`�悷�邾���̃v���O����
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
 * xyz�t�@�C����ǂݍ��ނ���
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

		/* �s����؂� */
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r ,"), boost::token_compress_on);

		/* �v�f�������傤��3�����󂯕t���Ȃ� */
		if (st.size() != 3){
			//std::cout << "    can't find 3 elements. skip this line. \n    ("<< line << "\n";
			continue;
		}

		/* PointXYZ ������āA���̂܂�cloud�ɉ������� */
		cloud->push_back(pcl::PointXYZ(float(atof(st[0].c_str())), float(atof(st[1].c_str())), float(atof(st[2].c_str()))));
	}
	fs.close();

	cloud->width = uint32_t(cloud->size());
	cloud->height = 1;
	cloud->is_dense = true;

	return (true);
}

/*
 * [1] PCLVisualizer�̍ł��ȒP�ȃT���v��
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	/* 1: viewer�I�u�W�F�N�g�𐶐����� */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	/* 2: �w�i�F�̐ݒ�
	 *     �f�t�H���g��(0, 0, 0)�Ȃ̂ŁA���̓R�[�����Ȃ��Ă����ʂ͓���
	 */
	viewer->setBackgroundColor(0, 0, 0);
	
	/* 3:[�ŏd�v] viewer�ɓ_�Q�f�[�^��ǉ�����
	 *     �R�[�����邽�тɁA�_�Q���ǉ�����Ă���
	 *     �ǉ����邽�тɁA�_�Q�����ʂ���ID��string�ŗ^���邱��
	 */
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

	/* 4: �_�̑傫�����w�肷�� */
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	
	/* 5: ���W����\������ �T�C�Y��1.0
	 *    pcl�͉�����VTK��p���Ă���AVTK��OpenGL�ō���Ă���
	 *    --->�E����W�n�ɂȂ�
	 */
	viewer->addCoordinateSystem(1.0);
	
	/* 6: ����������camera�p�����[�^�𒲐� */
	viewer->initCameraParameters();

	return (viewer);
}

/*
 * [2] �F�������
 *     [in]PointXYZRGB�^�ɕς���Ă��邱�Ƃɒ��� �F�t���̓_�Q�f�[�^
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	/* 1: rgb�Ƃ����n���h�����쐬���� */
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	
	/* 2: �n���h���𗘗p����add���� */
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");

	/* 3: �T�C�Y��3�ɂ��� */
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

/*
 * [3] �J�X�^���̐F�Â������邽�߂ɁA�n���h����C�ӂ̐F�ō쐬
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	
	/* 1: �ΐF�ɂȂ�悤�ɐF�n���h���𐶐� */
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	
	/* 2: �쐬�����n���h����p����add */
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
	/* �R�}���h���C�������̏��� */
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}

	bool simple(true), custom_c(false), savess(false);
	std::string xyzname("D:/work/pcl_viewer/cow/cow.xyz");
	if (pcl::console::find_argument(argc, argv, "-d") >= 0)
	{
		xyzname = "D:/work/pcl_viewer/dosei/dosei.xyz";
	}
	else if (pcl::console::find_argument(argc, argv, "-f") >= 0)
	{
		xyzname = "D:/work/pcl_viewer/ff/n901.xyz";
	}

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

	//std::string xyzdir = "D:/work/pcl_viewer/dosei_sweep";
	
	std::string xyzdir = ".";
	if (pcl::console::parse(argc, argv, "-D", xyzdir) >= 0)
	{
		cout << "Using Directory:" << xyzdir << ".\n";
	}
	else{
		PCL_ERROR("You must set target directory with -D.\n");
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
		PCL_ERROR("No .xyz file exists under %s !\n", xyzdir.c_str());
		return 0;
	}

	std::cout << xyzs.size() << " files found.\n";

	xyzname = xyzs[0];

	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	if (!loadCloud(xyzname, basic_cloud_ptr)){
		std::cout << "Reading xyz failed (" << xyzname << ") .....\n\n";
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

	viewer->addText(xyzname, 0, 20, "fname");

	std::cout << "Press h key to show VTK help.\n\n";

	unsigned int num = 1;
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		/* �\���̍X�V */
		viewer->spinOnce(100);

		/* wait */
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

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