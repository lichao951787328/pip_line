#pragma once
#pragma warning(disable: 4996)
#pragma warning(disable: 4819)
#define _CRT_SECURE_NO_WARNINGS

#include <map>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>
#include <pcl/io/openni_grabber.h>

#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>
#include "AHCPlaneFitter_AHFP.hpp"

namespace AHFP{
// #include <polytopic/polytopic.h>
using ahc::utils::Timer;

// pcl::PointCloud interface for our ahc::PlaneFitter
template<class PointT>
struct OrganizedImage3D {
	const pcl::PointCloud<PointT>& cloud;
	//note: ahc::PlaneFitter assumes mm as unit!!!
	const double unitScaleFactor;

	OrganizedImage3D(const pcl::PointCloud<PointT>& c) : cloud(c), unitScaleFactor(1) {}
	OrganizedImage3D(const OrganizedImage3D& other) : cloud(other.cloud), unitScaleFactor(other.unitScaleFactor) {}

	inline int width() const { return cloud.width; }
	inline int height() const { return cloud.height; }
	inline bool get(const int row, const int col, double& x, double& y, double& z) const {
		const PointT& pt=cloud.at(col,row);
		x=pt.x*unitScaleFactor; y=pt.y*unitScaleFactor; z=pt.z*unitScaleFactor; //TODO: will this slowdown the speed?
		// return pcl_isnan(z)==0; //return false if current depth is NaN
		return !std::isnan(z);
	}
};
typedef OrganizedImage3D<pcl::PointXYZ> ImageXYZ;
typedef ahc::PlaneFitter< ImageXYZ > PlaneFitter;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

namespace global_AstarHierarchicalFootstepPlanner {
std::map<std::string, std::string> ini;
PlaneFitter pf;
bool showWindow = false;

#ifdef _WIN32
const char filesep = '\\';
#else
const char filesep = '/';
#endif

// similar to matlab's fileparts
// if in=parent/child/file.txt
// then path=parent/child
// name=file, ext=txt
void fileparts(const std::string& str, std::string* pPath=0,
	std::string* pName=0, std::string* pExt=0)
{
	std::string::size_type last_sep = str.find_last_of(filesep);
	std::string::size_type last_dot = str.find_last_of('.');
	if (last_dot<last_sep) // "D:\parent\child.folderA\file", "D:\parent\child.folderA\"
		last_dot = std::string::npos;

	std::string path, name, ext;

	if (last_sep==std::string::npos) {
		path = "";
		if(last_dot==std::string::npos) { // "test"
			name = str;
			ext = "";
		} else { // "test.txt"
			name = str.substr(0, last_dot);
			ext = str.substr(last_dot+1);
		}
	} else {
		path = str.substr(0, last_sep);
		if(last_dot==std::string::npos) { // "d:/parent/test", "d:/parent/child/"
			name = str.substr(last_sep+1);
			ext = "";
		} else { // "d:/parent/test.txt"
			name = str.substr(last_sep+1, last_dot-last_sep-1);
			ext = str.substr(last_dot+1);
		}
	}

	if(pPath!=0) {
		*pPath = path;
	}
	if(pName!=0) {
		*pName = name;
	}
	if(pExt!=0) {
		*pExt = ext;
	}
}

//"D:/test/test.txt" -> "D:/test/"
std::string getFileDir(const std::string &fileName)
{
	std::string path;
	fileparts(fileName, &path);
	return path;
}

//"D:/parent/test.txt" -> "test"
//"D:/parent/test" -> "test"
std::string getNameNoExtension(const std::string &fileName)
{
	std::string name;
	fileparts(fileName, 0, &name);
	return name;
}

void iniLoad(std::string iniFileName) {
	std::ifstream in(iniFileName);
	if(!in.is_open()) {
		std::cout<<"[iniLoad] "<<iniFileName<<" not found, use default parameters!"<<std::endl;
		return;
	}
	while(in) {
		std::string line;
		std::getline(in, line);
		if(line.empty() || line[0]=='#') continue;
		std::string key, value;
		size_t eqPos = line.find_first_of("=");
		if(eqPos == std::string::npos || eqPos == 0) {
			std::cout<<"[iniLoad] ignore line:"<<line<<std::endl;
			continue;
		}
		key = line.substr(0,eqPos);
		value = line.substr(eqPos+1);
		std::cout<<"[iniLoad] "<<key<<"=>"<<value<<std::endl;
		ini[key]=value;
	}
}

template<class T>
T iniGet(std::string key, T default_value) {
	std::map<std::string, std::string>::const_iterator itr=ini.find(key);
	if(itr!=ini.end()) {
		std::stringstream ss;
		ss<<itr->second;
		T ret;
		ss>>ret;
		return ret;
	}
	return default_value;
}

template<> std::string iniGet(std::string key, std::string default_value) {
	std::map<std::string, std::string>::const_iterator itr=ini.find(key);
	if(itr!=ini.end()) {
		return itr->second;
	}
	return default_value;
}
}//global
int index_image = 1;


// vector<polytopic> result_polytopics;
CloudXYZRGB segColorCloud;
cv::Mat seg_image;
std::vector<cv::Mat> planepixel;
cv::Mat seg_unfeasible;
void processOneFrame(pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& outputFilePrefix)
{
	using global_AstarHierarchicalFootstepPlanner::pf;
	cv::Mat seg(cloud.height, cloud.width, CV_8UC3);

	//run PlaneFitter on the current frame of point cloud
	ImageXYZ Ixyz(cloud);
	Timer timer(1000);
	timer.tic();
	pf.run(&Ixyz, 0, &seg);
	double process_ms=timer.toc();
	std::cout<<process_ms<<" ms"<<std::endl;
	planepixel = pf.planes;
	seg_unfeasible = pf.unfeasible_image;
	// for (auto & poly_2d : pf.extractedPlanes)
	// {
	// 	// LOG(INFO)<<"CONTOURS SIZE "<<poly_2d->contours_2d.size();
	// 	// for (auto & point_2d_v : poly_2d->contours_2d)
	// 	// {
	// 	// 	LOG(INFO)<<"point size "<<point_2d_v.size();
	// 	// 	for (auto & point_2d : point_2d_v)
	// 	// 	{
	// 	// 		seg.at<cv::Vec3b>(point_2d) = cv::Vec3b(255, 255, 255);		
	// 	// 	}
	// 	// }
	// 	cv::drawContours(seg, poly_2d->contours_2d, -1, cv::Scalar(0, 0, 255), 2);
	// }
// cv::imshow("seg",seg);
// cv::waitKey(0);
	//save seg image
	cv::cvtColor(seg,seg,cv::COLOR_RGB2BGR);
	seg_image = seg.clone();
	// cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/elevation_mapping/debug_pcl/preprocess/" + std::to_string(index_image) + ".seg.png", seg);
	// std::cout<<"output: "<<outputFilePrefix<<".seg.png"<<std::endl;
	// result_polytopics.clear();
	// LOG(INFO)<<"plane size "<<pf.extractedPlanes.size();
	size_t index = 0;
	// for (auto & poly:pf.extractedPlanes)
	// {
	// 	// LOG(INFO)<<"emplace back polytopic";
	// 	Eigen::Vector3f normal(poly->normal[0], poly->normal[1], poly->normal[2]);
	// 	Eigen::Vector3f center(poly->center[0], poly->center[1], poly->center[2]);
	// 	// LOG(INFO)<<"start intial poly";
	// 	LOG(INFO)<<poly->contours.at(0).size();
	// 	if (poly->contours.size() == 0)
	// 	{
	// 		continue;
	// 	}
	// 	// 其实只要返回的轮廓为是固定顺序的，那么先判断顺序，再把他们理成顺时针也是可以的，前提是，他们必须是有序的
		
	// 	// cout<<poly->contours.size()<<endl;
	// 	polytopic tmppolytopic(normal, poly->contours, poly->N, center, poly->mse);
	// 	// LOG(INFO)<<tmppolytopic.outer_points.size();
	// 	// LOG(INFO)<<tmppolytopic.getOuter().size();
	// 	result_polytopics.emplace_back(tmppolytopic);
	// 	// LOG(INFO)<<(result_polytopics.end()-1)->getOuter().size();
	// }
	// LOG(INFO)<<"result_polytopics first outer size "<<result_polytopics.at(0).getOuter().size()<<endl;
	// LOG(INFO)<<"give color";
	//save seg cloud
	// std::cout<<"heree...***."<<std::endl;
	CloudXYZRGB xyzrgb(cloud.width, cloud.height);
	for(int r=0; r<(int)xyzrgb.height; ++r) {
		for(int c=0; c<(int)xyzrgb.width; ++c) {
			if (seg.at<cv::Vec3b>(r, c) != cv::Vec3b(0, 0, 0))
			{
				pcl::PointXYZRGB& pix = xyzrgb.at(c, r);
				const pcl::PointXYZ& pxyz = cloud.at(c, r);
				const cv::Vec3b& prgb = seg.at<cv::Vec3b>(r,c);
				pix.x=pxyz.x;
				pix.y=pxyz.y;
				pix.z=pxyz.z;
				pix.r=prgb(2);
				pix.g=prgb(1);
				pix.b=prgb(0);
			}
		}
	}
	// std::cout<<"heree...."<<std::endl;
	segColorCloud = xyzrgb;
	// std::cout<<"heree..。。.."<<std::endl;
	// LOG(INFO)<<"save pcd";
	// pcl::io::savePCDFileBinary(outputFilePrefix + std::to_string(index_image) + ".seg.pcd", xyzrgb);
	index_image++;

	if(global_AstarHierarchicalFootstepPlanner::showWindow) {
		//show frame rate
		std::stringstream stext;
		stext<<"Frame Rate: "<<(1000.0/process_ms)<<"Hz";
		cv::putText(seg, stext.str(), cv::Point(15,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));

		cv::imshow("seg", seg);
		cv::waitKey(10);
	}
}

int process(pcl::PointCloud<pcl::PointXYZ> & orderd_cloud, ahc::FitterAllParams & parameters) {
	// global::iniLoad("../plane_fitter_pcd.ini");
	const double unitScaleFactor = global_AstarHierarchicalFootstepPlanner::iniGet<double>("unitScaleFactor", 1.0f);
    std::cout<<"unitScaleFactor: "<<unitScaleFactor<<std::endl;
	const std::string outputDir = global_AstarHierarchicalFootstepPlanner::iniGet<std::string>("outputDir", ".");
	{//create outputDir
#ifdef _WIN32
		std::string cmd="mkdir "+outputDir;
#else
		std::string cmd="mkdir -p "+outputDir;
#endif
		system(cmd.c_str());
	}

	using global_AstarHierarchicalFootstepPlanner::pf;
	//setup fitter
	pf.minSupport = parameters.minSupport;
	pf.windowWidth = parameters.windowWidth;
	pf.windowHeight = parameters.windowHeight;
	pf.doRefine = parameters.doRefine;

	pf.params.initType = parameters.para.initType;

	//T_mse
	pf.params.stdTol_merge = parameters.para.stdTol_merge;
	pf.params.stdTol_init = parameters.para.stdTol_init;
	pf.params.depthSigma = parameters.para.depthSigma;

	//T_dz
	pf.params.depthAlpha = parameters.para.depthAlpha;
	pf.params.depthChangeTol = parameters.para.depthChangeTol;

	//T_ang
	pf.params.z_near = parameters.para.z_near;
	pf.params.z_far = parameters.para.z_far;

	pf.params.angle_near = parameters.para.angle_near;
	pf.params.angle_far = parameters.para.angle_far;

	pf.params.similarityTh_merge = parameters.para.similarityTh_merge;
	pf.params.similarityTh_refine = parameters.para.similarityTh_refine;

	using global_AstarHierarchicalFootstepPlanner::showWindow;
	showWindow = global_AstarHierarchicalFootstepPlanner::iniGet("showWindow", false);
	if(showWindow)
		cv::namedWindow("seg");
    std::string outputFilePrefix = outputDir+global_AstarHierarchicalFootstepPlanner::filesep;
    pcl::transformPointCloud<pcl::PointXYZ>(
				orderd_cloud, orderd_cloud,
				Eigen::Affine3f(Eigen::UniformScaling<float>(
				(float)unitScaleFactor)));
    processOneFrame(orderd_cloud, outputFilePrefix);
	
	return 0;
}


class PlanarContourExtraction
{
private:
	/**
	 * input
	*/
	pcl::PointCloud<pcl::PointXYZ> cloud;
	ahc::FitterAllParams parameters;

	/**
	 * output
	*/
	// vector<polytopic> polytopics;
	cv::Mat image;
	pcl::PointCloud<pcl::PointXYZ> plane_cloud;
	vector<cv::Mat> seg_planes;
	cv::Mat seg_unfeasible_image;
public:
	PlanarContourExtraction()
	{

	}

	PlanarContourExtraction(pcl::PointCloud<pcl::PointXYZ> & input_cloud)
	{
		cloud = input_cloud;
	}

	PlanarContourExtraction(pcl::PointCloud<pcl::PointXYZ> & input_cloud, ahc::FitterAllParams & input_parameters)
	{
		cloud = input_cloud;
		parameters = input_parameters;
	}
	void run()
	{
		// LOG(INFO)<<"run";
		process(cloud, parameters);
		// LOG(INFO)<<"plane extraction";
		// LOG(INFO)<<result_polytopics.at(0).getOuter().size()<<endl;
		// polytopics = result_polytopics;
		// std::cout<<"heree.."<<std::endl;
		pcl::copyPointCloud(segColorCloud, plane_cloud);
		image = seg_image;
		seg_planes = planepixel;
		seg_unfeasible_image = seg_unfeasible;
		// std::cout<<"heree...."<<std::endl;
	}
	// inline vector<polytopic> get_poly()
	// {
	// 	return polytopics;
	// }

	inline pcl::PointCloud<pcl::PointXYZ> get_planeCloud()
	{
		return plane_cloud;
	}
	
	inline cv::Mat getSegImage()
	{
		return image;
	}
	inline vector<cv::Mat> getSegPlanes()
	{
		return seg_planes;
	}

	inline cv::Mat getUnfeasibleRegion()
	{
		return seg_unfeasible_image;
	}
	~PlanarContourExtraction()
	{
		cloud.clear();
		// polytopics.clear();
		plane_cloud.clear();
	}
};


}