/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "CameraSystem.h"
#include <Common.h>

#include <thread>

#include "PCLViewer.h"
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main ()
{
  Voxel::logger.setDefaultLogLevel(Voxel::LOG_INFO);
  
  Voxel::CameraSystem sys;
  Voxel::DepthCameraPtr depthCamera;
  
  const Voxel::Vector<Voxel::DevicePtr> &devices = sys.scan();
  
  if(devices.size() > 0)
    depthCamera = sys.connect(devices[0]); // Connect to first available device
  else
  {
    std::cerr << "SimplePCLViewer: Could not find a compatible device." << std::endl;
    return -1;
  }
  
  if(!depthCamera)
  {
    std::cerr << "SimplePCLViewer: Could not open a depth camera." << std::endl;
    return -1;
  }
    
  Voxel::PCLViewer v;
  
  v.setDepthCamera(depthCamera);
  
  v.start();

//  pcl::PointCloud<pcl::PointXYZI>::Ptr pclPtr = v.getPclPtr();
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> > pcl_shared_ptr = v.getPclSharedPtr();


  while(v.isRunning()) {

//	 if(!pclPtr){
//		pclPtr = v.getPclPtr();
//		continue;
//	}
//	printf("%f,%f,%f,%d\n",(pclPtr->at(120,120)).x,(pclPtr->at(120,120)).y,(pclPtr->at(120,120)).z,pclPtr->size());
	
	if(!pcl_shared_ptr) {
		pcl_shared_ptr = v.getPclSharedPtr();
		continue;
	}

	printf("%f,%f,%f,%d\n",(pcl_shared_ptr->at(120,120)).x,(pcl_shared_ptr->at(120,120)).y,(pcl_shared_ptr->at(120,120)).z,
			pcl_shared_ptr->size());

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
