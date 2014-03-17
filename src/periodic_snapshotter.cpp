/*********************************************************************
* Software License Agreement (BSD License
* Copyright (c) 2008, Willow Garage, Inc.)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*L'obiettivo è quello di inserire la libreria sensor_msgs, in modo da poter sfruttare anche 
* le funzioni di tale librearia che ci permettono di trasformare una PointCloud in PointCloud2.
* Ottenua la nuvola di punti in questo formato, non è necessaria una riconversione
* in PointCloud, dal momento che anche il formato PointCloud2 può essere
* visualizzato in rviz senza problemi.
* La coversione in PointCloud2 è necessaria perchè vogliamo sfruttare le funzioni offerte
* dalla libreria pcl_conversions, che ci permette di trasformare i sensor_msgs in pcl::PointCloud.
* In questo formato è possibile merge e altre operazioni utili, al fine di tenere traccia in maniera 
* ottimizzata delle scansioni fatte in precedenza.
*/

#include <cstdio>
#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans.h"

// Messages
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

//Conversione PCL
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>


/***
* This a simple test app that requests a point cloud from the
* point_cloud_assembler every 4 seconds, and then publishes the
* resulting data
*/



namespace laser_assembler
//using namespace pcl;

{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    //pub_ = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);
	
	//Il nuovo publisher pubblica dei messaggi di tipo PointCloud2, non PointCloud semplici come quello commentato
	//Nonostante i diversi tipi di messaggio, pubblica sempre sul topic "assembled_cloud".
	pub_ = n_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans>("assemble_scans");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(5,0), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void timerCallback(const ros::TimerEvent& e)
  {
		// Populate our service request based on our timer callback times
    AssembleScans srv;
	//Questa variabile sarà quella che verrà pubblicata e quella dove verrà inserita la PointCloud convertita.
		sensor_msgs::PointCloud2 pointCloud2;
	//Queste variabili servono per le varie conversioni.
		pcl::PCLPointCloud2 pcl_pc;
		pcl::PointCloud<pcl::PointXYZ> cloud;


 

    // We don't want to build a cloud the first callback, since we we
    // don't have a start and end time yet
	
	/*Il campionamento di srv.request.begin è stato messo all'interno di questo pezzo di codice condizionto.
	* Di fatto, il begin time viene campionato solo la prima volta, ovvero quando first_time viene settato a false
	* Tale tempo non verrà più modificato.
	* Se il tempo di tutti i nodi è sincornizzato (attenzione a questo!!), tutte le scansione verrano tenute in memoria
	* fino al riempimento del roller buffer nel laser_assembler
	*/
    if (first_time_)
    {
      first_time_ = false;
			//QUI HO FATTO UNA MODIFICA
			//srv.request.begin = e.last_real;
			//FINE DELLA MODIFICA
      return;
    }

    
    srv.request.begin = e.last_real;
    srv.request.end = e.current_real;

		std::cout << srv.request.begin << "\n";
		std::cout << srv.request.end << "\n";

    // Make the service call
    if (client_.call(srv))
    {
      ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
      
	  
	  //Qui è inserita la conversione vera e propria.
	 
	  
	  sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, pointCloud2);

		//Porto il messaggio PointCloud2 al tipo PointCloud<pcl::PointXYZ> per tenerne traccia in maniera
		//persistente, passando per il tipo pcl::PCLPointCloud2
		 
	  pcl_conversions::toPCL(pointCloud2, pcl_pc);
		pcl::fromPCLPointCloud2(pcl_pc, cloud);

		//Aggiungo la PointCloud ricavata alla variabile persistente
		
		persistent_cloud = cloud + persistent_cloud;

		//Converto in sensor_msgs::PointCloud2 la nuvola persistente
		pcl::toPCLPointCloud2(persistent_cloud, pcl_pc);
		pcl_conversions::fromPCL(pcl_pc, pointCloud2);
		
		std::basic_string<char> fixed_frame = "base_link";
		pointCloud2.header.frame_id = fixed_frame;

	  //Non pubblico più il messaggio di risposta del servizio, ma quello ottenuto dalla conversione.
	  //pub_.publish(srv.response.cloud);
		ROS_INFO("Header, for fixed frame %s", pointCloud2.header.frame_id.c_str()) ;
	  pub_.publish(pointCloud2);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
	pcl::PointCloud<pcl::PointXYZ> persistent_cloud;
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
