/*
 *  QUT cyphy linear Kalman Filter implementation for 2D tracking
 *  Copyright (C) 2012, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _USE_MATH_DEFINES
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/StdVector>
#include <iostream>
#include "Kalman.h"
#include <cmath>
#include <vector>
#include "ros/ros.h"


using namespace Eigen;
using namespace std;


Kalman::Kalman():ts_(0.005*WINDOW_SIZE)
{
	 ROS_INFO("Kalman Constructor");
	 m_KM_initial=false;
	 fp_med_vel = fopen("med_vel.data","w");
	 fp_cov=fopen("cov.data","w");
     fprintf(fp_cov,"(0 0),(1 0),(2 0),(3 0),(0 1),(1 1),(2 1),(3 1),(0 2),(1 2),(2 2),(3 2),(0 3),(1 3),(2 3),(3 3)\n");
     fprintf(fp_med_vel,"time,med_vx,med_vy\n");
}

void Kalman::init()
{
	m_A << 1,0,0,0,
		   0,1,0,0,
		   0,0,1,0,
		   0,0,0,1;
	


    m_H << 1,0,0,0,
		   0,1,0,0,
		   0,0,1,0,
		   0,0,0,1;
	

    Matrix4f process_variances;

	process_variances=process_variances.Identity();

	process_variances(0,0)=PRO_POSNOISE;
	process_variances(1,1)=PRO_POSNOISE;
	process_variances(2,2)=PRO_VELNOISE;
	process_variances(3,3)=PRO_VELNOISE;
	
	
	m_Q=m_Q.Identity()*process_variances;
	//m_R=15*m_R.Identity();
    //m_R=MEA_VELNOISE*m_R.Identity();
    m_R<< MEA_POSNOISE,0,0,0,
		  0,MEA_POSNOISE,0,0,
		  0,0,MEA_VELNOISE,0,
		  0,0,0,MEA_VELNOISE;

	m_x=m_x.Zero();
	vmeasure[0].clear();
	vmeasure[1].clear();
	pmeasure[0].clear();
	pmeasure[1].clear();

	m_P=20*m_P.Identity();
	//m_P=100*m_P.Identity();
	m_KM_initial=false;
	m_LPF_initial=false;
	m_AVG_initial=false;
	m_MOVING_AVG_initial=false;
	m_total_num_samp=0;
	m_moving_num_samp=0;
}

Matrix<float, 4, 1> Kalman::process(Matrix<float, 4, 1> measure,double yaw,bool* invertable)
{
	 
	 Matrix<float, 4, 4>Pp;
	 Matrix<float, 4, 4>K;
	 Matrix<float, 4, 1>z;
	 Matrix<float, 4, 1>innov;
	 Matrix<float, 4, 4> s;


	m_A<< 1,0,ts_*cos(yaw),-ts_*sin(yaw),
		  0,1,ts_*sin(yaw), ts_*cos(yaw),
		  0,0,      1     ,     0      ,
		  0,0,      0     ,     1      ;
	


	 m_x=m_A*m_x;
	 Pp=m_A*m_P*m_A.transpose()+m_Q;
	 z=measure;
	 innov=z-m_H*m_x;
	 s=m_H*Pp*m_H.transpose()+m_R;
	 if(s.determinant()!=0)
	 {
	     *invertable=1;
		 K=Pp*m_H.transpose()*s.inverse();
		 m_x=m_x+K*innov;
		 m_P=Pp-K*m_H*Pp;
		 fprintf(fp_cov,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",m_P(0 ,0),m_P(1, 0),m_P(2, 0),m_P(3, 0),m_P(0, 1),m_P(1 ,1),m_P(2, 1),m_P(3 ,1),m_P(0, 2),m_P(1, 2),m_P(2, 2),m_P(3, 2),m_P(0, 3),m_P(1, 3),m_P(2 ,3),m_P(3 ,3));
		 return m_x;
	 }
	 else
	 {
		 *invertable=0;
	 }
}

Matrix<float, 2, 1> Kalman::LPF(Matrix<float, 2, 1> measure)
{
   if(!m_LPF_initial)
   {
     prevX_lpf=measure;
	 m_LPF_initial=true;
	 return prevX_lpf;
   }
   else
   {
	     lpf_x=alpha*prevX_lpf+(1-alpha)*measure;
		 prevX_lpf=lpf_x;
		 return lpf_x;
   }
}

Matrix<float, 2, 1> Kalman::AVG(Matrix<float, 2, 1> measure)
{
   if(!m_AVG_initial)
   {
     m_total_num_samp++;
	 prevX_avg=measure;
	 m_AVG_initial=true;
	 return prevX_avg;
   }
   else
   {
     m_total_num_samp++;
     float alp=(m_total_num_samp-1)/m_total_num_samp;
	 avg_x=alp*prevX_avg+(1-alp)*measure;
	 prevX_avg=avg_x;
	 return avg_x;   
   }
	
}

Matrix<float, 2, 1> Kalman::median(Matrix<float, 2, 1> measure,bool* flag)
{
     ros::Time now = ros::Time::now();
	 Matrix<float, 2, 1> ret;
	 if(vmeasure[0].size()==WINDOW_SIZE-1)
	 {
		//cout<<"vx = "<<vmeasure[0]<<"vy = "<<vmeasure[1]<<endl;
		sort(vmeasure[0].begin(), vmeasure[0].end());
		sort(vmeasure[1].begin(), vmeasure[1].end());

		#if 0
		//cout<<"vx = "<<vmeasure[0]<<"vy = "<<vmeasure[1]<<endl;
	    for(int i =0;i<vmeasure[0].size();i++)
	    {
		    cout<<vmeasure[0][i]<<endl;
			cout<<vmeasure[1][i]<<endl;
	    }
		#endif
		
		
		ret(0) = vmeasure[0][vmeasure[0].size()/2];
		ret(1) = vmeasure[1][vmeasure[1].size()/2];
		vmeasure[0].clear();
		vmeasure[1].clear();
		fprintf(fp_med_vel,"%f,%f,%f\n",now.toSec(),ret(0),ret(1));
		//cout<<"vx = "<<ret(0)<<"vy = "<<ret(1)<<endl;
		*flag=1;
		return ret;	
	 }
	 else
	 {
		 vmeasure[0].push_back(measure(0)); //vx
		 vmeasure[1].push_back(measure(1)); //vy
		 //*flag=0;
	 }
	 
}

Matrix<float, 2, 1> Kalman::MOVING_AVG(Matrix<float, 2, 1> measure)
{
  return measure;
	
}
Kalman::~Kalman()
  {
  }
