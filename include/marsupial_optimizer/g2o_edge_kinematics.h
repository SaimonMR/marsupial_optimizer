#pragma once

#ifndef G2O_KINEMATICS_EDGE_H
#define G2O_KINEMATICS_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"


namespace g2o 
{

	class G2OKinematicsEdge : public BaseMultiEdge<3, double>
	{
	public:
		G2OKinematicsEdge();
		
		double factor_ = 10.0;

		void computeError()
		{
			const VertexPointXYZ* pose1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
			const VertexPointXYZ* pose2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
			const VertexPointXYZ* pose3 = static_cast<const VertexPointXYZ*>(_vertices[2]);

			Eigen::Vector3d vector1, vector2;

			vector1 = pose2->estimate() - pose1->estimate();
			vector2 = pose2->estimate() - pose3->estimate();

			double dot_product = (vector2.x() * vector1.x()) + (vector2.y() * vector1.y()) + (vector2.z() * vector1.z());
			double norm_vector1 = vector1.norm();
			double norm_vector2 = vector2.norm();

			double angle = acos(dot_product / (norm_vector1*norm_vector2));

			double bound1 = M_PI - _measurement;
			double bound2 = M_PI + _measurement;

			// if (dot_product > 0.0 && ( (angle < bound1) || (angle > bound2)  ) )
			if ( (angle < bound1) || (angle > bound2) ) 
				_error[0] = factor_ / angle ;
			else
				_error[0] = 0.0;
			
			// printf("KinematicsEdge: _error[0]=[%f] angle=[%f] _measurement=[%f] bound=[%f %f] vertex=[%i,%i,%i] dot_product=[%f] norm_vector1=[%f] norm_vector2=[%f]  \n",_error[0],angle,_measurement,bound1,bound2,pose1->id(),pose2->id(),pose3->id(),dot_product,norm_vector1,norm_vector2);




			// double dist12_xy = sqrt(pow((pose2->estimate().x()-pose1->estimate().x()),2)+pow((pose2->estimate().y()-pose1->estimate().y()),2));
			// double dist23_xy = sqrt(pow((pose3->estimate().x()-pose2->estimate().x()),2)+pow((pose3->estimate().y()-pose2->estimate().y()),2));
			// double dist12_xz = sqrt(pow((pose2->estimate().x()-pose1->estimate().x()),2)+pow((pose2->estimate().z()-pose1->estimate().z()),2));
			// double dist23_xz = sqrt(pow((pose3->estimate().x()-pose2->estimate().x()),2)+pow((pose3->estimate().z()-pose2->estimate().z()),2));


			// double angle_x_12 =acos((pose2->estimate().x()-pose1->estimate().x())/ dist12_xy);
		    // double angle_x_23 =acos((pose3->estimate().x()-pose2->estimate().x())/ dist23_xy);

		    // double angle_y_12 =asin((pose2->estimate().y()-pose1->estimate().y())/ dist12_xy);
		    // double angle_y_23 =asin((pose3->estimate().y()-pose2->estimate().y())/ dist23_xy);

   		    // double angle_z_12 =asin((pose2->estimate().z()-pose1->estimate().z())/ dist12_xz);
		    // double angle_z_23 =asin((pose3->estimate().z()-pose2->estimate().z())/ dist23_xz);


			// if ((angle_x_12 - angle_x_23) < _measurement)
			// 	_error[0] = factor_ * (angle_x_12 - angle_x_23) ;
			// else
			// 	_error[0] = 0.0;
			
			// if	((angle_y_12 - angle_y_23) < _measurement)
			// 	_error[1] = factor_ * (angle_y_12 - angle_y_23) ;
			// else
			// 	_error[1] = 0.0;

			// if ((angle_z_12 - angle_z_23) < _measurement)
			// 	_error[2] = factor_ * (angle_z_12 - angle_z_23) ;
			// else
			// 	_error[2] = 0.0;

		}


		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const double& m) {
            _measurement = m;
		}
	};
} 

#endif