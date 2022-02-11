#pragma once

#ifndef G2O_EQUI_DISTANCE_EDGE_H
#define G2O_EQUI_DISTANCE_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"


namespace g2o 
{

	class G2OEquiDistanceEdge : public BaseMultiEdge<3, double>
	{
	public:
		G2OEquiDistanceEdge();

		double factor_ = 10.0;

		void computeError()
		{
			const VertexPointXYZ* pose1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
			const VertexPointXYZ* pose2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
			const VertexPointXYZ* pose3 = static_cast<const VertexPointXYZ*>(_vertices[2]);
			const VertexPointXYZ* pose4 = static_cast<const VertexPointXYZ*>(_vertices[3]);

			double _d1 = (pose2->estimate()-pose1->estimate()).norm();
			double _d2 = (pose3->estimate()-pose2->estimate()).norm();
			double _d3 = (pose4->estimate()-pose3->estimate()).norm();

			double average = (_d1 + _d2 + _d3) / 3.0;

			_error[0] = factor_ *(average - _d1);
			_error[1] = factor_ *(average - _d2);
			_error[2] = factor_ *(average - _d3);
			
			// printf("EquiDistance: error=[%f] [%i-%i]_d1 = [%f]  [%i-%i]_d2 = [%f] \n",_error[0],pose2->id(),pose1->id(),_d1,pose3->id(),pose2->id(),_d2);
		}


		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const double& m) {
            _measurement = m;
		}
	};
} 

#endif