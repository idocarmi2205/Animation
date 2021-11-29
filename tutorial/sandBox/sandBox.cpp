#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>





SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
			

			//initSimplification();
			//data().clear();
			//data().set_mesh(data().V, data().F);
			//data().set_face_based(true);
			//end of 703
		}

		nameFileout.close();

	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

//void SandBox::InitSimplification()
//{
//	//from 703
//	igl::edge_flaps(data().F, data().E, data().EMAP, data().EF, data().EI);
//	data().C.resize(data().E.rows(), data().V.cols());
//	Eigen::VectorXd costs(data().E.rows());
//	// https://stackoverflow.com/questions/2852140/priority-queue-clear-method
//	// Q.clear();
//	data().Q = {};
//	data().EQ = Eigen::VectorXi::Zero(data().E.rows());
//	{
//		Eigen::VectorXd costs(data().E.rows());
//		igl::parallel_for(data().E.rows(), [&](const int e)
//		{
//			double cost = e;
//			Eigen::RowVectorXd p(1, 3);
//			igl::shortest_edge_and_midpoint(e, data().V, data().F, data().E, data().EMAP, data().EF, data().EI, cost, p);
//			data().C.row(e) = p;
//			costs(e) = cost;
//		}, 10000);
//		for (int e = 0; e < data().E.rows(); e++)
//		{
//			data().Q.emplace(costs(e), e, 0);
//		}
//	}
//
//	data().num_collapsed = 0;
//}

SandBox::~SandBox()
{
	
}

void SandBox::simplify() {
	//printf("entered simplify");
	// If animating then collapse 10% of edges
	if ( !data().Q.empty())
	{
		//printf("entered if");
		bool something_collapsed = false;
		// collapse edge
		const int max_iter = std::ceil(0.005 * data().Q.size());
		int e, e1, e2, f1, f2;
		for (int j = 0; j < max_iter; j++)
		{
			if (!igl::collapse_edge_by_algo(data().V, data().F, data().E, data().EMAP,
				data().EF, data().EI, data().Q, data().EQ, data().C,data().vertexCosts))
			{
				break;
			}
			something_collapsed = true;
			data().num_collapsed++;
		}

		if (something_collapsed)
		{
			//data().clear();
			data().set_mesh(data().V, data().F);
			data().set_face_based(true);
			data().dirty = 157;
		}
	}
}

void SandBox::Animate()
{
	if (isActive)
	{
		
		
		
	}
}





