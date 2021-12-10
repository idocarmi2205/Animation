#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <math.h>




SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	Xdir = 0;
	Ydir = 0;
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
			data().show_overlay = true;

			data().MyTranslate(Eigen::Vector3d(pow((- 1),data_list.size()), 0, 0), true);
			

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
		data().MyTranslate(Eigen::Vector3d(Xdir, Ydir, 0), true);

		int currIndex = selected_data_index;
		for (size_t i = 0; i < data_list.size() && isActive; i++)
		{
			if (i != currIndex) {
				if (CheckCollision(data_list[i],currIndex, i)) {
					isActive = !isActive;
				}
			}
		}

		
	}
}





