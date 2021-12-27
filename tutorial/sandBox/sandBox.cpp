#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <math.h>




SandBox::SandBox()
{


}

void SandBox::Init(const std::string& config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	Xdir = 0;
	Ydir = 0;
	linksNum = 3;
	//nameFileout.open(config);
	//if (!nameFileout.is_open())
	//{
	//	std::cout << "Can't open file "<<config << std::endl;
	//}
	//else
	//{
	//	
	//	while (nameFileout >> item_name)
	//	{
	//		std::cout << "openning " << item_name << std::endl;
	//		load_mesh_from_file(item_name);
	//		
	//		parents.push_back(-1);
	//		data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
	//		data().show_overlay_depth = false;
	//		data().show_overlay = true;

	//		data().MyTranslate(Eigen::Vector3d(pow((- 1),data_list.size()), 0, 0), true);
	//		

	//		data().set_visible(false, 1);
	//		
	//		

	//		//initSimplification();
	//		//data().clear();
	//		//data().set_mesh(data().V, data().F);
	//		//data().set_face_based(true);
	//		//end of 703
	//	}

	//	nameFileout.close();

	//}
	//MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	//
	//data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));	
	load_mesh_from_file("C:/Users/97254/Documents/Animation/EngineForAnimationCourse/tutorial/data/sphere.obj");
	parents.push_back(-1);
	data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
	data().show_overlay_depth = false;
	data().show_overlay = true;
	//data().MyScale(Eigen::Vector3d(0.05, 0.05, 0.05));

	data().MyTranslate(Eigen::Vector3d(5, 0, 0), true);


	data().set_visible(false, 1);


	//initSimplification();
	//data().clear();
	//data().set_mesh(data().V, data().F,false);
	data().set_face_based(true);
	data().initEuler();
	initAxes();
	tipPos.push_back(Eigen::Vector4d(0, 0, 0,1));
	for (int i = 0; i < linksNum; i++) {
		load_mesh_from_file("C:/Users/97254/Documents/Animation/EngineForAnimationCourse/tutorial/data/zcylinder.obj");

		data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
		data().show_overlay_depth = false;
		data().show_overlay = true;

		if (i != 0) {
			data().MyTranslate(Eigen::Vector3d(0, 0, (-1.6)), true);
		}


		data().set_visible(false, 1);


		//initSimplification();
		//data().clear();
		//data().set_mesh(data().V, data().F);
		//data().set_face_based(true);
		initLinkAxes();
		Eigen::Vector3d center = data().GetCenter();
		//printf("base position: (%f,%f,%f) \n", center(0), center(1), center(2));
		data().SetCenter(Eigen::Vector3d(0, 0, 0.8));
		//center = data().GetCenter();
		//printf("base position: (%f,%f,%f) \n", center(0), center(1), center(2));
		if (i != 0) {
			parents.push_back(data_list.size() - 2);
			data().parentId = data_list.size() - 2;
		}
		else {
			parents.push_back(-1);
			data().parentId = -1;
			Eigen::MatrixXd V_box(1, 3);
			V_box <<
				center.x(), center.y(), center.z();

			data().add_points(V_box, Eigen::RowVector3d(1, 0, 0));
		}
		//data().SetCenter(Eigen::Vector3f(data().V., data().V.colwise().minCoeff()[1], data().V.colwise().mean()[2]));

		tipPos.push_back(Eigen::Vector4d(0, 0, (i + 1) * 1.6,1));
	}
	// +(Eigen::Vector3d(0, 0, -1.6))
	//Eigen::Vector4d rCenter(data().GetCenter()[0], data().GetCenter()[1], data().GetCenter()[2], 1);
	//tipPos = CalcParentsTrans(data_list.size() - 1) * data_list[data_list.size() - 1].MakeTransd() * rCenter;
	updateDestPos();
	updateTipPos();

}




SandBox::~SandBox()
{

}

void SandBox::simplify() {
	//printf("entered simplify");
	// If animating then collapse 10% of edges
	if (!data().Q.empty())
	{
		//printf("entered if");
		bool something_collapsed = false;
		// collapse edge
		const int max_iter = std::ceil(0.005 * data().Q.size());
		int e, e1, e2, f1, f2;
		for (int j = 0; j < max_iter; j++)
		{
			if (!igl::collapse_edge_by_algo(data().V, data().F, data().E, data().EMAP,
				data().EF, data().EI, data().Q, data().EQ, data().C, data().vertexCosts))
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


void SandBox::Collide() {
	data().MyTranslate(Eigen::Vector3d(Xdir, Ydir, 0), true);

	int currIndex = selected_data_index;
	for (size_t i = 0; i < data_list.size() && isActive; i++)
	{
		if (i != currIndex) {
			if (CheckCollision(data_list[i], currIndex, i)) {
				isActive = !isActive;
			}
		}
	}
}



void SandBox::CCD() {
	double delta = 0.1;
	Eigen::Vector4d tipToDest = tipPos[linksNum] - destPos;
	Eigen::Vector4d baseToTip;
	Eigen::Vector4d baseToDest;
	Eigen::Vector4d rotationVector;
	double angle,A;

	baseToDest = tipPos[0] - destPos; //will check if possible to reach destination
	if (baseToDest.norm() > linksNum * 1.6) {
		isActive = false;
		printf("can't reach destination position (try adding more links)\n");
		return;
	}

	for (int i = linksNum; i > 0; i--)
	{
		baseToDest = destPos - tipPos[i - 1];
		baseToTip = tipPos[linksNum] - tipPos[i - 1];
		A = baseToTip.normalized().dot(baseToDest.normalized());
		if (A > 1) {
			A = 1;
		}
		if (A < -1) {
			A = -1;
		}
		angle = acos(A);
		rotationVector = baseToTip.cross3(baseToDest);
		data_list[i].MyRotate(((CalcParentsTrans(i)* data_list[i].MakeTransd()).inverse()*rotationVector).head(3), angle / 10);
		updateTipPos();
	}

	tipToDest = tipPos[linksNum] - destPos;
	if (tipToDest.norm() < delta)
		isActive = false;
	fixRotation();
}

void SandBox::Fabrik() {
	double delta = 0.1;
	Eigen::Vector4d tipToDest = tipPos[linksNum] - destPos;
	Eigen::Vector4d baseToTip;
	Eigen::Vector4d baseToDest;
	Eigen::Vector4d rotationVector;
	double R, lambda;
	std::vector<Eigen::Vector4d> ftipPos(tipPos);

	baseToDest = tipPos[0] - destPos; //will check if possible to reach destination
	if (baseToDest.norm() > linksNum * 1.6) {
		isActive = false;
		printf("can't reach destination position (try adding more links)\n");
		return;
	}

	ftipPos[linksNum] = destPos;

	for (int i = linksNum-1; i >= 0; i--)
	{
		R=(ftipPos[i + 1] - ftipPos[i]).norm();
		lambda = 1.6 / R;
		ftipPos[i] = (1 - lambda) * ftipPos[i + 1] + lambda * ftipPos[i];
	}

	ftipPos[0] = tipPos[0];

	for (int i =0; i < linksNum; i++)
	{
		R = (ftipPos[i + 1] - ftipPos[i]).norm();
		lambda = 1.6 / R;
		ftipPos[i+1] = (1 - lambda) * ftipPos[i] + lambda * ftipPos[i + 1];
	}

	updateLinksToTips(ftipPos);

	tipToDest = tipPos[linksNum] - destPos;
	if (tipToDest.norm() < delta)
		isActive = false;
	fixRotation();
}



void SandBox::Animate()
{
	if (isActive)
	{
		//Fabrik();
		CCD();

	}
}





