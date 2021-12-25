// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>
#include <math.h>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/vertex_triangle_adjacency.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
	namespace opengl
	{
		namespace glfw
		{

			void Viewer::Init(const std::string config)
			{

			}

			IGL_INLINE Viewer::Viewer() :
				data_list(1),
				selected_data_index(0),
				next_data_id(1),
				isPicked(false),
				isActive(false)
			{
				data_list.front().id = 0;



				// Temporary variables initialization
			   // down = false;
			  //  hack_never_moved = true;
				scroll_position = 0.0f;

				// Per face
				data().set_face_based(false);


#ifndef IGL_VIEWER_VIEWER_QUIET
				const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
				);
				std::cout << usage << std::endl;
#endif
			}

			IGL_INLINE Viewer::~Viewer()
			{
			}

			IGL_INLINE bool Viewer::load_mesh_from_file(
				const std::string& mesh_file_name_string)
			{

				// Create new data slot and set to selected
				if (!(data().F.rows() == 0 && data().V.rows() == 0))
				{
					append_mesh();
				}
				data().clear();

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}

				std::string extension = mesh_file_name_string.substr(last_dot + 1);
				std::string objName = mesh_file_name_string.substr(last_dot - 6, last_dot);

				if (extension == "off" || extension == "OFF")
				{
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;
					if (!igl::readOFF(mesh_file_name_string, V, F))
						return false;


					data().set_mesh(V, F);

					data().set_face_based(true);

					data().dirty |= MeshGL::DIRTY_UV;

				}
				else if (extension == "obj" || extension == "OBJ")
				{
					Eigen::MatrixXd corner_normals;
					Eigen::MatrixXi fNormIndices;

					Eigen::MatrixXd UV_V;
					Eigen::MatrixXi UV_F;
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;

					if (!(
						igl::readOBJ(
							mesh_file_name_string,
							V, UV_V, corner_normals, F, UV_F, fNormIndices)))
					{
						return false;
					}


					if (objName == "sphere.obj") {
						data().set_mesh(V, F, true);
					}
					else if (objName == "rat_v2.obj") {
						data().set_mesh(V, F, false);

					}
					else {
						data().set_mesh(V, F);
					}
					if (UV_V.rows() > 0)
					{
						data().set_uv(UV_V, UV_F);
					}


				}
				else
				{
					// unrecognized file type
					printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}

				data().compute_normals();
				//data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
				//	Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
				//	Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

				 //Alec: why?
				if (data().V_uv.rows() == 0)
				{
					data().grid_texture();
				}

				//initSimplification();
				//initCosts();
				//initTree();
				//initAxes();
				//initLinkAxes();
				data().point_size = 7.5;
				data().line_width = 1.5;


				//for (unsigned int i = 0; i<plugins.size(); ++i)
				//  if (plugins[i]->post_load())
				//    return true;

				return true;
			}

			IGL_INLINE bool Viewer::save_mesh_to_file(
				const std::string& mesh_file_name_string)
			{
				// first try to load it with a plugin
				//for (unsigned int i = 0; i<plugins.size(); ++i)
				//  if (plugins[i]->save(mesh_file_name_string))
				//    return true;

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					// No file type determined
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}
				std::string extension = mesh_file_name_string.substr(last_dot + 1);
				if (extension == "off" || extension == "OFF")
				{
					return igl::writeOFF(
						mesh_file_name_string, data().V, data().F);
				}
				else if (extension == "obj" || extension == "OBJ")
				{
					Eigen::MatrixXd corner_normals;
					Eigen::MatrixXi fNormIndices;

					Eigen::MatrixXd UV_V;
					Eigen::MatrixXi UV_F;

					return igl::writeOBJ(mesh_file_name_string,
						data().V,
						data().F,
						corner_normals, fNormIndices, UV_V, UV_F);
				}
				else
				{
					// unrecognized file type
					printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}
				return true;
			}

			IGL_INLINE bool Viewer::load_scene()
			{
				std::string fname = igl::file_dialog_open();
				if (fname.length() == 0)
					return false;
				return load_scene(fname);
			}

			IGL_INLINE bool Viewer::load_scene(std::string fname)
			{
				// igl::deserialize(core(),"Core",fname.c_str());
				igl::deserialize(data(), "Data", fname.c_str());
				return true;
			}

			IGL_INLINE bool Viewer::save_scene()
			{
				std::string fname = igl::file_dialog_save();
				if (fname.length() == 0)
					return false;
				return save_scene(fname);
			}

			IGL_INLINE bool Viewer::save_scene(std::string fname)
			{
				//igl::serialize(core(),"Core",fname.c_str(),true);
				igl::serialize(data(), "Data", fname.c_str());

				return true;
			}

			IGL_INLINE void Viewer::open_dialog_load_mesh()
			{
				int savedIndx = selected_data_index;
				load_mesh_from_file("C:/Users/97254/Documents/Animation/EngineForAnimationCourse/tutorial/data/zcylinder.obj");
				data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
				data().show_overlay_depth = false;
				data().show_overlay = true;
				data().MyTranslate(Eigen::Vector3d(0, 0, (-1.6)), true);
				data().set_visible(false, 1);
				data().set_mesh(data().V, data().F);
				data().set_visible(true, 2);
				data().show_faces = 3;
				data().set_face_based(true);

				parents.push_back(data_list.size() - 2);
				data().parentId = data_list.size() - 2;

				data().SetCenter(Eigen::Vector3d(0, 0, 0.8));
				data().show_texture = 3;
				tipPos.push_back(Eigen::Vector4d(0, 0, 0, 1));
				updateTipPos();
				selected_data_index = savedIndx;
				initLinkAxes();
				linksNum++;
			}

			IGL_INLINE void Viewer::open_dialog_save_mesh()
			{
				std::string fname = igl::file_dialog_save();

				if (fname.length() == 0)
					return;

				this->save_mesh_to_file(fname.c_str());
			}

			IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
			{
				assert(!data_list.empty() && "data_list should never be empty");
				int index;
				if (mesh_id == -1)
					index = selected_data_index;
				else
					index = mesh_index(mesh_id);

				assert((index >= 0 && index < data_list.size()) &&
					"selected_data_index or mesh_id should be in bounds");
				return data_list[index];
			}

			IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
			{
				assert(!data_list.empty() && "data_list should never be empty");
				int index;
				if (mesh_id == -1)
					index = selected_data_index;
				else
					index = mesh_index(mesh_id);

				assert((index >= 0 && index < data_list.size()) &&
					"selected_data_index or mesh_id should be in bounds");
				return data_list[index];
			}

			IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
			{
				assert(data_list.size() >= 1);

				data_list.emplace_back();
				selected_data_index = data_list.size() - 1;
				data_list.back().id = next_data_id++;
				//if (visible)
				//    for (int i = 0; i < core_list.size(); i++)
				//        data_list.back().set_visible(true, core_list[i].id);
				//else
				//    data_list.back().is_visible = 0;
				return data_list.back().id;
			}

			IGL_INLINE bool Viewer::erase_mesh(const size_t index)
			{
				assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
				assert(data_list.size() >= 1);
				if (data_list.size() == 1)
				{
					// Cannot remove last mesh
					return false;
				}
				data_list[index].meshgl.free();
				data_list.erase(data_list.begin() + index);
				if (selected_data_index >= index && selected_data_index > 0)
				{
					selected_data_index--;
				}

				return true;
			}

			IGL_INLINE size_t Viewer::mesh_index(const int id) const {
				for (size_t i = 0; i < data_list.size(); ++i)
				{
					if (data_list[i].id == id)
						return i;
				}
				return 0;
			}

			Eigen::Matrix4d Viewer::CalcParentsTrans(int indx)
			{
				Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

				for (int i = indx; parents[i] >= 0; i = parents[i])
				{
					//std::cout << "parent matrix:\n" << data_list[parents[i]].MakeTransd() << std::endl;
					prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
				}

				return prevTrans;
			}

			//Assignment 1

			void Viewer::initSimplification()
			{
				//from 703
				igl::edge_flaps(data().F, data().E, data().EMAP, data().EF, data().EI);
				data().C.resize(data().E.rows(), data().V.cols());
				Eigen::VectorXd costs(data().E.rows());
				// https://stackoverflow.com/questions/2852140/priority-queue-clear-method
				// Q.clear();
				data().Q = {};
				data().EQ = Eigen::VectorXi::Zero(data().E.rows());
				{
					Eigen::VectorXd costs(data().E.rows());
					igl::parallel_for(data().E.rows(), [&](const int e)
					{
						double cost = e;
						Eigen::RowVectorXd p(1, 3);
						igl::shortest_edge_and_midpoint(e, data().V, data().F, data().E, data().EMAP, data().EF, data().EI, cost, p);
						data().C.row(e) = p;
						costs(e) = cost;
					}, 10000);
					for (int e = 0; e < data().E.rows(); e++)
					{
						data().Q.emplace(costs(e), e, 0);
					}
				}

				data().num_collapsed = 0;
			}

			void Viewer::initCosts()
			{
				//from 703
				igl::edge_flaps(data().F, data().E, data().EMAP, data().EF, data().EI);
				data().C.resize(data().E.rows(), data().V.cols());
				Eigen::VectorXd costs(data().E.rows());
				// https://stackoverflow.com/questions/2852140/priority-queue-clear-method
				// Q.clear();
				data().Q = {};
				data().EQ = Eigen::VectorXi::Zero(data().E.rows());
				{
					//Eigen::VectorXd costs(data().V.rows());
					std::vector<std::vector<int> > VF, VFI;
					igl::vertex_triangle_adjacency(data().V, data().F, VF, VFI);
					for (int v = 0; v < data().V.rows(); v++)
					{
						computeVertexMat(v, VF);
					}
					for (int e = 0; e < data().E.rows(); e++)
					{
						int v1 = data().E(e, 0);
						int v2 = data().E(e, 1);
						//computes cost for moving vertex to v1 or v2
						double costV1 = computeCost(v1, v2, e);
						//double costV2 = computeCost(v2,v1);
						//double costV3 = computeCost(v2, v1);
						//if (costV1 < costV2) {
						//	data().C.row(e) = data().V.row(v1);
						//	data().Q.emplace(costV1, e, 0);
						//}
						//else {
						//	data().C.row(e) = data().V.row(v2);
						//	data().Q.emplace(costV2, e, 0);
						//}


					}
				}

				data().num_collapsed = 0;
			}

			//computes QuadricError matrix for each vertex
			void Viewer::computeVertexMat(int vertex, std::vector<std::vector<int> > VF) {
				Eigen::Matrix4d* cost = new Eigen::Matrix4d();
				Eigen::Matrix4d temp = Eigen::Matrix4d::Zero();

				for each (int f in VF[vertex]) {
					Eigen::RowVector3d normal = data().F_normals.row(f).normalized();
					double d = -(data().V.row(vertex) * normal.transpose())(0, 0);
					temp(0, 0) = normal.x() * normal.x();
					temp(0, 1) = normal.x() * normal.y();
					temp(0, 2) = normal.x() * normal.z();
					temp(0, 3) = normal.x() * d;
					temp(1, 0) = normal.y() * normal.x();
					temp(1, 1) = normal.y() * normal.y();
					temp(1, 2) = normal.y() * normal.z();
					temp(1, 3) = normal.y() * d;
					temp(2, 0) = normal.z() * normal.x();
					temp(2, 1) = normal.z() * normal.y();
					temp(2, 2) = normal.z() * normal.z();
					temp(2, 3) = normal.z() * d;
					temp(3, 0) = d * normal.x();
					temp(3, 1) = d * normal.y();
					temp(3, 2) = d * normal.z();
					temp(3, 3) = d * d;
					*cost = *cost + temp;
				}
				//data().vertexCosts[vertex] = 5;
				//Eigen::RowVector4d v = data().V.row(vertex);
				auto itPos = data().vertexCosts.begin() + vertex;
				data().vertexCosts.insert(itPos, cost);
			}

			//gets 2 vertexes and computes cost of moving to the first vertex
			double Viewer::computeCost(int vFirst, int vSecond, int e) {

				Eigen::MatrixXd Q1 = *(data().vertexCosts[vFirst]);
				Eigen::MatrixXd Q2 = *(data().vertexCosts[vSecond]);
				Eigen::MatrixXd matQ = Q1 + Q2;
				Eigen::VectorXd newV;

				if (matQ.determinant() != 0) {
					matQ.row(matQ.rows() - 1)(0) = 0;
					matQ.row(matQ.rows() - 1)(1) = 0;
					matQ.row(matQ.rows() - 1)(2) = 0;
					matQ.row(matQ.rows() - 1)(3) = 1;
					Eigen::MatrixXd Qinv = matQ.inverse();
					Eigen::VectorXd v(4);

					v(0) = 0.0;
					v(1) = 0.0;
					v(2) = 0.0;
					v(3) = 1.0;
					newV = Qinv * v;
				}
				else {
					newV = 0.5 * (data().V.row(vFirst) + data().V.row(vSecond));
				}
				newV.conservativeResize(4);
				Eigen::RowVectorXd temp = newV.transpose() * matQ;
				data().Q.emplace((temp * newV)(0, 0), e, 0);
				data().C(e, 0) = newV(0);
				data().C(e, 1) = newV(1);
				data().C(e, 2) = newV(2);



				return 0;

			}

			//Assignment 2

			void Viewer::initTree()
			{
				data().tree.init(data().V, data().F);
				draw_box(data().tree.m_box, data(), Eigen::RowVector3d(1, 0, 0));
			}

			bool Viewer::CheckCollision(ViewerData& obj, int data1_Index, int data2_Index)
			{
				Eigen::MatrixXd A = data().GetRotation(), B = obj.GetRotation();
				return CheckCollision(data().tree, obj.tree, A, B, data().MakeTransd(), obj.MakeTransd(), data1_Index, data2_Index, Eigen::Vector4d(0, 0, 0, 0));

			}

			bool Viewer::CheckCollision(igl::AABB<Eigen::MatrixXd, 3>& tree1, igl::AABB<Eigen::MatrixXd, 3>& tree2, Eigen::MatrixXd& rot1, Eigen::MatrixXd& rot2, Eigen::Matrix4d& transd1, Eigen::Matrix4d& transd2, int data1_Index, int data2_Index, Eigen::Vector4d& old_D) {
				Eigen::AlignedBox3d box1 = tree1.m_box, box2 = tree2.m_box;
				Eigen::MatrixXd C = rot1.transpose() * rot2;
				Eigen::RowVectorXd C0asd = box1.center(), C1asd = box2.center();
				Eigen::Vector4d C0(C0asd(0), C0asd(1), C0asd(2), 1);
				Eigen::Vector4d C1(C1asd(0), C1asd(1), C1asd(2), 1);
				Eigen::Vector4d D = (transd2 * C1) - (transd1 * C0);
				if (old_D != Eigen::Vector4d(0, 0, 0, 0) && old_D.innerSize() > D.innerSize()) {
					return false;
				}
				double a[] = { box1.sizes().x() / 2,box1.sizes().y() / 2, box1.sizes().z() / 2 };
				double b[] = { box2.sizes().x() / 2, box2.sizes().y() / 2, box2.sizes().z() / 2 };
				double R0 = 0, R1 = 0, R = 0;
				Eigen::MatrixXd& valMarix = Eigen::MatrixXd(15, 3);
				init_val_mat(valMarix, a, b, rot1, rot2, C, D);
				for (int i = 0; i < 15; i++) {
					R0 = valMarix(i, 0);
					R1 = valMarix(i, 1);
					R = valMarix(i, 2);
					if (R > R0 + R1) {
						return false;
					}
				}

				if (tree1.is_leaf() || tree2.is_leaf()) {
					printf("tree1 is leaf: %d, tree2 is leaf: %d\n", tree1.is_leaf(), tree2.is_leaf());
					if (!tree1.is_leaf()) {
						return (CheckCollision(*(tree1.m_right), tree2, rot1, rot2, transd1, transd2, data1_Index, data2_Index, D)) || (CheckCollision(*(tree1.m_left), tree2, rot1, rot2, transd1, transd2, data1_Index, data2_Index, D));
					}
					if (!tree2.is_leaf()) {
						return (CheckCollision(tree1, *(tree2.m_left), rot1, rot2, transd1, transd2, data1_Index, data2_Index, D)) || (CheckCollision(tree1, *(tree2.m_right), rot1, rot2, transd1, transd2, data1_Index, data2_Index, D));
					}
					Eigen::RowVector3d collisionColor = Eigen::RowVector3d::Random().normalized();
					draw_box(box1, data_list[data1_Index], collisionColor);
					draw_box(box2, data_list[data2_Index], collisionColor);
					return true;
				}
				return (CheckCollision(*(tree1.m_right), *(tree2.m_right), rot1, rot2, transd1, transd2, data1_Index, data2_Index, D)) || (CheckCollision(*(tree1.m_left), *(tree2.m_left), rot1, rot2, transd1, transd2, data1_Index, data2_Index, D))
					|| (CheckCollision(*(tree1.m_right), *(tree2.m_left), rot1, rot2, transd1, transd2, data1_Index, data2_Index, D)) || (CheckCollision(*(tree1.m_left), *(tree2.m_right), rot1, rot2, transd1, transd2, data1_Index, data2_Index, D));
			}

			void Viewer::draw_box(Eigen::AlignedBox3d& box, ViewerData& obj, Eigen::RowVector3d colors) {
				Eigen::MatrixXd V_box(8, 3);
				V_box <<
					box.corner(box.BottomLeftFloor)[0],
					box.corner(box.BottomLeftFloor)[1],
					box.corner(box.BottomLeftFloor)[2],
					box.corner(box.BottomRightFloor)[0],
					box.corner(box.BottomRightFloor)[1],
					box.corner(box.BottomRightFloor)[2],
					box.corner(box.TopLeftFloor)[0],
					box.corner(box.TopLeftFloor)[1],
					box.corner(box.TopLeftFloor)[2],
					box.corner(box.TopRightFloor)[0],
					box.corner(box.TopRightFloor)[1],
					box.corner(box.TopRightFloor)[2],
					box.corner(box.BottomLeftCeil)[0],
					box.corner(box.BottomLeftCeil)[1],
					box.corner(box.BottomLeftCeil)[2],
					box.corner(box.BottomRightCeil)[0],
					box.corner(box.BottomRightCeil)[1],
					box.corner(box.BottomRightCeil)[2],
					box.corner(box.TopLeftCeil)[0],
					box.corner(box.TopLeftCeil)[1],
					box.corner(box.TopLeftCeil)[2],
					box.corner(box.TopRightCeil)[0],
					box.corner(box.TopRightCeil)[1],
					box.corner(box.TopRightCeil)[2];
				obj.add_points(V_box, colors);

				Eigen::MatrixXi E_box(12, 2);
				E_box <<
					0, 1,
					1, 3,
					2, 3,
					2, 0,
					4, 5,
					5, 7,
					6, 7,
					6, 4,
					0, 4,
					1, 5,
					2, 6,
					7, 3;

				for (unsigned i = 0; i < E_box.rows(); ++i)
					obj.add_edges
					(
						V_box.row(E_box(i, 0)),
						V_box.row(E_box(i, 1)),
						colors
					);
			}

			void Viewer::init_val_mat(Eigen::MatrixXd& mat, double a[], double b[], Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::Vector4d& D) {
				mat(0, 0) = a[0];                                                      mat(0, 1) = calculate(b[0], b[1], b[2], C(0, 0), C(0, 1), C(0, 2));     mat(0, 2) = calculate(D, A.col(0));
				mat(1, 0) = a[1];                                                      mat(1, 1) = calculate(b[0], b[1], b[2], C(1, 0), C(1, 1), C(1, 2));     mat(1, 2) = calculate(D, A.col(1));
				mat(2, 0) = a[2];                                                      mat(2, 1) = calculate(b[0], b[1], b[2], C(2, 0), C(2, 1), C(2, 2));     mat(2, 2) = calculate(D, A.col(2));
				mat(3, 0) = calculate(a[0], a[1], a[2], C(0, 0), C(1, 0), C(2, 0));    mat(3, 1) = b[0];                                                       mat(3, 2) = calculate(D, B.col(0));
				mat(4, 0) = calculate(a[0], a[1], a[2], C(0, 1), C(1, 1), C(2, 1));    mat(4, 1) = b[1];                                                       mat(4, 2) = calculate(D, B.col(1));
				mat(5, 0) = calculate(a[0], a[1], a[2], C(0, 2), C(1, 2), C(2, 2));    mat(5, 1) = b[2];                                                       mat(5, 2) = calculate(D, B.col(2));
				mat(6, 0) = calculate(a[1], a[2], C(2, 0), C(1, 0));                   mat(6, 1) = calculate(b[1], b[2], C(0, 2), C(0, 1));                    mat(6, 2) = calculate(C(1, 0), C(2, 0), D, A.col(2), A.col(1));
				mat(7, 0) = calculate(a[1], a[2], C(2, 1), C(1, 1));                   mat(7, 1) = calculate(b[0], b[2], C(0, 2), C(0, 0));                    mat(7, 2) = calculate(C(1, 1), C(2, 1), D, A.col(2), A.col(1));
				mat(8, 0) = calculate(a[1], a[2], C(2, 2), C(1, 2));                   mat(8, 1) = calculate(b[0], b[1], C(0, 1), C(0, 0));                    mat(8, 2) = calculate(C(1, 2), C(2, 2), D, A.col(2), A.col(1));
				mat(9, 0) = calculate(a[0], a[2], C(2, 0), C(0, 0));                   mat(9, 1) = calculate(b[1], b[2], C(1, 2), C(1, 1));                    mat(9, 2) = calculate(C(2, 0), C(0, 0), D, A.col(0), A.col(2));
				mat(10, 0) = calculate(a[0], a[2], C(2, 1), C(0, 1));                  mat(10, 1) = calculate(b[0], b[2], C(1, 2), C(1, 0));                   mat(10, 2) = calculate(C(2, 1), C(0, 1), D, A.col(0), A.col(2));
				mat(11, 0) = calculate(a[0], a[2], C(2, 2), C(0, 2));                  mat(11, 1) = calculate(b[0], b[1], C(1, 1), C(1, 0));                   mat(11, 2) = calculate(C(2, 2), C(0, 2), D, A.col(0), A.col(2));
				mat(12, 0) = calculate(a[0], a[1], C(1, 0), C(0, 0));                  mat(12, 1) = calculate(b[1], b[2], C(2, 2), C(2, 1));                   mat(12, 2) = calculate(C(0, 0), C(1, 0), D, A.col(1), A.col(0));
				mat(13, 0) = calculate(a[0], a[1], C(1, 1), C(0, 1));                  mat(13, 1) = calculate(b[0], b[2], C(2, 2), C(2, 0));                   mat(13, 2) = calculate(C(0, 1), C(1, 1), D, A.col(1), A.col(0));
				mat(14, 0) = calculate(a[0], a[1], C(1, 2), C(0, 2));                   mat(14, 1) = calculate(b[0], b[1], C(2, 1), C(2, 0));                  mat(14, 2) = calculate(C(0, 2), C(1, 2), D, A.col(1), A.col(0));
			}

			double Viewer::calculate(double x1, double x2, double x3, double y1, double y2, double y3) {
				return x1 * abs(y1) + x2 * abs(y2) + x3 * abs(y3);
			}

			double Viewer::calculate(double x1, double x2, double y1, double y2) {
				return x1 * abs(y1) + x2 * abs(y2);
			}

			double Viewer::calculate(double x1, double x2, Eigen::Vector4d D, Eigen::VectorXd Y1, Eigen::VectorXd Y2) {
				Y1.conservativeResize(4);
				Y2.conservativeResize(4);
				return abs((x1 * Y1.transpose() * D - x2 * Y2.transpose() * D).value());
			}

			double Viewer::calculate(Eigen::Vector4d D, Eigen::VectorXd Y1) {
				//maybe determinanta?
				Y1.conservativeResize(4);
				return abs(Y1.transpose() * D);
			}

			//Assignment 3

			void Viewer::initAxes() {

				Eigen::MatrixXd V_box(6, 3);
				V_box <<
					1.6, 0, 0,
					-1.6, 0, 0,
					0, 1.6, 0,
					0, -1.6, 0,
					0, 0, 1.6,
					0, 0, -1.6;

				data().add_points(V_box, Eigen::RowVector3d(1, 0, 0));


				Eigen::MatrixXi E_box(3, 2);
				E_box <<
					0, 1,
					2, 3,
					4, 5;

				for (unsigned i = 0; i < E_box.rows(); ++i)
					data().add_edges
					(
						V_box.row(E_box(i, 0)),
						V_box.row(E_box(i, 1)),
						Eigen::RowVector3d(1, 0, 0)
					);
			}

			void Viewer::initLinkAxes() {

				Eigen::MatrixXd V_box(6, 3);
				V_box <<
					0.8, 0, -0.8,
					-0.8, 0, -0.8,
					0, 0.8, -0.8,
					0, -0.8, -0.8,
					0, 0, -0.8,
					0, 0, -2.4;

				data().add_points(V_box, Eigen::RowVector3d(0, 0, 1));


				Eigen::MatrixXi E_box(3, 2);
				E_box <<
					0, 1,
					2, 3,
					4, 5;

				for (unsigned i = 0; i < E_box.rows(); ++i)
					data().add_edges
					(
						V_box.row(E_box(i, 0)),
						V_box.row(E_box(i, 1)),
						Eigen::RowVector3d(0, 0, 1)
					);
			}

			void Viewer::printRotation() {
				Eigen::Matrix3d rotMat;
				if (isPicked || selected_data_index == 0) {
					printf("scene rotation matrix:\n");
					rotMat = GetRotation();
				}
				else {
					printf("link %d rotation matrix:\n", selected_data_index);
					rotMat = data().GetRotation();
				}
				for (int i = 0; i < 3; i++) {
					printf("(%f\t%f\t%f)\n", rotMat(i, 0), rotMat(i, 1), rotMat(i, 2));
				}
			}

			void Viewer::printTip() {
				for (size_t i = 0; i < tipPos.size(); i++)
				{
					if (i != 0) {
						printf("tip position %d: (%f,%f,%f) \n", i, tipPos[i](0), tipPos[i](1), tipPos[i](2));
					}
					else {
						printf("base position: (%f,%f,%f) \n", tipPos[i](0), tipPos[i](1), tipPos[i](2));
					}
				}
			}

			//if stuck maybe improve efficiency
			void Viewer::updateTipPos() {
				Eigen::Vector3d c =data_list[1].GetCenter();
				Eigen::Vector4d center(c.x(), c.y(), c.z(), 1);
				Eigen::Matrix3d rot= Eigen::Matrix3d().Identity();

				//Eigen::Vector3d O = (data_list[1].MakeTransd() *  center).head(3);

				//Eigen::Vector4d O = (data_list[1].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1));
				Eigen::Vector4d O = data_list[1].MakeTransd()* center - Eigen::Vector4d(0, 0, 0.8, 0);
				tipPos[0] = O;
				for (int i = 1; i < data_list.size(); i++) {
					rot = rot * data_list[i].GetRotation() ;
					//for (int j = i - 1; j > 0; j--)
					//	rot = data_list[j].GetRotation() * rot;
					Eigen::Vector3d tmp = (rot * Eigen::Vector3d(0, 0, -1.6));
					O = O + Eigen::Vector4d(tmp(0), tmp(1), tmp(2), 0);
					tipPos[i] = O;
				}
				//tipPos[0] = data_list[1].MakeTransd()* data_list[1].MakeTransd() * tipPos[1];
				//for (int i = 1; i < data_list.size(); i++) {
				//	tipPos[i] = CalcParentsTrans(i)* data_list[i].MakeTransd() * tipPos[i-1];
				//}
			}

			void Viewer::updateDestPos() {
				destPos = (data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1));
			}

			void Viewer::printDest() {
				printf("destination position: (%f,%f,%f) \n", destPos(0), destPos(1), destPos(2));
			}

			void Viewer::updateLinksToTips(std::vector<Eigen::Vector4d> newPos) {
				double angle,A;
				Eigen::Vector4d currVec;
				Eigen::Vector4d newVec;
				Eigen::Vector4d rotationVector;
				for (int i = 0; i < linksNum; i++)
				{
					currVec = tipPos[i + 1] - tipPos[i];
					newVec = newPos[i + 1] - newPos[i];
					A = currVec.normalized().dot(newVec.normalized());
					if (A > 1) {
						A = 1;
					}
					if (A < -1) {
						A = -1;
					}
					angle = acos(A);
					rotationVector = currVec.cross3(newVec);
					data_list[i+1].MyRotate(((CalcParentsTrans(i+1) * data_list[i+1].MakeTransd()).inverse() * rotationVector).head(3), angle/10);
					updateTipPos();
				}

			}

			void Viewer::rotateAroundY(bool clockwise) {
				double dir = 0;
				if (clockwise) {
					dir = 0.1;
				}
				else {
					dir = -0.1;
				}
				//Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
				//for (int i = 1; i < selected_data_index; i++)
				//	rot = data_list[i].GetRotation() * rot;
				data().MyRotate(data().GetRotation().inverse() * Eigen::Vector3d(0, 0, 1), dir);
				//Eigen::Vector3d angles=data().GetRotation().eulerAngles(2, 0, 2);
				//angles(1) = angles(1) + dir;
				//data().MyRotate(computeEulerMatrix(angles));
				updateTipPos();
			}

			void Viewer::rotateAroundX(bool clockwise) {
				double dir = 0;
				if (clockwise) {
					dir = 0.1;
				}
				else {
					dir = -0.1;
				}
				Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
				for (int i = 1; i < selected_data_index; i++)
					rot = data_list[i].GetRotation() * rot;
				data().MyRotate(data().GetRotation().inverse() * Eigen::Vector3d(1, 0, 0), dir);
			}

			Eigen::Matrix3d Viewer::computeEulerMatrix(Eigen::Vector3d angles) {
				double phi, theta, ksi;
				phi = angles.x();
				theta = angles.y();
				ksi = angles.z();
				//Eigen::Matrix3d A1, A2, A3;
				Eigen::Matrix3d A1(3, 3);
				A1 <<
					1, 0, 0,
					0, cos(phi), -sin(phi),
					0, sin(phi), cos(phi);
				Eigen::Matrix3d A2(3, 3);
				A2 <<
					cos(theta), -sin(theta), 0,
					sin(theta), cos(theta), 0,
					0, 0, 1;
				Eigen::Matrix3d A3(3, 3);
				A3 <<
					1, 0, 0,
					0, cos(ksi), -sin(ksi),
					0, sin(ksi), cos(ksi);

				Eigen::Matrix3d A = A1 * A2 * A3;
				return A;
			}




		} // end namespace
	} // end namespace
}
