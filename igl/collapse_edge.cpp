// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2015 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "collapse_edge.h"
#include "circulation.h"
#include "edge_collapse_is_valid.h"
#include <vector>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/circulation.h>
#include <igl\opengl\glfw\Viewer.h>

IGL_INLINE bool igl::collapse_edge(
	const int e,
	const Eigen::RowVectorXd& p,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	int& a_e1,
	int& a_e2,
	int& a_f1,
	int& a_f2)
{
	// Assign this to 0 rather than, say, -1 so that deleted elements will get
	// draw as degenerate elements at vertex 0 (which should always exist and
	// never get collapsed to anything else since it is the smallest index)
	using namespace Eigen;
	using namespace std;
	const int eflip = E(e, 0) > E(e, 1);
	// source and destination
	const int s = eflip ? E(e, 1) : E(e, 0);
	const int d = eflip ? E(e, 0) : E(e, 1);

	if (!edge_collapse_is_valid(e, F, E, EMAP, EF, EI))
	{
		return false;
	}

	// Important to grab neighbors of d before monkeying with edges
	const std::vector<int> nV2Fd = circulation(e, !eflip, EMAP, EF, EI);

	// The following implementation strongly relies on s<d
	assert(s < d && "s should be less than d");
	// move source and destination to midpoint
	V.row(s) = p;
	V.row(d) = p;

	// Helper function to replace edge and associate information with NULL
	const auto& kill_edge = [&E, &EI, &EF](const int e)
	{
		E(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		E(e, 1) = IGL_COLLAPSE_EDGE_NULL;
		EF(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		EF(e, 1) = IGL_COLLAPSE_EDGE_NULL;
		EI(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		EI(e, 1) = IGL_COLLAPSE_EDGE_NULL;
	};

	// update edge info
	// for each flap
	const int m = F.rows();
	for (int side = 0; side < 2; side++)
	{
		const int f = EF(e, side);
		const int v = EI(e, side);
		const int sign = (eflip == 0 ? 1 : -1) * (1 - 2 * side);
		// next edge emanating from d
		const int e1 = EMAP(f + m * ((v + sign * 1 + 3) % 3));
		// prev edge pointing to s
		const int e2 = EMAP(f + m * ((v + sign * 2 + 3) % 3));
		assert(E(e1, 0) == d || E(e1, 1) == d);
		assert(E(e2, 0) == s || E(e2, 1) == s);
		// face adjacent to f on e1, also incident on d
		const bool flip1 = EF(e1, 1) == f;
		const int f1 = flip1 ? EF(e1, 0) : EF(e1, 1);
		assert(f1 != f);
		assert(F(f1, 0) == d || F(f1, 1) == d || F(f1, 2) == d);
		// across from which vertex of f1 does e1 appear?
		const int v1 = flip1 ? EI(e1, 0) : EI(e1, 1);
		// Kill e1
		kill_edge(e1);
		// Kill f
		F(f, 0) = IGL_COLLAPSE_EDGE_NULL;
		F(f, 1) = IGL_COLLAPSE_EDGE_NULL;
		F(f, 2) = IGL_COLLAPSE_EDGE_NULL;
		// map f1's edge on e1 to e2
		assert(EMAP(f1 + m * v1) == e1);
		EMAP(f1 + m * v1) = e2;
		// side opposite f2, the face adjacent to f on e2, also incident on s
		const int opp2 = (EF(e2, 0) == f ? 0 : 1);
		assert(EF(e2, opp2) == f);
		EF(e2, opp2) = f1;
		EI(e2, opp2) = v1;
		// remap e2 from d to s
		E(e2, 0) = E(e2, 0) == d ? s : E(e2, 0);
		E(e2, 1) = E(e2, 1) == d ? s : E(e2, 1);
		if (side == 0)
		{
			a_e1 = e1;
			a_f1 = f;
		}
		else
		{
			a_e2 = e1;
			a_f2 = f;
		}
	}

	// finally, reindex faces and edges incident on d. Do this last so asserts
	// make sense.
	//
	// Could actually skip first and last, since those are always the two
	// collpased faces.
	for (auto f : nV2Fd)
	{
		for (int v = 0; v < 3; v++)
		{
			if (F(f, v) == d)
			{
				const int flip1 = (EF(EMAP(f + m * ((v + 1) % 3)), 0) == f) ? 1 : 0;
				const int flip2 = (EF(EMAP(f + m * ((v + 2) % 3)), 0) == f) ? 0 : 1;
				assert(
					E(EMAP(f + m * ((v + 1) % 3)), flip1) == d ||
					E(EMAP(f + m * ((v + 1) % 3)), flip1) == s);
				E(EMAP(f + m * ((v + 1) % 3)), flip1) = s;
				assert(
					E(EMAP(f + m * ((v + 2) % 3)), flip2) == d ||
					E(EMAP(f + m * ((v + 2) % 3)), flip2) == s);
				E(EMAP(f + m * ((v + 2) % 3)), flip2) = s;
				F(f, v) = s;
				break;
			}
		}
	}
	// Finally, "remove" this edge and its information
	kill_edge(e);

	return true;
}

IGL_INLINE bool igl::collapse_edge(
	const int e,
	const Eigen::RowVectorXd& p,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI)
{
	int e1, e2, f1, f2;
	return collapse_edge(e, p, V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
}

IGL_INLINE bool igl::collapse_edge(
	const std::function<void(
		const int,
		const Eigen::MatrixXd&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		const Eigen::VectorXi&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		double&,
		Eigen::RowVectorXd&)>& cost_and_placement,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	std::set<std::pair<double, int> >& Q,
	std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
	Eigen::MatrixXd& C)
{
	int e, e1, e2, f1, f2;
	const auto always_try = [](
		const Eigen::MatrixXd&,/*V*/
		const Eigen::MatrixXi&,/*F*/
		const Eigen::MatrixXi&,/*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,/*EF*/
		const Eigen::MatrixXi&,/*EI*/
		const std::set<std::pair<double, int> >&,/*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
		const Eigen::MatrixXd&,/*C*/
		const int                                                        /*e*/
		) -> bool { return true; };
	const auto never_care = [](
		const Eigen::MatrixXd&,   /*V*/
		const Eigen::MatrixXi&,   /*F*/
		const Eigen::MatrixXi&,   /*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,  /*EF*/
		const Eigen::MatrixXi&,  /*EI*/
		const std::set<std::pair<double, int> >&,   /*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
		const Eigen::MatrixXd&,   /*C*/
		const int,   /*e*/
		const int,  /*e1*/
		const int,  /*e2*/
		const int,  /*f1*/
		const int,  /*f2*/
		const bool                                                  /*collapsed*/
		)-> void {};
	return
		collapse_edge(
			cost_and_placement, always_try, never_care,
			V, F, E, EMAP, EF, EI, Q, Qit, C, e, e1, e2, f1, f2);
}

IGL_INLINE bool igl::collapse_edge(
	const std::function<void(
		const int,
		const Eigen::MatrixXd&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		const Eigen::VectorXi&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		double&,
		Eigen::RowVectorXd&)>& cost_and_placement,
	const std::function<bool(
		const Eigen::MatrixXd&,/*V*/
		const Eigen::MatrixXi&,/*F*/
		const Eigen::MatrixXi&,/*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,/*EF*/
		const Eigen::MatrixXi&,/*EI*/
		const std::set<std::pair<double, int> >&,/*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
		const Eigen::MatrixXd&,/*C*/
		const int                                                        /*e*/
		)>& pre_collapse,
	const std::function<void(
		const Eigen::MatrixXd&,   /*V*/
		const Eigen::MatrixXi&,   /*F*/
		const Eigen::MatrixXi&,   /*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,  /*EF*/
		const Eigen::MatrixXi&,  /*EI*/
		const std::set<std::pair<double, int> >&,   /*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
		const Eigen::MatrixXd&,   /*C*/
		const int,   /*e*/
		const int,  /*e1*/
		const int,  /*e2*/
		const int,  /*f1*/
		const int,  /*f2*/
		const bool                                                  /*collapsed*/
		)>& post_collapse,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	std::set<std::pair<double, int> >& Q,
	std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
	Eigen::MatrixXd& C)
{
	int e, e1, e2, f1, f2;
	return
		collapse_edge(
			cost_and_placement, pre_collapse, post_collapse,
			V, F, E, EMAP, EF, EI, Q, Qit, C, e, e1, e2, f1, f2);
}


IGL_INLINE bool igl::collapse_edge(
	const std::function<void(
		const int,
		const Eigen::MatrixXd&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		const Eigen::VectorXi&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		double&,
		Eigen::RowVectorXd&)>& cost_and_placement,
	const std::function<bool(
		const Eigen::MatrixXd&,/*V*/
		const Eigen::MatrixXi&,/*F*/
		const Eigen::MatrixXi&,/*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,/*EF*/
		const Eigen::MatrixXi&,/*EI*/
		const std::set<std::pair<double, int> >&,/*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
		const Eigen::MatrixXd&,/*C*/
		const int                                                        /*e*/
		)>& pre_collapse,
	const std::function<void(
		const Eigen::MatrixXd&,   /*V*/
		const Eigen::MatrixXi&,   /*F*/
		const Eigen::MatrixXi&,   /*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,  /*EF*/
		const Eigen::MatrixXi&,  /*EI*/
		const std::set<std::pair<double, int> >&,   /*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
		const Eigen::MatrixXd&,   /*C*/
		const int,   /*e*/
		const int,  /*e1*/
		const int,  /*e2*/
		const int,  /*f1*/
		const int,  /*f2*/
		const bool                                                  /*collapsed*/
		)>& post_collapse,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	std::set<std::pair<double, int> >& Q,
	std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
	Eigen::MatrixXd& C,
	int& e,
	int& e1,
	int& e2,
	int& f1,
	int& f2)
{
	using namespace Eigen;
	if (Q.empty())
	{
		// no edges to collapse
		return false;
	}
	std::pair<double, int> p = *(Q.begin());
	if (p.first == std::numeric_limits<double>::infinity())
	{
		// min cost edge is infinite cost
		return false;
	}
	Q.erase(Q.begin());
	e = p.second;
	Qit[e] = Q.end();
	std::vector<int> N = circulation(e, true, EMAP, EF, EI);
	std::vector<int> Nd = circulation(e, false, EMAP, EF, EI);
	N.insert(N.begin(), Nd.begin(), Nd.end());
	bool collapsed = true;
	if (pre_collapse(V, F, E, EMAP, EF, EI, Q, Qit, C, e))
	{
		collapsed = collapse_edge(e, C.row(e), V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
	}
	else
	{
		// Aborted by pre collapse callback
		collapsed = false;
	}
	post_collapse(V, F, E, EMAP, EF, EI, Q, Qit, C, e, e1, e2, f1, f2, collapsed);
	if (collapsed)
	{
		// Erase the two, other collapsed edges
		Q.erase(Qit[e1]);
		Qit[e1] = Q.end();
		Q.erase(Qit[e2]);
		Qit[e2] = Q.end();
		// update local neighbors
		// loop over original face neighbors
		for (auto n : N)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// get edge id
					const int ei = EMAP(v * F.rows() + n);
					// erase old entry
					Q.erase(Qit[ei]);
					// compute cost and potential placement
					double cost;
					RowVectorXd place;
					cost_and_placement(ei, V, F, E, EMAP, EF, EI, cost, place);
					// Replace in queue
					Qit[ei] = Q.insert(std::pair<double, int>(cost, ei)).first;
					C.row(ei) = place;
				}
			}
		}
	}
	else
	{
		// reinsert with infinite weight (the provided cost function must **not**
		// have given this un-collapsable edge inf cost already)
		p.first = std::numeric_limits<double>::infinity();
		Qit[e] = Q.insert(p).first;
	}
	return collapsed;
}

//New: from toturial 703
IGL_INLINE bool igl::collapse_edge(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	igl::min_heap< std::tuple<double, int, int> >& Q,
	Eigen::VectorXi& EQ,
	Eigen::MatrixXd& C)
{
	int e, e1, e2, f1, f2;
	return collapse_edge(V, F, E, EMAP, EF, EI, Q, EQ, C, e, e1, e2, f1, f2);
}

//New: from toturial 703
IGL_INLINE bool igl::collapse_edge(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	igl::min_heap< std::tuple<double, int, int> >& Q,
	Eigen::VectorXi& EQ,
	Eigen::MatrixXd& C,
	int& e,
	int& e1,
	int& e2,
	int& f1,
	int& f2)
{
	using namespace Eigen;
	using namespace igl;
	std::tuple<double, int, int> p;
	while (true)
	{
		// Check if Q is empty
		if (Q.empty())
		{
			// no edges to collapse
			e = -1;
			return false;
		}
		// pop from Q
		p = Q.top();
		// 
		if (std::get<0>(p) == std::numeric_limits<double>::infinity())
		{
			e = -1;
			// min cost edge is infinite cost
			return false;
		}
		Q.pop();
		e = std::get<1>(p);
		// Check if matches timestamp | 
		if (std::get<2>(p) == EQ(e))
		{
			break;
		}
		// must be stale or dead.
		assert(std::get<2>(p) < EQ(e) || EQ(e) == -1);
		// try again.
	}

	// Why is this computed up here?
	// If we just need original face neighbors of edge, could we gather that more
	// directly than gathering face neighbors of each vertex?
	std::vector<int>  Nse, Nsf, Nsv;

	igl::circulation(e, true, F, EMAP, EF, EI /* Nse*/, Nsv, Nsf);
	std::vector<int>  Nde, Ndf, Ndv;
	igl::circulation(e, false, F, EMAP, EF, EI, /*Nde, */ Ndv, Ndf);


	bool collapsed = true;
	collapsed = collapse_edge(
		e, C.row(e),
		Nsv, Nsf, Ndv, Ndf,
		V, F, E, EMAP, EF, EI, e1, e2, f1, f2);



	if (collapsed)
	{
		// Erase the two, other collapsed edges by marking their timestamps as -1
		EQ(e1) = -1;
		EQ(e2) = -1;
		// TODO: visits edges multiple times, ~150% more updates than should
		//
		// update local neighbors
		// loop over original face neighbors
		//
		// Can't use previous computed Nse and Nde because those refer to EMAP
		// before it was changed...
		std::vector<int> Nf;
		Nf.reserve(Nsf.size() + Ndf.size()); // preallocate memory
		Nf.insert(Nf.end(), Nsf.begin(), Nsf.end());
		Nf.insert(Nf.end(), Ndf.begin(), Ndf.end());
		// https://stackoverflow.com/a/1041939/148668
		std::sort(Nf.begin(), Nf.end());
		Nf.erase(std::unique(Nf.begin(), Nf.end()), Nf.end());
		// Collect all edges that must be updated
		std::vector<int> Ne;
		Ne.reserve(3 * Nf.size());
		for (auto& n : Nf)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// get edge id
					const int ei = EMAP(v * F.rows() + n);
					Ne.push_back(ei);
				}
			}
		}
		// Only process edge once
		std::sort(Ne.begin(), Ne.end());
		Ne.erase(std::unique(Ne.begin(), Ne.end()), Ne.end());
		for (auto& ei : Ne)
		{
			// compute cost and potential placement
			double cost;
			RowVectorXd place;
			igl::shortest_edge_and_midpoint(ei, V, F, E, EMAP, EF, EI, cost, place);
			// Increment timestamp
			EQ(ei)++;
			// Replace in queue
			Q.emplace(cost, ei, EQ(ei));
			C.row(ei) = place;
		}
	}
	else
	{
		// reinsert with infinite weight (the provided cost function must *not*
		// have given this un-collapsable edge inf cost already)
		// Increment timestamp
		EQ(e)++;
		// Replace in queue
		Q.emplace(std::numeric_limits<double>::infinity(), e, EQ(e));
	}
	return collapsed;
}

//New: from toturial 703
IGL_INLINE bool igl::collapse_edge(
	const int e,
	const Eigen::RowVectorXd& p,
	/* const */ std::vector<int>& Nsv,
	const std::vector<int>& Nsf,
	/* const*/ std::vector<int>& Ndv,
	const std::vector<int>& Ndf,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	int& a_e1,
	int& a_e2,
	int& a_f1,
	int& a_f2)
{
	// Assign this to 0 rather than, say, -1 so that deleted elements will get
	// draw as degenerate elements at vertex 0 (which should always exist and
	// never get collapsed to anything else since it is the smallest index)
	using namespace Eigen;
	using namespace std;
	const int eflip = E(e, 0) > E(e, 1); // false when e1>e0
	// source and destination
	const int s = eflip ? E(e, 1) : E(e, 0);// small index
	const int d = eflip ? E(e, 0) : E(e, 1);//big index

	if (!igl::edge_collapse_is_valid(Nsv, Ndv))
	{
		return false;
	}

	// OVERLOAD: caller may have just computed this
	//
	// Important to grab neighbors of d before monkeying with edges
	const std::vector<int>& nV2Fd = (!eflip ? Nsf : Ndf);

	// The following implementation strongly relies on s<d
	assert(s < d && "s should be less than d");
	// move source and destination to placement
	V.row(s) = p;
	V.row(d) = p;

	// Helper function to replace edge and associate information with NULL
	const auto& kill_edge = [&E, &EI, &EF](const int e)
	{
		E(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		E(e, 1) = IGL_COLLAPSE_EDGE_NULL;
		EF(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		EF(e, 1) = IGL_COLLAPSE_EDGE_NULL;
		EI(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		EI(e, 1) = IGL_COLLAPSE_EDGE_NULL;
	};

	// update edge info
	// for each flap
	const int m = F.rows();
	for (int side = 0; side < 2; side++)
	{
		const int f = EF(e, side);
		const int v = EI(e, side);
		const int sign = (eflip == 0 ? 1 : -1) * (1 - 2 * side);
		// next edge emanating from d
		const int e1 = EMAP(f + m * ((v + sign * 1 + 3) % 3));
		// prev edge pointing to s
		const int e2 = EMAP(f + m * ((v + sign * 2 + 3) % 3));





		assert(E(e1, 0) == d || E(e1, 1) == d);
		assert(E(e2, 0) == s || E(e2, 1) == s);
		// face adjacent to f on e1, also incident on d
		const bool flip1 = EF(e1, 1) == f;
		const int f1 = flip1 ? EF(e1, 0) : EF(e1, 1);
		assert(f1 != f);
		assert(F(f1, 0) == d || F(f1, 1) == d || F(f1, 2) == d);
		// across from which vertex of f1 does e1 appear?
		const int v1 = flip1 ? EI(e1, 0) : EI(e1, 1);
		// Kill e1
		kill_edge(e1);
		// Kill f
		F(f, 0) = IGL_COLLAPSE_EDGE_NULL;
		F(f, 1) = IGL_COLLAPSE_EDGE_NULL;
		F(f, 2) = IGL_COLLAPSE_EDGE_NULL;
		// map f    1's edge on e1 to e2
		assert(EMAP(f1 + m * v1) == e1);

		EMAP(f1 + m * v1) = e2;
		// side opposite f2, the face adjacent to f on e2, also incident on s
		const int opp2 = (EF(e2, 0) == f ? 0 : 1);
		assert(EF(e2, opp2) == f);
		EF(e2, opp2) = f1;
		EI(e2, opp2) = v1;
		// remap e2 from d to s



		E(e2, 0) = E(e2, 0) == d ? s : E(e2, 0);
		E(e2, 1) = E(e2, 1) == d ? s : E(e2, 1);






		if (side == 0)
		{
			a_e1 = e1;
			a_f1 = f;
		}
		else
		{
			a_e2 = e1;
			a_f2 = f;
		}
	}

	// finally, reindex faces and edges incident on d. Do this last so asserts
	// make sense.
	//
	// Could actually skip first and last, since those are always the two
	// collpased faces. Nah, this is handled by (F(f,v) == d)
	//
	// Don't attempt to use Nde,Nse here because EMAP has changed
	{
		int p1 = -1;
		for (auto f : nV2Fd)
		{
			for (int v = 0; v < 3; v++)
			{
				if (F(f, v) == d)
				{
					const int e1 = EMAP(f + m * ((v + 1) % 3));
					const int flip1 = (EF(e1, 0) == f) ? 1 : 0;
					assert(E(e1, flip1) == d || E(e1, flip1) == s);
					E(e1, flip1) = s;
					const int e2 = EMAP(f + m * ((v + 2) % 3));
					// Skip if we just handled this edge (claim: this will be all except
					// for the first non-trivial face)
					if (e2 != p1)
					{
						const int flip2 = (EF(e2, 0) == f) ? 0 : 1;
						assert(E(e2, flip2) == d || E(e2, flip2) == s);
						E(e2, flip2) = s;
					}

					F(f, v) = s;
					p1 = e1;
					break;
				}
			}
		}
	}
	// Finally, "remove" this edge and its information
	kill_edge(e);
	return true;
}

IGL_INLINE bool igl::collapse_edge_by_algo(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	igl::min_heap< std::tuple<double, int, int> >& Q,
	Eigen::VectorXi& EQ,
	Eigen::MatrixXd& C,
	std::vector<Eigen::Matrix4d*>& VC)
{
	int e, e1, e2, f1, f2;
	return collapse_edge_by_algo(V, F, E, EMAP, EF, EI, Q, EQ, C, e, e1, e2, f1, f2,VC);
}

IGL_INLINE bool igl::collapse_edge_by_algo(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	igl::min_heap< std::tuple<double, int, int> >& Q,
	Eigen::VectorXi& EQ,
	Eigen::MatrixXd& C,
	int& e,
	int& e1,
	int& e2,
	int& f1,
	int& f2,
	std::vector<Eigen::Matrix4d*>& VC)
{
	using namespace Eigen;
	using namespace igl;
	std::tuple<double, int, int> p;
	while (true)
	{
		// Check if Q is empty
		if (Q.empty())
		{
			// no edges to collapse
			e = -1;
			return false;
		}
		// pop from Q
		p = Q.top();
		// 
		if (std::get<0>(p) == std::numeric_limits<double>::infinity())
		{
			e = -1;
			// min cost edge is infinite cost
			return false;
		}
		Q.pop();
		e = std::get<1>(p);
		// Check if matches timestamp | 
		if (std::get<2>(p) == EQ(e))
		{
			break;
		}
		// must be stale or dead
		if (std::get<2>(p) < EQ(e) || EQ(e) == -1) {
			//std::printf("bad edge = %d, EQ(e) = %d, Eq from Q = %d \n", e, EQ(e), std::get<2>(p));
		}
		// try again.
	}

	// Why is this computed up here?
	// If we just need original face neighbors of edge, could we gather that more
	// directly than gathering face neighbors of each vertex?
	std::vector<int>  Nse, Nsf, Nsv;

	igl::circulation(e, true, F, EMAP, EF, EI /* Nse*/, Nsv, Nsf);
	std::vector<int>  Nde, Ndf, Ndv;
	igl::circulation(e, false, F, EMAP, EF, EI, /*Nde, */ Ndv, Ndf);


	bool collapsed = true;
	//compute the cost before removing edge
	Eigen::Matrix4d* newCostMatrix = new Eigen::Matrix4d( * (VC[E(e, 0)]) + *(VC[E(e, 1)]));
	collapsed = collapse_edge(
		e, C.row(e),
		Nsv, Nsf, Ndv, Ndf,
		V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
	std::printf("edge %d, cost = %d, new v position = (%d %d %d)\n", e, std::get<0>(p), C.row(e).x(), C.row(e).y(), C.row(e).z());


	if (collapsed)
	{
		// Erase the two, other collapsed edges by marking their timestamps as -1
		EQ(e1) = -1;
		EQ(e2) = -1;
		// TODO: visits edges multiple times, ~150% more updates than should
		//
		// update local neighbors
		// loop over original face neighbors
		//
		// Can't use previous computed Nse and Nde because those refer to EMAP
		// before it was changed...
		std::vector<int> Nf;
		Nf.reserve(Nsf.size() + Ndf.size()); // preallocate memory
		Nf.insert(Nf.end(), Nsf.begin(), Nsf.end());
		Nf.insert(Nf.end(), Ndf.begin(), Ndf.end());
		// https://stackoverflow.com/a/1041939/148668
		std::sort(Nf.begin(), Nf.end());
		Nf.erase(std::unique(Nf.begin(), Nf.end()), Nf.end());
		// Collect all edges that must be updated
		std::vector<int> Ne;
		Ne.reserve(3 * Nf.size());
		for (auto& n : Nf)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// get edge id
					const int ei = EMAP(v * F.rows() + n);
					Ne.push_back(ei);
				}
			}
		}
		// Only process edge once
		std::sort(Ne.begin(), Ne.end());
		Ne.erase(std::unique(Ne.begin(), Ne.end()), Ne.end());

		//checks that the edge is updated to be a vertex
		assert(E(e, 0) == E(e, 1));

		VC[E(e, 0)] = newCostMatrix;
		VC[E(e, 1)] = newCostMatrix;

		for (auto& ei : Ne)
		{
			EQ(ei)++;
			//computes cost for moving vertex to v1+ v2 /2
			double costV1 = computeCost(E(ei, 0), E(ei, 1),V,VC,Q,C,e,EQ(ei));
			//double costV2 = computeCost(E(ei, 1), E(ei, 0),V,VC);
			//if (costV1 < costV2) {
			//	C.row(ei) = V.row(E(ei, 0));
			//	Q.emplace(costV1, ei, EQ(ei));
			//}
			//else {
			//	C.row(ei) = V.row(E(ei, 1));
			//	Q.emplace(costV2, ei, EQ(ei));
			//}
			 
			//// compute cost and potential placement
			//double cost;
			//RowVectorXd place;
			//igl::shortest_edge_and_midpoint(ei, V, F, E, EMAP, EF, EI, cost, place);
			//// Increment timestamp
			//EQ(ei)++;
			//// Replace in queue
			//Q.emplace(cost, ei, EQ(ei));
			//C.row(ei) = place;
		}
	}
	else
	{
		// reinsert with infinite weight (the provided cost function must *not*
		// have given this un-collapsable edge inf cost already)
		// Increment timestamp
		EQ(e)++;
		// Replace in queue
		Q.emplace(std::numeric_limits<double>::infinity(), e, EQ(e));
	}
	return collapsed;
}

double igl::computeCost(int vFirst, int vSecond, Eigen::MatrixXd& V,
	std::vector<Eigen::Matrix4d*>& VC, igl::min_heap< std::tuple<double, int, int> >& Q,
	Eigen::MatrixXd& C, int& e, int EQ) {


	Eigen::MatrixXd Q1 = *(VC[vFirst]);
	Eigen::MatrixXd Q2 = *(VC[vSecond]);
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
		newV = 0.5* (V.row(vFirst) + V.row(vSecond));
	}
	newV.conservativeResize(4);
	Eigen::RowVectorXd temp = newV.transpose()* matQ;
	Q.emplace((temp* newV)(0, 0), e, EQ);
	C(e, 0) = newV(0);
	C(e, 1) = newV(1);
	C(e, 2) = newV(2);


	//double x = (V(vFirst, 0) + V(vSecond, 0)) / 2;
	//double y = (V(vFirst, 1) + V(vSecond, 1)) / 2;
	//double z = (V(vFirst, 2) + V(vSecond, 2)) / 2;
	//Eigen::RowVector4d vertex = Eigen::RowVector4d(x, y, z, 1);
	//Q.emplace(vertex * (*(VC[vFirst]) + *(VC[vSecond])) * vertex.transpose(), e, EQ);
	//C.row(e) = Eigen::RowVector3d(x, y, z);

	return 0;
}
