/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#include <fstream>
#include <sstream>
#include "bbTrans/bound.h"
#include "bbTrans/branch_and_bound.h"


template<class Node>
void WriteBounds(bb::Bound<Node>& lower_bound, bb::Bound<Node>&
    upper_bound, bb::Bound<Node>& upper_bound_convex, const
    std::list<Node>& nodes) {
    std::stringstream ss;
    ss << "./bb_bounds_" << nodes.begin()->GetSpace() << "_t0.csv";
    std::ofstream out(ss.str());
    for (auto& node : nodes) {
      double lb = lower_bound.Evaluate(node);
      double ub = upper_bound.Evaluate(node);
      double ubC = upper_bound_convex.Evaluate(node);
      out << lb << " " << ub << " " << ubC << std::endl;
    }
    out.close();
};
