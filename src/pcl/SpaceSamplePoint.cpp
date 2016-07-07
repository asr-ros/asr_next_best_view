/*
 * SpaceSamplePoint.hpp
 *
 *  Created on: Jun 30, 2016
 *      Author: Daniel Stroh
 */

#include "typedef.hpp"

namespace next_best_view {

    std::ostream& operator<<(std::ostream &strm, const next_best_view::SpaceSamplePoint &p) {
        strm << "point: {" << endl << p.getPoint() << "}" << endl;

        if (p.child_indices != nullptr)
            strm << "child_indices.size: " << p.child_indices->size() << endl;
        else
            strm << "child_indices: nullptr" << endl;

        return strm;
    }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::SpaceSamplePointPtr &p) {
        return strm << *p;
    }

}

