/*
 * RealObjectPoint.hpp
 *
 *  Created on: Jun 30, 2016
 *      Author: Daniel Stroh
 */

#include "typedef.hpp"

namespace next_best_view {

    std::ostream& operator<<(std::ostream &strm, const next_best_view::RealObjectPoint &p) {
        strm << "pose: {" << endl << p.getPose() << "}" << endl;

        strm << "type: " << p.type << endl;
        if (p.normal_vectors != nullptr)
            strm << "normal_vectors.size: " << p.normal_vectors->size() << endl;
        else
            strm << "normal_vectors: nullptr" << endl;

        if (p.active_normal_vectors != nullptr)
            strm << "active_normal_vectors.size: " << p.active_normal_vectors->size() << endl;
        else
            strm << "active_normal_vectors: nullptr" << endl;

        strm << "color: {" << endl << p.color << "}" << endl;

        strm << "intermediate_object_weight: " << p.intermediate_object_weight << endl;

        return strm;
    }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::RealObjectPointPtr &p) {
        return strm << *p;
    }
}
