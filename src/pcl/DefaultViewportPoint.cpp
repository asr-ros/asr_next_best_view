/*
 * ViewportPoint.hpp
 *
 *  Created on: Jun 30, 2016
 *      Author: Daniel Stroh
 */

#include "typedef.hpp"

namespace next_best_view {

    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultViewportPoint &p) {
        strm << "pose: {" << endl << p.getPose() << "}" << endl;

        if (p.score != nullptr)
            strm << "score: {" << endl << p.score << endl << "}" << endl;
        else
            strm << "score: nullptr" << endl;

        if (p.child_point_cloud != nullptr)
            strm << "child_point_cloud.size: " << p.child_point_cloud->size() << endl;
        else
            strm << "child_point_cloud: nullptr" << endl;

        if (p.point_cloud != nullptr)
            strm << "point_cloud.size: " << p.point_cloud->size() << endl;
        else
            strm << "point_cloud: nullptr" << endl;

        if (p.child_indices != nullptr)
            strm << "child_indices.size: " << p.child_indices->size() << endl;
        else
            strm << "child_indices: nullptr" << endl;

        if (p.object_type_set != nullptr) {
            strm << "object_type_set.size: " << p.object_type_set->size() << endl;
            strm << "object_type_set: {" << endl;
            for (std::string object_type : *p.object_type_set) {
                strm << "  " << object_type << endl;
            }
            strm << "}" << endl;
        }
        else
            strm << "object_type_set: nullptr" << endl;
        return strm;
    }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultViewportPointPtr &p) {
        return strm << *p;
    }
}
