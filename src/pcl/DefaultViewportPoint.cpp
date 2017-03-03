/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
        strm << "oldIdx: " << p.oldIdx << endl;
        return strm;
    }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultViewportPointPtr &p) {
        return strm << *p;
    }
}
