/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolARHomographyValidation.h"
#include "xpcf/component/ComponentFactory.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARHomographyValidation);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

    SolARHomographyValidation::SolARHomographyValidation():ConfigurableBase(xpcf::toUUID<SolARHomographyValidation>())
    {
        declareInterface<api::solver::pose::IHomographyValidation>(this);
        declareProperty("oppositeSideRatio",m_oppositeSideRatio);
        declareProperty("surfaceRatio",m_surfaceRatio);
        declareProperty("maxOppositeDotProduct",m_maxOppositeDotProduct);
    }

    float computeSurface(std::vector<Point2Df> points){
        size_t size=points.size();
        int i,j;
        float s = 0;
        for(i=0 ; i<size ; ++i)
        {
            j = (i+1) % size;
            s += points[i][0] * points[j][1]-points[j][0] * points[i][1];

        }
        if(s<0){
            s=0;
        }

        return sqrt(s);
    }

    float computeDistance(const Point2Df & a, const Point2Df & b){
        float x2,y2,d=0;
        x2 = (a[0]-b[0]);
        x2 = x2*x2;
        y2 = (a[1]-b[1]);
        y2 = y2*y2;
        d = sqrt(x2+y2);
        return d;
    }

    bool SolARHomographyValidation::isValid(const std::vector<Point2Df> & ref2DSquaredMarkerCorners, const std::vector<Point2Df> & projected2DSquaredMarkerCorners)
    {
        // check sides proportions: should be almost equal
        float d1,d2,r;

        d1=computeDistance(projected2DSquaredMarkerCorners[0],projected2DSquaredMarkerCorners[1]);
        d2=computeDistance(projected2DSquaredMarkerCorners[2],projected2DSquaredMarkerCorners[3]);

        r=(d1<d2)?d1/d2:d2/d1;

        if(r<m_oppositeSideRatio)
            return false;

        d1=computeDistance(projected2DSquaredMarkerCorners[1],projected2DSquaredMarkerCorners[2]);
        d2=computeDistance(projected2DSquaredMarkerCorners[0],projected2DSquaredMarkerCorners[3]);

        r=(d1<d2)?d1/d2:d2/d1;

        if(r<m_oppositeSideRatio)
            return false;

        // check surfaces
        float s2=computeSurface(ref2DSquaredMarkerCorners);
        float s1=computeSurface(projected2DSquaredMarkerCorners);
        if(s1<=0)
            return false;

        if((s1/s2)<m_surfaceRatio){
            return false;
        }

        //study the direction of each line
        Point2Df dir_0, dir_1, dir_2, dir_3;
        dir_0 = projected2DSquaredMarkerCorners[1]-projected2DSquaredMarkerCorners[0];
        dir_0.normalize();

        dir_1 = projected2DSquaredMarkerCorners[2]-projected2DSquaredMarkerCorners[1];
        dir_1.normalize();

        dir_2 = projected2DSquaredMarkerCorners[3]-projected2DSquaredMarkerCorners[2];
        dir_2.normalize();

        dir_3 = projected2DSquaredMarkerCorners[0]-projected2DSquaredMarkerCorners[3];
        dir_3.normalize();

        // test the aspect shape of the homography
        return fabs( dir_0.dot(dir_2) ) > m_maxOppositeDotProduct && fabs( dir_1.dot(dir_3) ) > m_maxOppositeDotProduct;
    }

}
}
}  // end of namespace Solar
