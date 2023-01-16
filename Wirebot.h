#include "SpaceTransforms/WirebotTransformation.h"

struct Actuators{
    float values[3] = {0};
    float speeds[3] = {0};
};

class Wirebot : LLAMA::SpaceTransformations::WirebotTransformations{
    private:
        Actuators _actuators;

    public:
        Wirebot(BLA::Matrix<3> a1, BLA::Matrix<3> a2, BLA::Matrix<3> a3) : LLAMA::SpaceTransformations::WirebotTransformations(a1, a2, a3){

        }
};