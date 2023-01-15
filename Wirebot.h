#include "SpaceTransforms/WirebotTransformation.h"

//no need for a template -- time crunch and we know it's 3

using ActuatorStates = BLA::Matrix<3>;

class Actuators{
    public:
        Actuators(){}
        virtual bool setPosition(float position) = 0;
        virtual bool setSpeed(float speed) = 0;
        virtual bool runToPosition() = 0;
        virtual bool runAtSpeed() = 0;

        virtual BLA::Matrix<3> getPosition() = 0;
        virtual BLA::Matrix<3> getSpeed() = 0;

        BLA::Matrix<3>& operator BLA::Matrix<3>(){
            return getPosition();
        }
};

class virtualActuators{
    public:
    private:

}

class Wirebot{
    LLAMA::SpaceTransformations::WirebotTransformations* kinm;
    Actuators* _actuators;
    BLA::Matrix<3> _qX;
    BLA::Matrix<3> _cX;

    BLA::VVF<3,1>* _traj;

    public:

    Wirebot(Anchors anchors, Actuators* actuators){
        kinm = new LLAMA::SpaceTransformations::WirebotTransformations(anchors(0), anchors(1), anchors(2));
        BLA::Matrix<3> initX = (*actuators);
        _cX = kinm->FK(initX);
        _qX = initX;
    }

    BLA::Matrix<3> getTraverser();
    
    BLA::Matrix<3> getTraverser(ActuatorStates q);

    void goTo(BLA::Matrix<3> cx);

    bool run();
    
    void setSpeed(BLA::Matrix<3> dX);

    BLA::Matrix<3> getSpeed();

    void setTrajectory(BLA::VVF<3,1>* traj){_traj = traj;}

    void followTrajectory(float t);
};






