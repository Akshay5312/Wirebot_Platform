#include "SpaceTransforms/WirebotTransformation.h"

//no need for a template -- time crunch and we know it's 3

using ActuatorStates = BLA::Matrix<3>;

class Actuators{
    public:
        Actuators(){}
        virtual void setPosition(BLA::Matrix<3> position) = 0;
        virtual void setSpeed(BLA::Matrix<3> speed) = 0;
        virtual void runToPosition(float dt) = 0;
        virtual void runAtSpeed(float dt) = 0;

        virtual BLA::Matrix<3>& getPosition() = 0;
        virtual BLA::Matrix<3> getSpeed() = 0;

        operator BLA::Matrix<3>&() {return getPosition();}

};

class virtualActuators : public Actuators{
    private:
        BLA::Matrix<3> _position;
        BLA::Matrix<3> _speed;

        float maxActuatorSpeed = 5;

        BLA::Matrix<3> _targ_position;

        bool runRoutine(float dt){
            _targ_position = _targ_position + _speed * dt;
            return true;
        }

    public:
        virtualActuators() : Actuators(){}

        void operator = (const BLA::Matrix<3> &toSet){
            _position = toSet;
        }

        void setPosition(BLA::Matrix<3> position){_targ_position = position;}
        void setSpeed(BLA::Matrix<3> speed){_speed = speed;}
        void runToPosition(float dt){
            BLA::Matrix<3> direction = BLA::Zeros<3>();
            BLA::Matrix<3> disp = (_targ_position - _position);
            for(int i = 0; i < 3; i++){
                direction(i) = disp(i) / abs(disp(i));
            }

            setSpeed(direction * maxActuatorSpeed);
        }
        void runAtSpeed(float dt){
            runRoutine(dt);
        }

        BLA::Matrix<3>& getPosition(){
            return _position;
        }
        BLA::Matrix<3> getSpeed(){return _speed;}

};

class Wirebot{
    LLAMA::SpaceTransformations::WirebotTransformations* kinm;
    Actuators* _actuators;
    BLA::Matrix<3> _qX;
    BLA::Matrix<3> _cX;

    BLA::Matrix<3> _dX;

    
    BLA::Matrix<3> _setpoint;
    BLA::VVF<3,1>* _traj;

    
    void runRoutine(){
        _qX = (*_actuators);
        _cX = kinm->FK(_qX,_cX,30);
    }

    public:

    Wirebot(BLA::Matrix<3> anchor1, BLA::Matrix<3> anchor2, BLA::Matrix<3> anchor3){
        kinm = new LLAMA::SpaceTransformations::WirebotTransformations(anchor1, anchor2, anchor3);
        
    }

    void initActuators(Actuators* actuators){
        _actuators = actuators;
        BLA::Matrix<3> initX = (*actuators);
        _cX = kinm->FK(initX);
        _qX = initX;
    }
    BLA::Matrix<3> getTraverser();

    BLA::Matrix<3> getJoints();
    
    BLA::Matrix<3> getTraverser(ActuatorStates q);

    void goTo(BLA::Matrix<3> cx);

    bool run(bool );

    LLAMA::SpaceTransformations::WirebotTransformations* getKinematics(){
        return kinm;
    }

    bool runSpeed();

    void setSpeed(BLA::Matrix<3> dX);

    BLA::Matrix<3> getSpeed();

    void setTrajectory(BLA::VVF<3,1>* traj){
        if(_traj == nullptr){}else{delete _traj;}
        _traj = traj;
    }

    void followTrajectory(float t);
};



    BLA::Matrix<3> Wirebot::getTraverser(){
        return _cX;
    }

    BLA::Matrix<3> Wirebot::getJoints(){
        return _qX;
    }
    
    BLA::Matrix<3> Wirebot::getTraverser(ActuatorStates q){
        return kinm->FK(q,_cX,200);
    }

    void Wirebot::goTo(BLA::Matrix<3> cx){
        _setpoint = cx;
    }

    bool Wirebot::run(bool purelyPositional = false){
        runRoutine();
        _actuators->setPosition(kinm->IK(_setpoint));
        if(!purelyPositional){
            setSpeed((_setpoint - _cX) * 5);

            runSpeed();
        }else{
            _actuators->runToPosition(0.1);
        }
    }

    bool Wirebot::runSpeed(){
        runRoutine();
        // actuators->setPosition(kinm->IK(_setpoint));
        _actuators->setSpeed(kinm->InverseJacobian(_cX) * _dX);

        _actuators->runAtSpeed(0.1);
    }

    void Wirebot::setSpeed(BLA::Matrix<3> dX){
        _dX = dX;
    }

    BLA::Matrix<3> Wirebot::getSpeed(){
        return kinm->Jacobian(_cX) * _actuators->getSpeed();
    }

    void Wirebot::followTrajectory(float t){
        //currently, just setting the setpoint is good enough
        BLA::Matrix<3> curr_setpoint = _traj->vv_f({t});
        goTo(curr_setpoint);
    }






