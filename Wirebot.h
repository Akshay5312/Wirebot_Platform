#include "SpaceTransforms/WirebotTransformation.h"


/**
 * @brief A representation of velocity controlled actuators
 * 
 * @tparam n number of actuators
 */
template <int n>
class Actuators{
    BLA::Matrix<n> _pos;
    BLA::Matrix<n> _vel;
    
    public:

    Actuators(BLA::Matrix<n> pos = {0,0,0}) : _pos(pos){
        _vel = BLA::Zeros<n>();
    }

    operator BLA::Matrix<n>& () {
        return _pos;
    }

    void setVel(BLA::Matrix<n> vel){_vel = vel;}

    BLA::Matrix<n> step(float dt){
        return (_pos = _pos + _vel * dt); 
    }
};

/**
 * @brief A class representing the wirebot
 * 
 */
class Wirebot : public LLAMA::SpaceTransformations::WirebotTransformations{
    private:
        //the actuators
        Actuators<3> _actuators;

        //the position
        BLA::Matrix<3> _p;
        //the target position
        BLA::Matrix<3> _targetp;
        //the target speed
        BLA::Matrix<3> _targetdp;

        
        BLA::Matrix<3> updatePosition(int accuracyHeuristic = 25){
            _p = FK(_actuators, _p, accuracyHeuristic);
            return _p;
        }
        
    public:
        Wirebot(BLA::Matrix<3> a1, BLA::Matrix<3> a2, BLA::Matrix<3> a3, BLA::Matrix<3> initConfig = {0,0,0}) : LLAMA::SpaceTransformations::WirebotTransformations(a1, a2, a3){
            _p = initConfig;

            _actuators = IK(_p);
        }

        Wirebot& run(bool runposition, float dt = 0.01){
            updatePosition();
            BLA::Matrix<3> setdp;
            if(runposition){
                setdp = (_targetp - _p) * 10;
            }else{
                setdp = _targetdp;
            }

            BLA::Matrix<3,3> currIJ = InverseJacobian(_p);

            BLA::Matrix<3> setdq = currIJ * setdp;

            _actuators.setVel(setdq);

            _actuators.step(dt);

            return (*this);
        }


        /**
         * @brief cast this to a vector by returning it's position
         * 
         * @return BLA::Matrix<3> the position of the end effector
         */
        operator BLA::Matrix<3>(){return _p;}

        /**
         * @brief assign the target position
         * 
         * @param target 
         * @return Wirebot& 
         */
        Wirebot& operator = (const BLA::Matrix<3> &target){ _targetp = target; return (*this);}

        void setVel(BLA::Matrix<3> Vel){
            _targetdp = Vel;
        }
};