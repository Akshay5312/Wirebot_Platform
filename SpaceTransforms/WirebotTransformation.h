#pragma once 

#include "SpaceTransform.h"

namespace LLAMA{
    namespace SpaceTransformations{
    /**
     * @brief transformation from configuration space to joint space of a wirebot
     * 
     * @tparam inDOF (configurations)
     * @tparam outDOF (joints)
     * @tparam inType must cast to BLA::Matrix<inDOF>
     * @tparam outType must cast to BLA::Matrix<outDOF>
     */
    class WirebotTransformations : public SpaceTransformation<3, 3>{
        private:
            BLA::Matrix<3> _a1;
            BLA::Matrix<3> _a2;
            BLA::Matrix<3> _a3;
        public:
            BLA::Matrix<3> _y(BLA::Matrix<3> c){
                // -42.5704
                // -2.903

                BLA::Matrix<3> q;
                q(0) = BLA::Norm<3>(c - _a1);
                q(1) = BLA::Norm<3>(c - _a2);
                q(2) = BLA::Norm<3>(c - _a3); 

                return q;
            }
        public:

            /**
             * @brief Construct a new Wirebot Transformation
             * 
             * @param a1 anchor location
             * @param a2 anchor location
             * @param a3 anchor location
             */
            WirebotTransformations(BLA::Matrix<3> a1, BLA::Matrix<3> a2, BLA::Matrix<3> a3) : SpaceTransformation<3,3>(){
                _a1 = a1;
                _a2 = a2;
                _a3 = a3;
            }

            /**
             * @brief inverse kinematics, transformation from task space to joint space
             * 
             * @param c 
             * @return BLA::Matrix<3> 
             */
            BLA::Matrix<3> IK(BLA::Matrix<3> c){return this->FT(c);}

            /**
             * @brief get the inverse jacobian, linear transformation of task space velocity to joint space velocity at C
             * 
             * @param c position
             * @return BLA::Matrix<3, 3> 
             */
            BLA::Matrix<3, 3> InverseJacobian(BLA::Matrix<3> c){
                return BLA::Jacobian<3,3>((*(BLA::VVF<3, 3>*)(this)), c);
            }

            /**
             * @brief get the inverse jacobian, linear transformation of joint space velocity to task space velocity at C
             * 
             * @param c position
             * @return BLA::Matrix<3,3> 
             */
            BLA::Matrix<3,3> Jacobian(BLA::Matrix<3> c){
                return BLA::Inverse(InverseJacobian(c));
            }

            /**
             * @brief transformation of joint space to task space using an optimization based solver
             * 
             * @param q 
             * @return BLA::Matrix<3> 
             */
            BLA::Matrix<3> FK(BLA::Matrix<3> q){
                return this->IT(q);
            }

            /**
             * @brief transformation of joint space to task space using an optimization based solver
             * 
             * @param q 
             * @param bestGuess 
             * @param timeComplex 
             * @return BLA::Matrix<3> 
             */
            BLA::Matrix<3> FK(BLA::Matrix<3> q, BLA::Matrix<3> bestGuess, int timeComplex = 10000){
                return this->IT(q, bestGuess, timeComplex);
            }            

    };
    }
}