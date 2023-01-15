#pragma once 

#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"
namespace LLAMA{
    template<int inDOF, int outDOF, class inType = BLA::Matrix<inDOF>, class outType = BLA::Matrix<outDOF>>
    class SpaceTransformation : public BLA::VVF<outDOF, inDOF>{
        private:
            OPT::Solver<outDOF, inDOF, SpaceTransformation>* _inverter;// = OPT::Solver<outDOF, inDOF>(this)
            
        public:
        
            BLA::Matrix<outDOF> vv_f (BLA::Matrix<inDOF> x){
                return _y(x);
            }
        
            SpaceTransformation() : BLA::VVF<outDOF, inDOF>(){
                _inverter = new OPT::Solver<outDOF, inDOF, SpaceTransformation>(this);
            }
            virtual outType _y(inType x) = 0;

            virtual outType FT(inType x){return _y(x);}

            virtual BLA::Matrix<outDOF, inDOF> Jacobian(inType x){
                return BLA::Jacobian<outDOF,inDOF>((*(BLA::VVF<outDOF, inDOF>*)(this)), x);
            }
            //to implement

            virtual inType IT(outType y){
                _inverter->setTarget(y);
                _inverter->gd()->setInit(inType());
                return _inverter->gd()->solve();
            }
            virtual inType IT(outType y, outType best_guess, int maxSteps){
                _inverter->setTarget(y);
                _inverter->gd()->setInit(best_guess);

                return _inverter->gd()->stepAmount(maxSteps);
            }

            OPT::Solver<outDOF, inDOF, SpaceTransformation>* getSolver(){return _inverter;}

    };
}