// #include "../WirebotTransformation.h"


// // struct actuatorPin{
// //     int dirPin;
// //     int stepPin;
// // };

// class Anchors{
//     BLA::Matrix<3> _anchorPos[3] = {{0,0,0}, {0,0,0}, {0,0,0}}; 
    
//     public:
//     Anchors(BLA::Matrix<3> anchor1, BLA::Matrix<3> anchor2, BLA::Matrix<3> anchor3){
//         _anchorPos[0] = anchor1;_anchorPos[1] = anchor2;_anchorPos[2] = anchor3;
//     }

//     BLA::Matrix<3>& operator ()(const int &i){
//         return _anchorPos[i];
//     }
// };

// class ActuatorStates{

// };

// class Wirebot{
//     LLAMA::SpaceTransformations::WirebotTransformations* kinm;


//     BLA::Matrix<3> _qX;
//     BLA::Matrix<3> _cX;

//     public:

//     Wirebot(Anchors anchors, BLA::Matrix<3> initX, bool configSpace = false){
//         kinm = new LLAMA::SpaceTransformations::WirebotTransformations(anchors(0), anchors(1), anchors(2));
//         if(configSpace){
//             _qX = kinm->IK(initX);
//             _cX = initX;
//         }else{
//             _cX = kinm->FK(initX);
//             _qX = initX;
//         }
//     }

//     LLAMA::GEOM::SpatialT getTraverser();
    
//     LLAMA::GEOM::SpatialT getTraverser(ActuatorStates q);

//     void goTo(BLA::Matrix<3> cx);

//     bool run();

//     void setSpeed(BLA::Matrix<3> dX);

//     BLA::Matrix<3> getSpeed();

//     void setTrajectory(BLA::VVF<3,1>* traj);

//     void followTrajectory(float t);
// };
