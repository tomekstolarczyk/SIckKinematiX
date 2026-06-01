#include <stdio.h>

#include "inverseKinematics.h"

void inverseKinematicsCCD(RobotArm6DoF* ramie, double* thetas, const Vector3D* target)
{
    Matrix44 fkResults[6];
    int max_iters = 5000;
    double tolerance = 0.001;

    // 1. iteracje zewnetrzne 
    for(size_t iter = 0; iter < max_iters ; iter++)
    {
        // sprawdzamy gdzie obecnie sie znajdujemy
        forwardKinematics(ramie, thetas, fkResults);
        Vector3D currentEndEffectorPosition = {fkResults[5].data[3], fkResults[5].data[7], fkResults[5].data[11]};

        // sprawdzamy czy cel trafiony
        Vector3D distanceVector;
        myVectorSub(&currentEndEffectorPosition, target, &distanceVector);
        if(myVectorLength(&distanceVector)<=tolerance)
        {
            break;
        }

        // 2. jesli cel nietrafiony to petla wewnetrzna po stawach od konca do bazy
        for(int i = 5 ; i>=0; i--)
        {
            Vector3D currentJointPosition = {fkResults[i].data[3], fkResults[i].data[7], fkResults[i].data[11]};

            Vector3D relativeToCurrentJointTargetPosition;
            myVectorSub(target, &currentJointPosition, &relativeToCurrentJointTargetPosition);

            Vector3D relativeToCurrentJointGripperPosition;
            myVectorSub(&currentEndEffectorPosition, &currentJointPosition, &relativeToCurrentJointGripperPosition);

            Vector3D rotationAxis = {fkResults[i].data[2], fkResults[i].data[6], fkResults[i].data[10]};
            Vector3D projectedTarget, projectedGripper;
            myVectorPlaneProjection(&relativeToCurrentJointTargetPosition, &rotationAxis, &projectedTarget);
            myVectorPlaneProjection(&relativeToCurrentJointGripperPosition, &rotationAxis, &projectedGripper);
            
            // zabezpieczamy sie przed sytuacja gdzie target lezy na osi obrotu 
            double projectedTargetPosLen = myVectorLength(&projectedTarget);
            double projectedGripperPosLen = myVectorLength(&projectedGripper);
            if(projectedTargetPosLen < 1e-7 || projectedGripperPosLen < 1e-7)
            {continue;}
            
            myVectorNormalization(&projectedTarget); 
            myVectorNormalization(&projectedGripper);

            double cosTeta = myVectorDotProduct(&projectedTarget,&projectedGripper);
            cosTeta = fmax(-1.0, fmin(1.0, cosTeta));
            double teta = acos(cosTeta);

            Vector3D crossHelperVector;
            myVectorCrossProduct(&projectedGripper, &projectedTarget, &crossHelperVector);
            double crossHelperDir = myVectorDotProduct(&crossHelperVector, &rotationAxis);
            if(crossHelperDir < 0) {teta = -teta;}

            thetas[i] += teta;
            thetas[i] = atan2(sin(thetas[i]), cos(thetas[i])); // angle normalization

            forwardKinematics(ramie, thetas, fkResults);
            currentEndEffectorPosition.x = fkResults[5].data[3];
            currentEndEffectorPosition.y = fkResults[5].data[7];
            currentEndEffectorPosition.z = fkResults[5].data[11];
        }
    }
}

void inverseKinematicsDLS(RobotArm6DoF* ramie, double* thetas, const Matrix44* targetPose)
{

    // 1. where are we now?
    Matrix44 fkResults[6];
    forwardKinematics(ramie, thetas, &fkResults);

    // 2.1 error for xyz 

        

    // 2.2 error for rotation

    // 3. are we on target?

    // 4. calculate Jacobian

    // 5. inverse Jacobian - find DLS coeff

    // 6. find delta theta

    // 7. udpate thetas


}