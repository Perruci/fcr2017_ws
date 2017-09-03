#include "../include/movement/movement.h"

Movement::Movement(int argc, char *argv[])
{
    this->kin_ = new Kinematics(argc, argv);
}

Movement::~Movement()
{
    delete kin_;
}

void Movement::moveSquare(float moveSpeed, float timeForward)
{
    for(size_t i = 0; i < 4; i++)
    {
        runAndStop(moveSpeed, timeForward);
        this->kin_->sleep();
        spin90degrees(moveSpeed);
        this->kin_->sleep();
    }
}

/* Circle functions */
double Movement::timeTo360deg(float angularVel)
{
    double timeToAngle = 2*PI/angularVel;
    return timeToAngle;
}
 void Movement::moveCircle(float linear, float angular)
{
    double delayTime = timeTo360deg(angular);
    /* Set up movement */
    ros::Duration delay(delayTime);
    /* Move */
    this->kin_->moveAndSpin(linear, angular);
    /* Wait */
    delay.sleep();
    /* Stop */
    this->kin_->moveStop();
}

void Movement::runAndStop(float vel, float moveTime)
{
    ros::Duration delay(moveTime);
    this->kin_->moveLinear(vel);
    delay.sleep();
    this->kin_->moveStop();
}

double Movement::timeTo90deg(float rotationSpeed)
{
    float angularVel = rotationSpeed;
    double timeToAngle = (PI/2)/angularVel;
    return timeToAngle;
}

void Movement::spin90degrees(float rotationSpeed)
{
    double rotationTime = timeTo90deg(std::abs(rotationSpeed));
    /* Set up movement */
    ros::Duration delay(rotationTime);
    this->kin_->moveAngular(rotationSpeed);
    delay.sleep();
    this->kin_->moveStop();
}

// int main(int argc, char *argv[])
// {
//     Movement move(argc, argv);
//     ros::Rate loop_rate(100);
//     loop_rate.sleep();
//     float vel = 0.2;
//     float angVel = 0.2;
//     int Time = 5;
//     while(ros::ok())
//     {
//         char c = 0;
//         std::cout << "Say the command\n-> ";
//         std::cin >> c;
//         switch (c)
//         {
//         case 'w':
//             move.runAndStop(vel, Time);
//             break;
//         case 'a':
//             move.spin90degrees(angVel);
//             break;
//         case 'd':
//             move.spin90degrees(-angVel);
//             break;
//         // case 's':
//         //     move.kin_->moveStop();
//             break;
//         default:
//             break;
//         }
//         if(c == 'q')
//             break;
//     }
// }
