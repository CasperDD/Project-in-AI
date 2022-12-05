#include <selforg/controller_misc.h>
#include <ode_robots/hexabotsensormotordefinition.h>
#include "MyController.h"

MyController::MyController(const MyControllerConf& _conf)
    : AbstractController("MyController", "$Id: MyController.cpp,v 0.1 $"),
      conf(_conf)
{
    t = 0;

    outputH1 = 0.001;
    outputH2 = 0.001;
};

MyController::~MyController()
{
}

void MyController::init(int sensornumber, int motornumber, RandGen* randGen)
{
    // Tripodgait for 18 DOF Hexapod
    assert(motornumber >= 18);
};

/// performs one step (includes learning).
/// Calulates motor commands from sensor inputs.
void MyController::step(const sensor* x_, int number_sensors,
          motor* y_, int number_motors)
{
    stepNoLearning(x_, number_sensors, y_, number_motors);
};

/// performs one step without learning. Calulates motor commands from sensor
/// inputs.
void MyController::stepNoLearning(const sensor* x_, int number_sensors,
                    motor* y_, int number_motors)
{
    // Tripodgait for 18 DOF Hexapod

    assert(number_sensors >= 18);
    assert(number_motors >= 18);

    double activityH1 = conf.WeightH1_H1 * outputH1 + conf.WeightH1_H2 * outputH2 + 0.01;
    double activityH2 = conf.WeightH2_H2 * outputH2 + conf.WeightH2_H1 * outputH1 + 0.01;

    outputH1 = tanh(activityH1);
    outputH2 = tanh(activityH2);

    // generate motor commands
    // right rear coxa (knee) forward-backward joint
    y_[HEXABOT::T3_m] = outputH2 * conf.fact + conf.bias;
    y_[HEXABOT::C3_m] = -outputH1 * conf.fact * conf.direction;
    y_[HEXABOT::F3_m] = y_[HEXABOT::T3_m];
    // left rear coxa (knee) forward-backward joint
    y_[HEXABOT::T6_m] = -outputH2 * conf.fact + conf.bias;
    y_[HEXABOT::C6_m] = -outputH1 * conf.fact * conf.direction;
    y_[HEXABOT::F6_m] = -y_[HEXABOT::T6_m];
    // right middle coxa (knee) forward-backward joint
    y_[HEXABOT::T2_m] = outputH2 * conf.fact + conf.bias;
    y_[HEXABOT::C2_m] = -outputH1 * conf.fact * conf.direction;
    y_[HEXABOT::F2_m] = y_[HEXABOT::T2_m];
    // left middle coxa (knee) forward-backward joint
    y_[HEXABOT::T5_m] = -outputH2 * conf.fact + conf.bias;
    y_[HEXABOT::C5_m] = -outputH1 * conf.fact * conf.direction;
    y_[HEXABOT::F5_m] = -y_[HEXABOT::T5_m];
    // right front coxa (knee) forward-backward joint
    y_[HEXABOT::T1_m] = outputH2 * conf.fact + conf.bias;
    y_[HEXABOT::C1_m] = -outputH1 * conf.fact * conf.direction;
    y_[HEXABOT::F1_m] = y_[HEXABOT::T1_m];
    // left front coxa (knee) forward-backward joint
    y_[HEXABOT::T4_m] = -outputH2 * conf.fact + conf.bias;
    y_[HEXABOT::C4_m] = -outputH1 * conf.fact * conf.direction;
    y_[HEXABOT::F4_m] = -y_[HEXABOT::T4_m];

    // update step counter
    t++;

    // // right rear coxa (knee) forward-backward joint
    // y_[HEXABOT::T3_m] = 0.3;
    // y_[HEXABOT::C3_m] = -0.3;
    // y_[HEXABOT::F3_m] = y_[HEXABOT::T3_m];
    // // right front coxa (knee) forward-backward joint
    // y_[HEXABOT::T1_m] = 0.3;
    // y_[HEXABOT::C1_m] = -0.3;
    // y_[HEXABOT::F1_m] = y_[HEXABOT::T1_m];
    // // left middle coxa (knee) forward-backward joint
    // y_[HEXABOT::T5_m] = -0.3;
    // y_[HEXABOT::C5_m] = -0.3;
    // y_[HEXABOT::F5_m] = -y_[HEXABOT::T5_m];

    // // right middle coxa (knee) forward-backward joint
    // y_[HEXABOT::T2_m] = 0.3;
    // y_[HEXABOT::C2_m] = -0.3;
    // y_[HEXABOT::F2_m] = y_[HEXABOT::T2_m];
    // // left rear coxa (knee) forward-backward joint
    // y_[HEXABOT::T6_m] = -0.3;
    // y_[HEXABOT::C6_m] = -0.3;
    // y_[HEXABOT::F6_m] = -y_[HEXABOT::T6_m];
    // // left front coxa (knee) forward-backward joint
    // y_[HEXABOT::T4_m] = -0.3;
    // y_[HEXABOT::C4_m] = -0.3;
    // y_[HEXABOT::F4_m] = -y_[HEXABOT::T4_m];

    // // update step counter
    // t++;
};

/***** STOREABLE ****/
/** stores the controller values to a given file. */
bool MyController::store(FILE* f) const
{
    return true;
};
/** loads the controller values from a given file. */
bool MyController::restore(FILE* f)
{
    return true;
};