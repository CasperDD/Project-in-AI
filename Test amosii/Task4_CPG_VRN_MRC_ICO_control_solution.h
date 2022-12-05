#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>

///////// Save text////////////////////////
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
///////////////////////////////////////////

#include <selforg/matrix.h>

#define pi 3.14159265
//#define adaptiveCPGwithPlasticity
#define CPG

/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 */
class EmptyController : public AbstractController
{

public:
  EmptyController()
      : AbstractController("EmptyController", "$Id: tripodgait18dof.cpp,v 0.1 $")
  {
    t = 0;
    /// Neural Control I: CPG implementation, for students ///
    outputH1 = 0.2;
    outputH2 = 0.2;
    activityH1 = 0.0;
    activityH2 = 0.0;

    //  Connections from CPG to motor neurons
    bias_tjoint = 0.0;
    bias_cjoint = 0.5;
    bias_fjoint = -0.9;

    WeightM0_H2 = -0.5;
    WeightM1_H2 = 0.5;
    WeightM2_H2 = -0.5;

    WeightM3_H2 = 0.5;
    WeightM4_H2 = -0.5;
    WeightM5_H2 = 0.5;

    WeightM6_H1 = -0.2;
    WeightM7_H1 = 0.2;
    WeightM8_H1 = -0.2;

    WeightM9_H1 = 0.2;
    WeightM10_H1 = -0.2;
    WeightM11_H1 = 0.2;

    WeightM12_H1 = -0.2;
    WeightM13_H1 = 0.2;
    WeightM14_H1 = -0.2;

    WeightM15_H1 = 0.2;
    WeightM16_H1 = -0.2;
    WeightM17_H1 = 0.2;

    percount = 1;
    UPDATE = 1;

    outputH1_Chaos = 0.001;
    outputH2_Chaos = 0.001;
    activityH1_Chaos = 0.0;
    activityH2_Chaos = 0.0;

    alph = 1.5;
    phi = 0.3 * pi; // 0< x < pi
    MI = 0.01;

    saveFile1.open("savedata1.txt", ios::out);
    saveFile2.open("savedata2.txt", ios::out);

    // plot parameters using GUI "To display GUI, in terminal, type ./start -g 1 "
    addInspectableValue("CPGoutputH1", &outputH1, "CPGoutputH1");
    addInspectableValue("CPGoutputH2", &outputH2, "CPGoutputH2");
  };

  virtual void init(int sensornumber, int motornumber, RandGen *randGen = 0)
  {
    // Tripodgait for 18 DOF Hexapod
    assert(motornumber >= 18);
  };

  virtual ~EmptyController(){};

  /// returns the name of the object (with version number)
  virtual paramkey getName() const
  {
    return name;
  }
  /// returns the number of sensors the controller was initialised with or 0
  /// if not initialised
  virtual int getSensorNumber() const
  {
    return number_channels;
  }
  /// returns the mumber of motors the controller was initialised with or 0 if
  // not initialised
  virtual int getMotorNumber() const
  {
    return number_channels;
  }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor *x_, int number_sensors,
                    motor *y_, int number_motors)
  {
    stepNoLearning(x_, number_sensors, y_, number_motors);
  };

  /// performs one step without learning. Calulates motor commands from sensor
  /// inputs.
  virtual void stepNoLearning(const sensor *x_, int number_sensors,
                              motor *y_, int number_motors)
  {
    // Tripodgait for 18 DOF Hexapod
    int static iii = 0;
    assert(number_sensors >= 18);
    assert(number_motors >= 18);

    //----Students--------Adding your Neural Controller here------------------------------------------//

    // sensory inputs ,e.g, x_[FL_us] = left antenna, x_[FR_us] = right antenna (see also the list below)
    // x_[G0x_s] , x_[G0y_s], x_[G0z_s] = relative position to reference object 1 (red ball)

    // Final outputs of your controller should be set to the following y_[xx] parameters to control leg joints

    /*******************************************************************************
     *  MODULE 1 CPG
     *******************************************************************************/

    /// Neural Control I: CPG implementation, for students ///

#ifdef CPG

    if (++percount % UPDATE == 0)
    {

      // Chaotic CPG
      WeightH1_H1_Chaos = -5.5;  // 1.5;
      WeightH2_H2_Chaos = 0.0;   // 1.5;
      WeightH1_H2_Chaos = 1.48;  //-0.4;
      WeightH2_H1_Chaos = -1.65; // 0.4;

      BiasH1_Chaos = -5.73;
      BiasH2_Chaos = 0.25;

      activityH1_Chaos = WeightH1_H1_Chaos * outputH1_Chaos + WeightH1_H2_Chaos * outputH2_Chaos + BiasH1_Chaos;
      activityH2_Chaos = WeightH2_H2_Chaos * outputH2_Chaos + WeightH2_H1_Chaos * outputH1_Chaos + BiasH2_Chaos;

      outputH1_Chaos = (exp(2 * activityH1_Chaos) - 1) / (exp(2 * activityH1_Chaos) + 1); // tanh(activityH1);
      outputH2_Chaos = (exp(2 * activityH2_Chaos) - 1) / (exp(2 * activityH2_Chaos) + 1); // tanh(activityH2);

      // Periodic CPG
      WeightH1_H1 = alph * cos(phi);  // 1.5;1.4;
      WeightH2_H2 = alph * cos(phi);  // 1.5;1.4;
      WeightH1_H2 = -alph * sin(phi); //-0.4;-0.19
      WeightH2_H1 = alph * sin(phi);  // 0.4;0.19

      // WeightH1_H1  = 1.4;
      // WeightH2_H2  = 1.4;
      // WeightH1_H2  = -0.18-MI;
      // WeightH2_H1  = 0.18+MI;

      // WeightH1_H1  = 1.5;
      // WeightH2_H2  = 1.5;
      // WeightH1_H2  = -0.4;
      // WeightH2_H1  = 0.4;

      activityH1 = WeightH1_H1 * outputH1 + WeightH1_H2 * outputH2;
      activityH2 = WeightH2_H2 * outputH2 + WeightH2_H1 * outputH1;

      outputH1 = (exp(2 * activityH1) - 1) / (exp(2 * activityH1) + 1); // tanh(activityH1);
      outputH2 = (exp(2 * activityH2) - 1) / (exp(2 * activityH2) + 1); // tanh(activityH2);

      // usleep(1000000);
    }

    printf("CPG %f %f\n", WeightH1_H1, WeightH1_H2);

    // printf("CPG %f %f\n",outputH1, outputH2);
#endif

    /*******************************************************************************
     *  MODULE Motor neurons
     *******************************************************************************/

    /// Neural Control I: CPG implementation, for students ///
    // M0
    // y_[TR0_m] = WeightM0_H2*outputH2_Chaos + bias_tjoint;
    y_[TR0_m] = WeightM0_H2 * outputH2 + bias_tjoint;
    // M1
    y_[TR1_m] = WeightM1_H2 * outputH2 + bias_tjoint;
    // M2
    y_[TR2_m] = WeightM2_H2 * outputH2 + bias_tjoint;

    // M3
    y_[TL0_m] = WeightM3_H2 * outputH2 + bias_tjoint;
    // M4
    y_[TL1_m] = WeightM4_H2 * outputH2 + bias_tjoint;
    // M5
    y_[TL2_m] = WeightM5_H2 * outputH2 + bias_tjoint;

    // M6
    y_[CR0_m] = WeightM6_H1 * outputH1 + bias_cjoint;
    // M7
    y_[CR1_m] = WeightM7_H1 * outputH1 + bias_cjoint;
    // M8
    y_[CR2_m] = WeightM8_H1 * outputH1 + bias_cjoint;

    // M9
    y_[CL0_m] = WeightM9_H1 * outputH1 + bias_cjoint;
    // M10
    y_[CL1_m] = WeightM10_H1 * outputH1 + bias_cjoint;
    // M11
    y_[CL2_m] = WeightM11_H1 * outputH1 + bias_cjoint;

    // M12
    y_[FR0_m] = WeightM12_H1 * outputH1 + bias_fjoint;
    // M13
    y_[FR1_m] = WeightM13_H1 * outputH1 + bias_fjoint;
    // M14
    y_[FR2_m] = WeightM14_H1 * outputH1 + bias_fjoint;

    // M15
    y_[FL0_m] = WeightM15_H1 * outputH1 + bias_fjoint;
    // M16
    y_[FL1_m] = WeightM16_H1 * outputH1 + bias_fjoint;
    // M17
    y_[FL2_m] = WeightM17_H1 * outputH1 + bias_fjoint;

    // backbone joint
    y_[BJ_m] = 0;

    // update step counter
    t++;

    saveFile1 << outputH1 << "  " << outputH2 << "  " << y_[TR0_m] << " " << y_[TR1_m] << " " << y_[TR2_m] << "   \n"
              << flush; // SAVE DATA
    saveFile2 << outputH1 << "  " << outputH2 << "  " << y_[TR0_m] << " " << y_[TR1_m] << " " << y_[TR2_m] << "   \n"
              << flush; // SAVE DATA
  };

  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE *f) const
  {
    return true;
  };
  /** loads the controller values from a given file. */
  virtual bool restore(FILE *f)
  {
    return true;
  };

protected:
  unsigned short number_channels;

  int t;
  paramkey name;
  /// Neural Control I: CPG implementation, for students ///

  double alph;
  double phi;
  double MI;
  double WeightH1_H1;
  double WeightH2_H2;
  double WeightH1_H2;
  double WeightH2_H1;

  double BiasH1;
  double BiasH2;

  double activityH1;
  double activityH2;

  double outputH1;
  double outputH2;

  double bias_tjoint;
  double bias_cjoint;
  double bias_fjoint;

  double WeightM0_H2;
  double WeightM1_H2;
  double WeightM2_H2;

  double WeightM3_H2;
  double WeightM4_H2;
  double WeightM5_H2;

  double WeightM6_H1;
  double WeightM7_H1;
  double WeightM8_H1;

  double WeightM9_H1;
  double WeightM10_H1;
  double WeightM11_H1;

  double WeightM12_H1;
  double WeightM13_H1;
  double WeightM14_H1;

  double WeightM15_H1;
  double WeightM16_H1;
  double WeightM17_H1;

  int percount;
  int UPDATE;

  double WeightH1_H1_Chaos;
  double WeightH2_H2_Chaos;
  double WeightH1_H2_Chaos;
  double WeightH2_H1_Chaos;

  double BiasH1_Chaos;
  double BiasH2_Chaos;

  double activityH1_Chaos;
  double activityH2_Chaos;

  double outputH1_Chaos;
  double outputH2_Chaos;

  // --- Save text------------//
  ofstream saveFile1;
  ofstream saveFile2;
  //-------------------------//
};
#endif
