#ifndef __MyController_H
#define __MyController_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <selforg/matrix.h>

typedef struct MyControllerConf {
    double WeightH1_H1;
    double WeightH2_H2;
    double WeightH1_H2;
    double WeightH2_H1;
    double fact;
    double direction;
    double bias;
} MyControllerConf;

class MyController : public AbstractController {

  public:
    MyController(const MyControllerConf& conf = getDefaultConf());
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~MyController();

    /// returns the name of the object (with version number)
    virtual paramkey getName() const {
      return name;
    }
    /// returns the number of sensors the controller was initialised with or 0
    /// if not initialised
    virtual int getSensorNumber() const {
      return number_channels;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const {
      return number_channels;
    }

    /// performs one step (includes learning).
    /// Calulates motor commands from sensor inputs.
    virtual void step(const sensor*, int number_sensors, motor*,
        int number_motors);

    /// performs one step without learning. Calulates motor commands from sensor
    /// inputs.
    virtual void stepNoLearning(const sensor*, int number_sensors,
        motor*, int number_motors);

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);

    static MyControllerConf getDefaultConf() {
      MyControllerConf c;
      c.WeightH1_H1 = 1.;
      c.WeightH2_H2 = 1.;
      c.WeightH1_H2 = 0.3;
      c.WeightH2_H1 = -0.3;
      c.fact = 0.3; //0.7;
      c.direction = -1;
      c.bias = 0.0; //negative is legs up

      return c;
    }

  protected:
    unsigned short number_channels;

    int t;
    paramkey name;
    double outputH1;
    double outputH2;

    MyControllerConf conf;
};

#endif

