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
// #define adaptiveCPGwithPlasticity
#define CPG
// Note That: For decoupled CPGs implementation, the noise parameter in main.cpp
// should be set to a small value, otherwise the GRF cannot be identified from noise
#define PhaseModulation
// #define PhaseResetting

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
        outputH1 = 0.001;
        outputH2 = 0.001;
        activityH1 = 0.0;
        activityH2 = 0.0;

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

        /// Neural Control II: VRN implementation, for students ///
        activityH3 = 0.0;
        activityH4 = 0.0;
        activityH5 = 0.0;
        activityH6 = 0.0;
        activityH7 = 0.0;
        activityH8 = 0.0;
        activityH9 = 0.0;
        activityH10 = 0.0;
        activityH11 = 0.0;
        activityH12 = 0.0;
        activityH13 = 0.0;
        activityH14 = 0.0;

        outputH3 = 0.0;
        outputH4 = 0.0;
        outputH5 = 0.0;
        outputH6 = 0.0;
        outputH7 = 0.0;
        outputH8 = 0.0;

        outputH9 = 0.0;
        outputH10 = 0.0;
        outputH11 = 0.0;
        outputH12 = 0.0;
        outputH13 = 0.0;
        outputH14 = 0.0;

        BiasH5 = -2.48285;
        BiasH6 = -2.48285;
        BiasH7 = -2.48285;
        BiasH8 = -2.48285;

        BiasH9 = -2.48285;
        BiasH10 = -2.48285;
        BiasH11 = -2.48285;
        BiasH12 = -2.48285;

        WeightH5_H2 = 1.7246;
        WeightH6_H2 = -1.7246;
        WeightH7_H2 = 1.7246;
        WeightH8_H2 = -1.7246;

        WeightH9_H2 = 1.7246;
        WeightH10_H2 = -1.7246;
        WeightH11_H2 = 1.7246;
        WeightH12_H2 = -1.7246;

        WeightH5_H3 = 1.7246;
        WeightH6_H3 = -1.7246;
        WeightH7_H3 = -1.7246;
        WeightH8_H3 = 1.7246;

        WeightH9_H4 = 1.7246;
        WeightH10_H4 = -1.7246;
        WeightH11_H4 = -1.7246;
        WeightH12_H4 = 1.7246;

        WeightH13_H5 = 0.5;
        WeightH13_H6 = 0.5;
        WeightH13_H7 = -0.5;
        WeightH13_H8 = -0.5;

        WeightH14_H9 = 0.5;
        WeightH14_H10 = 0.5;
        WeightH14_H11 = -0.5;
        WeightH14_H12 = -0.5;

        Input1 = 0.0;
        Input2 = 0.0;

        WeightM0_H13 = -0.5;
        WeightM1_H13 = 0.5;
        WeightM2_H13 = -0.5;

        WeightM3_H14 = 0.5;
        WeightM4_H14 = -0.5;
        WeightM5_H14 = 0.5;

        leg_num = 6;
        mod_activityH5.resize(leg_num, 0.0);
        mod_activityH6.resize(leg_num, 0.0);
        mod_activityH7.resize(leg_num, 0.0);
        mod_activityH8.resize(leg_num, 0.0);
        mod_activityH13.resize(leg_num, 0.0);

        mod_outputH5.resize(leg_num, 0.0);
        mod_outputH6.resize(leg_num, 0.0);
        mod_outputH7.resize(leg_num, 0.0);
        mod_outputH8.resize(leg_num, 0.0);
        mod_outputH13.resize(leg_num, 0.0);

        mod_activityH9.resize(leg_num, 0.0);
        mod_activityH10.resize(leg_num, 0.0);
        mod_activityH11.resize(leg_num, 0.0);
        mod_activityH12.resize(leg_num, 0.0);
        mod_activityH14.resize(leg_num, 0.0);

        mod_outputH9.resize(leg_num, 0.0);
        mod_outputH10.resize(leg_num, 0.0);
        mod_outputH11.resize(leg_num, 0.0);
        mod_outputH12.resize(leg_num, 0.0);
        mod_outputH14.resize(leg_num, 0.0);

        /// Neural Control III: Neural preprocessing implementation, for students ///

        activityI1 = 0.0;
        activityI2 = 0.0;
        outputI1 = 0.0;
        outputI2 = 0.0;

        WeightI1_I1 = 5.4;
        WeightI2_I2 = 5.4;
        WeightI1_I2 = -3.55;
        WeightI2_I1 = -3.55;

        WeightI1_IR1 = 7.0;
        WeightI2_IR2 = 7.0;

        IR1_mapping = 0.0;
        IR2_mapping = 0.0;

        outputI1_final = 0.0;
        outputI2_final = 0.0;

        /// Neural Learning and Adaptation I: ICO implementation, for students ///

        outputI1_ico = 0.0;
        outputI2_ico = 0.0;

        exp_output = 0.0;
        exploration_g = 0.0;
        exploration_lowpass_g = 0.0;
        exploration_lowpass_old_g = 0.0;

        u_ico = 0.0;
        w_ico = 0.0;

        deri_xt_reflex_angle = 0.0;
        reflex_old = 0.0;

        predictive_signal = 0.0;
        distance = 0.0;
        alpha_tmp = 0.0;
        alpha = 0.0;
        deri_alpha = 0.0;
        input_angle_s = 0.0;
        input_distance_s = 0.0;
        xt_reflex_angle = 0.0;

        reflexive_signal = 0.0;
        deri_xt_reflex_angle = 0.0;
        reflex_old = 0.0;
        u_ico_out = 0.0;

        reset = 0;

        exp_decay = 1.0;

        /// Neural control: Multiple decoupled CPGs with phase modulation or phase resetting, for students ///
#if defined(PhaseModulation)
        // @Description: the following variables CPGs_outputH1, CPGs_outputH2, ..., can be regared as
        // arraies which stored the corresponding states of all CPGs.
        // @Note that: gamma represents the sensory feedback gains which directly influence the speed of the
        //  autonomous gait convergence. STudents can try to change their value to see what will happens

        leg_num = 6;                   // the number of the legs, for hexapod leg_num=6
        CPGs_outputH1.resize(leg_num); // The H1 neuron activity for leg_num CPGs
        CPGs_outputH2.resize(leg_num);
        CPGs_activityH1.resize(leg_num);
        CPGs_activityH2.resize(leg_num);
        GRF.resize(leg_num); // GRF means the ground reaction force of a robot foot detected
        fc.resize(leg_num);
        fs.resize(leg_num);
        gamma1.resize(leg_num);
        gamma2.resize(leg_num);

        gamma1.at(0) = 0.42;
        gamma2.at(0) = 0.32;
        gamma1.at(1) = 0.32;
        gamma2.at(1) = 0.3;
        gamma1.at(2) = 0.35;
        gamma2.at(2) = 0.32;
        gamma1.at(3) = 0.42;
        gamma2.at(3) = 0.32;
        gamma1.at(4) = 0.32;
        gamma2.at(4) = 0.3;
        gamma1.at(5) = 0.35;
        gamma2.at(5) = 0.32;
#endif
#if defined(PhaseResetting)
        // @Description: the following variables CPGs_outputH1, CPGs_outputH2, ..., can be regared as
        // arraies which stored the corresponding states of all CPGs.
        // @Note that: GRF_threshold directly determines the phase resetting timing. It influences the
        // the functionality of the mechanism. STudents can try to change its value to see what will happen
        // The GRF_threshold should be smammler than the maximum GRF value

        leg_num = 6;                   // the number of the legs, for hexapod leg_num=6
        CPGs_outputH1.resize(leg_num); // The H1 neuron activity for leg_num CPGs
        CPGs_outputH2.resize(leg_num);
        CPGs_activityH1.resize(leg_num);
        CPGs_activityH2.resize(leg_num);
        GRF.resize(leg_num); // GRF means the ground reaction force of a robot foot detected
        GRF_old.resize(leg_num);
        resetting1.resize(leg_num);
        resetting2.resize(leg_num);
        Diracs = 0;
        GRF_threshold = 0.25; // GRF_threshold determine
#endif

        saveFile1.open("de_savedata1.txt", ios::out);
        saveFile2.open("de_savedata2.txt", ios::out);
        saveFile3.open("de_savedata3.txt", ios::out);
        saveFile4.open("de_savedata4.txt", ios::out);

        // plot parameters using GUI "To display GUI, in terminal, type ./start -g 1 "
        addInspectableValue("CPGoutputH1", &outputH1, "CPGoutputH1");
        addInspectableValue("CPGoutputH2", &outputH2, "CPGoutputH2");

        addInspectableValue("VRNoutputH13R", &outputH13, "VRNoutputH13R");
        addInspectableValue("VRNoutputH14L", &outputH14, "VRNoutputH14L");

        addInspectableValue("IRright", &IR1_mapping, "IRright");
        addInspectableValue("IRleft", &IR2_mapping, "IRleft");

        addInspectableValue("IRoutput1R", &outputI1, "IRoutput1R");
        addInspectableValue("IRoutput2L", &outputI2, "IRoutput2L");

        addInspectableValue("uicoout", &u_ico_out, "uicoout");
        addInspectableValue("inputangles", &input_angle_s, "inputangles");

        addInspectableValue("wico", &w_ico, "wico");
        addInspectableValue("dreflex", &deri_xt_reflex_angle, "dreflex");
        addInspectableValue("reflex", &reflexive_signal, "reflex");
        addInspectableValue("predictive", &predictive_signal, "predictive");
        addInspectableValue("exp_decay", &exp_decay, "exp_decay");
        addInspectableValue("exp_output", &exp_output, "exp_output");

#if defined(PhaseModulation)
        addInspectableValue("R0_CPGH1", &CPGs_outputH1.at(0), "R0_CPGs_H1");
        addInspectableValue("R0_CPGH2", &CPGs_outputH2.at(0), "R0_CPGs_H2");
        addInspectableValue("fc", &fc.at(0), "fc"); // 0 means the right front leg
        addInspectableValue("fs", &fs.at(0), "fs");
        addInspectableValue("R0_GRF", &GRF.at(0), "R0_GRF");
        addInspectableValue("R1_GRF", &GRF.at(1), "R1_GRF");
#endif

#if defined(PhaseResetting)
        addInspectableValue("R0_CPGH1", &CPGs_outputH1.at(0), "R0_CPGs_H1");
        addInspectableValue("R0_CPGH2", &CPGs_outputH2.at(0), "R0_CPGs_H2");
        addInspectableValue("R0_GRF", &GRF.at(0), "R0_GRF");
        addInspectableValue("R1_GRF", &GRF.at(1), "R1_GRF");
        addInspectableValue("R0_GRF_old", &GRF_old.at(0), "R0_GRF_old");
        addInspectableValue("R1_GRF_old", &GRF_old.at(1), "R1_GRF_old");
#endif
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

        // Goal 1 // red

        if (sign(x_[G0x_s] /*X axis*/) > 0)
        {
            alpha = atan(x_[G0y_s] / x_[G0x_s]) * 180 / M_PI; // angle in degrees
        }
        else
        {                                                              // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
            alpha_tmp = -1 * atan(x_[G0y_s] / x_[G0x_s]) * 180 / M_PI; // angle in degrees
            if (alpha_tmp <= 0)
            { // left
                alpha = (-90 + (-90 - alpha_tmp));
            }
            else
            { // right
                alpha = (90 + (90 - alpha_tmp));
            }
        }
        input_angle_s = alpha / 180. * M_PI; // map sensor input to +-pi

        // Red
        distance = (sqrt(pow(x_[G0y_s], 2) + pow(x_[G0x_s], 2)));
        input_distance_s = distance / 100;

        // if the angle deviation is less than 0.2 then we set it to 0.0 for stable learning
        if (abs(input_angle_s) < 0.3)
        {
            input_angle_s = 0.0;
        }
        else
        {
            input_angle_s = input_angle_s;
        }

        //-----------------short distance----Calculating relative orientation to objects----------//

        // Angle reflex signals
        // 1) goal 1 RED

        double range_reflex = 0.01;
        if (input_distance_s < range_reflex /*0.04 ~0.001 very close to target*/)
        {
            reflexive_signal = input_angle_s;
        }
        else
        {
            reflexive_signal = 0.0;
        }

        if (input_distance_s < 0.008 || iii > 250)
        {
            reflexive_signal = 0.0;
        }

        predictive_signal = input_angle_s;

        double lp_gain = 0.99;
        double scale_exploration = 2.0;
        exploration_g = gauss();
        exploration_lowpass_old_g = exploration_lowpass_g;
        exploration_lowpass_g = exploration_lowpass_old_g * lp_gain + (1 - lp_gain) * exploration_g;
        exp_output = scale_exploration * exploration_lowpass_g;

        /*******************************************************************************
         *  MODULE 4 Neural learning and adaptation
         *******************************************************************************/

        /// Neural Learning and Adaptation I: ICO implementation, for students ///

        double rate_ico = 0.5;

        //----ICO learning-------//
        u_ico = w_ico * predictive_signal + reflexive_signal; // Red //----- Student

        deri_xt_reflex_angle = reflexive_signal - reflex_old; //----- Student

        // Weights of ICO learning modify these by implementing ICO learning rule!!

        w_ico += rate_ico * deri_xt_reflex_angle * predictive_signal; // Red //----- Student

        // exp_output*exp(-w_ico)
        // Exponential decay of exploration
        exp_decay = exp_output * exp(-w_ico);

        u_ico_out = exp_decay + u_ico; //----- Student

        // printf("w_ico %d u_ico_out %d\n",w_ico, u_ico_out);

        //////////////////////////////////////////////////////////////////////////////////////

        reflex_old = reflexive_signal;

        // Exploration and goal direction navigation control policy
        if (u_ico_out >= 0.3)
        {
            outputI1_ico = 0.1; // 0.1
            outputI2_ico = -1;
        }
        else if (u_ico_out <= -0.3)
        {
            outputI1_ico = -1;
            outputI2_ico = 0.1; // 0.1 test
        }
        else
        {
            outputI1_ico = -1;
            outputI2_ico = -1;
        }

        //////////////////////////////////////////////////////////

        // if (input_distance_s < 0.005)
        // {
        //     reset = 1;
        // }

        // if (reset == 1)
        // {

        //     if (exp_output >= 0.15)
        //     { // Turn left
        //         outputI1_ico = -1;
        //         outputI2_ico = 0.1;
        //     }
        //     else if (exp_output <= -0.15)
        //     { // Turn right
        //         outputI1_ico = 0.1;
        //         outputI2_ico = -1;
        //     }
        //     else
        //     { // Forward
        //         outputI1_ico = -1;
        //         outputI2_ico = -1;
        //     }

        //     iii++;
        // }

        // if (iii == 2000)
        // {
        //     iii = 0;
        //     reset = 0;
        // }

        // printf("reset %d iii %d\n",reset, iii);

        /*******************************************************************************
         *  MODULE 3 Sensory preprocessing
         *******************************************************************************/

        /// Neural Control III: Neural preprocessing implementation, for students ///

        // Obstacle avoidance
        // IR1 on the right [0,...,1]
        IR1_mapping = x_[FR_us] * 2 - 1;
        // IR2 on the left [0,...,1]
        IR2_mapping = x_[FL_us] * 2 - 1;

        activityI1 = WeightI1_I1 * outputI1 + WeightI1_I2 * outputI2 + WeightI1_IR1 * IR1_mapping;
        activityI2 = WeightI2_I2 * outputI2 + WeightI2_I1 * outputI1 + WeightI2_IR2 * IR2_mapping;
        outputI1 = tanh(activityI1); // Left  motors
        outputI2 = tanh(activityI2); // Right motors

        outputI1_final = outputI1; // Right IR --> go to left TC motors
        outputI2_final = outputI2; // Left IR  --> go to right TC motors

        /*******************************************************************************
         *  MODULE 5 Sensory fusion
         *******************************************************************************/
        /*if(input_distance_s < 0.1)
          {
          outputI1 = -1.0;
          outputI2 = -1.0;
          }
          else
          {
          outputI1 = outputI1;
          outputI2 = outputI2;
          }*/

        if (outputI1 > 0.9 || outputI2 > 0.9)
        {
            outputI1_ico = -1;
            outputI2_ico = -1;
        }

        outputI1_final = -outputI1 * outputI1_ico;
        outputI2_final = -outputI2 * outputI2_ico;

        // outputI1_final = outputI1;
        // outputI2_final = outputI2;

        // outputI1_final = outputI1_ico;
        // outputI2_final = outputI2_ico;

        // printf("x_[FR_us] %e x_[FL_us] %e IR1_mapping %e IR2_mapping %e\n", x_[FR_us], x_[FL_us], IR1_mapping, IR2_mapping);
        // printf("outputI1 %e outputI2 %e\n", outputI1, outputI2);

        /*******************************************************************************
         *  MODULE 1 CPG
         *******************************************************************************/

        /// Neural Control I: CPG implementation, for students ///

#ifdef CPG
        alph = 1.5;
        phi = 0.25;

        WeightH1_H1 = 1.5;  // alph*cos(phi);
        WeightH2_H2 = 1.5;  // alph*cos(phi);
        WeightH1_H2 = -0.4; // alph*sin(phi);
        WeightH2_H1 = 0.4;  //-alph*sin(phi);

        activityH1 = WeightH1_H1 * outputH1 + WeightH1_H2 * outputH2;
        activityH2 = WeightH2_H2 * outputH2 + WeightH2_H1 * outputH1;

        outputH1 = tanh(activityH1);
        outputH2 = tanh(activityH2);

        // printf("CPG %f %f\n",outputH1, outputH2);
#endif

        /*******************************************************************************
         *  MODULE 1 Multiple decoupled CPGs
         *******************************************************************************/

        /// Neural Control VI: Decoupled CPG with phase modulation or phase resetting implementation, for students ///
#if defined(PhaseModulation)
        alph = 1.5;
        phi = 0.25;

        WeightH1_H1 = 1.5;  // alph*cos(phi);
        WeightH2_H2 = 1.5;  // alph*cos(phi);
        WeightH1_H2 = -0.4; // alph*sin(phi);
        WeightH2_H1 = 0.4;  //-alph*sin(phi);
        BiasH1 = 0.01;
        BiasH2 = 0.01;
        for (unsigned int idx = 0; idx < leg_num; idx++)
        {
            GRF.at(idx) = x_[R0_fs + idx];
            fc.at(idx) = cos(CPGs_activityH1.at(idx));
            fs.at(idx) = sin(CPGs_activityH2.at(idx));

            CPGs_activityH1.at(idx) = WeightH1_H1 * CPGs_outputH1.at(idx) + WeightH1_H2 * CPGs_outputH2.at(idx) + BiasH1 - gamma1.at(idx) * GRF.at(idx) * fc.at(idx);
            CPGs_activityH2.at(idx) = WeightH2_H2 * CPGs_outputH2.at(idx) + WeightH2_H1 * CPGs_outputH1.at(idx) + BiasH2 - gamma2.at(idx) * GRF.at(idx) * fs.at(idx);

            CPGs_outputH1.at(idx) = tanh(CPGs_activityH1.at(idx));
            CPGs_outputH2.at(idx) = tanh(CPGs_activityH2.at(idx));
        }
#endif

#if defined(PhaseResetting)
        alph = 1.5;
        phi = 0.25;

        WeightH1_H1 = 1.5;  // alph*cos(phi);
        WeightH2_H2 = 1.5;  // alph*cos(phi);
        WeightH1_H2 = -0.4; // alph*sin(phi);
        WeightH2_H1 = 0.4;  //-alph*sin(phi);
        BiasH1 = 0.01;
        BiasH2 = 0.01;

        for (unsigned int idx = 0; idx < leg_num; idx++)
        {
            GRF.at(idx) = 0.2 * x_[R0_fs + idx] + 0.8 * GRF_old.at(idx); // low-pass filter, STudents can changes the parameters 0.05 and 0.95 to adjust the filter
            if ((GRF.at(idx) > GRF_threshold) && (GRF_old.at(idx) <= GRF_threshold))
            {
                Diracs = 1;
                // printf("Resetting, GRF:%f, GRF_old:%f\n", GRF.at(idx), GRF_old.at(idx));
            }
            else
            {
                Diracs = 0;
                // printf("No Resetting\n");
            }
            GRF_old.at(idx) = GRF.at(idx);

            CPGs_activityH1.at(idx) = WeightH1_H1 * CPGs_outputH1.at(idx) + WeightH1_H2 * CPGs_outputH2.at(idx) + BiasH1;
            CPGs_activityH2.at(idx) = WeightH2_H2 * CPGs_outputH2.at(idx) + WeightH2_H1 * CPGs_outputH1.at(idx) + BiasH2;

            resetting1.at(idx) = -CPGs_activityH1.at(idx) * Diracs;
            resetting2.at(idx) = (1.0 - CPGs_activityH2.at(idx)) * Diracs;

            CPGs_activityH1.at(idx) = CPGs_activityH1.at(idx) + resetting1.at(idx);
            CPGs_activityH2.at(idx) = CPGs_activityH2.at(idx) + resetting2.at(idx);

            // CPGs_activityH1.at(idx) = WeightH1_H1 * CPGs_outputH1.at(idx) + WeightH1_H2 * CPGs_outputH2.at(idx) + BiasH1 + resetting1.at(idx);
            // CPGs_activityH2.at(idx) = WeightH2_H2 * CPGs_outputH2.at(idx) + WeightH2_H1 * CPGs_outputH1.at(idx) + BiasH2 + resetting2.at(idx);
            CPGs_outputH1.at(idx) = tanh(CPGs_activityH1.at(idx));
            CPGs_outputH2.at(idx) = tanh(CPGs_activityH2.at(idx));
        }
#endif

        /*******************************************************************************
         *  MODULE 2 VRNs
         *******************************************************************************/

        /// Neural Control II: VRN implementation, for students ///

        //**********VRN***************

        Input1 = -1 * outputI1_final; // 1 = backward, -1 = forward, -1 = TR, 1 = TL
        Input2 = -1 * outputI2_final; // 1 = backward, -1 = forward, 1 = TR, -1 = TL

        // VRN 1 (Right)

        activityH3 = Input2;
        outputH3 = activityH3; // Linear neuron

        activityH5 = WeightH5_H2 * outputH2 + WeightH5_H3 * outputH3 + BiasH5;
        activityH6 = WeightH6_H2 * outputH2 + WeightH6_H3 * outputH3 + BiasH6;
        activityH7 = WeightH7_H2 * outputH2 + WeightH7_H3 * outputH3 + BiasH7;
        activityH8 = WeightH8_H2 * outputH2 + WeightH8_H3 * outputH3 + BiasH8;

        activityH13 = WeightH13_H5 * outputH5 + WeightH13_H6 * outputH6 + WeightH13_H7 * outputH7 + WeightH13_H8 * outputH8;

        outputH5 = tanh(activityH5);
        outputH6 = tanh(activityH6);
        outputH7 = tanh(activityH7);
        outputH8 = tanh(activityH8);

        outputH13 = tanh(activityH13);

        // VRN 2 (Left)

        activityH4 = Input1;
        outputH4 = activityH4; // Linear neuron

        activityH9 = WeightH9_H2 * outputH2 + WeightH9_H4 * outputH4 + BiasH9;
        activityH10 = WeightH10_H2 * outputH2 + WeightH10_H4 * outputH4 + BiasH10;
        activityH11 = WeightH11_H2 * outputH2 + WeightH11_H4 * outputH4 + BiasH11;
        activityH12 = WeightH12_H2 * outputH2 + WeightH12_H4 * outputH4 + BiasH12;

        activityH14 = WeightH14_H9 * outputH9 + WeightH14_H10 * outputH10 + WeightH14_H11 * outputH11 + WeightH14_H12 * outputH12;

        outputH9 = tanh(activityH9);
        outputH10 = tanh(activityH10);
        outputH11 = tanh(activityH11);
        outputH12 = tanh(activityH12);

        outputH14 = tanh(activityH14);

        //*******VRN end***************

        /*******************************************************************************
         *  MODULE 2 VRNs modified
         *******************************************************************************/

        /// Neural Control II: VRN implementation, for students ///

        //**********VRN***************

        Input1 = -1 * outputI1_final; // 1 = backward, -1 = forward, -1 = TR, 1 = TL
        Input2 = -1 * outputI2_final; // 1 = backward, -1 = forward, 1 = TR, -1 = TL

        // VRN 1 (Right)

        activityH3 = Input2;
        outputH3 = activityH3; // Linear neuron

        for (unsigned int idx = 0; idx < leg_num; idx++)
        {

            mod_activityH5.at(idx) = WeightH5_H2 * CPGs_outputH2.at(idx) + WeightH5_H3 * outputH3 + BiasH5;
            mod_activityH6.at(idx) = WeightH6_H2 * CPGs_outputH2.at(idx) + WeightH6_H3 * outputH3 + BiasH6;
            mod_activityH7.at(idx) = WeightH7_H2 * CPGs_outputH2.at(idx) + WeightH7_H3 * outputH3 + BiasH7;
            mod_activityH8.at(idx) = WeightH8_H2 * CPGs_outputH2.at(idx) + WeightH8_H3 * outputH3 + BiasH8;

            mod_activityH13.at(idx) = WeightH13_H5 * mod_outputH5.at(idx) + WeightH13_H6 * mod_outputH6.at(idx) + WeightH13_H7 * mod_outputH7.at(idx) + WeightH13_H8 * mod_outputH8.at(idx);

            mod_outputH5.at(idx) = tanh(mod_activityH5.at(idx));
            mod_outputH6.at(idx) = tanh(mod_activityH6.at(idx));
            mod_outputH7.at(idx) = tanh(mod_activityH7.at(idx));
            mod_outputH8.at(idx) = tanh(mod_activityH8.at(idx));

            mod_outputH13.at(idx) = tanh(mod_activityH13.at(idx));
        }

        // VRN 2 (Left)

        activityH4 = Input1;
        outputH4 = activityH4; // Linear neuron

        for (unsigned int idx = 0; idx < leg_num; idx++)
        {

            mod_activityH9.at(idx) = WeightH9_H2 * CPGs_outputH2.at(idx) + WeightH9_H4 * outputH4 + BiasH9;
            mod_activityH10.at(idx) = WeightH10_H2 * CPGs_outputH2.at(idx) + WeightH10_H4 * outputH4 + BiasH10;
            mod_activityH11.at(idx) = WeightH11_H2 * CPGs_outputH2.at(idx) + WeightH11_H4 * outputH4 + BiasH11;
            mod_activityH12.at(idx) = WeightH12_H2 * CPGs_outputH2.at(idx) + WeightH12_H4 * outputH4 + BiasH12;

            mod_activityH14.at(idx) = WeightH14_H9 * mod_outputH9.at(idx) + WeightH14_H10 * mod_outputH10.at(idx) + WeightH14_H11 * mod_outputH11.at(idx) + WeightH14_H12 * mod_outputH12.at(idx);

            mod_outputH9.at(idx) = tanh(mod_activityH9.at(idx));
            mod_outputH10.at(idx) = tanh(mod_activityH10.at(idx));
            mod_outputH11.at(idx) = tanh(mod_activityH11.at(idx));
            mod_outputH12.at(idx) = tanh(mod_activityH12.at(idx));

            mod_outputH14.at(idx) = tanh(mod_activityH14.at(idx));
        }

        // printf("mod_activityH14.at(0) %e mod_activityH14.at(1) %e mod_activityH14.at(2) %e mod_activityH14.at(3) %e mod_activityH14.at(4) %e mod_activityH14.at(5) %e\n",
        //        mod_activityH14.at(0), mod_activityH14.at(1), mod_activityH14.at(2), mod_activityH14.at(3), mod_activityH14.at(4), mod_activityH14.at(5));

        //*******VRN end***************

        /*******************************************************************************
         *  MODULE Motor neurons
         *******************************************************************************/

        // if (outputI1 > 0.9 || outputI2 > 0.9)
        // {
        //     /// Neural Control II: VRN implementation, for students ///
        //     // M0
        //     y_[TR0_m] = WeightM0_H13 * outputH13; // outputH2 goes through VRN_right and output outputH13
        //     // M1
        //     y_[TR1_m] = WeightM1_H13 * outputH13; // outputH2 goes through VRN_right and output outputH13
        //     // M2
        //     y_[TR2_m] = WeightM2_H13 * outputH13; // outputH2 goes through VRN_right and output outputH13

        //     // M3
        //     y_[TL0_m] = WeightM3_H14 * outputH14; // outputH2 goes through VRN_left and output outputH14
        //     // M4
        //     y_[TL1_m] = WeightM4_H14 * outputH14; // outputH2 goes through VRN_left and output outputH14
        //     // M5
        //     y_[TL2_m] = WeightM5_H14 * outputH14; // outputH2 goes through VRN_left and output outputH14

        //     // M6
        //     y_[CR0_m] = WeightM6_H1 * outputH1 + 0.5;
        //     // M7
        //     y_[CR1_m] = WeightM7_H1 * outputH1 + 0.5;
        //     // M8
        //     y_[CR2_m] = WeightM8_H1 * outputH1 + 0.5;

        //     // M9
        //     y_[CL0_m] = WeightM9_H1 * outputH1 + 0.5;
        //     // M10
        //     y_[CL1_m] = WeightM10_H1 * outputH1 + 0.5;
        //     // M11
        //     y_[CL2_m] = WeightM11_H1 * outputH1 + 0.5;

        //     // M12
        //     y_[FR0_m] = WeightM12_H1 * outputH1 - 0.9;
        //     // M13
        //     y_[FR1_m] = WeightM13_H1 * outputH1 - 0.9;
        //     // M14
        //     y_[FR2_m] = WeightM14_H1 * outputH1 - 0.9;

        //     // M15
        //     y_[FL0_m] = WeightM15_H1 * outputH1 - 0.9;
        //     // M16
        //     y_[FL1_m] = WeightM16_H1 * outputH1 - 0.9;
        //     // M17
        //     y_[FL2_m] = WeightM17_H1 * outputH1 - 0.9;
        // }
        // else
        // {
        //             /// Neural Control VI: multiple decoupled CPGs with phase modulation or resetting implementation, for students ///
        // #if defined(PhaseModulation) || defined(PhaseResetting)
        //             // M0
        //             y_[TR0_m] = 0.5 * CPGs_outputH2.at(0); // The second output of the first CPG
        //             // M1
        //             y_[TR1_m] = 0.5 * CPGs_outputH2.at(1); // The second output of the second CPG
        //             // M2
        //             y_[TR2_m] = 0.5 * CPGs_outputH2.at(2); // The second output of the thrid CPG

        //             // M3
        //             y_[TL0_m] = 0.5 * CPGs_outputH2.at(3);
        //             // M4
        //             y_[TL1_m] = 0.5 * CPGs_outputH2.at(4);
        //             // M5
        //             y_[TL2_m] = 0.5 * CPGs_outputH2.at(5);

        //             // M6
        //             y_[CR0_m] = 0.2 * CPGs_outputH1.at(0) + 0.5;
        //             // M7
        //             y_[CR1_m] = 0.2 * CPGs_outputH1.at(1) + 0.5;
        //             // M8
        //             y_[CR2_m] = 0.2 * CPGs_outputH1.at(2) + 0.5;

        //             // M9
        //             y_[CL0_m] = 0.2 * CPGs_outputH1.at(3) + 0.5;
        //             // M10
        //             y_[CL1_m] = 0.2 * CPGs_outputH1.at(4) + 0.5;
        //             // M11
        //             y_[CL2_m] = 0.2 * CPGs_outputH1.at(5) + 0.5;

        //             // M12
        //             y_[FR0_m] = 0.2 * CPGs_outputH1.at(0) - 0.9;
        //             // M13
        //             y_[FR1_m] = 0.2 * CPGs_outputH1.at(1) - 0.9;
        //             // M14
        //             y_[FR2_m] = 0.2 * CPGs_outputH1.at(2) - 0.9;

        //             // M15
        //             y_[FL0_m] = 0.2 * CPGs_outputH1.at(3) - 0.9;
        //             // M16
        //             y_[FL1_m] = 0.2 * CPGs_outputH1.at(4) - 0.9;
        //             // M17
        //             y_[FL2_m] = 0.2 * CPGs_outputH1.at(5) - 0.9;

        // #endif

#if defined(PhaseModulation) || defined(PhaseResetting)
        // M0
        y_[TR0_m] = 0.5 * mod_outputH13.at(0); // The second output of the first CPG
        // M1
        y_[TR1_m] = 0.5 * mod_outputH13.at(1); // The second output of the second CPG
        // M2
        y_[TR2_m] = 0.5 * mod_outputH13.at(2); // The second output of the thrid CPG

        // M3
        y_[TL0_m] = 0.5 * mod_outputH14.at(3);
        // M4
        y_[TL1_m] = 0.5 * mod_outputH14.at(4);
        // M5
        y_[TL2_m] = 0.5 * mod_outputH14.at(5);

        // M6
        y_[CR0_m] = 0.2 * CPGs_outputH1.at(0) + 0.5;
        // M7
        y_[CR1_m] = 0.2 * CPGs_outputH1.at(1) + 0.5;
        // M8
        y_[CR2_m] = 0.2 * CPGs_outputH1.at(2) + 0.5;

        // M9
        y_[CL0_m] = 0.2 * CPGs_outputH1.at(3) + 0.5;
        // M10
        y_[CL1_m] = 0.2 * CPGs_outputH1.at(4) + 0.5;
        // M11
        y_[CL2_m] = 0.2 * CPGs_outputH1.at(5) + 0.5;

        // M12
        y_[FR0_m] = 0.2 * CPGs_outputH1.at(0) - 0.9;
        // M13
        y_[FR1_m] = 0.2 * CPGs_outputH1.at(1) - 0.9;
        // M14
        y_[FR2_m] = 0.2 * CPGs_outputH1.at(2) - 0.9;

        // M15
        y_[FL0_m] = 0.2 * CPGs_outputH1.at(3) - 0.9;
        // M16
        y_[FL1_m] = 0.2 * CPGs_outputH1.at(4) - 0.9;
        // M17
        y_[FL2_m] = 0.2 * CPGs_outputH1.at(5) - 0.9;

#endif
        // }

        // backbone joint
        y_[BJ_m] = 0;

        // update step counter
        t++;

        saveFile1 << y_[TR0_m] << "  " << y_[TR1_m] << "  " << y_[TR2_m] << "  " << y_[TL0_m] << "  " << y_[TL1_m] << "  " << y_[TL2_m] << "  "
                  << y_[CR0_m] << "  " << y_[CR1_m] << "  " << y_[CR2_m] << "  " << y_[CL0_m] << "  " << y_[CL1_m] << "  " << y_[CL2_m] << "  "
                  << y_[FR0_m] << "  " << y_[FR1_m] << "  " << y_[FR2_m] << "  " << y_[FL0_m] << "  " << y_[FL1_m] << "  " << y_[FL2_m] << "  "
                                                                                                                                           "   \n"
                  << flush; // SAVE DATA
        saveFile2 << outputH1 << "  " << outputH2 << "  " << outputH13 << " " << outputH14 << "   \n"
                  << flush; // SAVE DATA
        saveFile3 << outputI1 << " " << outputI2 << " " << outputI1_final << " " << outputI2_final << " " << outputI1_ico << " " << outputI2_ico << "   \n"
                  << flush; // SAVE DATA
        saveFile4 << reflexive_signal << " " << predictive_signal << " " << u_ico << " " << deri_xt_reflex_angle << " "
                  << w_ico << " " << exp_decay << "   \n"
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

    /*************************************************************
     *  Gaussian random variable: just a sum of uniform distribution
     *  Average = 0, Variance = 1, (1.2)
     *************************************************************/
    virtual double gauss()
    {
        double sum;
        int i;

        for (sum = i = 0; i < 12; i++)
            sum += (1.0 * (double)rand() / (RAND_MAX));
        return (sum - 6.0);
    }

protected:
    unsigned short number_channels;

    int t;
    paramkey name;

    /// Neural Control I: CPG implementation, for students ///

    double alph;
    double phi;
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

    // Adaptive CPG//
    double mu;
    double wd;
    double tt;
    double activityH3_;
    double WeightH1_H3_;
    double WeightH1_H3_t0;
    double outputH3_;
    double WeightH3__H1;
    double WeightH3__H1_t0;
    double Weight2F;
    double Weight2F_t0;
    double F;
    double A20;
    double A02;
    double A2F;

    double B20;
    double B02;
    double B2F;
    double intrinsicFreq;

    /// Neural Control II: VRN implementation, for students ///

    double activityH3;
    double activityH4;

    double activityH5;
    double activityH6;
    double activityH7;
    double activityH8;

    double activityH9;
    double activityH10;
    double activityH11;
    double activityH12;

    double activityH13;
    double activityH14;

    double outputH3;
    double outputH4;
    double outputH5;
    double outputH6;
    double outputH7;
    double outputH8;

    double outputH9;
    double outputH10;
    double outputH11;
    double outputH12;
    double outputH13;
    double outputH14;

    double BiasH5;
    double BiasH6;
    double BiasH7;
    double BiasH8;

    double BiasH9;
    double BiasH10;
    double BiasH11;
    double BiasH12;

    double WeightH5_H2;
    double WeightH6_H2;
    double WeightH7_H2;
    double WeightH8_H2;

    double WeightH9_H2;
    double WeightH10_H2;
    double WeightH11_H2;
    double WeightH12_H2;

    double WeightH5_H3;
    double WeightH6_H3;
    double WeightH7_H3;
    double WeightH8_H3;

    double WeightH9_H3;
    double WeightH10_H3;
    double WeightH11_H3;
    double WeightH12_H3;

    double WeightH5_H4;
    double WeightH6_H4;
    double WeightH7_H4;
    double WeightH8_H4;

    double WeightH9_H4;
    double WeightH10_H4;
    double WeightH11_H4;
    double WeightH12_H4;

    double WeightH13_H5;
    double WeightH13_H6;
    double WeightH13_H7;
    double WeightH13_H8;

    double WeightH14_H9;
    double WeightH14_H10;
    double WeightH14_H11;
    double WeightH14_H12;

    double Input1;
    double Input2;

    double WeightM0_H13;
    double WeightM1_H13;
    double WeightM2_H13;

    double WeightM3_H14;
    double WeightM4_H14;
    double WeightM5_H14;

    std::vector<double> mod_activityH5;
    std::vector<double> mod_activityH6;
    std::vector<double> mod_activityH7;
    std::vector<double> mod_activityH8;
    std::vector<double> mod_activityH13;

    std::vector<double> mod_outputH5;
    std::vector<double> mod_outputH6;
    std::vector<double> mod_outputH7;
    std::vector<double> mod_outputH8;
    std::vector<double> mod_outputH13;

    std::vector<double> mod_activityH9;
    std::vector<double> mod_activityH10;
    std::vector<double> mod_activityH11;
    std::vector<double> mod_activityH12;
    std::vector<double> mod_activityH14;

    std::vector<double> mod_outputH9;
    std::vector<double> mod_outputH10;
    std::vector<double> mod_outputH11;
    std::vector<double> mod_outputH12;
    std::vector<double> mod_outputH14;

    /// Neural Control III: Neural preprocessing implementation, for students ///

    double activityI1;
    double activityI2;
    double outputI1;
    double outputI2;
    double outputI1_final;
    double outputI2_final;

    double WeightI1_I1;
    double WeightI2_I2;
    double WeightI1_I2;
    double WeightI2_I1;

    double WeightI1_IR1;
    double WeightI2_IR2;

    double IR1_mapping;
    double IR2_mapping;

    /// Neural Learning and Adaptation I: ICO implementation, for students ///
    double predictive_signal;
    double distance;
    double alpha_tmp;
    double alpha;
    double deri_alpha;
    double input_angle_s;
    double input_distance_s;
    double xt_reflex_angle;
    double u_ico;
    double w_ico;
    double reflexive_signal;
    double deri_xt_reflex_angle;
    double reflex_old;
    double u_ico_out;
    double outputI1_ico;
    double outputI2_ico;
    int reset;

    /// Neural control: Multiple decoupled CPGs with phase modulation or phase resetting, for students ///
#if defined(PhaseModulation)
    unsigned int leg_num;
    std::vector<double> CPGs_outputH1;
    std::vector<double> CPGs_outputH2;
    std::vector<double> CPGs_activityH1;
    std::vector<double> CPGs_activityH2;
    std::vector<double> gamma1;
    std::vector<double> gamma2;
    std::vector<double> GRF;
    std::vector<double> fc;
    std::vector<double> fs;
#endif
#if defined(PhaseResetting)
    unsigned int leg_num;
    std::vector<double> CPGs_outputH1;
    std::vector<double> CPGs_outputH2;
    std::vector<double> CPGs_activityH1;
    std::vector<double> CPGs_activityH2;
    std::vector<double> GRF;
    std::vector<double> GRF_old;
    std::vector<double> resetting1;
    std::vector<double> resetting2;
    unsigned int Diracs;
    double GRF_threshold;

#endif

    //----Exploration noise---------------//

    double exp_output;
    double exploration_g;
    double exploration_lowpass_g;
    double exploration_lowpass_old_g;
    double exp_decay;

    // --- Save text------------//
    ofstream saveFile1;
    ofstream saveFile2;
    ofstream saveFile3;
    ofstream saveFile4;
    //-------------------------//
};

#endif

/*
 * List of available Sensors and their numbers/enum's to be used for example as x_[TR0_as] in the step function
 *
// Angle sensors (for actoric-sensor board (new board))
TR0_as=0, //Thoracic joint of right front leg
TR1_as=1, //Thoracic joint of right middle leg
TR2_as=2, //Thoracic joint of right hind leg

TL0_as=3, //Thoracic joint of left front leg
TL1_as=4, //Thoracic joint of left middle leg
TL2_as=5, //Thoracic joint of left hind leg

CR0_as=6, //Coxa joint of right front leg
CR1_as=7, //Coxa joint of right middle leg
CR2_as=8, //Coxa joint of right hind leg

CL0_as=9,  //Coxa joint of left hind leg
CL1_as=10, //Coxa joint of left hind leg
CL2_as=11, //Coxa joint of left hind leg

FR0_as=12, //Fibula joint of right front leg
FR1_as=13, //Fibula joint of right middle leg
FR2_as=14, //Fibula joint of right hind leg

FL0_as=15, //Fibula joint of left front leg
FL1_as=16, //Fibula joint of left middle leg
FL2_as=17, //Fibula joint of left hind leg

BJ_as= 18, //Backbone joint angle

//Foot contact sensors (AMOSII v1 and v2)
R0_fs= 19, //Right front foot
R1_fs= 20, //Right middle foot
R2_fs= 21, //Right hind foot
L0_fs= 22, //Left front foot
L1_fs= 23, //Left middle foot
L2_fs= 24, //Left hind foot

// US sensors (AMOSII v1 and v2)
FR_us=25, //Front Ultrasonic sensor (right)
FL_us=26, //Front Ultrasonic sensor (left)

// IR reflex sensors at legs (AMOSIIv2)
R0_irs=31,
R1_irs=29,
R2_irs=27,
L0_irs=32,
L1_irs=30,
L2_irs=28,

// goal orientation sensors (relative position to reference object 1 (red), e.g. camera)
G0x_s=62,
G0y_s=63,
G0z_s=64,

//Body speed sensors (only simulation)
BX_spd= 66,
BY_spd= 67,
BZ_spd= 68,

// goal orientation sensors (relative position to reference object 2 (green), e.g. camera)
G1x_s=72,
G1y_s=73,
G1z_s=74,

// goal orientation sensors (relative position to reference object 3 (blue), e.g. camera)
G2x_s=78,
G2y_s=79,
G2z_s=80,

*/
