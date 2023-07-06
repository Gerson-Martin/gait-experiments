#ifndef FCONTROL_H
#define FCONTROL_H


/*! \mainpage fcontrol
 *
 * This project is a control library.
 * Currently two types of blocks can be used:
 * - \subpage SystemBlock "z transfer function block"
 *
 * - \subpage PIDBlock "pid controller block"
 *
 */ 

/*! \page SystemBlock z transfer function block
 *
 * z transfer function block
 *
 */

/*! \page PIDBlock pid controller block
 *
 * This page is about how program pid.
 * Following sections describe parameters:
 * - \ref P "Proportional parameter"
 * - \ref I "Integral param"
 */


#include "SystemBlock.h"
#include "FSystemBlock.h"
#include "FactorSystemBlock.h"
#include "ControllerBlock.h"
#include "TransferFunction.h"
#include "PIDBlock.h"
#include "StateVariable.h"
#include "FractionalController1DOF.h"
#include "FractionalDerivative.h"
#include "OnlineSystemIdentification.h"
#include "ToolsFControl.h"
#include "FPDBlock.h"
#include "FPDTuner.h"
#include "LinearSystem.h"
#include "StateSpace.h"
//#include "StateObserver.h"



#endif // FCONTROL_H

