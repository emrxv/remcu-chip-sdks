/**************************************************************************//**
 * @file em_device.h
 * @brief CMSIS Cortex-M Peripheral Access Layer for Silicon Laboratories
 *        microcontroller devices
 *
 * This is a convenience header file for defining the part number on the
 * build command line, instead of specifying the part specific header file.
 *
 * @verbatim
 * Example: Add "-DEFM32G890F128" to your build options, to define part
 *          Add "#include "em_device.h" to your source files
 *
 *
 * @endverbatim
 * @version 5.1.2
 ******************************************************************************
 * @section License
 * <b>Copyright 2017 Silicon Laboratories, Inc. http://www.silabs.com</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of any
 * kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties against
 * infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 *****************************************************************************/

#ifndef EM_DEVICE_H
#define EM_DEVICE_H

#if defined(EFR32FG1V032F128GM32)
#include "efr32fg1v032f128gm32.h"

#elif defined(EFR32FG1V032F256GM32)
#include "efr32fg1v032f256gm32.h"

#elif defined(EFR32FG1V131F128GM32)
#include "efr32fg1v131f128gm32.h"

#elif defined(EFR32FG1V131F128GM48)
#include "efr32fg1v131f128gm48.h"

#elif defined(EFR32FG1V131F256GM32)
#include "efr32fg1v131f256gm32.h"

#elif defined(EFR32FG1V131F256GM48)
#include "efr32fg1v131f256gm48.h"

#elif defined(EFR32FG1V131F32GM32)
#include "efr32fg1v131f32gm32.h"

#elif defined(EFR32FG1V131F32GM48)
#include "efr32fg1v131f32gm48.h"

#elif defined(EFR32FG1V131F64GM32)
#include "efr32fg1v131f64gm32.h"

#elif defined(EFR32FG1V131F64GM48)
#include "efr32fg1v131f64gm48.h"

#elif defined(EFR32FG1V132F128GM32)
#include "efr32fg1v132f128gm32.h"

#elif defined(EFR32FG1V132F128GM48)
#include "efr32fg1v132f128gm48.h"

#elif defined(EFR32FG1V132F256GM32)
#include "efr32fg1v132f256gm32.h"

#elif defined(EFR32FG1V132F256GM48)
#include "efr32fg1v132f256gm48.h"

#elif defined(EFR32FG1V132F32GM32)
#include "efr32fg1v132f32gm32.h"

#elif defined(EFR32FG1V132F32GM48)
#include "efr32fg1v132f32gm48.h"

#elif defined(EFR32FG1V132F64GM32)
#include "efr32fg1v132f64gm32.h"

#elif defined(EFR32FG1V132F64GM48)
#include "efr32fg1v132f64gm48.h"

#else
#error "em_device.h: PART NUMBER undefined"
#endif
#endif /* EM_DEVICE_H */
