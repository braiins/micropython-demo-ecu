#include <stdint.h>
#include <stdbool.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "lib/utils/pyexec.h"
#include "lib/mp-readline/readline.h"

#include "micropython-freertos-hal/mp/gc_helper.h"
#include "micropython-freertos-hal/mp/mp_hal.h"
#include "micropython-freertos-hal/upy_hal.h"

/* initialize the garbage collector with the top of our stack */
extern void gc_collect_init(mp_uint_t sp);


void micropython_task_method(void *unused) {
    gc_collect_init(gc_helper__get_sp());

    // MicroPython init
    mp_init();
    mp_obj_list_init(mp_sys_path, 0);
    mp_obj_list_init(mp_sys_argv, 0);
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_)); // current dir (or base dir of the script)

    // execute all basic initializations
    readline_init0();
    mp_hal_init();
    upy_hal__init();

    pyexec_frozen_module("boot");
    // main script is finished, so now go into REPL mode.
    // the REPL mode can change, or it can request a soft reset.
    for ( ; ; ) {
        if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
            if (pyexec_raw_repl() != 0) {
                break;
            }
        } else {
            if (pyexec_friendly_repl() != 0) {
                break;
            }
        }
    }
}

