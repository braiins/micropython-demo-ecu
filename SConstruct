import os

global_env = Environment(CROSS_COMPILE='arm-none-eabi-',
                         CCFLAGS_OPT='-Os',
                         VARIANT_DIR='build-X')
Export('global_env')

global_env.Tool('sbbs', toolpath=[os.path.join('..', '..', 'sbbs/')])
global_env.LoadConfig()

global_env.LoadProject(kconfig_prefix='FREERTOS')
global_env.LoadProject(kconfig_prefix='FREERTOS_DRIVERS')
global_env.LoadProject(kconfig_prefix='LIB_RTOS')
global_env.LoadProject(kconfig_prefix='LIB_ALGORITHMS')
global_env.LoadProject(kconfig_prefix='LIB_DAQ')
global_env.LoadProject(kconfig_prefix='LIB_IC')
global_env.LoadProject(kconfig_prefix='LIB_NMEA')
global_env.LoadProject(kconfig_prefix='LIB_RC')
global_env.LoadProject(kconfig_prefix='LIB_COBJS')
global_env.LoadProject(kconfig_prefix='BSP')
global_env.LoadProject(kconfig_prefix='MICROPYTHON')
global_env.LoadProject(kconfig_prefix='MICROPYTHON_FREERTOS_HAL')


global_env.Append(CCFLAGS='-save-temps=obj')
global_env.SConscript('SConscript', variant_dir=global_env['VARIANT_DIR'],
                      duplicate=0)

project_kconfig_prefices = ['FREERTOS',
                            'FREERTOS_DRIVERS',
                            'LIB_RTOS',
                            'LIB_ALGORITHMS',
                            'LIB_DAQ',
                            'LIB_IC',
                            'LIB_NMEA',
                            'LIB_RC',
                            'LIB_COBJS',
                            'BSP',
                            'MICROPYTHON_FREERTOS_HAL',
                            'MICROPYTHON']

global_env.ProjectSConscript(project_kconfig_prefices)


global_env.ComponentProgram('#$VARIANT_DIR/firmware.elf')

#global_env.Program('firmware.elf', global_env['SBBS_BUILTINS'])

#print global_env
#print global_env.Dump()

