import os

def setup_build_env(env):
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

    env.LoadProject(project_kconfig_prefices)

    env.Append(CCFLAGS='-save-temps=obj')
    env.SConscript('SConscript', variant_dir=global_env['VARIANT_DIR'],
                   duplicate=0)
    env.ProjectSConscript(project_kconfig_prefices)

    env.ComponentProgram('#$VARIANT_DIR/firmware.elf')


global_env = Environment(CROSS_COMPILE='arm-none-eabi-',
                         CCFLAGS_OPT='-Os',
                         VARIANT_DIR='build')
Export('global_env')

global_env.Tool('sbbs', toolpath=[os.path.join('..', '..', 'sbbs/')])
global_env.LoadBuildEnv(setup_build_env)
