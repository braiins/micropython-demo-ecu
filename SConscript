import os

Import('global_env')

env = global_env.Clone()
Export('env')


#env.QstrHeader('#myqstr.h')

env.FeatureObject(source=['main.c',
                          'mptask.c',
                          'rpm.c',
                          'frozen_scripts.c'])

env.QstrFeatureObject(source=['app.c'])


board_path = os.path.join('board', env['CONFIG'].BSP_TARGET)
env.Append(CPPPATH=os.path.realpath(board_path))

env.FeatureSConscript(dirs=[board_path])

env.FrozenScripts(dir=['scripts'])

env.BuiltInObject(global_env)
