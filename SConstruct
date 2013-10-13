import os
env = Environment(ENV = {'PATH' : os.environ['PATH'],
                         'TERM' : os.environ['TERM'],
                         'HOME' : os.environ['HOME']})

commons = os.environ['COMMONS']

env['CCFLAGS'] = ['-g', '-Wall', '-O2', '-std=c++11', '-fPIC', '-Wno-unused-local-typedefs', '-D USE_BASH_COLORS']
env['CPPPATH'] = ['.', commons,'/usr/include/eigen3']
env['LIBS']    = ['dl', 'GL', 'GLU', 'boost_regex', 'boost_filesystem', 'boost_program_options', 'boost_system']

obj = [Glob('GUI/*.cpp'), Glob('GUI/Property/*.cpp'), Glob('GUI/Mode/*.cpp'), Glob('FW/*.cpp'), Glob('FW/View/*.cpp'), Glob('FW/Events/*.cpp')]
#cmns = [Glob(commons+'/IO/*.cpp'), commons+'/Random/RNG.cpp']
lib = [Glob('Library/Buffer/*.cpp'), Glob('Library/Rendered/*.cpp'), Glob('Library/Shader/*.cpp')]

# library
env.Library("graphene", obj+lib)
# executable
env['LIBS'] += ['graphene']
env['LIBPATH'] = ['.']
env.Program("graphene", ["graphene.cpp"])
