import os
env = Environment(ENV = {'PATH' : os.environ['PATH'],
                         'TERM' : os.environ['TERM'],
                         'HOME' : os.environ['HOME']})
env['CCFLAGS'] = ['-g', '-Wall', '-O2', '-std=c++11', '-fPIC', '-Wno-unused-local-typedefs', '-D ENABLE_SCREENCAST']
env['CPPPATH'] = ['.','/usr/local/include/commons','/usr/include/eigen3']
env['LIBPATH'] = ['/usr/local/lib/commons']
env['LIBS']    = ['dl', 'IO', 'boost_filesystem', 'boost_program_options', 'boost_system', 'GL', 'GLU', 'boost_regex']

obj = [Glob('GUI/*.cpp'), Glob('GUI/Property/*.cpp'), Glob('GUI/Mode/*.cpp'), Glob('FW/*.cpp'), Glob('FW/View/*.cpp'), Glob('FW/Events/*.cpp')]
lib = [Glob('Library/Buffer/*.cpp'), Glob('Library/Rendered/*.cpp'), Glob('Library/Shader/*.cpp')]

env.Program("graphene", ["graphene.cpp"]+obj)
env.Library("graphene", obj+lib)
