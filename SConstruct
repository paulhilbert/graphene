import os

def CleanFlagIsSet() :
	import SCons.Script.Main
	return SCons.Script.Main.GetOption('clean')

def LaunchedFromTopDir() :
	return DefaultEnvironment().GetLaunchDir() == Dir('#')

def CleanAction(name,action) :
	if CleanFlagIsSet() :
		if len(COMMAND_LINE_TARGETS) == 0 or name in COMMAND_LINE_TARGETS or LaunchedFromTopDir() and '.' in COMMAND_LINE_TARGETS :
			Execute(action)



env = Environment(ENV = {'PATH' : os.environ['PATH'],
                         'TERM' : os.environ['TERM'],
                         'HOME' : os.environ['HOME']})

commons = os.environ['COMMONS']

env['CCFLAGS'] = ['-g', '-Wall', '-O2', '-std=c++11', '-fPIC', '-Wno-unused-local-typedefs']#, '-D OPENGL_EFFECTS']
env['CPPPATH'] = ['.', commons,'/usr/include/eigen3']
env['LIBS']    = ['dl', 'GL', 'GLU', 'GLEW', 'boost_regex', 'boost_filesystem', 'boost_program_options', 'boost_system']

obj = [Glob('GUI/*.cpp'), Glob('GUI/Property/*.cpp'), Glob('GUI/Mode/*.cpp'), Glob('FW/*.cpp'), Glob('FW/View/*.cpp'), Glob('FW/Events/*.cpp')]
#cmns = [Glob(commons+'/IO/*.cpp'), commons+'/Random/RNG.cpp']
lib = [Glob('Library/Buffer/*.cpp'), Glob('Library/Rendered/*.cpp'), Glob('Library/Shader/*.cpp')]

# library
env.Library("graphene", obj+lib)
# executable
env['LIBS'] += ['graphene']
env['LIBPATH'] = ['.']
env.Program("graphene", ["graphene.cpp"])

# documentation
if 'doc' in COMMAND_LINE_TARGETS:
	Execute('rm -rf doc');
	env.Command ('doc/external', '', 'doxygen DoxExternal')
	env.Command ('doc/internal', '', 'doxygen DoxInternal')

CleanAction('doc', Action(['rm -rf doc']))
