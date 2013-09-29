Graphene Visualization Framework
==========

Welcome to graphene, a modular visualization framework developed as a private project
by two student researchers at the University of Bonn primarily to aid algorithm development
by providing fast and adaptable visualization of algorithm output.


Target Audience
==========

First of all: The primary target audience for graphene is us (i.e. the developers).
However there is a secondary target audience - and that is you, if

- you somehow work in the field of geometry processing and need a fast and adaptable
way to visualize your work

- you prefer fast compile times and documentation-by-example (or code audit) over large,
slow and intrusive frameworks like ogre3d, xgrt or coin3d (these have partly been the reason we initially wrote graphene)

- you work under linux systems (actually we compiled and successfully ran it under windows, but we neither have
nor will test it there and don't expect compiling graphene to be easy (see Windows section below for some hints and rants)
Our opinion regarding Windows and especially Visual Studio is nothing we are able to talk about politely - therefore don't
expect much of us regarding that area)

- you don't expect to hack graphene yourself and expect instabilities and compatibility breaking commits. Graphene is not
intended for the masses and probably never will - we modify/extend/destroy it almost every day which obviously doesn't always have benefits.

- you love (and we don't mean tolerate, or know, or have heard of, but *love*) the functional programming aspects of c++ and especially
c++11/c++14. Graphene is intended to be used with lambda expressions a lot (and it heavily relies on automatic type derivation, initializer lists and variadic templates).
A typical line looks like

auto btn = gui()->properties()->add<Button>("Label");
btn->setCallback([&]() { gui()->log()->info("Button pressed"); });

That's it. Consider yourself warned.


Framework Structure
==========

The graphene framework is divided into four major parts of which (probably only one) is written by yourself:

1. The Core (graphene main application and library). Though this constitutes the core of the framework it is merely a large chunk of interface specifications and should (if at all) concern you only if you need to look up some functions.

2. The Backend. One modularity aspect of graphene is that it uses a plugin mechanism for anything GUI related. The backend is a huge blob of code responsible for drawing the entire GUI whilst fulfilling the interface specifications given by the graphene core.
While we supply a Qt5 backend, anyone may write other backends (how about a CLI backend or an android implementation?).

3. The Library. This is simply a folder in the graphene root containing some useful helper classes. Use them.

4. The visualizers. Graphene loads visualizers using a plugin mechanism on startup. When writing visualizers you preferably create them using the bash script createVisualizer in bin/ using the skeleton visualizer you'll find in the repository
https://github.com/paulhilbert/visualizer - this gives you a ready-to-compile setup in order to compile a new visualizer as a shared library that is loader by graphene on startup.


Guess what: Number 4 is where you'll spend most probably all of your time coding.


Dependencies
==========

There are a few dependencies in order to compile graphene and own visualizers, namely

- C++11 support (the list of functionaly of c++11 we use is too long to put here, or in other words: Use gcc 4.7 or higher, newer clang versions or visual studio 2013)

- Boost (both)

- Boost Extension (both; this is an unofficial addition to Boost we use for the plugin API)

- Eigen 3 (both; you'll really want to use Eigen for Linear Algebra - we certainly do)

- Qt5 (graphene; this repository contains under Backends/Qt5 a reference backend implementation using Qt5 - you
can always try to implement your own backend using another toolkit; we initially used Gtk2/3 so that is certainly possible, but we currently have no code for that)

- commons (this is another repo here on github, but it is linked as an external repository, so if you update the submodules in this repo you will automagically get commons)


Compiling Graphene
==========

(look at the end of this section for a screencast of a prototypical build)

This section is meant for linux systems and we have tested this only on Arch Linux distributions. For some hints regarding Windows see the corresponding section below.

First of all clone the graphene repository and commons using

% git clone https://github.com/paulhilbert/graphene

% cd graphene

% git submodule update --init --recursive

It is recommended to also checkout the visualizer repository you'll find at
https://github.com/paulhilbert/visualizer and then change your environment variables. More specifically

- set both $GRAPHENE_ROOT to the graphene root directory and $GRAPHENE_VIS to the visualizer root directory

- and add $GRAPHENE_ROOT/bin to your $PATH (this one is really not important, but it will ease the use of the createVisualizer bash script)

Finally cd to the graphene root and if all dependencies and your environment variables are set up it should suffice to execute

% ./build.sh

And here is everything in video form:

http://sicgraph.org/graphene_install.ogv


Testing Graphene
==========

In order to test graphene, do

% cd "$GRAPHENE_VIS/examples/Rendering"

% scons

% cd "$GRAPHENE_VIS"

% ln -s examples/Rendering/visRendering.so

% cd "$GRAPHENE_ROOT"

% ./graphene --visPath "$GRAPHENE_VIS" --visExt ".so" --backend Backends/libbackendQt5.so

If this works consider putting these parameters into a "graphene.conf" file using a param=value per line syntax.



Using Graphene
==========

To be written...
Actually there will probably be a screencast linked here.



Windows
==========

To be written...
