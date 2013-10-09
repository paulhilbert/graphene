Graphene Visualizer
==========

This repository contains example visualizer as well as the skeleton visualizer
for the graphene scientific visualization framework.

For more information regarding graphene take a look at
https://github.com/paulhilbert/graphene


Usage
=======

The main purpose for this repository is to provide the skeleton files used
by the createVisualizer bash script found in the graphene repository under
bin/ in order to spawn new custom visualizers.

For this purpose it is highly recommended to define environment variables
GRAPHENE_ROOT (which should point to the folder you cloned graphene to) and
GRAPHENE_VIS (which should point to the folder you cloned this repository to)
and add "$GRAPHENE_ROOT/bin" to your PATH.

An example scenario then looks like:

% cd $GRAPHENE_VIS

% createVisualizer VisualizerName ./

This creates a new visualizer named "VisualizerName", builds it and symlinks
it to ./ (which in this case is $GRAPHENE_VIS and should be the same folder
you point graphene's visPath command-line parameter to).

When starting graphene you should now see this new visualizer loaded (though
it has no functionality yet).

At this point you can modify VisualizerName/VisualizerName.{h,cpp} to your needs
using the example visualizers in the examples/ directory as inspiration.

That's it - have fun.
