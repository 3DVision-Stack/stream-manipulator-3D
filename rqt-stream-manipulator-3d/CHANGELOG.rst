^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_stream_manipulator_3d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2016-08-04)
------------------
* Updates & Bugfixes
  Bugfix: CropBox not updating Translation properly
  Add Pause/Resume functionality:
  - Pause button will block processing and all plugins, it also
  unsubscribe from input_topic.
  - Resume will resubscribe and resume all normal functionality.
  - While stream_manipulator_3d is paused you cannot modify configuration
  or plugin specific settings
  Add Saving/Loading configuration functionality:
  - Saving config to rosparams and then dumpng to yaml file
  - Loading yaml file into rosparams, then update configuration from them.
* Add two more filter plugins
  * Add Statistical Outlier Removal Plugin
  Wrapping  of  pcl::StatisticalOutlierRemoval  filter.  StatisticalOutlierRemoval
  uses  point  neighborhood  statistics  to filter  outlier  data.  The  algorithm
  iterates through  the entire  input twice:  During the  first iteration  it will
  compute the average distance that each point has to its nearest k neighbors. The
  value of k  can be set using  nr_k_neighbors param. Next, the  mean and standard
  deviation of all  these distances are computed in order  to determine a distance
  threshold. The distance threshold will be equal to: mean + stddev_mult * stddev.
  The multiplier  for the  standard deviation can  be set  using stddev_multiplier
  parameter. During the next iteration the  points will be classified as inlier or
  outlier if  their average  neighbor distance  is below  or above  this threshold
  respectively. Outliers will then be removed from the stream. The neighbors found
  for each query point will be found amongst ALL input points.
  * Add Radius Outlier Removal Plugin
  Wrapping  of  pcl::RadiusOutlierRemoval   filter.  RadiusOutlierRemoval  filters
  points in a cloud  based on the number of neighbors  they have. Iterates through
  the entire  input once, and  for each point,  retrieves the number  of neighbors
  within a certain radius.  The point will be considered an outlier  if it has too
  few neighbors, as determined by  k_neighbors_threshold parameter. The radius can
  be changed using  radius_search param. The neighbors found for  each query point
  will be found amongst ALL points of input stream.
* Contributors: Federico Spinelli

0.1.6 (2016-07-29)
------------------
* Add some more Plugins:
  * Frustum Culling Filter Plugin
  Removes everything inside/outside a frustum,  defined by two aperture angles and
  two parallel planes.
  * Median Filter Plugin
  The  median filter  is  one of  the simplest  and  wide-spread image  processing
  filters. It is known to perform  well with "shot"/impulse noise (some individual
  pixels having extreme  values), it does not reduce contrast  across steps in the
  function  (as compared  to filters  based  on averaging),  and it  is robust  to
  outliers. Furthermore, it is simple to implement and efficient, as it requires a
  single pass over  the image. It consists  of a moving window of  fixed size that
  replaces the pixel in the center with the median inside the window.
  NOTE: This filters  only the depth (z-component) of  organized and untransformed
  (i.e., in  camera coordinates) point  clouds. An error  will be outputted  if an
  unorganized cloud is given.
  * PassThrough Filter Plugin
  PassThrough passes  points in a  cloud based  on constraints for  one particular
  field of the  point type. Iterates through the entire  input once, automatically
  filtering non-finite points and the points outside the interval specified by the
  filter limits, which applies only to the field specified by field type.
* Contributors: Federico Spinelli

0.1.4 (2016-07-23)
------------------
* Fix build errors
  Add proper catkin dependencies
* Contributors: Federico Spinelli

0.1.2 (22-07-2016)
------------------
  * Fix VoxelGrid, now works properly:
  Trick was to instantiate pcl class from scratch into apply() method
  * Add delay timer measuring total time spent during processing:
  sm3d GUI now shows this value as an user indication that he's
  pulling in too many plugins.
  * Delay gui progressively colors from green to red according to the
  delay it shows.
  * Actual shown delay is a moving average of all instant total delays
  measured. Moving average window size is hardcoded to 20
  * Fix some quirks in plugins gui, like allignments.

0.1.1 (20-07-2016)
------------------
  * Add Plugn Chain moving:
  New chain elements get copied from old chain if they were present. This preservs
  user  set  parameters and  configuration.  As  a  result, rearrangments  can  be
  executed  easily with  Plugins params  gettin preserved.  New Plugins  are still
  created from scratch. To  achieve this, a copy of old  chain description must be
  kept in StreamManipulator Class.
  * Add Voxel Grid Plugin:
  NEW  Plugin that  wraps pcl::VoxelGrid,  i.e.  downsample the  stream with  user
  specified parameters.
  * Change process() and apply() layouts:
  Drop the use of shared_ptr on output, cause that was confusing VoxelGrid Plugin,
  outputting an  empty cloud.  Not sure  why, thought,  it may  be dependant  of a
  particular PCL version (1.7.1).
  * Remove condition variable on chain changes:
  Not a real need for it, Reconfiguration can be accomplished simply by
  looking at a boolean, hence Condition Variable was probably overkill.

0.1.0 (04-07-2016)
------------------
  * stream_manipulator_3d package (sm3d)
    - Main ROS spinner process: configuration checking/updating,
    marker publishing, stream chain assembling.
    - Processing async Thread: reads chain and applies filters(plugins)
    one by one as fast as possible.
    - Async threaded ROS Subscriber: provides input to the chain and notifies
    processing thread
  * rqt_stream_manipulator_3d package (rqt_sm3d)
    - Rqt plugin to configure sm3d in real time
    - Provides GUI through ui files
    - Lets the user interact with the chain of Plugins:
    erasing,creating,modifying it.
    - A list of all available plugins is displayed for user convenience.
    - sm3d Subscriber can also be changed in realtime from here.
  * PluginLib Infrastructure for sm3d
    - Each filter/output/application is a dynamic plugin to sm3d::Plugin
    base class.
    - Plugins have their own configuration and namespace, to allow
    multiple instances of the same plugin to coexist
    - Plugins are loaded/unloaded at runtime based on configuration of
    stream_manipulator_3d node.
    - Plugins chained togheter form the stream chain
  * PluginLib Infrastructure for rqt_sm3d
    - Each Plugin  of sm3d is mirrored also a Plugin for rqt_sm3d.
    This allows the GUI to dynamically grow/shrink according to the
    current loaded chain.
    - Provide ui files for each Plugin.
    - Each Plugin interacts only with its counterpart in sm3d
  * Configuration is allocated is shared memory using Boost Interprocess
    - sm3d configuration lives in shared memory, rqt_sm3d reads/modifies
    it.
    - Each sm3d plugin creates a configuration object in shared memory
    when it is spawned. rqt_sm3d respective plugins read/modify them.
    - Rosparams are used to define an initial configuration, then they are
    not touched anymore. This is done intentionally for now, to remove
    ros master overhead of constantly querying/changing Rosparams.
    - Dynamic_reconfigure is intentionally unused, because its is not
    flexible enough for this kind of application. Particurarly GUI is to
    minimalistic/unconfigurable.
  * Plugins implemented so far
    - Cropbox filter: wrapping of PCL one, remove anything inside/outside a box
    - Ros Publisher: publish stream on any point of the chain

