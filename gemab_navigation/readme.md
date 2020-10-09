This overall package is for all of the navigation activities for the GeMaB robot.  This includes some libraries to assist with navigation.

RoomManager:
This class is for housing a semantic map of a house.

It contains a vector of labeled rooms (defined by a vector of vertices that define the room polygon), a vector of entries (named with defined location and rooms each connects), and a vector of objects that have been identified around an environment.  These can easily be created and linked to each other in order to have a semantic map of a defined environment as well as its contents.

The 'room_detector.cpp' file gives some examples of how to create a map, export it to a json for persistance, and import it for use.  It can be compiled by running : 'make' from the gemab_navigation folder, then executing: 'build/apps/program' after compilation (on ubuntu 18, it should just work with the tools required for ROS).  The test.json file has a trivial example for demonstration/debugging purposes.

I have copied in the json.hpp file from https://github.com/nlohmann/json/releases/tag/v3.9.1.  I should probably have a submodule or something else rather than copying in and revision controlling the file.

Currently the usage in a ROS node is to import an existing json map and identify the name of the room you are in.  You can also create and add objects or other rooms/entries, but there is more work to do to enable more functionality.  Eventually I may port all of this to a mongodb instead of several vectors in a class, but it should be fine for small projects.