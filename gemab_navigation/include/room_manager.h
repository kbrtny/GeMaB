#ifndef ROOM_MANAGER_H
#define ROOM_MANAGER_H

#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>
#include <iostream>

struct Point {
	float x;
	float y;
	float z;
};

struct RoomPolygon {
	int id;
	std::string currentRoomName;

	uint32_t numRoomVertex;
    std::vector <float> roomX;
    std::vector <float> roomY;
    
    std::vector <float> constant;
    std::vector <float> multiple;
};

struct Room {
	int id;
	std::string roomName;
	std::vector <Point> vertex;

	std::vector <int> entryIds;
	std::vector <int> objectIds;

	Room () {}
	Room(std::string name, std::vector <Point> vertices) : roomName(name), vertex(vertices)
	{
	}
};

struct Entry {
	int id;
	std::string entryName;
	std::vector <Point> vertex;
	int connectedRoomIds[2];
	Entry () {}
	Entry(std::string name, std::vector <Point> vertices) : entryName(name), vertex(vertices)
	{
	}
};

struct Objects {
	int id;
	std::string objectName;
	std::vector <std::string> properties;
	Objects () {}
	Objects(std::string name) : objectName(name)
	{
	}
};

class RoomManager
{
    public:
        RoomManager(){}
        ~RoomManager(){};
        Room CreateRoom(std::string, std::vector<Point>);
        Entry CreateEntry(std::string, std::vector<Point>, int rooms[]);
        Objects CreateObject(std::string, std::vector<std::string>);
        int AddRoom(Room room_to_add);
        int AttachEntry(Entry entry_to_add);
        int AddObject(Objects object_to_add, int room_id);
        void PrintRoom(int id);
        void PrintEntry(int id);
        void PrintObject(int id);
        int UpdateRoomName(int roomId, std::string new_name);
        //void LoadMap(const std::string& map_file);
        int GetCurrentRoomId(float x, float y);
        int GetInitialRoomId(float x, float y);
    private:
    	std::vector<Room> rooms;
    	std::vector<Entry> entries;
    	std::vector<Objects> objects;
    	std::string currentRoom;
    	int currentRoomid;
};



#endif