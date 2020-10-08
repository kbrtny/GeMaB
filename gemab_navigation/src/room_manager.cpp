#include "../include/room_manager.h"


Room RoomManager::CreateRoom(std::string name, std::vector <Point> vertices)
{
	Room new_room(name, vertices);
	new_room.id = AddRoom(new_room);
	return new_room;
}

Entry RoomManager::CreateEntry(std::string name, std::vector <Point> vertices, int rooms[])
{
	Entry new_entry(name, vertices);
	new_entry.connectedRoomIds[0] = rooms[0];
	new_entry.connectedRoomIds[1] = rooms[1];
	new_entry.id = AttachEntry(new_entry);
	return new_entry;

}

Objects RoomManager::CreateObject(std::string name, std::vector <std::string> parameters)
{
	Objects new_object(name);
	new_object.properties = parameters;
	new_object.id = objects.size();
	objects.push_back(new_object);
	return new_object;
}

int RoomManager::AddRoom(Room room_to_add)
{
	room_to_add.id = rooms.size();
	rooms.push_back(room_to_add);
	return room_to_add.id;
}

int RoomManager::AttachEntry(Entry entry_to_add)
{
	entry_to_add.id = entries.size();
	rooms[entry_to_add.connectedRoomIds[0]].entryIds.push_back(entry_to_add.id);
	rooms[entry_to_add.connectedRoomIds[1]].entryIds.push_back(entry_to_add.id);
	entries.push_back(entry_to_add);
	return entry_to_add.id;
}

int RoomManager::AddObject(Objects object_to_add, int room_id)
{
	rooms[room_id].objectIds.push_back(object_to_add.id);
	return object_to_add.id;
}


void RoomManager::PrintRoom(int id)
{
	std::vector<Point> myvertex = rooms[id].vertex;
	std::vector <int> myentry = rooms[id].entryIds;
	std::vector <int> myobject = rooms[id].objectIds;

	std::cout << "Room id: " << rooms[id].id << std::endl;
	std::cout << "Room name: " << rooms[id].roomName << std::endl;
	std::cout << "Vertices: " << std::endl;
	for(auto i=myvertex.begin(); i != myvertex.end(); ++i)
	{
		std::cout << "  " << i->x << " , "  << i->y << std::endl;
	}
	std::cout << "Entries: " << std::endl;
	std::cout << "    ";
	for(auto i=myentry.begin(); i != myentry.end(); ++i)
	{
		std::cout <<  *i << " , ";
	}
	std::cout << std::endl;
	std::cout << "Objects: " << std::endl;
	std::cout << "    ";
	for(auto i=myobject.begin(); i != myobject.end(); ++i)
	{
		std::cout <<  *i << " , ";
	}
	std::cout << std::endl;
	std::cout << "-----" << std::endl;
	
}

void RoomManager::PrintEntry(int id)
{
	std::vector<Point> myvertex = entries[id].vertex;

	std::cout << "Entry id: " << entries[id].id << std::endl;
	std::cout << "Entry name: " << entries[id].entryName << std::endl;
	std::cout << "Vertices: " << std::endl;
	for(auto i=myvertex.begin(); i != myvertex.end(); ++i)
	{
		std::cout << "  " << i->x << " , "  << i->y << std::endl;
	}
	std::cout << "Connected Room Ids: " << entries[id].connectedRoomIds[0] << " , " << entries[id].connectedRoomIds[1] << std::endl;
	std::cout << "-----" << std::endl;
}
void RoomManager::PrintObject(int id)
{
	std::vector<std::string> myproperties = objects[id].properties;

	std::cout << "Object id: " << objects[id].id << std::endl;
	std::cout << "Object name: " << objects[id].objectName << std::endl;
	std::cout << "Properties: " << std::endl;
	std::cout << "    ";
	for(auto i=myproperties.begin(); i != myproperties.end(); ++i)
	{
		std::cout << *i << " , ";
	}
	std::cout << std::endl;
	std::cout << "-----" << std::endl;
	
}
int RoomManager::UpdateRoomName(int roomId, std::string new_name)
{
	rooms[roomId].roomName = new_name;
	return 0;
}
/*void RoomManager::LoadMap(const std::string& map_file)
{

	std::ifstream fin(map_file.c_str());
	if (fin.fail()) {
		exit(-1);
	}
	YAML::Node doc = YAML::Load(fin);

}*/

int RoomManager::GetCurrentRoomId(float x, float y)
{
	std::cout << "Coords: " << x << " , " << y;
	return 0;
}

int RoomManager::GetInitialRoomId(float x, float y)
{
	std::cout << "Coords: " << x << " , " << y;
	return 0;
}
