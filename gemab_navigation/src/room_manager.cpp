#include "../include/room_manager.h"

void to_json(nlohmann::json& j, const Point& p)
{
	j = nlohmann::json{ { "x", p.x },{ "y", p.y },{ "z", p.z }};
}

void from_json(const nlohmann::json& j, Point& p)
{
	j.at("x").get_to(p.x);
	j.at("y").get_to(p.y);
	j.at("z").get_to(p.z);
}

void to_json(nlohmann::json& j, const Room& r)
{
	j = nlohmann::json{ 
		{ "id", r.id },
		{ "roomName", r.roomName },
		{ "vertex", r.vertex },
		{ "entryIds", r.entryIds },
		{ "objectIds", r.objectIds }
	};
}

void from_json(const nlohmann::json& j, Room& r)
{
	j.at("id").get_to(r.id);
	j.at("roomName").get_to(r.roomName);
	j.at("vertex").get_to(r.vertex);
	j.at("entryIds").get_to(r.entryIds);
	j.at("objectIds").get_to(r.objectIds);
}

void to_json(nlohmann::json& j, const Entry& r)
{
	j = nlohmann::json{ 
		{ "id", r.id },
		{ "entryName", r.entryName },
		{ "vertex", r.vertex },
		{ "connectedRoomIds", r.connectedRoomIds }
	};
}

void from_json(const nlohmann::json& j, Entry& r)
{
	j.at("id").get_to(r.id);
	j.at("entryName").get_to(r.entryName);
	j.at("vertex").get_to(r.vertex);
	j.at("connectedRoomIds").get_to(r.connectedRoomIds);
}

void to_json(nlohmann::json& j, const Objects& r)
{
	j = nlohmann::json{ 
		{ "id", r.id },
		{ "objectName", r.objectName },
		{ "properties", r.properties }
	};
}

void from_json(const nlohmann::json& j, Objects& r)
{
	j.at("id").get_to(r.id);
	j.at("objectName").get_to(r.objectName);
	j.at("properties").get_to(r.properties);
}

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
	PreCalcRoom(room_to_add.id);
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

void RoomManager::ExportToJSON(std::string filename)
{
	nlohmann::json j;
	j["Rooms"] = rooms;
	j["Entries"] = entries;
	j["Objects"] = objects;

	std::ofstream o;
	o.open(filename);

	o << std::setw(4) << j << std::endl;
	o.close();

}

int RoomManager::ImportFromJSON(std::string filename)
{
	nlohmann::json j;
	std::ifstream readFromFile(filename);
	if(readFromFile.is_open())
	{
		readFromFile >> j;
		rooms = j["Rooms"].get<std::vector<Room>>();
		entries = j["Entries"].get<std::vector<Entry>>();
		objects = j["Objects"].get<std::vector<Objects>>();
	}
	for(auto i=rooms.begin(); i != rooms.end(); ++i)
	{
		//PrintRoom(i->id);
		PreCalcRoom(i->id);
	}
	readFromFile.close();
	return 0;
}

//Base on http://alienryderflex.com/polygon/
void RoomManager::PreCalcRoom(int id)
{
	RoomPolygon new_roomPolygon(id, rooms[id].roomName, rooms[id].vertex.size());
	for(auto i=rooms[id].vertex.begin(); i != rooms[id].vertex.end(); ++i)
	{
		new_roomPolygon.polyX.push_back(i->x);
		new_roomPolygon.polyY.push_back(i->y);
	}
	uint i, j=rooms[id].vertex.size()-1;
	for(i=0; i < rooms[id].vertex.size(); i++)
	{
		if(new_roomPolygon.polyY[j]==new_roomPolygon.polyY[i])
		{
			new_roomPolygon.constant.push_back(new_roomPolygon.polyX[i]);
			new_roomPolygon.multiple.push_back(0);
		}else
		{
			new_roomPolygon.constant.push_back(new_roomPolygon.polyX[i]-(new_roomPolygon.polyY[i]*new_roomPolygon.polyX[j])/(new_roomPolygon.polyY[j]-new_roomPolygon.polyY[i])+(new_roomPolygon.polyY[j]*new_roomPolygon.polyX[i])/(new_roomPolygon.polyY[j]-new_roomPolygon.polyY[i]));
      		new_roomPolygon.multiple.push_back((new_roomPolygon.polyX[j]-new_roomPolygon.polyX[i])/(new_roomPolygon.polyY[j]-new_roomPolygon.polyY[i])); 
		}
		j = i;
	}
	computedRooms.push_back(new_roomPolygon);
}

std::string RoomManager::GetRoomNameFromId(int id)
{
	std::string name;
	for(auto i=rooms.begin(); i != rooms.end(); ++i)
	{
		if(i->id == id)
		{
			name = i->roomName;
		}
	}
	return name;
}

int RoomManager::GetCurrentRoomId(float x, float y)
{
	int currentroom = -1;
	for( auto i=computedRooms.begin(); i!= computedRooms.end(); ++i)
	{
		if(InRoom(x,y,*i))
		{
			currentroom = i->id;
			//std::cout << "Current Room is: " << i->roomName << std::endl;
		}
	}
	/*
	if(currentroom < 0)
	{
		std::cout << "Not in a room." << std::endl;
	}
	*/
	return currentroom;
}

bool RoomManager::InRoom(float x, float y, RoomPolygon test_room)
{
	uint i, j = test_room.polyCorners-1;
	bool oddNodes=0;

	for(i = 0; i<test_room.polyCorners; i++)
	{
		if((test_room.polyY[i] < y && test_room.polyY[j] >=y) 
			|| (test_room.polyY[j] < y && test_room.polyY[i] >=y))
		{
			oddNodes^=(y * test_room.multiple[i] + test_room.constant[i] < x);
		}
		j=i;
	}
	return oddNodes;
}

int RoomManager::GetInitialRoomId(float x, float y)
{
	std::cout << "Coords: " << x << " , " << y;
	return 0;
}
