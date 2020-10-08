#include "../include/room_manager.h"

int main() {
	RoomManager my_home;
	std::vector<Point>  roomacoords {
		{0, 0, 0},
		{10, 0, 0},
		{10, 5, 0},
		{0, 5, 0}
	};
	std::vector<Point>  roombcoords {
		{0, 5, 0},
		{10, 5, 0},
		{10, 10, 0},
		{0, 10, 0}
	};
	std::vector<Point>  entrycoords {
		{4, 5, 0},
		{6, 5, 0}
	};


	Room testrooma = my_home.CreateRoom("rooma", roomacoords);
	my_home.PrintRoom(testrooma.id);
	my_home.UpdateRoomName(testrooma.id, "Foyer");
	
	Room testroomb = my_home.CreateRoom("roomb", roombcoords);
	my_home.PrintRoom(testroomb.id);
	int roomsconnected[2] = {testrooma.id, testroomb.id};

	Entry firstentry = my_home.CreateEntry("firstentry", entrycoords, roomsconnected);
	my_home.PrintEntry(firstentry.id);

	Objects firstobject = my_home.CreateObject("book", {"red", "big"});
	my_home.AddObject(firstobject, testrooma.id);
	my_home.PrintObject(firstobject.id);

	my_home.PrintRoom(testrooma.id);
}