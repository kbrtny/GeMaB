#include "../include/room_manager.h"

//using Point = pt::Point;

int main() {
	RoomManager my_home;
	int temp;

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
	//my_home.PrintRoom(testrooma.id);
	//std::cout << my_home.RoomToJson(testrooma) << std::endl;
	my_home.UpdateRoomName(testrooma.id, "Foyer");

	Room testroomb = my_home.CreateRoom("roomb", roombcoords);
	//my_home.PrintRoom(testroomb.id);
	int roomsconnected[2] = {testrooma.id, testroomb.id};

	temp = my_home.GetCurrentRoomId(0.1,0.1);
	std::cout << "Currently in: " << my_home.GetRoomNameFromId(temp) << std::endl;

	my_home.GetCurrentRoomId(0,0);
	my_home.GetCurrentRoomId(5,6);
	my_home.GetCurrentRoomId(20,20);
	Entry firstentry = my_home.CreateEntry("firstentry", entrycoords, roomsconnected);
	//my_home.PrintEntry(firstentry.id);

	Objects firstobject = my_home.CreateObject("book", {"red", "big"});
	my_home.AddObject(firstobject, testrooma.id);
	//my_home.PrintObject(firstobject.id);

	//my_home.PrintRoom(testrooma.id);
}