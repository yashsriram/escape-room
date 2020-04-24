#ifndef ESCAPE_ROOM_CONFIGURATIONSPACE_H
#define ESCAPE_ROOM_CONFIGURATIONSPACE_H

using namespace ros;

struct ConfigurationSpace {
	const Room& room;

	explicit ConfigurationSpace(const Room& room): room(room) {}

	bool does_intersect(const Vector2f& end1, const Vector2f& end2) const {
		return false;
	}
};


#endif
