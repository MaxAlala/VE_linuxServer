#pragma once
#ifndef PROTOCOL_HPP_
#define PROTOCOL_HPP_

enum : unsigned
{
	MAX_IP_PACK_SIZE = 512,
	MAX_NICKNAME = 16,
	PADDING = 24,
	NUMBER_OF_ANGLES=2,
	NUMBER_OF_AXES = 2
};

class AxisBorders
{

public:
	AxisBorders(int l, int r, int h) {
		left = l;
		right = r;
		home = h;
	}

    int left;
    int right;
    int home;
};


#endif /* PROTOCOL_HPP_ */
