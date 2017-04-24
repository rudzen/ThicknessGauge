#pragma once


class Specs {

public:
	
	Specs() {
	};

private:

	const double pixMm_ = 3.45 * 0.001;

	const int defaultHeight_ = 2448;

	const int defaultWidth_ = 2050;

	typedef struct {
		int offset_x : 4;
		int offset_y : 4;
		int pos_x : 4;
		int pos_y : 4;
	} offset;

	offset offset_;




};