#pragma once

#include <vector>
#include "tg.h"

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

namespace validate {

#ifdef CV_VERSION

	/**
	 * \brief Validates a rectangle that its boundries are all valid numbers
	 * \tparam T The type of rectangle
	 * \param rect The rectangle to validate
	 * \return true if its values are above 0, otherweise false
	 */
	template <typename T>
	inline
	bool validate_rect(cv::Rect_<T>& rect) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		auto valid_rectangle = [rect]()-> bool {
			return rect.width > 0.0 && rect.height > 0.0 && rect.x >= 0.0 && rect.y >= 0.0;
		};

		return valid_rectangle();
	}

	/**
	 * \brief Validates a rectangle based on the boundries of an image
	 * \tparam T Type
	 * \param rect The rectangle to validate
	 * \param boundry The boundry matrix to compare the rectangle boundries to
	 * \return true if the rectangle seems ok
	 */
	template <typename T>
	inline
	bool validate_rect(cv::Rect_<T>& rect, cv::Mat& boundry) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		auto valid_rectangle = [rect](cv::Rect_<T>& boundry)-> bool {

			if (!validate_rect(rect) || !validate_rect(boundry))
				return false;

			if (!validate_rect(rect))
				return false;

			return (rect & boundry).area() == boundry.area();

		};

		return valid_rectangle(cv::Rect(cv::Point(0, 0), boundry.size()));

	}

	/**
	 * \brief Validates a vector of points that all containing points are above zero in both x and y
	 * \tparam T The type
	 * \param vec The vector of points to validate
	 * \return true if all pixels are above 0 in both x and y, otherwise false
	 */
	template <typename T>
	inline
	bool valid_pix_vec(std::vector<cv::Point_<T>>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		if (vec.empty())
			return false;

		auto invalid_pix = [](cv::Point_<T>& p) {
			return p.x < 0 || p.y < 0;
		};

		auto it = find_if(vec.begin(), vec.end(), invalid_pix);

		return it != vec.end();

	}

	template <typename T, int cn>
	inline
	bool valid_vec2(cv::Vec<T, cn>& v) {
        for (auto i = 0; i < cn; i++)
            if (v[i] < 0)
                return false;

		return true;
	}


	/**
	 * \brief Validates the entirety of the data structure
	 * \param data Pointer for the data
	 * \return true if a-okay, otherwise false
	 */
	inline
	bool valid_data(std::shared_ptr<tg::Data> data) {

		// logging is temporary !
		using namespace tg;
		using namespace std;

		bool failed = false;

		if (data->globName.empty()) {
			failed = true;
		}
		else {

		}


		if (data->cameraPtr == nullptr) {
			log_time << "cameraPtr failed." << endl;
			failed = true;
		}
		else {
			log_time << "cameraPtr ok." << endl;
		}

		if (!valid_pix_vec(data->centerPoints)) {
			log_time << "centerPoints failed." << endl;
			failed = true;
		}
		else {
			log_time << "centerPoints ok." << endl;
		}

		if (!valid_pix_vec(data->leftPoints)) {
            log_time << "leftPoints failed." << endl;
            failed = true;
		}
		else {
            log_time << "centerPoints ok." << endl;
        }

		if (!valid_pix_vec(data->rightPoints)) {
            log_time << "rightPoints failed." << endl;
            failed = true;
		}
		else {
            log_time << "centerPoints ok." << endl;
        }

		if (!valid_vec2(data->pointsStart)) {
            log_time << "pointsStart failed." << endl;
        }
		else {
            log_time << "centerPoints ok." << endl;
        }

		if (!valid_vec2(data->leftBorder)) {
            log_time << "leftBorder failed." << endl;
            failed = true;
		}
		else {
            log_time << "centerPoints ok." << endl;
        }
        if (!valid_vec2(data->rightBorder)) {
            log_time << "rightBorder failed." << endl;
            failed = true;
		}
		else {
            log_time << "centerPoints ok." << endl;
        }

        if (!valid_vec2(data->centerLine)) {
            log_time << "centerLine failed." << endl;
            failed = true;
        } else {
            log_time << "centerPoints ok." << endl;
        }

        if (data->leftAvg < 0.0) {
            log_time << "leftAvg failed." << endl;
            failed = true;
        } else {
            log_time << "centerPoints ok." << endl;
        }

        if (data->centerAvg < 0.0) {
            log_time << "centerAvg failed." << endl;
            failed = true;
        } else {
            log_time << "centerPoints ok." << endl;
        }

        if (data->rightAvg < 0.0) {
            log_time << "rightAvg failed." << endl;
            failed = true;
        } else {
            log_time << "centerPoints ok." << endl;
        }

        if (data->difference < 0.0) {
            log_time << "difference failed." << endl;
            failed = true;
        } else {
            log_time << "centerPoints ok." << endl;
        }


	}


#endif


}
