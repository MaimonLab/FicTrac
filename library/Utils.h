#ifndef UTILS_H_
#define UTILS_H_

#include <boost/shared_array.hpp>

#include <string>
#include <deque>
#include <stdint.h>

class Utils
{
	public:
		static void				SET_PROCESS_PRIORITY(int priority);

		static bool				READ_FILE_TO_STRING(const std::string fn, std::string& ret);
		static std::string		GET_TOKEN(std::string& str, std::string dlim = "\t\n, ");
		static void				TOKENISE(const std::string& str,
										std::deque<std::string>& tokens,
										std::string dlim = "\t\n, ");

		static double			STR2NUM(std::string str);
		static std::string		NUM2STR(double num);

		static double			GEN_RAND_DBL(double min, double max);
		static double			GEN_RAND_GSN(double sigma, double mean=0);
		
		static double			GET_CLOCK();
		static std::string		GET_DATE_STRING();
};

#endif /*UTILS_H_*/

